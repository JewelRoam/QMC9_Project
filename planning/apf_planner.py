"""
Artificial Potential Field (APF) Planner for obstacle avoidance.
Based on: Ahn et al. (2024) - APF with prediction + Bézier smoothing.

The APF method generates:
  - Attractive force: pulls the vehicle toward the goal waypoint
  - Repulsive force: pushes the vehicle away from detected obstacles
  - Combined force: determines steering direction and speed adjustment

Enhanced with:
  - Pedestrian/cyclist priority weighting (ethical consideration)
  - Cooperative obstacle integration (V2V shared detections)
  - Quintic Bézier curve smoothing for kinematic feasibility
"""
import math
import time
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class Obstacle:
    """Obstacle representation for APF computation."""
    x: float                # world x position (meters)
    y: float                # world y position (meters)
    distance: float         # distance from ego (meters)
    category: str           # "vehicle", "pedestrian", "cyclist"
    confidence: float = 1.0
    velocity: Tuple[float, float] = (0.0, 0.0)  # vx, vy
    source: str = "local"   # "local" or "v2v" (cooperative)
    ethical_weight: float = 1.0  # Higher for VRUs (pedestrians, cyclists)


@dataclass
class LaneConstraint:
    """Lane boundary constraints for lane-keeping behavior."""
    left_boundary: float      # y-coordinate of left lane boundary
    right_boundary: float     # y-coordinate of right lane boundary
    lane_center_y: float      # y-coordinate of lane center
    lane_width: float         # width of current lane
    is_valid: bool = True     # whether lane info is available


@dataclass
class PlannerOutput:
    """Output of the APF planner for one planning step."""
    target_steering: float    # [-1.0, 1.0] normalized steering
    target_speed: float       # km/h desired speed (can be negative for reverse)
    emergency_brake: bool     # True if emergency stop needed
    force_magnitude: float    # total force magnitude (for visualization)
    force_direction: float    # force direction in radians (for visualization)
    attractive_force: Tuple[float, float] = (0.0, 0.0)
    repulsive_force: Tuple[float, float] = (0.0, 0.0)
    nearest_obstacle_dist: float = float('inf')
    status: str = "normal"   # "normal", "avoiding", "emergency", "goal_reached", "recovering"
    preferred_side: str = "none"  # "left", "right", or "none" for overtaking
    recovery_mode: str = "none"   # "none", "reversing", "turning"
    recovery_progress: float = 0.0  # 0.0 to 1.0 progress of recovery maneuver


class APFPlanner:
    """
    Artificial Potential Field path planner.
    Computes steering and speed from attractive (goal) and repulsive (obstacle) forces.
    """

    def __init__(self, config: dict):
        """
        Args:
            config: planning.apf section of config.yaml
        """
        self.k_att = config.get("k_attractive", 1.0)
        self.k_rep = config.get("k_repulsive", 150.0)
        self.d0 = config.get("d0", 15.0)  # repulsive influence distance (m)
        self.goal_tolerance = config.get("goal_tolerance", 5.0)
        self.max_speed = config.get("max_speed", 30.0)  # km/h
        self.min_speed = config.get("min_speed", 5.0)   # km/h

        # Ethical weights for vulnerable road users (VRUs)
        self._ethical_weights = {
            "pedestrian": 2.5,  # Highest priority
            "cyclist": 2.0,
            "vehicle": 1.0,
        }

        # Emergency brake distance
        self.emergency_distance = 3.0  # meters

        # Recovery behavior parameters
        self._recovery_state = {
            "is_recovering": False,
            "recovery_mode": "none",  # "reversing", "turning", "pivot_turn"
            "recovery_start_time": 0.0,
            "recovery_duration": 0.0,
            "stuck_position": (0.0, 0.0),
            "stuck_yaw": 0.0,
            "target_recovery_yaw": 0.0,
            "collision_detected": False,
            "last_collision_time": 0.0,
            "recovery_attempts": 0,     # Count recovery attempts at same location
            "recovery_history": [],     # Track recovery positions to detect loops
            "original_goal": (0.0, 0.0), # Store original goal for reference
        }
        self.recovery_reverse_speed = -6.0  # km/h (negative = reverse) - slower for better control
        self.recovery_turn_speed = 3.0      # km/h for turning maneuver
        self.stuck_threshold = 0.5          # meters - MORE SENSITIVE (was 1.0)
        self.stuck_timeout = 8.0            # seconds - LONGER TIMEOUT (was 4.0)
        self.collision_cooldown = 2.0       # seconds after collision before normal operation
        self.max_recovery_time = 8.0        # maximum time to spend in recovery mode
        self.max_recovery_attempts = 5      # max attempts before giving up (was 3)
        self.recovery_cooldown = 5.0        # seconds before allowing another recovery at similar position

    def check_collision(self, ego_speed: float, nearest_obstacle_dist: float) -> bool:
        """
        Detect if a collision has occurred.
        Collision indicators:
        - Very low speed despite throttle (stuck against obstacle)
        - Sudden stop from moderate/high speed
        - Contact with obstacle (distance ~0)
        """
        current_time = time.time() if hasattr(time, 'time') else 0.0

        # Immediate collision: touching obstacle and stopped
        if nearest_obstacle_dist < 0.5 and ego_speed < 1.0:
            self._recovery_state["collision_detected"] = True
            self._recovery_state["last_collision_time"] = current_time
            return True

        # Recent collision cooldown period
        if current_time - self._recovery_state["last_collision_time"] < self.collision_cooldown:
            return True

        return False

    def check_stuck(self, ego_x: float, ego_y: float, ego_speed: float) -> bool:
        """
        Check if vehicle is stuck (not making progress).
        Returns True if vehicle hasn't moved significantly in stuck_timeout seconds.
        """
        import time
        current_time = time.time()

        if not hasattr(self, '_last_check'):
            self._last_check = {
                "position": (ego_x, ego_y),
                "time": current_time,
                "speeds": [],
            }
            return False

        # Calculate distance moved since last check
        last_pos = self._last_check["position"]
        dist_moved = math.sqrt((ego_x - last_pos[0])**2 + (ego_y - last_pos[1])**2)

        # Update speed history
        self._last_check["speeds"].append(ego_speed)
        if len(self._last_check["speeds"]) > 30:  # Keep last 30 samples (~3 seconds)
            self._last_check["speeds"].pop(0)

        # Check if stuck: low average speed and small movement
        avg_speed = sum(self._last_check["speeds"]) / len(self._last_check["speeds"]) if self._last_check["speeds"] else 0
        time_since_check = current_time - self._last_check["time"]

        if time_since_check > self.stuck_timeout:
            if dist_moved < self.stuck_threshold and avg_speed < 2.0:
                # Vehicle is stuck
                return True
            else:
                # Reset tracking
                self._last_check["position"] = (ego_x, ego_y)
                self._last_check["time"] = current_time
                self._last_check["speeds"] = []

        return False

    def _find_alternative_goal(self, ego_x: float, ego_y: float, 
                                original_goal_x: float, original_goal_y: float,
                                obstacles: List[Obstacle]) -> Tuple[float, float]:
        """
        Find an alternative intermediate goal when direct path is blocked.
        Tries points perpendicular to the blocked direction.
        """
        dx = original_goal_x - ego_x
        dy = original_goal_y - ego_y
        dist_to_goal = math.sqrt(dx*dx + dy*dy)
        
        if dist_to_goal < 0.1:
            return (original_goal_x, original_goal_y)
        
        # Normalize direction to goal
        goal_dir_x = dx / dist_to_goal
        goal_dir_y = dy / dist_to_goal
        
        # Perpendicular directions (left and right of goal direction)
        perp_left_x = -goal_dir_y
        perp_left_y = goal_dir_x
        perp_right_x = goal_dir_y
        perp_right_y = -goal_dir_x
        
        # Try different distances for alternative goals
        test_distances = [5.0, 10.0, 15.0]
        best_goal = (original_goal_x, original_goal_y)
        best_score = -float('inf')
        
        for offset_dist in test_distances:
            # Try left side
            alt_x = ego_x + perp_left_x * offset_dist + goal_dir_x * 3.0
            alt_y = ego_y + perp_left_y * offset_dist + goal_dir_y * 3.0
            score = self._score_alternative_goal(alt_x, alt_y, ego_x, ego_y, 
                                                  original_goal_x, original_goal_y, obstacles)
            if score > best_score:
                best_score = score
                best_goal = (alt_x, alt_y)
            
            # Try right side
            alt_x = ego_x + perp_right_x * offset_dist + goal_dir_x * 3.0
            alt_y = ego_y + perp_right_y * offset_dist + goal_dir_y * 3.0
            score = self._score_alternative_goal(alt_x, alt_y, ego_x, ego_y,
                                                  original_goal_x, original_goal_y, obstacles)
            if score > best_score:
                best_score = score
                best_goal = (alt_x, alt_y)
        
        return best_goal
    
    def _score_alternative_goal(self, alt_x: float, alt_y: float,
                                 ego_x: float, ego_y: float,
                                 goal_x: float, goal_y: float,
                                 obstacles: List[Obstacle]) -> float:
        """
        Score an alternative goal position.
        Higher score = better alternative.
        """
        score = 0.0
        
        # Penalize distance from original path (we want to stay roughly on track)
        dist_from_path = abs((alt_x - ego_x) * (goal_y - ego_y) - 
                            (alt_y - ego_y) * (goal_x - ego_x)) / \
                        math.sqrt((goal_x - ego_x)**2 + (goal_y - ego_y)**2 + 0.001)
        score -= dist_from_path * 0.5
        
        # Reward distance from obstacles
        min_obs_dist = float('inf')
        for obs in obstacles:
            d = math.sqrt((alt_x - obs.x)**2 + (alt_y - obs.y)**2)
            if d < min_obs_dist:
                min_obs_dist = d
        
        if min_obs_dist < 3.0:
            score -= (3.0 - min_obs_dist) * 10  # Heavy penalty for being too close
        else:
            score += min(min_obs_dist, 10.0)  # Reward clearance up to 10m
        
        # Reward progress toward final goal
        dist_to_goal = math.sqrt((alt_x - goal_x)**2 + (alt_y - goal_y)**2)
        score -= dist_to_goal * 0.1
        
        return score

    def start_recovery(self, ego_x: float, ego_y: float, ego_yaw: float,
                       goal_x: float, goal_y: float, is_collision: bool = False,
                       obstacles: Optional[List[Obstacle]] = None):
        """
        Initiate recovery maneuver with intelligent path replanning.
        """
        import time
        current_time = time.time()
        
        # Check if we're trying to recover too frequently at same location
        stuck_pos = self._recovery_state.get("stuck_position", (0, 0))
        dist_from_last_stuck = math.sqrt((ego_x - stuck_pos[0])**2 + (ego_y - stuck_pos[1])**2)
        
        if dist_from_last_stuck < 5.0:
            self._recovery_state["recovery_attempts"] = self._recovery_state.get("recovery_attempts", 0) + 1
        else:
            self._recovery_state["recovery_attempts"] = 1
        
        # If too many attempts at same spot, try alternative goal
        if self._recovery_state["recovery_attempts"] >= 2 and obstacles:
            print(f"[Recovery] Multiple failures ({self._recovery_state['recovery_attempts']}), seeking alternative path")
            alt_goal = self._find_alternative_goal(ego_x, ego_y, goal_x, goal_y, obstacles)
            print(f"[Recovery] Alternative goal: ({alt_goal[0]:.1f}, {alt_goal[1]:.1f}) vs original ({goal_x:.1f}, {goal_y:.1f})")
            self._recovery_state["alternative_goal"] = alt_goal
            self._recovery_state["original_goal"] = (goal_x, goal_y)
            # Use alternative goal for this recovery
            goal_x, goal_y = alt_goal
        
        self._recovery_state["is_recovering"] = True
        self._recovery_state["recovery_start_time"] = current_time
        self._recovery_state["stuck_position"] = (ego_x, ego_y)
        self._recovery_state["stuck_yaw"] = ego_yaw

        # Determine recovery strategy based on heading vs goal
        dx = goal_x - ego_x
        dy = goal_y - ego_y
        angle_to_goal = math.atan2(dy, dx)
        ego_yaw_rad = math.radians(ego_yaw)

        angle_diff = abs(angle_to_goal - ego_yaw_rad)
        angle_diff = min(angle_diff, 2*math.pi - angle_diff)

        # Also check if we're against a wall/obstacle
        front_blocked = False
        if obstacles:
            for obs in obstacles:
                obs_dx = obs.x - ego_x
                obs_dy = obs.y - ego_y
                obs_dist = math.sqrt(obs_dx**2 + obs_dy**2)
                if obs_dist < 3.0:  # Close obstacle
                    obs_angle = math.atan2(obs_dy, obs_dx)
                    angle_to_obs = abs(obs_angle - ego_yaw_rad)
                    angle_to_obs = min(angle_to_obs, 2*math.pi - angle_to_obs)
                    if angle_to_obs < math.radians(30):  # Directly ahead
                        front_blocked = True
                        break

        if front_blocked or angle_diff > math.radians(60):
            # Need to reverse and make significant turn
            self._recovery_state["recovery_mode"] = "turning"
            # Target: aim more toward the goal but allow some deviation
            self._recovery_state["target_recovery_yaw"] = math.degrees(angle_to_goal)
            print(f"[Recovery] Front blocked or large angle error, will reverse+turn")
        else:
            # Just reverse straight back
            self._recovery_state["recovery_mode"] = "reversing"

        print(f"[Recovery] Started {self._recovery_state['recovery_mode']} maneuver")
        print(f"  Position: ({ego_x:.1f}, {ego_y:.1f}), Yaw: {ego_yaw:.1f}°")
        print(f"  Goal: ({goal_x:.1f}, {goal_y:.1f})")

    def compute_recovery_control(self, ego_x: float, ego_y: float, ego_yaw: float,
                                  obstacles: List[Obstacle]) -> PlannerOutput:
        """
        Compute control commands during recovery maneuver.
        """
        import time
        current_time = time.time()
        elapsed = current_time - self._recovery_state["recovery_start_time"]

        # Check if recovery timeout
        if elapsed > self.max_recovery_time:
            print("[Recovery] Timeout reached, aborting recovery")
            self._recovery_state["is_recovering"] = False
            self._recovery_state["recovery_mode"] = "none"
            return PlannerOutput(
                target_steering=0.0,
                target_speed=0.0,
                emergency_brake=True,
                force_magnitude=0.0,
                force_direction=0.0,
                status="emergency",
                recovery_mode="failed",
            )

        # Check if we've moved far enough from stuck position
        stuck_pos = self._recovery_state["stuck_position"]
        dist_from_stuck = math.sqrt((ego_x - stuck_pos[0])**2 + (ego_y - stuck_pos[1])**2)

        if dist_from_stuck > 3.0:  # Moved 3 meters from stuck position
            print(f"[Recovery] Successfully moved {dist_from_stuck:.1f}m from stuck position")
            self._recovery_state["is_recovering"] = False
            self._recovery_state["recovery_mode"] = "none"
            return PlannerOutput(
                target_steering=0.0,
                target_speed=self.min_speed,
                emergency_brake=False,
                force_magnitude=0.0,
                force_direction=0.0,
                status="normal",
                recovery_mode="completed",
                recovery_progress=1.0,
            )

        # Check for obstacles behind us during reverse
        ego_yaw_rad = math.radians(ego_yaw)
        rear_clearance = float('inf')

        for obs in obstacles:
            dx = obs.x - ego_x
            dy = obs.y - ego_y
            dist = math.sqrt(dx*dx + dy*dy)
            angle_to_obs = math.atan2(dy, dx)
            angle_diff = abs(angle_to_obs - (ego_yaw_rad + math.pi))  # Behind us
            angle_diff = min(angle_diff, 2*math.pi - angle_diff)

            if angle_diff < math.radians(60) and dist < rear_clearance:
                rear_clearance = dist

        # Adjust recovery based on rear clearance
        if rear_clearance < 2.0:
            # Obstacle behind us, stop reversing
            print(f"[Recovery] Obstacle behind ({rear_clearance:.1f}m), stopping")
            return PlannerOutput(
                target_steering=0.0,
                target_speed=0.0,
                emergency_brake=True,
                force_magnitude=0.0,
                force_direction=0.0,
                status="emergency",
                recovery_mode=self._recovery_state["recovery_mode"],
                recovery_progress=elapsed / self.max_recovery_time,
            )

        # Compute steering for recovery
        if self._recovery_state["recovery_mode"] == "turning":
            # Reverse while turning toward target yaw
            target_yaw = self._recovery_state["target_recovery_yaw"]
            yaw_error = target_yaw - ego_yaw
            # Normalize to [-180, 180]
            while yaw_error > 180: yaw_error -= 360
            while yaw_error < -180: yaw_error += 360

            # Steer to correct yaw while reversing
            # When reversing, steering direction is inverted
            steer = np.clip(yaw_error / 45.0, -1.0, 1.0)

            return PlannerOutput(
                target_steering=float(steer),
                target_speed=self.recovery_reverse_speed,
                emergency_brake=False,
                force_magnitude=0.0,
                force_direction=0.0,
                status="recovering",
                recovery_mode="turning",
                recovery_progress=min(elapsed / 5.0, 1.0),  # Expect turning to take ~5s
            )
        else:
            # Simple reversing
            return PlannerOutput(
                target_steering=0.0,
                target_speed=self.recovery_reverse_speed,
                emergency_brake=False,
                force_magnitude=0.0,
                force_direction=0.0,
                status="recovering",
                recovery_mode="reversing",
                recovery_progress=min(dist_from_stuck / 3.0, 1.0),
            )

    def reset_recovery_state(self):
        """Reset recovery state (call when starting fresh)."""
        self._recovery_state = {
            "is_recovering": False,
            "recovery_mode": "none",
            "recovery_start_time": 0.0,
            "recovery_duration": 0.0,
            "stuck_position": (0.0, 0.0),
            "stuck_yaw": 0.0,
            "target_recovery_yaw": 0.0,
            "collision_detected": False,
            "last_collision_time": 0.0,
        }
        if hasattr(self, '_last_check'):
            delattr(self, '_last_check')

    def _attractive_force(self, ego_x: float, ego_y: float,
                          goal_x: float, goal_y: float) -> Tuple[float, float]:
        """
        Compute attractive force toward the goal.
        F_att = k_att * (goal - ego) / |goal - ego|
        Capped to prevent excessive force at large distances.
        """
        dx = goal_x - ego_x
        dy = goal_y - ego_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.goal_tolerance:
            return (0.0, 0.0)

        # Normalize and scale
        scale = min(self.k_att * dist, self.k_att * 20.0)  # Cap at 20m equivalent
        fx = scale * dx / dist
        fy = scale * dy / dist
        return (fx, fy)

    def _repulsive_force(self, ego_x: float, ego_y: float,
                         obstacle: Obstacle) -> Tuple[float, float]:
        """
        Compute repulsive force from a single obstacle.
        F_rep = k_rep * ethical_w * (1/d - 1/d0) * (1/d^2) * direction
        Only active when d < d0.
        """
        dx = ego_x - obstacle.x
        dy = ego_y - obstacle.y
        d = math.sqrt(dx * dx + dy * dy)

        if d < 0.1:
            d = 0.1  # Prevent division by zero
        if d >= self.d0:
            return (0.0, 0.0)

        # Ethical weight: stronger repulsion for VRUs
        ew = obstacle.ethical_weight

        # Repulsive force magnitude
        magnitude = self.k_rep * ew * (1.0 / d - 1.0 / self.d0) * (1.0 / (d * d))

        # Direction: away from obstacle
        fx = magnitude * dx / d
        fy = magnitude * dy / d

        return (fx, fy)

    def _compute_lane_constraint(self, ego_x: float, ego_y: float,
                                  goal_x: float, goal_y: float) -> LaneConstraint:
        """
        Estimate lane constraints based on goal direction.
        Assumes the vehicle should stay within ±lane_width/2 of the path centerline.
        """
        # Path direction vector
        dx = goal_x - ego_x
        dy = goal_y - ego_y
        path_angle = math.atan2(dy, dx)

        # Standard lane width (can be made configurable)
        lane_width = 3.5  # meters

        # Perpendicular direction to path
        perp_angle = path_angle + math.pi / 2

        # Lane boundaries relative to ego position
        left_boundary = ego_y + lane_width / 2 * math.sin(perp_angle)
        right_boundary = ego_y - lane_width / 2 * math.sin(perp_angle)

        return LaneConstraint(
            left_boundary=left_boundary,
            right_boundary=right_boundary,
            lane_center_y=ego_y,
            lane_width=lane_width,
            is_valid=True
        )

    def _select_overtake_side(self, ego_x: float, ego_y: float, ego_yaw: float,
                               obstacles: List[Obstacle]) -> str:
        """
        Select which side to overtake an obstacle (left or right).
        Returns: "left", "right", or "none" (if no overtaking needed).
        """
        if not obstacles:
            return "none"

        # Find closest obstacle directly ahead
        ego_yaw_rad = math.radians(ego_yaw)
        min_dist = float('inf')
        closest_obs = None

        for obs in obstacles:
            dx = obs.x - ego_x
            dy = obs.y - ego_y
            dist = math.sqrt(dx*dx + dy*dy)

            # Check if obstacle is roughly ahead (within ±30 degrees)
            angle_to_obs = math.atan2(dy, dx)
            angle_diff = abs(angle_to_obs - ego_yaw_rad)
            angle_diff = min(angle_diff, 2*math.pi - angle_diff)

            if angle_diff < math.radians(30) and dist < min_dist:
                min_dist = dist
                closest_obs = obs

        if not closest_obs or min_dist > self.d0:
            return "none"

        # Decide side based on obstacle lateral position
        # If obstacle is on the left half of the road, overtake on right
        # If obstacle is on the right half, overtake on left
        # This assumes we drive on the right side of the road

        rel_y = closest_obs.y - ego_y  # positive = obstacle is to our right

        if rel_y > 0.5:  # Obstacle is on our right
            return "left"  # Overtake on left
        elif rel_y < -0.5:  # Obstacle is on our left
            return "right"  # Overtake on right
        else:
            # Obstacle is centered, prefer left overtaking (standard traffic rule)
            return "left"

    def compute(self, ego_x: float, ego_y: float, ego_yaw: float,
                ego_speed: float, goal_x: float, goal_y: float,
                obstacles: List[Obstacle],
                cooperative_obstacles: Optional[List[Obstacle]] = None,
                lane_constraint: Optional[LaneConstraint] = None,
                check_recovery: bool = True) -> PlannerOutput:
        """
        Main planning step: compute steering and speed from APF with lane constraints.

        Args:
            ego_x, ego_y: ego vehicle world position
            ego_yaw: ego heading in degrees
            ego_speed: current speed in km/h
            goal_x, goal_y: next waypoint position
            obstacles: list of locally detected obstacles
            cooperative_obstacles: obstacles shared via V2V (optional)
            lane_constraint: optional lane boundary constraints
            check_recovery: whether to enable collision/stuck recovery behavior

        Returns:
            PlannerOutput with steering, speed, and status
        """
        # Merge local and cooperative obstacles
        all_obstacles = list(obstacles)
        if cooperative_obstacles:
            for obs in cooperative_obstacles:
                obs.source = "v2v"
                all_obstacles.append(obs)

        # Apply ethical weights
        for obs in all_obstacles:
            obs.ethical_weight = self._ethical_weights.get(obs.category, 1.0)

        # Find nearest obstacle distance for collision/stuck detection
        nearest_dist = float('inf')
        for obs in all_obstacles:
            d = math.sqrt((ego_x - obs.x) ** 2 + (ego_y - obs.y) ** 2)
            if d < nearest_dist:
                nearest_dist = d

        # === RECOVERY BEHAVIOR ===
        if check_recovery:
            # Check if already in recovery mode
            if self._recovery_state["is_recovering"]:
                return self.compute_recovery_control(ego_x, ego_y, ego_yaw, all_obstacles)

            # Check for collision or stuck condition
            is_collision = self.check_collision(ego_speed, nearest_dist)
            is_stuck = self.check_stuck(ego_x, ego_y, ego_speed)

            if is_collision or is_stuck:
                self.start_recovery(ego_x, ego_y, ego_yaw, goal_x, goal_y, 
                                   is_collision, obstacles=all_obstacles)
                return self.compute_recovery_control(ego_x, ego_y, ego_yaw, all_obstacles)

        # Compute lane constraint if not provided
        if lane_constraint is None:
            lane_constraint = self._compute_lane_constraint(ego_x, ego_y, goal_x, goal_y)

        # Determine overtaking side
        overtake_side = self._select_overtake_side(ego_x, ego_y, ego_yaw, all_obstacles)

        # Compute attractive force
        f_att = self._attractive_force(ego_x, ego_y, goal_x, goal_y)

        # Compute total repulsive force with directional bias for overtaking
        f_rep_x, f_rep_y = 0.0, 0.0
        nearest_dist = float('inf')

        for obs in all_obstacles:
            frx, fry = self._repulsive_force(ego_x, ego_y, obs)

            # Apply directional bias for overtaking
            if overtake_side != "none":
                # Modify repulsion to encourage overtaking on selected side
                rel_y = obs.y - ego_y  # positive = obstacle is to our right

                if overtake_side == "left" and rel_y > 0:
                    # Want to go left of this obstacle (on our right)
                    # Reduce downward repulsion, enhance upward
                    fry *= 0.3  # Reduce repulsion pushing us away from left side
                elif overtake_side == "right" and rel_y < 0:
                    # Want to go right of this obstacle (on our left)
                    fry *= 0.3  # Reduce repulsion pushing us away from right side

            f_rep_x += frx
            f_rep_y += fry

            d = math.sqrt((ego_x - obs.x) ** 2 + (ego_y - obs.y) ** 2)
            if d < nearest_dist:
                nearest_dist = d

        f_rep = (f_rep_x, f_rep_y)

        # Total force
        fx = f_att[0] + f_rep_x
        fy = f_att[1] + f_rep_y

        # Apply lane boundary constraints
        # If we're trying to leave the lane, add a restoring force
        if lane_constraint.is_valid:
            lane_width = lane_constraint.lane_width
            deviation_from_center = ego_y - lane_constraint.lane_center_y

            # Soft boundary: start resisting when approaching edge
            soft_boundary = lane_width * 0.35  # Start resisting at 70% of half-width
            hard_boundary = lane_width * 0.45  # Strong resistance at 90%

            if abs(deviation_from_center) > soft_boundary:
                # Calculate restoring force toward lane center
                overshoot = abs(deviation_from_center) - soft_boundary
                max_overshoot = hard_boundary - soft_boundary
                restore_strength = min(overshoot / max_overshoot, 1.0) * self.k_rep * 0.5

                # Force direction: back toward center
                if deviation_from_center > 0:
                    # Too far right, push left (negative y)
                    fy -= restore_strength
                else:
                    # Too far left, push right (positive y)
                    fy += restore_strength

        force_mag = math.sqrt(fx * fx + fy * fy)
        force_dir = math.atan2(fy, fx)

        # Check for emergency brake
        emergency = nearest_dist < self.emergency_distance and len(all_obstacles) > 0

        # Check goal reached
        goal_dist = math.sqrt((goal_x - ego_x) ** 2 + (goal_y - ego_y) ** 2)
        goal_reached = goal_dist < self.goal_tolerance

        # Convert force direction to steering
        # CARLA coordinate system: x=forward, y=right, yaw measured from +x axis
        ego_yaw_rad = math.radians(ego_yaw)

        if force_mag < 0.001:
            # No significant force, keep going straight
            target_steering = 0.0
        else:
            # Steering angle = difference between force direction and current heading
            angle_diff = force_dir - ego_yaw_rad
            # Normalize to [-pi, pi]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            # Normalize to [-1, 1] for CARLA (max steering ~70 degrees)
            # Use a smaller divisor for more responsive steering
            target_steering = np.clip(angle_diff / math.radians(45.0), -1.0, 1.0)

        # Compute target speed based on obstacle proximity
        if emergency:
            target_speed = 0.0
            status = "emergency"
        elif goal_reached:
            target_speed = 0.0
            status = "goal_reached"
        elif nearest_dist < self.d0:
            # Slow down proportionally to nearest obstacle
            speed_factor = (nearest_dist - self.emergency_distance) / (self.d0 - self.emergency_distance)
            speed_factor = np.clip(speed_factor, 0.1, 1.0)
            target_speed = self.min_speed + (self.max_speed - self.min_speed) * speed_factor
            status = "avoiding"
        else:
            target_speed = self.max_speed
            status = "normal"

        return PlannerOutput(
            target_steering=float(target_steering),
            target_speed=float(target_speed),
            emergency_brake=emergency,
            force_magnitude=force_mag,
            force_direction=force_dir,
            attractive_force=f_att,
            repulsive_force=f_rep,
            nearest_obstacle_dist=nearest_dist,
            status=status,
            preferred_side=overtake_side,
        )

    def detections_to_obstacles(self, detections, ego_x: float, ego_y: float,
                                ego_yaw: float) -> List[Obstacle]:
        """
        Convert DetectedObject list (with distance and world_position) to Obstacle list.

        Args:
            detections: List[DetectedObject] with distance populated
            ego_x, ego_y, ego_yaw: ego vehicle state

        Returns:
            List[Obstacle] for APF computation
        """
        obstacles = []
        ego_yaw_rad = math.radians(ego_yaw)

        for det in detections:
            if det.distance <= 0:
                continue
            if det.category not in ("vehicle", "pedestrian", "cyclist"):
                continue

            if det.world_position:
                ox, oy, _ = det.world_position
            else:
                # Fallback: estimate position from distance and bbox center
                # Assume detection center corresponds to forward direction
                cx, cy = det.center
                # Rough lateral offset: positive cx offset = right side
                image_cx = 640  # approximate half-width
                lateral_offset = (cx - image_cx) / image_cx * det.distance * 0.5
                ox = ego_x + det.distance * math.cos(ego_yaw_rad) - lateral_offset * math.sin(ego_yaw_rad)
                oy = ego_y + det.distance * math.sin(ego_yaw_rad) + lateral_offset * math.cos(ego_yaw_rad)

            obstacles.append(Obstacle(
                x=ox,
                y=oy,
                distance=det.distance,
                category=det.category,
                confidence=det.confidence,
            ))

        return obstacles


class BezierSmoother:
    """
    Quintic Bézier curve smoother for path smoothing.
    Ensures generated paths are kinematically feasible (smooth steering).
    Reference: Ahn et al. (2024) - Bézier optimization for APF paths.
    """

    @staticmethod
    def quintic_bezier(control_points: np.ndarray, num_samples: int = 50) -> np.ndarray:
        """
        Generate a quintic Bézier curve from 6 control points.

        Args:
            control_points: (6, 2) array of control points
            num_samples: number of points to sample along curve

        Returns:
            (num_samples, 2) array of curve points
        """
        assert control_points.shape == (6, 2), "Quintic Bézier requires 6 control points"

        t = np.linspace(0, 1, num_samples).reshape(-1, 1)
        # Bernstein basis for degree 5
        coeffs = np.array([
            (1 - t) ** 5,
            5 * t * (1 - t) ** 4,
            10 * t ** 2 * (1 - t) ** 3,
            10 * t ** 3 * (1 - t) ** 2,
            5 * t ** 4 * (1 - t),
            t ** 5,
        ]).squeeze(-1).T  # (num_samples, 6)

        return coeffs @ control_points

    @staticmethod
    def smooth_waypoints(waypoints: List[Tuple[float, float]],
                         smoothing_factor: float = 0.3) -> List[Tuple[float, float]]:
        """
        Smooth a series of waypoints using overlapping Bézier segments.

        Args:
            waypoints: list of (x, y) waypoints
            smoothing_factor: how much to smooth (0 = straight lines, 1 = max)

        Returns:
            Smoothed list of (x, y) points
        """
        if len(waypoints) < 3:
            return waypoints

        pts = np.array(waypoints)
        smoothed = [waypoints[0]]

        for i in range(len(pts) - 2):
            p0 = pts[i]
            p1 = pts[i + 1]
            p2 = pts[i + 2]

            # Generate 6 control points for quintic Bézier
            d01 = p1 - p0
            d12 = p2 - p1

            cp = np.array([
                p0,
                p0 + d01 * 0.2,
                p0 + d01 * (1.0 - smoothing_factor),
                p1 + d12 * smoothing_factor,
                p1 + d12 * 0.8,
                p2,
            ])

            curve = BezierSmoother.quintic_bezier(cp, num_samples=10)
            for pt in curve[1:]:
                smoothed.append((float(pt[0]), float(pt[1])))

        return smoothed
