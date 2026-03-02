"""
Intersection Scenario Test for CARLA
Tests the TTI-based intersection coordination in a controlled scenario.

This script creates a specific intersection scenario where two vehicles
approach from perpendicular directions to test the cooperative planner.
"""
import os
import sys
import time
import yaml
import cv2
import math
import traceback
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation.carla_env import CarlaEnv
from perception.detector import YOLODetector
from perception.depth_estimator import DepthEstimator
from planning.apf_planner import APFPlanner
from control.vehicle_controller import VehicleController
from cooperation.v2v_message import V2VCommunicator
from cooperation.cooperative_planner import CooperativePlanner
from utils.logger import setup_logger, FPSCounter


def load_config(config_path: str = "config/config.yaml") -> dict:
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


class IntersectionTestAgent:
    """Simplified agent for intersection testing."""
    
    def __init__(self, vehicle_id: str, config: dict, managed_vehicle):
        self.vehicle_id = vehicle_id
        self.config = config
        self.vehicle = managed_vehicle
        
        # Planning & Control
        self.apf_planner = APFPlanner(config['planning']['apf'])
        self.controller = VehicleController(config['control'], platform="carla")
        
        # Cooperation
        comm_config = config.get('cooperation', {}).get('communication', {})
        comm_config['protocol'] = 'in_process'
        self.v2v_comm = V2VCommunicator(vehicle_id, comm_config)
        self.coop_planner = CooperativePlanner(vehicle_id, config.get('cooperation', {}))
        
        # State
        self.latest_coop_decision = None
        self.decision_history = []
        
    def step(self):
        """Execute one step."""
        # Get ego state
        ego_x, ego_y, _ = self.vehicle.get_location()
        ego_yaw = self.vehicle.get_yaw()
        ego_speed = self.vehicle.get_speed_kmh()
        
        # V2V Communication
        msg = self.v2v_comm.create_message(
            position=(ego_x, ego_y, ego_yaw),
            velocity=(ego_speed, ego_yaw),
            detections=[],
            intent="cruising",
        )
        self.v2v_comm.broadcast(msg)
        
        # Receive and process
        v2v_messages = self.v2v_comm.receive_all()
        self.latest_coop_decision = self.coop_planner.process(
            ego_x, ego_y, ego_yaw, ego_speed, [], v2v_messages
        )
        
        # Record decision
        self.decision_history.append({
            'time': time.time(),
            'position': (ego_x, ego_y),
            'speed': ego_speed,
            'decision': self.latest_coop_decision.action if self.latest_coop_decision else None,
            'reason': self.latest_coop_decision.reason if self.latest_coop_decision else None,
        })
        
        # Simple waypoint following
        self.vehicle.advance_waypoint(threshold=3.0)
        goal = self.vehicle.get_next_waypoint()
        
        if goal:
            goal_x, goal_y = goal
            
            # Compute control toward goal
            dx = goal_x - ego_x
            dy = goal_y - ego_y
            target_yaw = math.degrees(math.atan2(dy, dx))
            
            yaw_error = target_yaw - ego_yaw
            while yaw_error > 180:
                yaw_error -= 360
            while yaw_error < -180:
                yaw_error += 360
            
            steering = max(-1.0, min(1.0, yaw_error / 45.0))
            
            # Apply speed adjustment from cooperative decision
            base_speed = 20.0  # km/h
            if self.latest_coop_decision:
                base_speed *= self.latest_coop_decision.speed_adjustment
            
            control = {
                'throttle': 0.5 if ego_speed < base_speed else 0.0,
                'brake': 0.5 if self.latest_coop_decision and self.latest_coop_decision.action == "yield" else 0.0,
                'steer': steering,
            }
            
            self.controller.apply_carla_control(self.vehicle.actor, control)
        
        return self.vehicle.get_rgb_image()


def spawn_at_location(env, vehicle_id, x, y, yaw, blueprint_filter=None):
    """Spawn a vehicle at a specific location with collision check retry."""
    import carla
    
    if blueprint_filter is None:
        blueprint_filter = env.config.get('ego_vehicle', {}).get('blueprint_filter', 'vehicle.tesla.model3')
    
    blueprint = env.world.get_blueprint_library().filter(blueprint_filter)[0]
    
    # Try multiple Z heights to avoid collision
    z_heights = [0.5, 1.0, 2.0, 5.0]
    actor = None
    
    for z in z_heights:
        transform = carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(yaw=yaw)
        )
        
        # Use try_spawn_actor which returns None on collision instead of raising
        actor = env.world.try_spawn_actor(blueprint, transform)
        if actor is not None:
            print(f"[CARLA] Spawned {vehicle_id} at ({x:.1f}, {y:.1f}, {z:.1f})")
            break
        else:
            print(f"[CARLA] Spawn attempt at z={z} failed for {vehicle_id}, retrying...")
    
    if actor is None:
        raise RuntimeError(f"Failed to spawn {vehicle_id} at ({x}, {y}) - collision at all heights")
    
    # Wrap in managed vehicle - need to pass world as 3rd argument
    from simulation.carla_env import ManagedVehicle
    managed = ManagedVehicle(vehicle_id, actor, env.world, spawn_transform=transform)
    env.vehicles[vehicle_id] = managed
    
    return managed


def create_intersection_route(env, vehicle, waypoints_xy):
    """Create a route through specified waypoints (x,y tuples)."""
    import carla
    
    # Convert (x,y) tuples to CARLA Waypoint objects
    carla_waypoints = []
    for x, y in waypoints_xy:
        loc = carla.Location(x=x, y=y, z=0.5)
        wp = env.map.get_waypoint(loc, project_to_road=True)
        if wp:
            carla_waypoints.append(wp)
    
    if len(carla_waypoints) < 2:
        print(f"[WARNING] Only {len(carla_waypoints)} valid waypoints found")
    
    vehicle.set_route(carla_waypoints)
    print(f"[CARLA] Set route with {len(carla_waypoints)} waypoints for {vehicle.vehicle_id}")


def main():
    """Run intersection scenario test."""
    config = load_config()
    logger = setup_logger("IntersectionTest", "INFO")
    
    # Reset V2V
    V2VCommunicator.reset_shared_bus()
    
    logger.info("Connecting to CARLA...")
    env = CarlaEnv(config)
    env.connect()
    
    try:
        logger.info("Setting up intersection scenario...")
        
        # Scenario: Two vehicles approaching intersection from perpendicular directions
        # Intersection center at (0, 0)
        # Vehicle A: South to North, starting at (0, -50)
        # Vehicle B: West to East, starting at (-50, 0)
        
        logger.info("Spawning Vehicle A (South approach)...")
        vehicle_a = spawn_at_location(env, "vehicle_A", 0.0, -40.0, 0.0)
        
        logger.info("Spawning Vehicle B (West approach)...")
        vehicle_b = spawn_at_location(env, "vehicle_B", -40.0, 0.0, 90.0)
        
        # Attach cameras
        sensor_cfg = config.get('sensors', {})
        vehicle_a.attach_rgb_camera(sensor_cfg.get('rgb_camera', {}))
        vehicle_b.attach_rgb_camera(sensor_cfg.get('rgb_camera', {}))
        
        # Create routes through intersection
        # Vehicle A: (0, -40) -> (0, 0) -> (0, 40)
        route_a = [(0.0, -30.0), (0.0, -15.0), (0.0, 0.0), (0.0, 15.0), (0.0, 30.0)]
        create_intersection_route(env, vehicle_a, route_a)
        
        # Vehicle B: (-40, 0) -> (0, 0) -> (40, 0)
        route_b = [(-30.0, 0.0), (-15.0, 0.0), (0.0, 0.0), (15.0, 0.0), (30.0, 0.0)]
        create_intersection_route(env, vehicle_b, route_b)
        
        # Create agents
        logger.info("Creating agents...")
        agent_a = IntersectionTestAgent("vehicle_A", config, vehicle_a)
        agent_b = IntersectionTestAgent("vehicle_B", config, vehicle_b)
        agents = [agent_a, agent_b]
        
        # Setup display
        cv2.namedWindow("Vehicle A", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Vehicle B", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Vehicle A", 640, 480)
        cv2.resizeWindow("Vehicle B", 640, 480)
        
        logger.info("=" * 60)
        logger.info("Intersection Scenario Test Started")
        logger.info("Vehicle A: Approaching from South")
        logger.info("Vehicle B: Approaching from West")
        logger.info("Press 'q' to stop")
        logger.info("=" * 60)
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            env.tick()
            
            # Step all agents
            for i, agent in enumerate(agents):
                rgb = agent.step()
                
                if rgb is not None:
                    display = rgb.copy()
                    h, w = display.shape[:2]
                    
                    # Add info overlay
                    speed = agent.vehicle.get_speed_kmh()
                    loc = agent.vehicle.get_location()
                    coop = agent.latest_coop_decision
                    
                    cv2.rectangle(display, (0, 0), (w, 100), (0, 0, 0), -1)
                    cv2.putText(display, f"{agent.vehicle_id}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(display, f"Speed: {speed:.1f} km/h", (10, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(display, f"Pos: ({loc[0]:.1f}, {loc[1]:.1f})", (10, 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    if coop:
                        color = (0, 255, 0) if coop.action == "proceed" else \
                                (0, 0, 255) if coop.action == "yield" else (0, 165, 255)
                        cv2.putText(display, f"Action: {coop.action.upper()}", (w-200, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        cv2.putText(display, coop.reason[:40], (w-300, 55),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                    
                    win_name = f"Vehicle {'A' if i == 0 else 'B'}"
                    cv2.imshow(win_name, cv2.cvtColor(display, cv2.COLOR_RGB2BGR))
            
            # Log status every 2 seconds
            elapsed = time.time() - start_time
            if frame_count % 40 == 0:  # At 20 FPS, every 2 seconds
                dist = math.sqrt(
                    (agent_a.vehicle.get_location()[0] - agent_b.vehicle.get_location()[0])**2 +
                    (agent_a.vehicle.get_location()[1] - agent_b.vehicle.get_location()[1])**2
                )
                logger.info(f"t={elapsed:.1f}s | Distance between vehicles: {dist:.1f}m")
                for agent in agents:
                    if agent.latest_coop_decision:
                        logger.info(f"  {agent.vehicle_id}: {agent.latest_coop_decision.action} - {agent.latest_coop_decision.reason}")
            
            frame_count += 1
            
            key = cv2.waitKey(50) & 0xFF  # 20 FPS
            if key == ord('q'):
                break
            
            # Stop after 30 seconds
            if elapsed > 30:
                logger.info("Test duration reached (30s)")
                break
        
        # Print summary
        logger.info("\n" + "=" * 60)
        logger.info("Test Summary")
        logger.info("=" * 60)
        for agent in agents:
            logger.info(f"\n{agent.vehicle_id} Decision History:")
            for record in agent.decision_history[::10]:  # Every 10th record
                logger.info(f"  t={record['time']-start_time:.1f}s: {record['decision']} - {record['reason'][:50]}")
        
    except Exception as e:
        logger.error(f"Error: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        V2VCommunicator.reset_shared_bus()
        env.cleanup()
        logger.info("Test finished")


if __name__ == '__main__':
    main()
