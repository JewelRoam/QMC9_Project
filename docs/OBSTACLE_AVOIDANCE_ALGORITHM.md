# Obstacle Avoidance Algorithm Documentation

## Overview

This document describes the Artificial Potential Field (APF) based obstacle avoidance algorithm implemented in `planning/apf_planner.py`. The algorithm is enhanced with lane constraints, ethical considerations, collision recovery, and V2V cooperative perception.

**Reference**: Ahn et al. (2024) - APF with prediction + Bézier smoothing

---

## Core Algorithm: Artificial Potential Field

### Basic Principle

The APF method treats the vehicle as a particle moving in a potential field where:
- **Goal (attractor)**: Creates an attractive force pulling the vehicle toward the destination
- **Obstacles (repulsors)**: Create repulsive forces pushing the vehicle away
- **Result**: The vehicle follows the gradient of the combined potential field

### Mathematical Formulation

#### Attractive Force (Goal)
```
F_att = k_att * (goal - ego) / |goal - ego|
```
Where:
- `k_att`: Attractive force coefficient (configurable, default: 1.0)
- Capped at 20m equivalent to prevent excessive force at large distances

#### Repulsive Force (Obstacles)
```
F_rep = k_rep * w_ethical * (1/d - 1/d0) * (1/d²) * direction
```
Where:
- `k_rep`: Repulsive force coefficient (configurable, default: 150.0)
- `w_ethical`: Ethical weight for vulnerable road users
- `d`: Distance to obstacle
- `d0`: Influence distance threshold (default: 15.0m)
- Active only when `d < d0`

#### Combined Force
```
F_total = F_att + Σ(F_rep_i)
```

---

## Enhancements

### 1. Lane Constraint System

**Problem**: Basic APF causes vehicles to swerve onto sidewalks or opposite lanes.

**Solution**: Soft lane boundary constraints that add restoring force when approaching lane edges.

```python
if deviation_from_center > soft_boundary:
    restore_strength = proportional_to_overshoot * k_rep * 0.5
    F_y += restore_strength  # Push back toward center
```

**Parameters**:
- Lane width: 3.5m (standard)
- Soft boundary: 70% of half-width (1.225m from center)
- Hard boundary: 90% of half-width (1.575m from center)

### 2. Overtaking Side Selection

When encountering obstacles, the algorithm decides which side to overtake:

1. Find closest obstacle within ±30° ahead
2. If obstacle is on right side → overtake on left
3. If obstacle is on left side → overtake on right
4. If centered → prefer left (standard traffic rule)

**Implementation**: Modifies repulsive force direction to bias toward selected side.

### 3. Ethical Weighting

Different obstacle types receive different repulsion strengths:

| Category      | Weight | Rationale                          |
|---------------|--------|------------------------------------|
| Pedestrian    | 2.5    | Highest priority (VRU protection)  |
| Cyclist       | 2.0    | High priority (VRU protection)     |
| Vehicle       | 1.0    | Standard priority                  |

### 4. Collision Recovery System

**Detection Conditions**:
- Collision: Distance < 0.5m AND speed < 1 km/h
- Stuck: No significant movement (< 1m) for 4 seconds

**Recovery Strategy Selection**:
The system analyzes the situation to choose the best recovery approach:
1. Check if front is blocked by obstacles (within 3m, ±30° cone)
2. Calculate angle difference between current heading and goal direction
3. If front blocked OR angle > 60°, use turning mode; otherwise simple reversing

**Recovery Modes**:

#### Mode 1: Simple Reversing
- Triggered when facing approximately toward goal (< 60° error) and path ahead is clear
- Action: Reverse straight back at -6 km/h
- Exit condition: Moved 3m from stuck position

#### Mode 2: Reverse + Turn
- Triggered when facing significantly away from goal (> 60° error) or front is blocked
- Action: Reverse while steering toward goal direction
- Steering inversion applied for reverse motion (intuitive control)

#### Mode 3: Alternative Path Planning (Smart Recovery)
When multiple recovery attempts fail at the same location (≥2 attempts within 5m):
1. System searches for alternative intermediate goals perpendicular to blocked path
2. Evaluates candidate points based on:
   - Distance from original path (penalty for deviation)
   - Clearance from obstacles (reward for safety margin)
   - Progress toward final goal
3. Selects best alternative and uses it as temporary goal
4. After reaching alternative goal, resumes navigation to original destination

**Safety Features**:
- Rear obstacle detection during reverse (60° cone behind vehicle)
- Abort if obstacle detected within 2m behind
- Maximum recovery time: 8 seconds per attempt
- Maximum 3 recovery attempts before giving up
- Recovery cooldown: 5 seconds between attempts at same location
- Timeout aborts with emergency brake

---

## Control Flow

```
┌─────────────────────────────────────────────────────────────┐
│                     PLANNING STEP                           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │ Check Recovery   │
                    │ State            │
                    └────────┬─────────┘
                             │
              ┌──────────────┴──────────────┐
              │ In Recovery?                │
              └────────┬────────────────────┘
                       │
         ┌─────────────┴─────────────┐
        YES                           NO
         │                            │
         ▼                            ▼
┌──────────────────┐      ┌──────────────────────┐
│ Execute Recovery │      │ Check Collision/Stuck│
│ Maneuver         │      │                      │
└────────┬─────────┘      └──────────┬───────────┘
         │                           │
         │              ┌────────────┴───────────┐
         │              │ Need Recovery?         │
         │              └────────┬───────────────┘
         │                       │
         │          ┌────────────┴───────────┐
         │         YES                        NO
         │          │                        │
         │          ▼                        ▼
         │  ┌──────────────┐      ┌────────────────────┐
         │  │ Start        │      │ Normal APF Compute │
         │  │ Recovery     │      │                    │
         │  └──────────────┘      └────────────────────┘
         │                               │
         └───────────────────────────────┤
                                         ▼
                              ┌────────────────────┐
                              │ Apply Lane         │
                              │ Constraints        │
                              └────────┬───────────┘
                                       │
                              ┌────────▼───────────┐
                              │ Convert to         │
                              │ Steering & Speed   │
                              └────────────────────┘
```

---

## Configuration Parameters

All parameters are configured in `config/config.yaml` under `planning.apf`:

```yaml
planning:
  apf:
    k_attractive: 1.0        # Goal attraction strength
    k_repulsive: 150.0       # Obstacle repulsion strength
    d0: 15.0                 # Repulsion influence distance (m)
    max_speed: 30.0          # Maximum speed (km/h)
    min_speed: 5.0           # Minimum speed (km/h)
    goal_tolerance: 5.0      # Waypoint reached threshold (m)
    emergency_distance: 3.0  # Emergency brake distance (m)
```

### Parameter Tuning Guidelines

| Scenario                  | k_att | k_rep | d0  | Effect                                  |
|---------------------------|-------|-------|-----|-----------------------------------------|
| Conservative (safe)       | 1.0   | 200   | 20  | Early avoidance, slower speeds          |
| Aggressive (fast)         | 1.5   | 100   | 12  | Late avoidance, higher speeds           |
| Narrow roads              | 1.0   | 180   | 15  | Stronger lane keeping                   |
| Highway driving           | 1.2   | 120   | 25  | Smooth lane changes                     |

Use the automated tuning framework to find optimal parameters:
```bash
python -m testing.automated_runner --tune --iterations 20
```

---

## Integration with Perception

### Input: DetectedObject List

The planner receives detections from YOLO with depth estimation:

```python
@dataclass
class DetectedObject:
    category: str           # "vehicle", "pedestrian", "cyclist"
    confidence: float       # Detection confidence [0, 1]
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[float, float]      # (cx, cy) in image coords
    distance: float         # Depth-estimated distance (m)
    world_position: Optional[Tuple[float, float, float]]  # (x, y, z)
```

### Conversion to Obstacles

The `detections_to_obstacles()` method:
1. Filters by valid categories (vehicle, pedestrian, cyclist)
2. Uses world_position if available, otherwise estimates from distance
3. Assigns ethical weights based on category

---

## Integration with Control

### Output: PlannerOutput

```python
@dataclass
class PlannerOutput:
    target_steering: float      # [-1.0, 1.0] normalized
    target_speed: float         # km/h (can be negative for reverse)
    emergency_brake: bool       # Immediate stop required
    force_magnitude: float      # For visualization
    force_direction: float      # For visualization
    status: str                 # "normal", "avoiding", "emergency", etc.
    preferred_side: str         # "left", "right", "none"
    recovery_mode: str          # "none", "reversing", "turning"
```

### Speed Adaptation

Speed is modulated based on obstacle proximity:

```
if nearest_dist < emergency_distance:
    speed = 0                    # Stop
elif nearest_dist < d0:
    speed = min_speed + (max_speed - min_speed) * factor
else:
    speed = max_speed            # Full speed
```

---

## V2V Cooperative Integration

### Cooperative Obstacles

Vehicles can share detected obstacles via V2V communication:

```python
# Merge local and cooperative obstacles
all_obstacles = local_obstacles + v2v_obstacles

for obs in all_obstacles:
    obs.ethical_weight = get_weight(obs.category)
```

### Benefits

1. **Occlusion Compensation**: See obstacles hidden behind other vehicles
2. **Extended Range**: Detect obstacles beyond sensor range
3. **Early Warning**: Receive alerts about distant hazards

---

## Testing and Validation

### Automated Test Scenarios

The testing framework (`testing/`) provides:

1. **Scenario Generator**: Creates diverse test cases
   - Straight road with obstacles
   - Intersection crossings
   - Occlusion scenarios
   - Pedestrian crossings

2. **Metrics Collector**: Records performance data
   - Collision count
   - Near misses
   - Average speed
   - Lane deviation

3. **Parameter Tuner**: Auto-optimizes APF coefficients

### Running Tests

```bash
# Quick validation (5 scenarios)
python -m testing.automated_runner --suite quick

# Full evaluation (15 scenarios)
python -m testing.automated_runner --suite standard

# Parameter optimization
python -m testing.automated_runner --tune --iterations 20

# Apply best configuration
python testing/apply_best_config.py
```

---

## Known Limitations and Future Work

### Current Limitations

1. **Local Minima**: APF can get stuck in local minima (rare with proper tuning)
2. **Lane Estimation**: Lane constraints are estimated from goal direction, not actual lane markings
3. **Static Obstacles Only**: No prediction of dynamic obstacle trajectories
4. **Single Lane**: No multi-lane overtaking logic

### Planned Improvements

1. **Trajectory Prediction**: Integrate obstacle velocity prediction
2. **Multi-Lane Planning**: Add lane change decision making
3. **Learning-Based Tuning**: Reinforcement learning for parameter optimization
4. **HD Map Integration**: Use actual lane boundaries from map data

---

## References

1. Khatib, O. (1986). Real-time obstacle avoidance for manipulators and mobile robots.
2. Ahn et al. (2024). Local Path Planning with Predictive Virtual Force Field.
3. Wang et al. (2024). Research on Autonomous Vehicle Obstacle Avoidance with Social Ethics.

---

## Change Log

| Date       | Version | Changes                                      |
|------------|---------|----------------------------------------------|
| 2025-02-28 | 1.0     | Initial APF implementation                   |
| 2025-03-01 | 1.1     | Added lane constraints and overtaking logic  |
| 2025-03-01 | 1.2     | Added collision recovery system              |

---

*Last Updated: 2025-03-01*
*Maintainer: Development Team*
