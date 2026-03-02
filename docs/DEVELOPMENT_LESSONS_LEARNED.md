# Development Lessons Learned - CARLA Simulation Project

## Overview
This document captures key lessons, rules, and best practices learned during the development and debugging of the autonomous vehicle simulation project.

---

## 1. CARLA Physics and Vehicle Control

### 1.1 Never Use set_transform on Moving Vehicles
**Problem**: Calling `actor.set_transform()` on a vehicle that already has physics simulation enabled causes the vehicle to flip/roll uncontrollably.

**Root Cause**: CARLA's physics engine maintains momentum and collision state. Teleporting the vehicle mid-simulation creates physical inconsistencies.

**Solution**: 
- Set vehicle orientation ONLY at spawn time using the spawn point's rotation
- If reorientation is needed, destroy and respawn the vehicle
- Alternatively, use `set_simulate_physics(False)` before transform change, then re-enable

**Example (WRONG)**:
```python
# DON'T DO THIS - causes vehicle to flip
vehicle.actor.set_transform(new_transform)
```

**Example (CORRECT)**:
```python
# Set orientation at spawn time only
spawn_point = carla.Transform(location, rotation)
actor = world.spawn_actor(blueprint, spawn_point)
```

### 1.2 Spawn Point Selection Matters
**Lesson**: CARLA spawn points have predefined rotations that align with road direction. Using random spawn points can lead to vehicles facing away from their intended path.

**Best Practice**: 
- Accept the natural spawn point rotation
- Design navigation logic to handle arbitrary initial orientations
- Skip waypoints that are too close/behind the vehicle rather than trying to reorient

---

## 2. Route Planning and Waypoint Management

### 2.1 GlobalRoutePlanner API
**Key Insight**: CARLA's `GlobalRoutePlanner` returns a list of `(waypoint, road_option)` tuples, not just waypoints.

**Correct Usage**:
```python
from agents.navigation.global_route_planner import GlobalRoutePlanner

grp = GlobalRoutePlanner(map, sampling_resolution=2.0)
route_trace = grp.trace_route(start_loc, end_location)
waypoints = [wp for wp, _ in route_trace]  # Extract just the waypoints
```

### 2.2 Initial Waypoint Skipping
**Problem**: When a vehicle spawns, the first few waypoints may be behind or very close to it, causing immediate recovery behavior.

**Solution**: Skip waypoints within a threshold distance (e.g., 20m) of the start:
```python
skip_count = 0
for i, wp in enumerate(waypoints):
    dist = math.sqrt((wp.transform.location.x - start_loc.x)**2 + 
                     (wp.transform.location.y - start_loc.y)**2)
    if dist < 20.0 and i < len(waypoints) - 5:
        skip_count = i + 1
    else:
        break

if skip_count > 0:
    waypoints = waypoints[skip_count:]
```

### 2.3 Route Start Location Consistency
**Critical**: Ensure the route planning uses the SAME location as where the vehicle spawned.

**Common Bug**: Using current vehicle position after spawn vs. original spawn point can cause mismatches if the vehicle moved slightly during initialization.

**Solution**: Store spawn transform and use it for route planning:
```python
class ManagedVehicle:
    def __init__(self, ..., spawn_transform=None):
        self.spawn_transform = spawn_transform  # Save original spawn
        
# Later when generating route:
start_loc = vehicle.spawn_transform.location  # Use saved spawn, not current pos
```

---

## 3. Recovery Behavior Tuning

### 3.1 Recovery Trigger Sensitivity
**Parameters to tune** (in `planning/apf_planner.py`):
- `stuck_threshold`: Distance threshold to consider vehicle stuck (default: 0.5m)
- `stuck_timeout`: Time before triggering recovery (default: 8.0s)
- `max_recovery_attempts`: Max retries before giving up (default: 5)

**Lesson**: More sensitive detection (lower threshold) with longer timeout prevents false positives while still catching real stuck situations.

### 3.2 Recovery Mode Types
The planner supports three recovery modes:
1. **reversing**: Simple backward movement
2. **turning**: Reverse while steering toward goal
3. **pivot_turn**: In-place rotation (not fully implemented)

**Selection Logic**: Based on angle difference to goal and front obstacle detection.

---

## 4. Multi-Vehicle Display Architecture

### 4.1 Window Layout Strategy
For N vehicles, create:
- Individual vehicle windows (RGB camera view)
- Single BEV (Bird's Eye View) window showing all vehicles

**Layout Formula**:
```python
# Arrange vehicle windows in a grid
grid_size = math.ceil(math.sqrt(num_vehicles))
window_x = (i % grid_size) * (WINDOW_WIDTH + PADDING)
window_y = (i // grid_size) * (WINDOW_HEIGHT + PADDING)
```

### 4.2 BEV Rendering Considerations
- Use OpenCV drawing functions (`cv2.circle`, `cv2.line`, `cv2.polylines`)
- Scale world coordinates to image coordinates with proper scaling factor
- Draw waypoint paths as polylines for smooth visualization
- Use different colors for different vehicles

### 4.3 Coordinate Transformations
**World to BEV Image**:
```python
def world_to_bev(x, y, center_x, center_y, scale, img_height):
    """Convert world coordinates to BEV image coordinates."""
    img_x = int((x - center_x) * scale + img_width / 2)
    img_y = int(img_height / 2 - (y - center_y) * scale)
    return (img_x, img_y)
```

---

## 5. Configuration Management

### 5.1 Config.yaml Structure
Keep all tunable parameters in `config/config.yaml`:
```yaml
planning:
  apf:
    k_attractive: 1.0
    k_repulsive: 150.0
    d0: 15.0
    max_speed: 30.0
    min_speed: 5.0
```

### 5.2 Parameter Tuning Workflow
1. Run `testing/parameter_tuner.py` to find optimal values
2. Results saved to `output/best_apf_config.json`
3. Apply with `testing/apply_best_config.py`
4. Backup old config before applying changes

---

## 6. Debugging Techniques

### 6.1 Visual Debugging
- Always display vehicle status (speed, FPS, control mode) on screen
- Use color coding: green=normal, yellow=avoiding, red=emergency/recovery
- Show waypoint paths in BEV view

### 6.2 Log Analysis
Key metrics to log:
- Perception latency (ms)
- Planning latency (ms)
- Total frame time (ms)
- Current speed (km/h)
- Status (normal/avoiding/emergency/recovering)

### 6.3 Common Issues and Solutions

| Issue | Likely Cause | Solution |
|-------|-------------|----------|
| Vehicle flips on start | `set_transform` called after spawn | Remove post-spawn transform changes |
| Immediate recovery trigger | First waypoint behind vehicle | Skip initial close waypoints |
| Vehicle doesn't move | Goal too far/speed too low | Check target_speed calculation |
| Jerky steering | PID gains too high | Reduce kp/kd in controller |
| False obstacle detection | Depth estimation error | Tune depth confidence threshold |

---

## 7. Code Organization Best Practices

### 7.1 Module Separation
- `perception/`: Detection and depth estimation
- `planning/`: APF planner and path generation
- `control/`: Vehicle control (PID, platform abstraction)
- `simulation/`: CARLA environment and demo scripts
- `cooperation/`: V2V communication and cooperative planning
- `testing/`: Evaluation and tuning tools

### 7.2 Platform Abstraction
The `VehicleController` class abstracts platform differences:
- CARLA: throttle/steer/brake commands
- Raspberry Pi: PWM motor commands
- Future: STM32/Micro-ROS interface

### 7.3 Type Hints and Documentation
Use type hints and docstrings for clarity:
```python
def compute(self, ego_x: float, ego_y: float, ego_yaw: float,
            ego_speed: float, goal_x: float, goal_y: float,
            obstacles: List[Obstacle]) -> PlannerOutput:
    """Main planning step: compute steering and speed from APF.
    
    Args:
        ego_x, ego_y: ego vehicle world position
        ego_yaw: ego heading in degrees
        ...
    
    Returns:
        PlannerOutput with steering, speed, and status
    """
```

---

## 8. Performance Optimization

### 8.1 Synchronous Mode
Always use synchronous mode for reproducible results:
```python
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05  # 20 FPS
```

### 8.2 Sensor Data Handling
- Process sensor data in callbacks
- Copy data immediately to avoid reference issues
- Use numpy arrays for efficient processing

### 8.3 Perception Pipeline
Current bottleneck: YOLO inference (~60ms)
- Consider TensorRT/OpenVINO optimization for deployment
- Use smaller model (YOLOv8n) for faster inference

---

## 9. Testing and Validation

### 9.1 Automated Testing
Use `testing/automated_runner.py` for systematic evaluation:
- Multiple scenarios
- Metrics collection
- Report generation

### 9.2 Key Metrics
- Success rate (reaching destination)
- Collision rate
- Average speed
- Smoothness (steering jerk)
- Perception latency

---

## 10. Future Improvements

### 10.1 Cooperative Perception
- Implement full V2V message exchange
- Fuse detections from multiple vehicles
- Handle communication delays

### 10.2 Real-World Deployment
- Test on Raspberry Pi with actual hardware
- Calibrate camera intrinsics
- Validate depth estimation accuracy

### 10.3 Advanced Planning
- Model Predictive Control (MPC) instead of pure APF
- Traffic rule compliance
- Pedestrian intention prediction

---

## Summary Checklist

Before running simulation:
- [ ] CARLA server is running
- [ ] Config file has correct parameters
- [ ] Model files exist (yolov8n.pt)
- [ ] Output directory exists

When adding new features:
- [ ] Update relevant module
- [ ] Add configuration parameters if needed
- [ ] Update documentation
- [ ] Test with single vehicle first
- [ ] Then test multi-vehicle scenario

When debugging:
- [ ] Check logs for error messages
- [ ] Verify coordinate transformations
- [ ] Confirm physics settings
- [ ] Review recovery behavior triggers

## 11. ROS 2 Integration (Windows)

### 11.1 Python Version Conflict Resolution
**Problem**: CARLA requires Python 3.7, but ROS 2 Jazzy requires Python 3.12.

**Failed Approaches**:
1. Installing rclpy in a virtual environment - DLL load failures
2. Using system Python 3.12 - missing ROS 2 specific paths
3. Direct imports across Python versions - interpreter crashes

**Solution**: ZeroMQ-based dual-environment architecture
```
CARLA (Py3.7) ←→ ZeroMQ ←→ ROS 2 (Py3.12)
```

### 11.2 Windows ROS 2 Environment Setup
**Critical Discovery**: ROS 2 Jazzy on Windows uses pixi's bundled Python 3.12
- **Correct Python Path**: `C:\pixi_ws\.pixi\envs\default\python.exe`
- **Must call**: `setup.bat` before importing rclpy
- **Do NOT create**: Separate virtual environments for ROS 2

**Working Pattern**:
```batch
@echo off
chcp 65001 >nul
set "ROS2_PATH=C:\pixi_ws\ros2-windows"

REM Source ROS 2 environment (REQUIRED!)
call %ROS2_PATH%\setup.bat

REM Use pixi's Python explicitly
set "PYTHON_EXE=C:\pixi_ws\.pixi\envs\default\python.exe"

REM Now rclpy will work
%PYTHON_EXE% -c "import rclpy; print('OK')"
```

### 11.3 Common ROS 2 Errors on Windows

| Error | Cause | Solution |
|-------|-------|----------|
| `DLL load failed: _rclpy_pybind11` | Not sourced setup.bat | Call setup.bat first |
| `cannot import 'Protocol' from 'typing'` | Using Python 3.7 for ROS 2 | Use Python 3.12 from pixi |
| `ModuleNotFoundError: No module named 'rclpy'` | Wrong Python environment | Use pixi's Python, not venv |
| `RTI Connext DDS not found` | Warning only, can ignore | Add to PATH or ignore |

### 11.4 ZeroMQ Communication Pattern
**Architecture**:
- CARLA Bridge (Py3.7): Publishes sensor data via ZMQ
- ZMQ-to-ROS Node (Py3.12): Subscribes and converts to ROS topics

**Port Assignment**:
- 5555: RGB images (PUB-SUB)
- 5556: Depth images (PUB-SUB)
- 5557: Odometry (PUB-SUB)
- 5558: Control commands (SUB-PUB reverse direction)

**Key Implementation Details**:
- Use multipart messages for image data: `[metadata_json, image_binary]`
- Set `zmq.LINGER` to 0 for clean shutdown
- Use `zmq.SNDHWM` and `zmq.RCVHWM` to prevent memory issues

### 11.5 Environment Cleanup
**Removed** (no longer needed):
- `.venv_ros2` - Virtual environment for ROS 2 (use pixi instead)
- `setup_ros2_env.bat` - Obsolete script
- References to `venv310` - Replaced with `venv_carla`

**Current Structure**:
- `venv_carla/` - Python 3.7 for CARLA bridge
- Pixi environment - Python 3.12 for ROS 2 (managed by ROS 2 installer)

---

*Last Updated: March 2, 2026*
*Author: Cline Assistant*
