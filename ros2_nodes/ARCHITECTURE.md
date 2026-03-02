
## Overview

This directory contains ROS 2 node wrappers for the existing QMC9 project. The design follows a **layered architecture** that maintains complete isolation between:

1. **Simulation Layer** (CARLA-specific)
2. **Algorithm Layer** (ROS-agnostic, pure Python)
3. **Interface Layer** (ROS 2 nodes as thin wrappers)

## Directory Structure

```
ros2_nodes/
├── README.md                    # Usage documentation
├── ARCHITECTURE.md              # This file - architectural overview
├── carla_ros_interface.py       # CARLA ↔ ROS 2 bridge (simulation layer)
├── perception_node.py           # Perception module wrapper
├── planning_node.py             # Planning module wrapper
├── control_node.py              # Control module wrapper
├── cooperation_node.py          # Cooperation module wrapper
├── run_simulation.py            # Simplified runner script
└── launch/
    └── simulation.launch.py     # ROS 2 launch file
```

## Layered Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         INTERFACE LAYER                                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────┐ │
│  │ perception_node │  │ planning_node   │  │    control_node         │ │
│  │                 │  │                 │  │                         │ │
│  │ Sub: /sensors/* │  │ Sub: /percept/* │  │ Sub: /planning/*        │ │
│  │ Pub: /percept/* │  │ Pub: /planning/*│  │ Pub: /control/cmd_vel   │ │
│  └────────┬────────┘  └────────┬────────┘  └────────────┬────────────┘ │
│           │                    │                        │               │
│           └────────────────────┼────────────────────────┘               │
│                                │                                        │
│                     ROS 2 Topics (Standard Interfaces)                   │
├────────────────────────────────┼────────────────────────────────────────┤
│                      SIMULATION INTERFACE LAYER                          │
│                                │                                        │
│              ┌─────────────────┴─────────────────┐                      │
│              │      carla_ros_interface.py       │                      │
│              │                                   │                      │
│              │  • Publishes sensor data          │                      │
│              │  • Subscribes to cmd_vel          │                      │
│              │  • Manages CARLA connection       │                      │
│              └─────────────────┬─────────────────┘                      │
│                                │                                        │
├────────────────────────────────┼────────────────────────────────────────┤
│                           CARLA SIMULATOR                                │
└────────────────────────────────┴────────────────────────────────────────┘
```

## Key Design Principles

### 1. Zero Modification to Existing Code

The existing modules (`perception/`, `planning/`, `control/`, `cooperation/`) remain completely unchanged. ROS 2 nodes are thin wrappers that:
- Subscribe to ROS topics
- Convert ROS messages to/from Python data structures
- Call existing module methods
- Publish results back to ROS topics

### 2. Single Responsibility

Each layer has one job:
- **Existing modules**: Implement algorithms (pure Python)
- **ROS 2 nodes**: Handle ROS communication
- **CARLA interface**: Bridge to simulator

### 3. Topic-Based Communication

All inter-module communication happens via ROS 2 topics:

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/sensors/camera/rgb/image_raw` | sensor_msgs/Image | carla_interface | perception_node |
| `/sensors/camera/depth/image_raw` | sensor_msgs/Image | carla_interface | perception_node |
| `/perception/detections` | vision_msgs/Detection2DArray | perception_node | planning_node |
| `/perception/obstacles` | visualization_msgs/MarkerArray | perception_node | planning_node, cooperation_node |
| `/planning/path` | nav_msgs/Path | planning_node | control_node |
| `/planning/target_speed` | std_msgs/Float64 | planning_node | control_node |
| `/control/cmd_vel` | geometry_msgs/Twist | control_node | carla_interface |
| `/v2v/outgoing` | std_msgs/String | cooperation_node | (network) |
| `/v2v/incoming` | std_msgs/String | (network) | cooperation_node |

## Running the System

### Prerequisites

1. **CARLA 0.9.13** installed at `../CARLA_0.9.13/`
2. **ROS 2 Humble** or newer installed and sourced
3. **Python dependencies**: `pip install numpy opencv-python`

### Quick Start

```bash
# Terminal 1: Start CARLA
cd CARLA_0.9.13/WindowsNoEditor
./CarlaUE4.exe -RenderOffScreen

# Terminal 2: Source ROS 2 and run
source /opt/ros/humble/setup.bash
cd /path/to/QMC9_Project
python ros2_nodes/run_simulation.py --mode simulation
```

### Algorithm-Only Mode (No CARLA)

For testing without CARLA:

```bash
# Terminal 1: Play recorded sensor data
ros2 bag play sensor_data.db3

# Terminal 2: Run algorithm stack only
python ros2_nodes/run_simulation.py --mode algorithm_only
```

## Testing Individual Components

### Test Perception Only

```bash
# Terminal 1: Publish test image
ros2 topic pub /sensors/camera/rgb/image_raw sensor_msgs/Image \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera'}, \
    height: 480, width: 640, encoding: 'rgb8', \
    step: 1920, data: <your_image_data>}"

# Terminal 2: Run perception node
python -c "from ros2_nodes.perception_node import main; main()"
```

### Monitor Topics

```bash
# List all active topics
ros2 topic list

# Echo specific topic
ros2 topic echo /perception/detections

# Visualize in RViz
ros2 run rviz2 rviz2
```

## Extending the System

### Adding a New Sensor

1. Add sensor setup in `carla_ros_interface.py`:
```python
def _setup_lidar(self, vehicle):
    lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
    # ... configure lidar
    lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
    lidar.listen(self._on_lidar_data)
    return lidar
```

2. Add publisher:
```python
self.lidar_pub = self.create_publisher(
    PointCloud2, '/sensors/lidar/points', qos
)
```

3. Add callback to convert and publish:
```python
def _on_lidar_data(self, data):
    # Convert CARLA lidar to ROS PointCloud2
    pointcloud = self._convert_lidar_to_ros(data)
    self.lidar_pub.publish(pointcloud)
```

### Adding a New Algorithm Node

1. Create new file `ros2_nodes/my_algorithm_node.py`:
```python
import rclpy
from rclpy.node import Node

class MyAlgorithmNode(Node):
    def __init__(self):
        super().__init__('my_algorithm_node')
        
        # Import your existing module
        from my_module import MyAlgorithm
        self.algorithm = MyAlgorithm()
        
        # Setup subscribers
        self.sub = self.create_subscription(
            InputMsg, '/input/topic', self._callback, 10
        )
        
        # Setup publishers
        self.pub = self.create_publisher(OutputMsg, '/output/topic', 10)
    
    def _callback(self, msg):
        # Convert ROS message to Python data
        data = self._ros_to_python(msg)
        
        # Call existing algorithm
        result = self.algorithm.process(data)
        
        # Convert result to ROS message
        output_msg = self._python_to_ros(result)
        self.pub.publish(output_msg)
```

2. Add to `run_simulation.py`:
```python
from ros2_nodes.my_algorithm_node import MyAlgorithmNode
# ... in start method
my_node = MyAlgorithmNode()
self.executor.add_node(my_node)
```

## Debugging Tips

### Enable Debug Logging

```bash
# Set log level to DEBUG
export RCUTILS_LOGGING_SEVERITY=DEBUG
python ros2_nodes/run_simulation.py
```

### Check Node Status

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /perception_node
```

### Profile Performance

```bash
# Monitor topic frequencies
ros2 topic hz /perception/detections

# Check for dropped messages
ros2 topic bw /sensors/camera/rgb/image_raw
```

## Integration with Real Vehicle

To deploy on Raspberry Pi:

1. Copy `ros2_nodes/` and required modules to RPi
2. Install ROS 2 on RPi (or use micro-ROS)
3. Modify `carla_ros_interface.py` to read from real sensors instead of CARLA
4. Keep algorithm nodes unchanged - they work with any sensor source!

See `rpi_deploy/` for deployment scripts.

## Summary

This architecture provides:
- ✅ **Clean separation** between simulation and algorithms
- ✅ **Testability** - each layer can be tested independently
- ✅ **Flexibility** - easy to swap CARLA for real sensors
- ✅ **Extensibility** - add new sensors/algorithms without changing existing code
- ✅ **Maintainability** - ROS-specific code isolated in wrapper nodes
