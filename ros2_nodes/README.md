# ROS 2 Nodes for QMC9

This directory contains ROS 2 node implementations that wrap the existing QMC9 modules.
The design follows **low coupling, high cohesion** principles with a **dual-environment architecture**
to handle Python version conflicts between CARLA (3.7) and ROS 2 Jazzy (3.12).

## Dual-Environment Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Python 3.7 Environment (CARLA)                       │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                    carla_bridge/carla_zmq_bridge.py                   │  │
│  │  • Connects to CARLA simulator                                         │  │
│  │  • Publishes sensor data via ZeroMQ (ports 5555-5558)                  │  │
│  │  • Receives control commands from ROS 2                                │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────┬─────────────────────────────────────────┘
                                    │ ZeroMQ IPC
                                    │ (Cross-Python communication)
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        Python 3.12 Environment (ROS 2)                       │
│                                                                              │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────────────┐  │
│  │   zmq_to_ros    │    │ perception_node │    │      planning_node      │  │
│  │    _node.py     │◄───│                 │◄───│                         │  │
│  │                 │    │ • YOLO detection │    │  • APF path planning    │  │
│  │ • Receives ZMQ  │    │ • Depth estimation│   │  • Obstacle avoidance   │  │
│  │ • Publishes ROS │    │ • Publishes      │    │  • Publishes path       │  │
│  │   topics        │    │   detections     │    │                         │  │
│  └────────┬────────┘    └─────────────────┘    └────────────┬────────────┘  │
│           │                                                  │               │
│           └──────────────────────────────────────────────────┘               │
│                              │                                               │
│                     ROS 2 Topics (Standard interfaces)                        │
│                                                                              │
│  Topics:                                                                     │
│    /sensors/camera/rgb/image_raw    → sensor_msgs/Image                      │
│    /sensors/camera/depth/image_raw  → sensor_msgs/Image                      │
│    /sensors/odometry                → nav_msgs/Odometry                      │
│    /perception/detections           → vision_msgs/Detection2DArray         │
│    /planning/path                   → nav_msgs/Path                          │
│    /control/cmd_vel                 → geometry_msgs/Twist                    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Directory Structure

```
ros2_nodes/
├── README.md                           # This file
├── ARCHITECTURE.md                     # Detailed architecture documentation
├── DUAL_ENV_SETUP.md                   # Dual environment setup guide
│
├── run_carla_bridge.bat                # Launch CARLA bridge (Python 3.7)
├── run_ros_stack.bat                   # Launch ROS 2 stack (Python 3.12)
├── setup_carla_env.bat                 # Setup CARLA Python 3.7 environment
│
├── carla_bridge/                       # Python 3.7 CARLA interface
│   ├── __init__.py
│   ├── carla_zmq_bridge.py             # CARLA ↔ ZeroMQ bridge
│   └── requirements.txt                # Python 3.7 dependencies
│
├── zmq_to_ros_node.py                  # ZeroMQ → ROS 2 bridge (Python 3.12)
├── perception_node.py                  # Perception ROS 2 node
├── planning_node.py                    # Planning ROS 2 node
├── control_node.py                     # Control ROS 2 node
├── cooperation_node.py                 # Multi-vehicle cooperation node
├── run_simulation.py                   # Main simulation runner
│
└── launch/
    └── simulation.launch.py            # ROS 2 launch file
```

## Quick Start

### Prerequisites

1. **CARLA 0.9.13** installed at `CARLA_0.9.13/`
2. **Python 3.7** for CARLA compatibility
3. **ROS 2 Jazzy** installed at `C:\pixi_ws\ros2-windows` (includes Python 3.12 via pixi)
4. **ZeroMQ** library (`pip install pyzmq` in both environments)

> **Important**: ROS 2 Jazzy on Windows uses its own Python 3.12 from `C:\pixi_ws\.pixi\envs\default\python.exe`. 
> Do NOT use a separate virtual environment for ROS 2 - use the pixi environment that comes with ROS 2.

### Option 1: Automated Setup

Run the setup script to create the CARLA Python 3.7 environment:

```batch
setup_carla_env.bat
```

This will:
- Install Python 3.7 if not present
- Create `.venv_carla` virtual environment
- Install required dependencies

### Option 2: Manual Setup

If you prefer to manually set up the Python 3.7 environment:

```batch
# Create and activate virtual environment
python -m venv venv_carla
venv_carla\Scripts\activate

# Install dependencies
pip install pyzmq numpy opencv-python
```

### ROS 2 Environment Setup (No Action Needed)

The `run_ros_stack.bat` script automatically:
- Sources ROS 2 environment via `setup.bat`
- Uses the correct Python 3.12 from pixi environment
- Installs any missing Python packages

## Usage

### Step 1: Start CARLA Simulator

```batch
cd CARLA_0.9.13\WindowsNoEditor
CarlaUE4.exe -RenderOffScreen
```

Or with display:
```batch
CarlaUE4.exe
```

### Step 2: Start CARLA Bridge (Terminal 1)

```batch
ros2_nodes\run_carla_bridge.bat
```

Or with mock data (no CARLA connection):
```batch
ros2_nodes\run_carla_bridge.bat --mock
```

### Step 3: Start ROS 2 Stack (Terminal 2)

```batch
ros2_nodes\run_ros_stack.bat
```

This launches:
- ZMQ to ROS bridge node
- Perception node (object detection)
- Planning node (APF path planning)
- Control node (vehicle control)
- Cooperation node (multi-vehicle coordination)

## Communication Protocol

### ZeroMQ Ports

| Port | Direction | Data |
|------|-----------|------|
| 5555 | CARLA → ROS | RGB camera images |
| 5556 | CARLA → ROS | Depth camera images |
| 5557 | CARLA → ROS | Vehicle odometry |
| 5558 | ROS → CARLA | Control commands |

### Message Format

Images are sent as multipart messages:
```
[metadata_json] [image_binary_data]
```

Metadata example:
```json
{
  "timestamp": 1234567890.123,
  "width": 1280,
  "height": 720,
  "encoding": "rgb8",
  "dtype": "uint8"
}
```

Control commands:
```json
{
  "timestamp": 1234567890.123,
  "throttle": 0.5,
  "steering": 0.1,
  "brake": 0.0,
  "hand_brake": false,
  "reverse": false
}
```

## ROS 2 Topics

### Published by ZMQ Bridge

| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/camera/rgb/image_raw` | sensor_msgs/Image | RGB camera feed |
| `/sensors/camera/depth/image_raw` | sensor_msgs/Image | Depth camera feed |
| `/sensors/odometry` | nav_msgs/Odometry | Vehicle pose and velocity |

### Published by Algorithm Nodes

| Topic | Type | Description |
|-------|------|-------------|
| `/perception/detections` | DetectionArray | Detected obstacles |
| `/planning/path` | nav_msgs/Path | Planned trajectory |
| `/planning/target_speed` | Float64 | Target speed |

### Subscribed by Control Node

| Topic | Type | Description |
|-------|------|-------------|
| `/control/cmd_vel` | geometry_msgs/Twist | Velocity commands |

## Troubleshooting

### "Failed to import rclpy"

Make sure you're running `run_ros_stack.bat` which properly sets up the ROS 2 environment.

### "Cannot connect to CARLA"

1. Verify CARLA is running: check Task Manager for `CarlaUE4.exe`
2. Check the host/port in `run_carla_bridge.bat`
3. Try mock mode: `run_carla_bridge.bat --mock`

### "ZMQ connection failed"

1. Ensure ports 5555-5558 are not blocked by firewall
2. Check that CARLA bridge is running before starting ROS stack
3. Verify ZeroMQ is installed in both environments

### Python Version Issues

The dual-environment design specifically avoids Python version conflicts:
- **Never** import CARLA modules in ROS 2 nodes
- **Never** import rclpy in CARLA bridge
- Use only ZeroMQ for cross-environment communication

## Design Principles

1. **Complete Isolation**: CARLA (Py3.7) and ROS 2 (Py3.12) never share memory or imports
2. **ZeroMQ IPC**: High-performance inter-process communication
3. **Standard Interfaces**: ROS 2 topics use standard message types
4. **Modularity**: Each component can be tested independently
5. **Extensibility**: Easy to add new sensors or algorithms

## Next Steps

1. Test the basic pipeline: CARLA → ZMQ → ROS 2
2. Add multi-vehicle support via cooperation node
3. Integrate with real Raspberry Pi vehicle using the same ROS 2 topics
4. Record and playback data using rosbag2

## References

- [DUAL_ENV_SETUP.md](DUAL_ENV_SETUP.md) - Detailed setup instructions
- [ARCHITECTURE.md](ARCHITECTURE.md) - Architecture documentation
- CARLA Documentation: https://carla.readthedocs.io/
- ROS 2 Documentation: https://docs.ros.org/
