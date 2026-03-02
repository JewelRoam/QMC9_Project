"""
CARLA to ZeroMQ Bridge (Python 3.7)
Bridges CARLA simulator data to ZeroMQ for cross-environment communication with ROS 2.

This module runs in Python 3.7 environment (for CARLA compatibility) and publishes
sensor data via ZeroMQ to be consumed by ROS 2 nodes running in Python 3.12.
"""

import sys
import os
import time
import json
import struct
import signal
import argparse
from typing import Optional, Dict, Any
from dataclasses import dataclass, asdict

import numpy as np
import zmq
import cv2

# Add CARLA PythonAPI to path
carla_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
    'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI', 'carla'
)
if carla_path not in sys.path:
    sys.path.insert(0, carla_path)

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    print("[WARNING] CARLA not available. Running in mock mode.")
    CARLA_AVAILABLE = False


@dataclass
class SensorConfig:
    """Configuration for CARLA sensors."""
    image_width: int = 1280
    image_height: int = 720
    fov: float = 110.0
    tick_interval: float = 0.05  # 20 Hz


@dataclass
class VehicleState:
    """Vehicle state message."""
    timestamp: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    velocity_x: float
    velocity_y: float
    velocity_z: float
    speed: float


@dataclass
class ControlCommand:
    """Control command message."""
    timestamp: float
    throttle: float
    steering: float
    brake: float
    hand_brake: bool = False
    reverse: bool = False


class CarlaZmqBridge:
    """
    Bridges CARLA simulator to ZeroMQ.
    
    Publishes:
        - RGB camera images (port 5555)
        - Depth camera images (port 5556)
        - Vehicle odometry (port 5557)
    
    Subscribes:
        - Control commands (port 5558)
    """
    
    def __init__(self, host='localhost', port=2000, config: Optional[SensorConfig] = None):
        self.host = host
        self.port = port
        self.config = config or SensorConfig()
        
        # ZeroMQ context and sockets
        self.zmq_context = zmq.Context()
        
        # Publishers
        self.rgb_pub = self.zmq_context.socket(zmq.PUB)
        self.rgb_pub.bind("tcp://127.0.0.1:5555")
        
        self.depth_pub = self.zmq_context.socket(zmq.PUB)
        self.depth_pub.bind("tcp://127.0.0.1:5556")
        
        self.odom_pub = self.zmq_context.socket(zmq.PUB)
        self.odom_pub.bind("tcp://127.0.0.1:5557")
        
        # Subscriber for control commands
        self.control_sub = self.zmq_context.socket(zmq.SUB)
        self.control_sub.bind("tcp://127.0.0.1:5558")
        self.control_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # CARLA client
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.vehicle: Optional[carla.Vehicle] = None
        self.sensors: list = []
        
        # Data buffers
        self.latest_rgb_image: Optional[np.ndarray] = None
        self.latest_depth_image: Optional[np.ndarray] = None
        self.latest_vehicle_state: Optional[VehicleState] = None
        
        # Control
        self.running = False
        self.current_control = ControlCommand(
            timestamp=time.time(),
            throttle=0.0,
            steering=0.0,
            brake=1.0
        )
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        print(f"\n[INFO] Received signal {signum}, shutting down...")
        self.stop()
    
    def connect_carla(self) -> bool:
        """Connect to CARLA server."""
        if not CARLA_AVAILABLE:
            print("[WARNING] CARLA not available, running in mock mode")
            return False
        
        try:
            print(f"[INFO] Connecting to CARLA at {self.host}:{self.port}...")
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            
            print(f"[OK] Connected to CARLA world: {self.world.get_map().name}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to connect to CARLA: {e}")
            return False
    
    def spawn_vehicle(self, vehicle_filter='vehicle.tesla.model3') -> bool:
        """Spawn ego vehicle in CARLA."""
        if not self.world:
            return False
        
        try:
            # Get spawn points
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                print("[ERROR] No spawn points available")
                return False
            
            # Get vehicle blueprint
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.filter(vehicle_filter)[0]
            vehicle_bp.set_attribute('role_name', 'ego')
            
            # Spawn vehicle
            spawn_point = spawn_points[0]
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            
            print(f"[OK] Spawned vehicle: {self.vehicle.type_id}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to spawn vehicle: {e}")
            return False
    
    def setup_sensors(self) -> bool:
        """Setup RGB and depth cameras."""
        if not self.world or not self.vehicle:
            return False
        
        try:
            blueprint_library = self.world.get_blueprint_library()
            
            # RGB Camera
            rgb_bp = blueprint_library.find('sensor.camera.rgb')
            rgb_bp.set_attribute('image_size_x', str(self.config.image_width))
            rgb_bp.set_attribute('image_size_y', str(self.config.image_height))
            rgb_bp.set_attribute('fov', str(self.config.fov))
            
            rgb_transform = carla.Transform(carla.Location(x=1.6, z=1.7))
            rgb_camera = self.world.spawn_actor(
                rgb_bp, rgb_transform, attach_to=self.vehicle
            )
            rgb_camera.listen(self._on_rgb_image)
            self.sensors.append(rgb_camera)
            
            # Depth Camera
            depth_bp = blueprint_library.find('sensor.camera.depth')
            depth_bp.set_attribute('image_size_x', str(self.config.image_width))
            depth_bp.set_attribute('image_size_y', str(self.config.image_height))
            depth_bp.set_attribute('fov', str(self.config.fov))
            
            depth_transform = carla.Transform(carla.Location(x=1.6, z=1.7))
            depth_camera = self.world.spawn_actor(
                depth_bp, depth_transform, attach_to=self.vehicle
            )
            depth_camera.listen(self._on_depth_image)
            self.sensors.append(depth_camera)
            
            print(f"[OK] Setup {len(self.sensors)} sensors")
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to setup sensors: {e}")
            return False
    
    def _on_rgb_image(self, image):
        """Callback for RGB camera."""
        # Convert to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        self.latest_rgb_image = array
        
        # Publish via ZeroMQ
        self._publish_image(self.rgb_pub, array, 'rgb8', image.timestamp)
    
    def _on_depth_image(self, image):
        """Callback for depth camera."""
        # Convert to numpy array (depth in meters)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        
        # Convert BGRA to depth (in meters)
        depth = array[:, :, 0] + array[:, :, 1] * 256 + array[:, :, 2] * 256 * 256
        depth = depth / (256 * 256 * 256 - 1) * 1000  # Convert to meters
        
        self.latest_depth_image = depth.astype(np.float32)
        
        # Publish via ZeroMQ
        self._publish_image(self.depth_pub, self.latest_depth_image, '32FC1', image.timestamp)
    
    def _publish_image(self, socket, image: np.ndarray, encoding: str, timestamp: float):
        """Publish image via ZeroMQ."""
        # Create metadata
        metadata = {
            'timestamp': timestamp,
            'width': image.shape[1],
            'height': image.shape[0],
            'encoding': encoding,
            'dtype': str(image.dtype)
        }
        
        # Serialize: [metadata_json | image_bytes]
        metadata_bytes = json.dumps(metadata).encode('utf-8')
        image_bytes = image.tobytes()
        
        # Send multipart message
        socket.send_multipart([
            metadata_bytes,
            image_bytes
        ])
    
    def _publish_odometry(self):
        """Publish vehicle odometry."""
        if not self.vehicle:
            return
        
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        
        state = VehicleState(
            timestamp=time.time(),
            x=transform.location.x,
            y=transform.location.y,
            z=transform.location.z,
            roll=transform.rotation.roll,
            pitch=transform.rotation.pitch,
            yaw=transform.rotation.yaw,
            velocity_x=velocity.x,
            velocity_y=velocity.y,
            velocity_z=velocity.z,
            speed=np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        )
        
        self.latest_vehicle_state = state
        
        # Publish via ZeroMQ
        self.odom_pub.send_json(asdict(state))
    
    def _receive_control(self):
        """Receive control commands from ZeroMQ."""
        try:
            # Non-blocking receive
            if self.control_sub.poll(0):
                msg = self.control_sub.recv_json()
                self.current_control = ControlCommand(**msg)
                
                # Apply control to vehicle
                if self.vehicle:
                    control = carla.VehicleControl(
                        throttle=self.current_control.throttle,
                        steer=self.current_control.steering,
                        brake=self.current_control.brake,
                        hand_brake=self.current_control.hand_brake,
                        reverse=self.current_control.reverse
                    )
                    self.vehicle.apply_control(control)
                    
        except zmq.Again:
            pass
        except Exception as e:
            print(f"[WARNING] Error receiving control: {e}")
    
    def run_mock_mode(self):
        """Run in mock mode without CARLA (for testing)."""
        print("[INFO] Running in MOCK mode (no CARLA connection)")
        print("[INFO] Publishing synthetic data...")
        
        self.running = True
        last_odom_time = time.time()
        
        while self.running:
            # Generate synthetic RGB image
            mock_rgb = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
            self._publish_image(self.rgb_pub, mock_rgb, 'rgb8', time.time())
            
            # Generate synthetic depth image
            mock_depth = np.random.uniform(0, 100, (720, 1280)).astype(np.float32)
            self._publish_image(self.depth_pub, mock_depth, '32FC1', time.time())
            
            # Publish mock odometry at lower rate
            if time.time() - last_odom_time > 0.05:
                mock_state = VehicleState(
                    timestamp=time.time(),
                    x=0.0, y=0.0, z=0.0,
                    roll=0.0, pitch=0.0, yaw=0.0,
                    velocity_x=0.0, velocity_y=0.0, velocity_z=0.0,
                    speed=0.0
                )
                self.odom_pub.send_json(asdict(mock_state))
                last_odom_time = time.time()
            
            # Check for control commands
            self._receive_control()
            
            time.sleep(0.033)  # ~30 FPS
    
    def run(self):
        """Main loop."""
        # Try to connect to CARLA
        carla_connected = self.connect_carla()
        
        if carla_connected:
            # Spawn vehicle and sensors
            if not self.spawn_vehicle():
                print("[WARNING] Could not spawn vehicle, running in mock mode")
                self.run_mock_mode()
                return
            
            if not self.setup_sensors():
                print("[WARNING] Could not setup sensors, running in mock mode")
                self.run_mock_mode()
                return
        else:
            # Run in mock mode
            self.run_mock_mode()
            return
        
        # Main loop with CARLA
        print("[INFO] Starting main loop...")
        print("[INFO] Press Ctrl+C to stop")
        
        self.running = True
        last_odom_time = time.time()
        
        try:
            while self.running:
                # Publish odometry
                if time.time() - last_odom_time > 0.05:
                    self._publish_odometry()
                    last_odom_time = time.time()
                
                # Receive control commands
                self._receive_control()
                
                time.sleep(0.001)  # Small delay to prevent CPU spinning
                
        except KeyboardInterrupt:
            print("\n[INFO] Keyboard interrupt received")
        finally:
            self.stop()
    
    def stop(self):
        """Cleanup and shutdown."""
        self.running = False
        print("[INFO] Shutting down...")
        
        # Destroy sensors
        for sensor in self.sensors:
            try:
                sensor.destroy()
            except:
                pass
        self.sensors.clear()
        
        # Destroy vehicle
        if self.vehicle:
            try:
                self.vehicle.destroy()
            except:
                pass
        
        # Close ZeroMQ sockets
        self.rgb_pub.close()
        self.depth_pub.close()
        self.odom_pub.close()
        self.control_sub.close()
        self.zmq_context.term()
        
        print("[OK] Shutdown complete")


def main():
    parser = argparse.ArgumentParser(
        description='CARLA to ZeroMQ Bridge',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Connect to local CARLA
  python carla_zmq_bridge.py

  # Connect to remote CARLA
  python carla_zmq_bridge.py --host 192.168.1.100 --port 2000

  # Mock mode (no CARLA)
  python carla_zmq_bridge.py --mock
        """
    )
    
    parser.add_argument('--host', default='localhost', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--mock', action='store_true', help='Run in mock mode without CARLA')
    parser.add_argument('--width', type=int, default=1280, help='Camera width')
    parser.add_argument('--height', type=int, default=720, help='Camera height')
    
    args = parser.parse_args()
    
    config = SensorConfig(
        image_width=args.width,
        image_height=args.height
    )
    
    bridge = CarlaZmqBridge(args.host, args.port, config)
    
    if args.mock:
        bridge.run_mock_mode()
    else:
        bridge.run()


if __name__ == '__main__':
    main()
