"""
CARLA-ROS Interface Layer
Bridges CARLA simulator with ROS 2 topics.
This is the ONLY module that imports both CARLA and ROS 2 libraries,
ensuring complete isolation between simulation and algorithm layers.

Design Principles:
1. Single Responsibility: Only handles CARLA ↔ ROS 2 conversion
2. No Algorithm Logic: Pure data transformation, no perception/planning/control
3. Configurable: All settings via ROS 2 parameters
4. Extensible: Easy to add new sensors or vehicles
"""
import sys
import os
import math
import threading
from typing import Optional, Dict, List
from dataclasses import dataclass

# Add CARLA Python API to path
carla_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                          'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI')
if carla_path not in sys.path:
    sys.path.insert(0, carla_path)

try:
    import carla
except ImportError:
    print("[WARNING] CARLA not available. Running in mock mode.")
    carla = None

# ROS 2 imports - gracefully handle if not available
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    
    # Sensor messages
    from sensor_msgs.msg import Image, CameraInfo, NavSatFix, Imu, PointCloud2
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, TransformStamped, Pose, Quaternion, Vector3
    from std_msgs.msg import Header, Float64, Bool
    from cv_bridge import CvBridge
    
    ROS2_AVAILABLE = True
except ImportError:
    print("[WARNING] ROS 2 not available. CarlaRosInterface will not function.")
    ROS2_AVAILABLE = False
    Node = object

import numpy as np
import cv2


@dataclass
class VehicleConfig:
    """Configuration for a vehicle in CARLA."""
    vehicle_id: str
    blueprint_filter: str = "vehicle.tesla.model3"
    spawn_transform: Optional[object] = None  # carla.Transform
    
    # Sensor configurations
    enable_rgb_camera: bool = True
    rgb_camera_position: tuple = (1.5, 0.0, 2.4)  # x, y, z relative to vehicle
    rgb_image_size: tuple = (1280, 720)
    rgb_fov: float = 110.0
    
    enable_depth_camera: bool = True
    depth_camera_position: tuple = (1.5, 0.0, 2.4)
    depth_image_size: tuple = (1280, 720)
    depth_fov: float = 110.0
    
    enable_gnss: bool = True
    enable_imu: bool = True
    enable_odometry: bool = True


class CarlaRosInterface(Node):
    """
    ROS 2 node that bridges CARLA simulator with ROS 2 topics.
    
    Published Topics:
        /sensors/camera/rgb/image_raw (sensor_msgs/Image): RGB camera feed
        /sensors/camera/depth/image_raw (sensor_msgs/Image): Depth camera feed
        /sensors/gnss/fix (sensor_msgs/NavSatFix): GPS position
        /sensors/imu/data (sensor_msgs/Imu): IMU data
        /sensors/odometry (nav_msgs/Odometry): Vehicle odometry
        /vehicle/status (std_msgs/Bool): Vehicle connection status
    
    Subscribed Topics:
        /control/cmd_vel (geometry_msgs/Twist): Velocity commands
        /vehicle/spawn (std_msgs/String): Spawn new vehicle request
        /vehicle/destroy (std_msgs/String): Destroy vehicle request
    
    Parameters:
        carla_host (str): CARLA server host (default: localhost)
        carla_port (int): CARLA server port (default: 2000)
        carla_timeout (float): Connection timeout (default: 10.0)
        synchronous_mode (bool): Enable synchronous mode (default: true)
        fixed_delta_seconds (float): Simulation step size (default: 0.05)
        map_name (str): CARLA map to load (default: Town03)
    """
    
    def __init__(self):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 is not available. Cannot create CarlaRosInterface.")
        
        super().__init__('carla_ros_interface')
        
        # Declare parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('carla_timeout', 10.0)
        self.declare_parameter('synchronous_mode', True)
        self.declare_parameter('fixed_delta_seconds', 0.05)
        self.declare_parameter('map_name', 'Town03')
        self.declare_parameter('vehicle_namespace', 'ego_vehicle')
        
        # Get parameters
        self.carla_host = self.get_parameter('carla_host').value
        self.carla_port = self.get_parameter('carla_port').value
        self.carla_timeout = self.get_parameter('carla_timeout').value
        self.synchronous_mode = self.get_parameter('synchronous_mode').value
        self.fixed_delta_seconds = self.get_parameter('fixed_delta_seconds').value
        self.map_name = self.get_parameter('map_name').value
        self.vehicle_ns = self.get_parameter('vehicle_namespace').value
        
        # CARLA client and world
        self.carla_client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.original_settings: Optional[carla.WorldSettings] = None
        
        # Vehicle management
        self.vehicles: Dict[str, dict] = {}  # vehicle_id -> {actor, sensors, config}
        self.cv_bridge = CvBridge()
        
        # Publishers
        self._setup_publishers()
        
        # Subscribers
        self._setup_subscribers()
        
        # Control command storage
        self.current_cmd_vel = Twist()
        self.cmd_lock = threading.Lock()
        
        # Connect to CARLA
        self._connect_to_carla()
        
        # Create timer for main loop
        self.timer = self.create_timer(self.fixed_delta_seconds, self._main_loop)
        
        self.get_logger().info("CarlaRosInterface initialized successfully")
    
    def _setup_publishers(self):
        """Setup ROS 2 publishers."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        ns = f"/sensors"
        
        self.rgb_pub = self.create_publisher(Image, f'{ns}/camera/rgb/image_raw', qos)
        self.depth_pub = self.create_publisher(Image, f'{ns}/camera/depth/image_raw', qos)
        self.gnss_pub = self.create_publisher(NavSatFix, f'{ns}/gnss/fix', 10)
        self.imu_pub = self.create_publisher(Imu, f'{ns}/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, f'{ns}/odometry', 10)
        self.status_pub = self.create_publisher(Bool, '/vehicle/status', 10)
        
        self.get_logger().debug("Publishers created")
    
    def _setup_subscribers(self):
        """Setup ROS 2 subscribers."""
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/control/cmd_vel', self._cmd_vel_callback, 10
        )
        self.spawn_sub = self.create_subscription(
            std_msgs.msg.String, '/vehicle/spawn', self._spawn_callback, 10
        )
        self.destroy_sub = self.create_subscription(
            std_msgs.msg.String, '/vehicle/destroy', self._destroy_callback, 10
        )
        
        self.get_logger().debug("Subscribers created")
    
    def _connect_to_carla(self):
        """Connect to CARLA server and setup world."""
        if carla is None:
            self.get_logger().error("CARLA Python API not available")
            return
        
        try:
            self.get_logger().info(f"Connecting to CARLA at {self.carla_host}:{self.carla_port}...")
            self.carla_client = carla.Client(self.carla_host, self.carla_port)
            self.carla_client.set_timeout(self.carla_timeout)
            
            # Get world
            self.world = self.carla_client.get_world()
            current_map = self.world.get_map().name
            
            # Load requested map if different
            if self.map_name not in current_map:
                self.get_logger().info(f"Loading map: {self.map_name}")
                self.world = self.carla_client.load_world(self.map_name)
            
            # Save original settings
            self.original_settings = self.world.get_settings()
            
            # Setup synchronous mode
            if self.synchronous_mode:
                settings = self.world.get_settings()
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = self.fixed_delta_seconds
                self.world.apply_settings(settings)
                self.get_logger().info(f"Synchronous mode enabled (dt={self.fixed_delta_seconds}s)")
            
            self.get_logger().info(f"Connected to CARLA. Map: {self.world.get_map().name}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            raise
    
    def _cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands."""
        with self.cmd_lock:
            self.current_cmd_vel = msg
    
    def _spawn_callback(self, msg):
        """Handle vehicle spawn request."""
        vehicle_id = msg.data
        self.get_logger().info(f"Spawning vehicle: {vehicle_id}")
        try:
            self.spawn_vehicle(vehicle_id)
        except Exception as e:
            self.get_logger().error(f"Failed to spawn vehicle {vehicle_id}: {e}")
    
    def _destroy_callback(self, msg):
        """Handle vehicle destroy request."""
        vehicle_id = msg.data
        self.get_logger().info(f"Destroying vehicle: {vehicle_id}")
        try:
            self.destroy_vehicle(vehicle_id)
        except Exception as e:
            self.get_logger().error(f"Failed to destroy vehicle {vehicle_id}: {e}")
    
    def spawn_vehicle(self, vehicle_id: str, config: Optional[VehicleConfig] = None):
        """
        Spawn a vehicle in CARLA with sensors.
        
        Args:
            vehicle_id: Unique identifier for the vehicle
            config: Vehicle configuration (uses defaults if None)
        """
        if config is None:
            config = VehicleConfig(vehicle_id=vehicle_id)
        
        if vehicle_id in self.vehicles:
            self.get_logger().warn(f"Vehicle {vehicle_id} already exists, destroying first")
            self.destroy_vehicle(vehicle_id)
        
        # Spawn vehicle actor
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter(config.blueprint_filter)[0]
        
        # Get spawn point
        if config.spawn_transform:
            spawn_point = config.spawn_transform
        else:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = spawn_points[0] if spawn_points else carla.Transform()
        
        vehicle_actor = self.world.spawn_actor(vehicle_bp, spawn_point)
        
        # Setup sensors
        sensors = {}
        
        if config.enable_rgb_camera:
            sensors['rgb_camera'] = self._setup_rgb_camera(
                vehicle_actor, config.rgb_camera_position, 
                config.rgb_image_size, config.rgb_fov
            )
        
        if config.enable_depth_camera:
            sensors['depth_camera'] = self._setup_depth_camera(
                vehicle_actor, config.depth_camera_position,
                config.depth_image_size, config.depth_fov
            )
        
        if config.enable_gnss:
            sensors['gnss'] = self._setup_gnss(vehicle_actor)
        
        if config.enable_imu:
            sensors['imu'] = self._setup_imu(vehicle_actor)
        
        # Store vehicle info
        self.vehicles[vehicle_id] = {
            'actor': vehicle_actor,
            'sensors': sensors,
            'config': config,
            'data': {
                'rgb_image': None,
                'depth_image': None,
                'gnss': None,
                'imu': None,
            }
        }
        
        self.get_logger().info(f"Vehicle {vehicle_id} spawned successfully")
    
    def _setup_rgb_camera(self, vehicle, position, image_size, fov):
        """Setup RGB camera sensor."""
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(image_size[0]))
        camera_bp.set_attribute('image_size_y', str(image_size[1]))
        camera_bp.set_attribute('fov', str(fov))
        
        transform = carla.Transform(carla.Location(x=position[0], y=position[1], z=position[2]))
        camera = self.world.spawn_actor(camera_bp, transform, attach_to=vehicle)
        
        # Setup callback
        def on_image(image):
            vehicle_id = None
            for vid, vinfo in self.vehicles.items():
                if vinfo['sensors'].get('rgb_camera') == camera:
                    vehicle_id = vid
                    break
            if vehicle_id:
                self.vehicles[vehicle_id]['data']['rgb_image'] = image
        
        camera.listen(on_image)
        return camera
    
    def _setup_depth_camera(self, vehicle, position, image_size, fov):
        """Setup depth camera sensor."""
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.depth')
        camera_bp.set_attribute('image_size_x', str(image_size[0]))
        camera_bp.set_attribute('image_size_y', str(image_size[1]))
        camera_bp.set_attribute('fov', str(fov))
        
        transform = carla.Transform(carla.Location(x=position[0], y=position[1], z=position[2]))
        camera = self.world.spawn_actor(camera_bp, transform, attach_to=vehicle)
        
        def on_image(image):
            for vid, vinfo in self.vehicles.items():
                if vinfo['sensors'].get('depth_camera') == camera:
                    self.vehicles[vid]['data']['depth_image'] = image
                    break
        
        camera.listen(on_image)
        return camera
    
    def _setup_gnss(self, vehicle):
        """Setup GNSS sensor."""
        blueprint_library = self.world.get_blueprint_library()
        gnss_bp = blueprint_library.find('sensor.other.gnss')
        transform = carla.Transform(carla.Location(z=2.0))
        gnss = self.world.spawn_actor(gnss_bp, transform, attach_to=vehicle)
        
        def on_gnss(data):
            for vid, vinfo in self.vehicles.items():
                if vinfo['sensors'].get('gnss') == gnss:
                    self.vehicles[vid]['data']['gnss'] = data
                    break
        
        gnss.listen(on_gnss)
        return gnss
    
    def _setup_imu(self, vehicle):
        """Setup IMU sensor."""
        blueprint_library = self.world.get_blueprint_library()
        imu_bp = blueprint_library.find('sensor.other.imu')
        transform = carla.Transform()
        imu = self.world.spawn_actor(imu_bp, transform, attach_to=vehicle)
        
        def on_imu(data):
            for vid, vinfo in self.vehicles.items():
                if vinfo['sensors'].get('imu') == imu:
                    self.vehicles[vid]['data']['imu'] = data
                    break
        
        imu.listen(on_imu)
        return imu
    
    def destroy_vehicle(self, vehicle_id: str):
        """Destroy a vehicle and its sensors."""
        if vehicle_id not in self.vehicles:
            self.get_logger().warn(f"Vehicle {vehicle_id} not found")
            return
        
        vehicle_info = self.vehicles[vehicle_id]
        
        # Destroy sensors
        for sensor_name, sensor in vehicle_info['sensors'].items():
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()
        
        # Destroy vehicle
        if vehicle_info['actor'].is_alive:
            vehicle_info['actor'].destroy()
        
        del self.vehicles[vehicle_id]
        self.get_logger().info(f"Vehicle {vehicle_id} destroyed")
    
    def _main_loop(self):
        """Main simulation loop - called by ROS 2 timer."""
        if self.world is None:
            return
        
        # Apply control commands to vehicles
        self._apply_vehicle_controls()
        
        # Publish sensor data
        self._publish_sensor_data()
        
        # Tick simulation if in synchronous mode
        if self.synchronous_mode:
            self.world.tick()
    
    def _apply_vehicle_controls(self):
        """Apply ROS cmd_vel to CARLA vehicles."""
        with self.cmd_lock:
            cmd = self.current_cmd_vel
        
        for vehicle_id, vehicle_info in self.vehicles.items():
            vehicle = vehicle_info['actor']
            
            # Convert Twist to CARLA VehicleControl
            control = carla.VehicleControl()
            
            # Linear velocity -> throttle/brake
            linear_x = cmd.linear.x
            if linear_x > 0:
                control.throttle = min(linear_x / 10.0, 1.0)  # Normalize to 0-1
                control.brake = 0.0
            elif linear_x < 0:
                control.throttle = 0.0
                control.brake = min(abs(linear_x) / 10.0, 1.0)
            else:
                control.throttle = 0.0
                control.brake = 0.0
            
            # Angular velocity -> steering
            control.steer = max(-1.0, min(1.0, cmd.angular.z))
            
            # Apply control
            vehicle.apply_control(control)
    
    def _publish_sensor_data(self):
        """Publish all sensor data to ROS topics."""
        stamp = self.get_clock().now().to_msg()
        
        for vehicle_id, vehicle_info in self.vehicles.items():
            data = vehicle_info['data']
            vehicle = vehicle_info['actor']
            
            # Publish RGB image
            if data['rgb_image'] is not None:
                self._publish_rgb_image(data['rgb_image'], stamp, vehicle_id)
            
            # Publish depth image
            if data['depth_image'] is not None:
                self._publish_depth_image(data['depth_image'], stamp, vehicle_id)
            
            # Publish GNSS
            if data['gnss'] is not None:
                self._publish_gnss(data['gnss'], stamp, vehicle_id)
            
            # Publish IMU
            if data['imu'] is not None:
                self._publish_imu(data['imu'], stamp, vehicle_id)
            
            # Publish odometry
            self._publish_odometry(vehicle, stamp, vehicle_id)
        
        # Publish status
        status = Bool()
        status.data = len(self.vehicles) > 0
        self.status_pub.publish(status)
    
    def _publish_rgb_image(self, carla_image, stamp, vehicle_id):
        """Convert and publish RGB image."""
        # Convert CARLA image to numpy array
        array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
        array = array.reshape((carla_image.height, carla_image.width, 4))  # BGRA
        rgb_array = array[:, :, :3][:, :, ::-1]  # Convert BGR to RGB
        
        # Convert to ROS Image
        ros_image = self.cv_bridge.cv2_to_imgmsg(rgb_array, encoding='rgb8')
        ros_image.header.stamp = stamp
        ros_image.header.frame_id = f'{vehicle_id}/camera_link'
        
        self.rgb_pub.publish(ros_image)
    
    def _publish_depth_image(self, carla_image, stamp, vehicle_id):
        """Convert and publish depth image."""
        # Convert CARLA depth to meters
        array = np.frombuffer(carla_image.raw_data, dtype=np.float32)
        depth_array = array.reshape((carla_image.height, carla_image.width))
        
        # Normalize to 16-bit for ROS
        depth_normalized = (depth_array * 1000).astype(np.uint16)  # mm
        
        ros_image = self.cv_bridge.cv2_to_imgmsg(depth_normalized, encoding='16UC1')
        ros_image.header.stamp = stamp
        ros_image.header.frame_id = f'{vehicle_id}/camera_depth_link'
        
        self.depth_pub.publish(ros_image)
    
    def _publish_gnss(self, gnss_data, stamp, vehicle_id):
        """Publish GNSS data."""
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = f'{vehicle_id}/gnss_link'
        msg.latitude = gnss_data.latitude
        msg.longitude = gnss_data.longitude
        msg.altitude = gnss_data.altitude
        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # SERVICE_GPS
        msg.position_covariance_type = 0  # COVARIANCE_TYPE_UNKNOWN
        
        self.gnss_pub.publish(msg)
    
    def _publish_imu(self, imu_data, stamp, vehicle_id):
        """Publish IMU data."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = f'{vehicle_id}/imu_link'
        
        # Accelerometer
        msg.linear_acceleration.x = imu_data.accelerometer.x
        msg.linear_acceleration.y = imu_data.accelerometer.y
        msg.linear_acceleration.z = imu_data.accelerometer.z
        
        # Gyroscope
        msg.angular_velocity.x = imu_data.gyroscope.x
        msg.angular_velocity.y = imu_data.gyroscope.y
        msg.angular_velocity.z = imu_data.gyroscope.z
        
        # Orientation (not provided by CARLA IMU, set to identity)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        
        self.imu_pub.publish(msg)
    
    def _publish_odometry(self, vehicle, stamp, vehicle_id):
        """Publish vehicle odometry."""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = f'{vehicle_id}/base_link'
        
        # Position
        transform = vehicle.get_transform()
        msg.pose.pose.position.x = transform.location.x
        msg.pose.pose.position.y = transform.location.y
        msg.pose.pose.position.z = transform.location.z
        
        # Orientation (convert CARLA rotation to quaternion)
        roll = math.radians(transform.rotation.roll)
        pitch = math.radians(transform.rotation.pitch)
        yaw = math.radians(transform.rotation.yaw)
        q = self._euler_to_quaternion(roll, pitch, yaw)
        msg.pose.pose.orientation = q
        
        # Velocity
        velocity = vehicle.get_velocity()
        msg.twist.twist.linear.x = velocity.x
        msg.twist.twist.linear.y = velocity.y
        msg.twist.twist.linear.z = velocity.z
        
        # Angular velocity
        angular_vel = vehicle.get_angular_velocity()
        msg.twist.twist.angular.x = math.radians(angular_vel.x)
        msg.twist.twist.angular.y = math.radians(angular_vel.y)
        msg.twist.twist.angular.z = math.radians(angular_vel.z)
        
        self.odom_pub.publish(msg)
    
    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.get_logger().info("Shutting down CarlaRosInterface...")
        
        # Destroy all vehicles
        for vehicle_id in list(self.vehicles.keys()):
            self.destroy_vehicle(vehicle_id)
        
        # Restore CARLA settings
        if self.world and self.original_settings:
            self.world.apply_settings(self.original_settings)
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    if not ROS2_AVAILABLE:
        print("ROS 2 is required but not available.")
        return 1
    
    rclpy.init(args=args)
    
    try:
        node = CarlaRosInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    import std_msgs
    main()
