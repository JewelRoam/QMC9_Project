"""
ZeroMQ to ROS 2 Bridge Node (Python 3.12)
Receives sensor data from CARLA via ZeroMQ and publishes as ROS 2 topics.

This node runs in the ROS 2 Python 3.12 environment and bridges data
from the CARLA Python 3.7 environment via ZeroMQ.
"""

import sys
import os
import time
import json
import struct
from typing import Optional
from dataclasses import dataclass

import numpy as np
import zmq

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    
    # Message types
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, Pose, Quaternion, Vector3
    from std_msgs.msg import Header
    
    ROS2_AVAILABLE = True
except ImportError:
    print("[ERROR] ROS 2 not available. This node requires ROS 2 environment.")
    ROS2_AVAILABLE = False
    sys.exit(1)


@dataclass
class ZmqConfig:
    """Configuration for ZeroMQ sockets."""
    rgb_port: str = "tcp://127.0.0.1:5555"
    depth_port: str = "tcp://127.0.0.1:5556"
    odom_port: str = "tcp://127.0.0.1:5557"
    control_port: str = "tcp://127.0.0.1:5558"


class ZmqToRosNode(Node):
    """
    Receives data from CARLA via ZeroMQ and publishes ROS 2 topics.
    
    Subscribes (ZeroMQ):
        - RGB images (port 5555)
        - Depth images (port 5556)
        - Vehicle odometry (port 5557)
    
    Publishes (ROS 2):
        - /sensors/camera/rgb/image_raw (sensor_msgs/Image)
        - /sensors/camera/depth/image_raw (sensor_msgs/Image)
        - /sensors/odometry (nav_msgs/Odometry)
    
    Subscribes (ROS 2):
        - /control/cmd_vel (geometry_msgs/Twist) -> forwards to ZeroMQ
    """
    
    def __init__(self):
        super().__init__('zmq_to_ros_bridge')
        
        # Declare parameters
        self.declare_parameter('rgb_port', 'tcp://127.0.0.1:5555')
        self.declare_parameter('depth_port', 'tcp://127.0.0.1:5556')
        self.declare_parameter('odom_port', 'tcp://127.0.0.1:5557')
        self.declare_parameter('control_port', 'tcp://127.0.0.1:5558')
        self.declare_parameter('publish_rate', 30.0)  # Hz
        
        # Get parameters
        config = ZmqConfig(
            rgb_port=self.get_parameter('rgb_port').value,
            depth_port=self.get_parameter('depth_port').value,
            odom_port=self.get_parameter('odom_port').value,
            control_port=self.get_parameter('control_port').value
        )
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Setup ZeroMQ
        self.zmq_context = zmq.Context()
        
        # Subscribers (receive from CARLA bridge)
        self.rgb_sub = self.zmq_context.socket(zmq.SUB)
        self.rgb_sub.connect(config.rgb_port)
        self.rgb_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.rgb_sub.setsockopt(zmq.RCVHWM, 10)  # Limit queue size
        
        self.depth_sub = self.zmq_context.socket(zmq.SUB)
        self.depth_sub.connect(config.depth_port)
        self.depth_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.depth_sub.setsockopt(zmq.RCVHWM, 10)
        
        self.odom_sub = self.zmq_context.socket(zmq.SUB)
        self.odom_sub.connect(config.odom_port)
        self.odom_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.odom_sub.setsockopt(zmq.RCVHWM, 10)
        
        # Publisher (send control to CARLA bridge)
        self.control_pub = self.zmq_context.socket(zmq.PUB)
        self.control_pub.connect(config.control_port)
        
        # ROS 2 Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.rgb_pub = self.create_publisher(Image, '/sensors/camera/rgb/image_raw', qos)
        self.depth_pub = self.create_publisher(Image, '/sensors/camera/depth/image_raw', qos)
        self.odom_pub = self.create_publisher(Odometry, '/sensors/odometry', qos)
        
        # ROS 2 Subscriber for control commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/control/cmd_vel',
            self._on_cmd_vel,
            10
        )
        
        # Timers for receiving ZMQ data
        period = 1.0 / self.publish_rate
        self.rgb_timer = self.create_timer(period, self._receive_rgb)
        self.depth_timer = self.create_timer(period, self._receive_depth)
        self.odom_timer = self.create_timer(0.05, self._receive_odom)  # 20 Hz
        
        self.get_logger().info("ZMQ to ROS bridge initialized")
        self.get_logger().info(f"Connected to ZMQ ports: RGB={config.rgb_port}, Depth={config.depth_port}, Odom={config.odom_port}")
    
    def _receive_rgb(self):
        """Receive RGB image from ZeroMQ and publish as ROS topic."""
        try:
            # Non-blocking receive with timeout
            if self.rgb_sub.poll(0):
                metadata_bytes, image_bytes = self.rgb_sub.recv_multipart()
                metadata = json.loads(metadata_bytes.decode('utf-8'))
                
                # Convert bytes to numpy array
                dtype = np.dtype(metadata['dtype'])
                image = np.frombuffer(image_bytes, dtype=dtype)
                image = image.reshape((metadata['height'], metadata['width'], 3))
                
                # Create ROS Image message
                msg = Image()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                msg.height = metadata['height']
                msg.width = metadata['width']
                msg.encoding = 'bgr8'  # OpenCV default
                msg.is_bigendian = False
                msg.step = msg.width * 3
                msg.data = image.tobytes()
                
                self.rgb_pub.publish(msg)
                
        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().warning(f"Error receiving RGB: {e}")
    
    def _receive_depth(self):
        """Receive depth image from ZeroMQ and publish as ROS topic."""
        try:
            if self.depth_sub.poll(0):
                metadata_bytes, image_bytes = self.depth_sub.recv_multipart()
                metadata = json.loads(metadata_bytes.decode('utf-8'))
                
                # Convert bytes to numpy array
                dtype = np.dtype(metadata['dtype'])
                image = np.frombuffer(image_bytes, dtype=dtype)
                image = image.reshape((metadata['height'], metadata['width']))
                
                # Create ROS Image message
                msg = Image()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                msg.height = metadata['height']
                msg.width = metadata['width']
                msg.encoding = '32FC1'  # Float depth
                msg.is_bigendian = False
                msg.step = msg.width * 4
                msg.data = image.tobytes()
                
                self.depth_pub.publish(msg)
                
        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().warning(f"Error receiving depth: {e}")
    
    def _receive_odom(self):
        """Receive odometry from ZeroMQ and publish as ROS topic."""
        try:
            if self.odom_sub.poll(0):
                state = self.odom_sub.recv_json()
                
                # Create ROS Odometry message
                msg = Odometry()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                msg.child_frame_id = 'base_link'
                
                # Position
                msg.pose.pose.position.x = state['x']
                msg.pose.pose.position.y = state['y']
                msg.pose.pose.position.z = state['z']
                
                # Orientation (simplified - just yaw)
                import math
                cy = math.cos(state['yaw'] * 0.5)
                sy = math.sin(state['yaw'] * 0.5)
                msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)
                
                # Velocity
                msg.twist.twist.linear = Vector3(
                    x=state['velocity_x'],
                    y=state['velocity_y'],
                    z=state['velocity_z']
                )
                
                self.odom_pub.publish(msg)
                
        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().warning(f"Error receiving odometry: {e}")
    
    def _on_cmd_vel(self, msg: Twist):
        """Receive ROS control command and forward to ZeroMQ."""
        try:
            # Convert Twist to control command
            control = {
                'timestamp': time.time(),
                'throttle': max(0.0, min(1.0, msg.linear.x)),  # Forward speed -> throttle
                'steering': max(-1.0, min(1.0, -msg.angular.z)),  # Angular velocity -> steering
                'brake': 0.0 if msg.linear.x > 0 else 0.5,
                'hand_brake': False,
                'reverse': msg.linear.x < 0
            }
            
            self.control_pub.send_json(control)
            
        except Exception as e:
            self.get_logger().warning(f"Error sending control: {e}")
    
    def destroy_node(self):
        """Cleanup."""
        self.get_logger().info("Shutting down ZMQ connections...")
        self.rgb_sub.close()
        self.depth_sub.close()
        self.odom_sub.close()
        self.control_pub.close()
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ZmqToRosNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
