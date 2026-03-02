"""
Planning ROS 2 Node
Wraps the APF planning module to provide ROS 2 topic interface.

Subscribed Topics:
    /perception/obstacles (visualization_msgs/MarkerArray): Detected obstacles
    /localization/pose (geometry_msgs/PoseStamped): Vehicle pose
    /navigation/goal (geometry_msgs/PoseStamped): Navigation goal

Published Topics:
    /planning/path (nav_msgs/Path): Planned trajectory
    /planning/target_speed (std_msgs/Float64): Target speed
    /planning/maneuver (std_msgs/String): Current maneuver type

Parameters:
    k_attractive (float): APF attractive force coefficient
    k_repulsive (float): APF repulsive force coefficient
    d0 (float): APF influence distance
    max_speed (float): Maximum vehicle speed
"""
import sys
import os
from typing import Optional, List
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    
    from visualization_msgs.msg import MarkerArray, Marker
    from geometry_msgs.msg import PoseStamped, Point
    from nav_msgs.msg import Path
    from std_msgs.msg import Float64, String
    
    ROS2_AVAILABLE = True
except ImportError:
    print("[WARNING] ROS 2 not available.")
    ROS2_AVAILABLE = False
    Node = object

import numpy as np

try:
    from planning.apf_planner import APFPlanner
    PLANNING_AVAILABLE = True
except ImportError:
    print("[WARNING] Planning module not available.")
    PLANNING_AVAILABLE = False


class PlanningNode(Node):
    """ROS 2 node for path planning using APF."""
    
    def __init__(self):
        if not ROS2_AVAILABLE or not PLANNING_AVAILABLE:
            raise RuntimeError("Required dependencies not available.")
        
        super().__init__('planning_node')
        
        # Parameters
        self.declare_parameter('k_attractive', 1.0)
        self.declare_parameter('k_repulsive', 100.0)
        self.declare_parameter('d0', 20.0)
        self.declare_parameter('max_speed', 30.0)
        self.declare_parameter('planning_rate', 10.0)
        
        # Initialize planner
        config = {
            'apf': {
                'k_attractive': self.get_parameter('k_attractive').value,
                'k_repulsive': self.get_parameter('k_repulsive').value,
                'd0': self.get_parameter('d0').value,
                'max_speed': self.get_parameter('max_speed').value,
            }
        }
        self.planner = APFPlanner(config)
        
        # State
        self.current_pose = None
        self.goal_pose = None
        self.obstacles = []
        self.state_lock = threading.Lock()
        
        # Subscribers
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.obstacle_sub = self.create_subscription(
            MarkerArray, '/perception/obstacles', self._obstacle_callback, qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/localization/pose', self._pose_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/navigation/goal', self._goal_callback, 10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planning/path', 10)
        self.speed_pub = self.create_publisher(Float64, '/planning/target_speed', 10)
        self.maneuver_pub = self.create_publisher(String, '/planning/maneuver', 10)
        
        # Timer
        rate = self.get_parameter('planning_rate').value
        self.timer = self.create_timer(1.0 / rate, self._plan_loop)
        
        self.get_logger().info("PlanningNode initialized")
    
    def _obstacle_callback(self, msg: MarkerArray):
        """Process obstacle markers."""
        obstacles = []
        for marker in msg.markers:
            obs = {
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'z': marker.pose.position.z,
                'type': marker.ns,
                'radius': max(marker.scale.x, marker.scale.y) / 2
            }
            obstacles.append(obs)
        
        with self.state_lock:
            self.obstacles = obstacles
    
    def _pose_callback(self, msg: PoseStamped):
        """Update current pose."""
        with self.state_lock:
            self.current_pose = msg
    
    def _goal_callback(self, msg: PoseStamped):
        """Update navigation goal."""
        with self.state_lock:
            self.goal_pose = msg
        self.get_logger().info(f"New goal set: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
    
    def _plan_loop(self):
        """Main planning loop."""
        with self.state_lock:
            pose = self.current_pose
            goal = self.goal_pose
            obstacles = self.obstacles.copy()
        
        if pose is None or goal is None:
            return
        
        try:
            # Extract position
            ego_x = pose.pose.position.x
            ego_y = pose.pose.position.y
            goal_x = goal.pose.position.x
            goal_y = goal.pose.position.y
            
            # Run APF planning
            result = self.planner.plan(
                ego_position=(ego_x, ego_y),
                goal_position=(goal_x, goal_y),
                obstacles=obstacles
            )
            
            # Publish results
            stamp = self.get_clock().now().to_msg()
            self._publish_path(result, stamp)
            self._publish_speed(result, stamp)
            self._publish_maneuver(result, stamp)
            
        except Exception as e:
            self.get_logger().error(f"Planning error: {e}")
    
    def _publish_path(self, result, stamp):
        """Publish planned path."""
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'map'
        
        if 'path' in result and result['path']:
            for point in result['path']:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.0
                path.poses.append(pose)
        
        self.path_pub.publish(path)
    
    def _publish_speed(self, result, stamp):
        """Publish target speed."""
        speed = Float64()
        speed.data = result.get('target_speed', 0.0)
        self.speed_pub.publish(speed)
    
    def _publish_maneuver(self, result, stamp):
        """Publish current maneuver."""
        maneuver = String()
        maneuver.data = result.get('maneuver', 'unknown')
        self.maneuver_pub.publish(maneuver)


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 required but not available.")
        return 1
    
    rclpy.init(args=args)
    try:
        node = PlanningNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
