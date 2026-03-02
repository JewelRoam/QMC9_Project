"""
Cooperation ROS 2 Node
Wraps the cooperative planning module for multi-vehicle coordination.

Subscribed Topics:
    /perception/obstacles (visualization_msgs/MarkerArray): Local obstacles
    /localization/pose (geometry_msgs/PoseStamped): Ego vehicle pose
    /navigation/goal (geometry_msgs/PoseStamped): Navigation goal
    /v2v/incoming (std_msgs/String): Incoming V2V messages (JSON)

Published Topics:
    /v2v/outgoing (std_msgs/String): Outgoing V2V messages (JSON)
    /cooperation/decision (std_msgs/String): Cooperative decision
    /planning/cooperative_path (nav_msgs/Path): Adjusted path considering cooperation

Parameters:
    vehicle_id (str): Unique vehicle identifier
    enable_cooperation (bool): Enable cooperative behaviors
    communication_range (float): V2V communication range in meters
    cooperation_mode (str): 'passive', 'active', or 'negotiated'
"""
import sys
import os
import json
import threading
from typing import Optional, Dict, Any

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

try:
    import rclpy
    from rclpy.node import Node
    
    from visualization_msgs.msg import MarkerArray
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import String
    
    ROS2_AVAILABLE = True
except ImportError:
    print("[WARNING] ROS 2 not available.")
    ROS2_AVAILABLE = False
    Node = object

try:
    from cooperation.cooperative_planner import CooperativePlanner
    from cooperation.v2v_message import V2VMessage
    COOPERATION_AVAILABLE = True
except ImportError:
    print("[WARNING] Cooperation module not available.")
    COOPERATION_AVAILABLE = False


class CooperationNode(Node):
    """ROS 2 node for multi-vehicle cooperation."""
    
    def __init__(self):
        if not ROS2_AVAILABLE or not COOPERATION_AVAILABLE:
            raise RuntimeError("Required dependencies not available.")
        
        super().__init__('cooperation_node')
        
        # Parameters
        self.declare_parameter('vehicle_id', 'ego_vehicle')
        self.declare_parameter('enable_cooperation', True)
        self.declare_parameter('communication_range', 100.0)
        self.declare_parameter('cooperation_mode', 'negotiated')
        self.declare_parameter('broadcast_rate', 10.0)
        
        self.vehicle_id = self.get_parameter('vehicle_id').value
        
        # Initialize planner
        config = {
            'vehicle_id': self.vehicle_id,
            'cooperation': {
                'enabled': self.get_parameter('enable_cooperation').value,
                'mode': self.get_parameter('cooperation_mode').value,
                'range': self.get_parameter('communication_range').value
            }
        }
        self.planner = CooperativePlanner(config)
        
        # State
        self.current_pose = None
        self.goal_pose = None
        self.local_obstacles = []
        self.neighbor_vehicles: Dict[str, Dict] = {}
        self.state_lock = threading.Lock()
        
        # Subscribers
        self.obstacle_sub = self.create_subscription(
            MarkerArray, '/perception/obstacles', self._obstacle_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/localization/pose', self._pose_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/navigation/goal', self._goal_callback, 10
        )
        self.v2v_in_sub = self.create_subscription(
            String, '/v2v/incoming', self._v2v_in_callback, 10
        )
        
        # Publishers
        self.v2v_out_pub = self.create_publisher(String, '/v2v/outgoing', 10)
        self.decision_pub = self.create_publisher(String, '/cooperation/decision', 10)
        self.path_pub = self.create_publisher(Path, '/planning/cooperative_path', 10)
        
        # Timers
        broadcast_rate = self.get_parameter('broadcast_rate').value
        self.broadcast_timer = self.create_timer(1.0 / broadcast_rate, self._broadcast_state)
        self.plan_timer = self.create_timer(0.1, self._cooperation_loop)
        
        self.get_logger().info(f"CooperationNode initialized for vehicle {self.vehicle_id}")
    
    def _obstacle_callback(self, msg: MarkerArray):
        """Process local obstacles."""
        obstacles = []
        for marker in msg.markers:
            obs = {
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'radius': max(marker.scale.x, marker.scale.y) / 2
            }
            obstacles.append(obs)
        
        with self.state_lock:
            self.local_obstacles = obstacles
    
    def _pose_callback(self, msg: PoseStamped):
        """Update current pose."""
        with self.state_lock:
            self.current_pose = msg
    
    def _goal_callback(self, msg: PoseStamped):
        """Update navigation goal."""
        with self.state_lock:
            self.goal_pose = msg
    
    def _v2v_in_callback(self, msg: String):
        """Handle incoming V2V message."""
        try:
            data = json.loads(msg.data)
            sender_id = data.get('vehicle_id')
            
            if sender_id and sender_id != self.vehicle_id:
                with self.state_lock:
                    self.neighbor_vehicles[sender_id] = {
                        'position': data.get('position'),
                        'velocity': data.get('velocity'),
                        'timestamp': self.get_clock().now().to_msg()
                    }
                
                # Process message through cooperative planner
                v2v_msg = V2VMessage.from_dict(data)
                self.planner.process_v2v_message(v2v_msg)
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Invalid V2V message: {e}")
    
    def _broadcast_state(self):
        """Broadcast ego vehicle state to neighbors."""
        with self.state_lock:
            pose = self.current_pose
        
        if pose is None:
            return
        
        # Create V2V message
        msg_data = {
            'vehicle_id': self.vehicle_id,
            'position': {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y
            },
            'velocity': {'x': 0.0, 'y': 0.0},  # TODO: Get from twist
            'timestamp': self.get_clock().now().to_msg().sec
        }
        
        # Publish
        ros_msg = String()
        ros_msg.data = json.dumps(msg_data)
        self.v2v_out_pub.publish(ros_msg)
    
    def _cooperation_loop(self):
        """Main cooperation loop."""
        with self.state_lock:
            pose = self.current_pose
            goal = self.goal_pose
            obstacles = self.local_obstacles.copy()
            neighbors = self.neighbor_vehicles.copy()
        
        if pose is None or goal is None:
            return
        
        try:
            # Run cooperative planning
            result = self.planner.plan_with_cooperation(
                ego_position=(pose.pose.position.x, pose.pose.position.y),
                goal_position=(goal.pose.position.x, goal.pose.position.y),
                local_obstacles=obstacles,
                neighbor_states=neighbors
            )
            
            # Publish cooperative path
            stamp = self.get_clock().now().to_msg()
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            
            if 'path' in result and result['path']:
                for point in result['path']:
                    pose_msg = PoseStamped()
                    pose_msg.header = path.header
                    pose_msg.pose.position.x = point[0]
                    pose_msg.pose.position.y = point[1]
                    path.poses.append(pose_msg)
            
            self.path_pub.publish(path)
            
            # Publish decision
            decision = String()
            decision.data = result.get('decision', 'proceed')
            self.decision_pub.publish(decision)
            
        except Exception as e:
            self.get_logger().error(f"Cooperation error: {e}")


def main(args=None):
    if not ROS2_AVAILABLE:
        print("ROS 2 required but not available.")
        return 1
    
    rclpy.init(args=args)
    try:
        node = CooperationNode()
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
