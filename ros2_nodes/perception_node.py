"""
Perception ROS 2 Node
Wraps the perception module to provide ROS 2 topic interface.
This node is completely isolated from CARLA - it only processes ROS messages.

Subscribed Topics:
    /sensors/camera/rgb/image_raw (sensor_msgs/Image): RGB camera image
    /sensors/camera/depth/image_raw (sensor_msgs/Image): Depth camera image

Published Topics:
    /perception/detections (vision_msgs/Detection2DArray): 2D bounding boxes
    /perception/obstacles (visualization_msgs/MarkerArray): 3D obstacles with depth
    /perception/debug/image (sensor_msgs/Image): Annotated debug image

Parameters:
    model_path (str): Path to YOLO model
    confidence_threshold (float): Detection confidence threshold
    use_onnx (bool): Use ONNX runtime instead of PyTorch
    device (str): Inference device (cpu/cuda)
"""
import sys
import os
from typing import Optional, List
import threading

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    
    # ROS messages
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
    from visualization_msgs.msg import MarkerArray, Marker
    from geometry_msgs.msg import Point
    from std_msgs.msg import Header, ColorRGBA
    from cv_bridge import CvBridge
    
    ROS2_AVAILABLE = True
except ImportError:
    print("[WARNING] ROS 2 not available. PerceptionNode will not function.")
    ROS2_AVAILABLE = False
    Node = object

import numpy as np
import cv2

# Import existing perception module
try:
    from perception.detector import YOLODetector
    from perception.depth_estimator import DepthEstimator
    PERCEPTION_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] Perception module not available: {e}")
    PERCEPTION_AVAILABLE = False


class PerceptionNode(Node):
    """
    ROS 2 node for obstacle detection and depth estimation.
    
    This node wraps the existing perception module without modification,
    demonstrating the low-coupling design principle.
    """
    
    def __init__(self):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS 2 is not available.")
        if not PERCEPTION_AVAILABLE:
            raise RuntimeError("Perception module is not available.")
        
        super().__init__('perception_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'model/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('use_onnx', False)
        self.declare_parameter('onnx_model_path', 'model/yolo11n.onnx')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('image_size', 416)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('max_detection_distance', 100.0)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        confidence_threshold = self.get_parameter('confidence_threshold').value
        use_onnx = self.get_parameter('use_onnx').value
        onnx_model_path = self.get_parameter('onnx_model_path').value
        device = self.get_parameter('device').value
        image_size = self.get_parameter('image_size').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.max_detection_distance = self.get_parameter('max_detection_distance').value
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Initialize perception modules
        self.get_logger().info("Initializing YOLO detector...")
        try:
            if use_onnx:
                self.detector = YOLODetector(
                    model_path=onnx_model_path,
                    confidence_threshold=confidence_threshold,
                    device='cpu'  # ONNX uses CPU by default
                )
            else:
                self.detector = YOLODetector(
                    model_path=model_path,
                    confidence_threshold=confidence_threshold,
                    device=device
                )
            self.get_logger().info("YOLO detector initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize detector: {e}")
            raise
        
        self.depth_estimator = DepthEstimator(max_distance=self.max_detection_distance)
        
        # Data storage with thread safety
        self.latest_rgb = None
        self.latest_depth = None
        self.image_lock = threading.Lock()
        
        # Setup subscribers with message synchronization
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        self.rgb_sub = self.create_subscription(
            Image, '/sensors/camera/rgb/image_raw', 
            self._rgb_callback, qos
        )
        self.depth_sub = self.create_subscription(
            Image, '/sensors/camera/depth/image_raw',
            self._depth_callback, qos
        )
        
        # Alternative: Use synchronized subscriber for both images
        # This ensures RGB and depth are processed together
        self.sync_enabled = True
        if self.sync_enabled:
            self.rgb_sync_sub = Subscriber(self, Image, '/sensors/camera/rgb/image_raw')
            self.depth_sync_sub = Subscriber(self, Image, '/sensors/camera/depth/image_raw')
            self.sync = ApproximateTimeSynchronizer(
                [self.rgb_sync_sub, self.depth_sync_sub],
                queue_size=10,
                slop=0.1  # Allow 100ms difference
            )
            self.sync.registerCallback(self._synced_callback)
        
        # Setup publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/perception/detections', 10
        )
        self.obstacle_pub = self.create_publisher(
            MarkerArray, '/perception/obstacles', 10
        )
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image, '/perception/debug/image', 10
            )
        
        # Create processing timer (10 Hz)
        self.timer = self.create_timer(0.1, self._process_frame)
        
        self.get_logger().info("PerceptionNode initialized successfully")
        self.frame_count = 0
        self.processing_time_ms = 0.0
    
    def _rgb_callback(self, msg: Image):
        """Handle incoming RGB image."""
        with self.image_lock:
            self.latest_rgb = msg
    
    def _depth_callback(self, msg: Image):
        """Handle incoming depth image."""
        with self.image_lock:
            self.latest_depth = msg
    
    def _synced_callback(self, rgb_msg: Image, depth_msg: Image):
        """Handle synchronized RGB and depth images."""
        with self.image_lock:
            self.latest_rgb = rgb_msg
            self.latest_depth = depth_msg
    
    def _process_frame(self):
        """Process latest frame and publish results."""
        with self.image_lock:
            rgb_msg = self.latest_rgb
            depth_msg = self.latest_depth
        
        if rgb_msg is None:
            return
        
        start_time = self.get_clock().now()
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            
            # Convert to BGR for YOLO (expects BGR)
            cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # Run detection using existing perception module
            detections = self.detector.detect(cv_image_bgr)
            
            # Process depth if available
            obstacles_3d = []
            if depth_msg is not None:
                depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
                # Convert mm to meters
                depth_meters = depth_image.astype(np.float32) / 1000.0
                
                # Estimate 3D positions
                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    distance = self.depth_estimator.estimate_distance(depth_meters, (x1, y1, x2, y2))
                    det['distance'] = distance
                    
                    # Calculate 3D position (simplified projection)
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    obstacles_3d.append({
                        'class': det['class'],
                        'confidence': det['confidence'],
                        'bbox': det['bbox'],
                        'distance': distance,
                        'position_3d': self._calculate_3d_position(center_x, center_y, distance, cv_image.shape)
                    })
            
            # Publish results
            stamp = rgb_msg.header.stamp
            self._publish_detections(detections, stamp, rgb_msg.header.frame_id)
            self._publish_obstacles(obstacles_3d, stamp)
            
            # Publish debug image
            if self.publish_debug_image:
                debug_image = self._create_debug_image(cv_image, detections, obstacles_3d)
                debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='rgb8')
                debug_msg.header = rgb_msg.header
                self.debug_image_pub.publish(debug_msg)
            
            # Update statistics
            self.frame_count += 1
            end_time = self.get_clock().now()
            dt = (end_time - start_time).nanoseconds / 1e6  # ms
            self.processing_time_ms = 0.9 * self.processing_time_ms + 0.1 * dt
            
            # Log periodically
            if self.frame_count % 30 == 0:
                self.get_logger().debug(
                    f"Processed frame {self.frame_count}: "
                    f"{len(detections)} detections, "
                    f"{self.processing_time_ms:.1f}ms latency"
                )
                
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")
    
    def _calculate_3d_position(self, pixel_x, pixel_y, distance, image_shape):
        """
        Calculate 3D position from pixel coordinates and depth.
        Simplified pinhole camera model.
        """
        height, width = image_shape[:2]
        
        # Principal point (center of image)
        cx, cy = width / 2, height / 2
        
        # Focal length (approximate for 110° FOV)
        fov_rad = np.radians(110)
        fx = fy = width / (2 * np.tan(fov_rad / 2))
        
        # Back-project to 3D
        x = (pixel_x - cx) * distance / fx
        y = (pixel_y - cy) * distance / fy
        z = distance
        
        return {'x': x, 'y': y, 'z': z}
    
    def _publish_detections(self, detections: List[dict], stamp, frame_id: str):
        """Publish 2D detections as ROS message."""
        msg = Detection2DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        
        for det in detections:
            detection_msg = Detection2D()
            detection_msg.header.stamp = stamp
            detection_msg.header.frame_id = frame_id
            
            # Bounding box
            x1, y1, x2, y2 = det['bbox']
            detection_msg.bbox.center.position.x = (x1 + x2) / 2
            detection_msg.bbox.center.position.y = (y1 + y2) / 2
            detection_msg.bbox.size_x = x2 - x1
            detection_msg.bbox.size_y = y2 - y1
            
            # Class and confidence
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class'])
            hypothesis.hypothesis.score = float(det['confidence'])
            detection_msg.results.append(hypothesis)
            
            # Add distance if available
            if 'distance' in det:
                detection_msg.results[0].pose.pose.position.z = det['distance']
            
            msg.detections.append(detection_msg)
        
        self.detection_pub.publish(msg)
    
    def _publish_obstacles(self, obstacles: List[dict], stamp):
        """Publish 3D obstacles as RViz markers."""
        marker_array = MarkerArray()
        
        for i, obs in enumerate(obstacles):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'ego_vehicle/base_link'
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position
            pos = obs['position_3d']
            marker.pose.position.x = pos['z']  # Forward in vehicle frame
            marker.pose.position.y = -pos['x']  # Left in vehicle frame
            marker.pose.position.z = 0.0
            
            # Size (approximate based on class)
            size_map = {
                'person': (0.5, 0.5, 1.7),
                'car': (4.5, 2.0, 1.5),
                'truck': (8.0, 2.5, 2.5),
                'bicycle': (1.5, 0.5, 1.0),
                'motorcycle': (2.0, 0.8, 1.2),
            }
            default_size = (1.0, 1.0, 1.0)
            size = size_map.get(obs['class'], default_size)
            marker.scale.x = size[0]
            marker.scale.y = size[1]
            marker.scale.z = size[2]
            
            # Color based on class
            color_map = {
                'person': ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7),      # Green
                'car': ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7),         # Red
                'truck': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7),       # Orange
                'bicycle': ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7),     # Blue
                'motorcycle': ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.7),  # Purple
            }
            marker.color = color_map.get(obs['class'], ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.7))
            
            # Lifetime
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = int(0.2 * 1e9)  # 200ms
            
            marker_array.markers.append(marker)
        
        self.obstacle_pub.publish(marker_array)
    
    def _create_debug_image(self, rgb_image: np.ndarray, detections: List[dict], 
                           obstacles_3d: List[dict]) -> np.ndarray:
        """Create annotated debug image."""
        debug_img = rgb_image.copy()
        
        for det, obs in zip(detections, obstacles_3d):
            x1, y1, x2, y2 = map(int, det['bbox'])
            
            # Draw bounding box
            color = (0, 255, 0)  # Green
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{det['class']}: {det['confidence']:.2f}"
            if 'distance' in det:
                label += f", {det['distance']:.1f}m"
            
            # Label background
            (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(debug_img, (x1, y1 - text_h - 10), (x1 + text_w, y1), color, -1)
            cv2.putText(debug_img, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        # Add FPS info
        fps_text = f"FPS: {1000.0 / max(self.processing_time_ms, 1):.1f}"
        cv2.putText(debug_img, fps_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return debug_img


def main(args=None):
    """Main entry point."""
    if not ROS2_AVAILABLE:
        print("ROS 2 is required but not available.")
        return 1
    
    rclpy.init(args=args)
    
    try:
        node = PerceptionNode()
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
    main()
