"""
Depth estimation module.
Supports CARLA depth camera (simulation) and future Depth Anything V2 (real-world).
Converts 2D detections into 3D obstacle positions for APF planning.
"""
import numpy as np
import math
from typing import List, Tuple, Optional


class DepthEstimator:
    """
    Estimates depth/distance for detected objects.
    In CARLA: uses depth camera sensor data.
    On real hardware: can use monocular depth estimation (Depth Anything V2).
    """

    def __init__(self, config: dict, camera_config: dict):
        """
        Args:
            config: depth section of config.yaml
            camera_config: sensors.rgb_camera section for intrinsic params
        """
        self.source = config.get("source", "carla_depth_camera")
        self.max_distance = config.get("max_distance", 100.0)

        # Camera intrinsics (derived from CARLA camera settings)
        self.image_w = camera_config.get("image_size_x", 1280)
        self.image_h = camera_config.get("image_size_y", 720)
        fov_deg = camera_config.get("fov", 110)
        self.fov_rad = math.radians(fov_deg)

        # Focal length in pixels
        self.focal_length = self.image_w / (2.0 * math.tan(self.fov_rad / 2.0))

        # Depth image buffer
        self._depth_image = None

    def update_depth_image(self, depth_data):
        """
        Update the internal depth image from CARLA depth camera.

        Args:
            depth_data: CARLA depth sensor image (raw_data)
                        CARLA encodes depth as (R + G*256 + B*256*256) / (256^3 - 1) * 1000m
        """
        arr = np.frombuffer(depth_data.raw_data, dtype=np.uint8)
        arr = arr.reshape((depth_data.height, depth_data.width, 4))

        # CARLA depth encoding: normalized = (R + G*256 + B*65536) / 16777215
        # depth_meters = normalized * 1000.0
        r = arr[:, :, 2].astype(np.float32)
        g = arr[:, :, 1].astype(np.float32)
        b = arr[:, :, 0].astype(np.float32)

        normalized = (r + g * 256.0 + b * 65536.0) / 16777215.0
        self._depth_image = normalized * 1000.0  # Convert to meters

    def get_distance_at_bbox(self, bbox: Tuple[int, int, int, int]) -> float:
        """
        Get the estimated distance for an object at a given bounding box.
        Uses the median depth in the central region of the bbox for robustness.

        Args:
            bbox: (x1, y1, x2, y2) pixel coordinates

        Returns:
            Distance in meters, or -1.0 if unavailable
        """
        if self._depth_image is None:
            return -1.0

        x1, y1, x2, y2 = bbox
        # Use the central 50% of the bounding box to avoid edge noise
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        w4 = max((x2 - x1) // 4, 1)
        h4 = max((y2 - y1) // 4, 1)

        roi_x1 = max(cx - w4, 0)
        roi_x2 = min(cx + w4, self._depth_image.shape[1])
        roi_y1 = max(cy - h4, 0)
        roi_y2 = min(cy + h4, self._depth_image.shape[0])

        roi = self._depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
        if roi.size == 0:
            return -1.0

        distance = float(np.median(roi))
        return min(distance, self.max_distance)

    def pixel_to_local_3d(self, px: int, py: int, depth: float) -> Tuple[float, float, float]:
        """
        Convert pixel coordinates + depth to 3D position in camera frame.
        Camera frame: x=right, y=down, z=forward.

        Args:
            px, py: pixel coordinates
            depth: distance in meters

        Returns:
            (x, y, z) in camera local frame (meters)
        """
        # Center of image
        cx = self.image_w / 2.0
        cy = self.image_h / 2.0

        x = (px - cx) * depth / self.focal_length
        y = (py - cy) * depth / self.focal_length
        z = depth

        return (x, y, z)

    def enrich_detections(self, detections, ego_transform=None):
        """
        Add distance and world position to detection objects.

        Args:
            detections: List[DetectedObject]
            ego_transform: CARLA Transform of the ego vehicle (optional, for world coords)

        Returns:
            Same list with distance and world_position fields populated
        """
        for det in detections:
            dist = self.get_distance_at_bbox(det.bbox)
            det.distance = dist

            if dist > 0:
                cx, cy = det.center
                local_pos = self.pixel_to_local_3d(cx, cy, dist)

                if ego_transform is not None:
                    # Transform from camera local to world coordinates
                    # Simplified: camera is roughly at ego position + offset
                    ego_loc = ego_transform.location
                    ego_rot = ego_transform.rotation
                    yaw_rad = math.radians(ego_rot.yaw)

                    # Rotate local_pos by ego yaw (2D rotation in x-z plane)
                    world_x = ego_loc.x + local_pos[2] * math.cos(yaw_rad) - local_pos[0] * math.sin(yaw_rad)
                    world_y = ego_loc.y + local_pos[2] * math.sin(yaw_rad) + local_pos[0] * math.cos(yaw_rad)
                    world_z = ego_loc.z + local_pos[1]

                    det.world_position = (world_x, world_y, world_z)

        return detections

    @property
    def has_depth(self) -> bool:
        return self._depth_image is not None

    def get_depth_image_normalized(self) -> Optional[np.ndarray]:
        """Return depth image normalized to 0-255 for visualization."""
        if self._depth_image is None:
            return None
        clipped = np.clip(self._depth_image, 0, self.max_distance)
        normalized = (clipped / self.max_distance * 255).astype(np.uint8)
        return normalized
