"""
YOLO-based Object Detector for obstacle detection.
Supports both PyTorch (.pt) and ONNX (.onnx) model formats.
Designed to work with both CARLA simulation and real camera input.
"""
import numpy as np
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class DetectedObject:
    """Represents a single detected object in the scene."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2 in pixels
    category: str  # "vehicle", "pedestrian", "cyclist", "traffic_sign", "other"
    distance: float = -1.0  # meters, -1 means unknown
    world_position: Optional[Tuple[float, float, float]] = None  # x, y, z in world coords
    velocity: Optional[Tuple[float, float]] = None  # vx, vy estimated

    @property
    def center(self) -> Tuple[int, int]:
        return ((self.bbox[0] + self.bbox[2]) // 2,
                (self.bbox[1] + self.bbox[3]) // 2)

    @property
    def width(self) -> int:
        return self.bbox[2] - self.bbox[0]

    @property
    def height(self) -> int:
        return self.bbox[3] - self.bbox[1]

    @property
    def area(self) -> int:
        return self.width * self.height


class YOLODetector:
    """
    Encapsulates YOLO model for object detection.
    Classifies detections into categories relevant for autonomous driving.
    """

    # Category color map for visualization (BGR format)
    CATEGORY_COLORS = {
        "vehicle": (0, 255, 0),       # Green
        "pedestrian": (0, 0, 255),    # Red
        "cyclist": (255, 165, 0),     # Blue-ish
        "traffic_sign": (0, 165, 255),  # Orange
        "other": (128, 128, 128),     # Gray
    }

    def __init__(self, config: dict):
        """
        Initialize the YOLO detector.

        Args:
            config: perception section of config.yaml
        """
        self.config = config
        self.imgsz = config.get("imgsz", 416)
        self.conf_threshold = config.get("confidence_threshold", 0.4)

        # Class ID to category mapping
        self._vehicle_ids = set(config.get("vehicle_classes", [2, 3, 5, 7]))
        self._pedestrian_ids = set(config.get("pedestrian_classes", [0]))
        self._cyclist_ids = set(config.get("cyclist_classes", [1]))
        self._traffic_sign_ids = set(config.get("traffic_sign_classes", [9, 11, 12]))

        # Load model
        self._load_model()

        # Performance tracking
        self._inference_times = []

    def _load_model(self):
        """Load YOLO model (PyTorch or ONNX)."""
        from ultralytics import YOLO

        use_onnx = self.config.get("use_onnx", False)
        if use_onnx:
            model_path = self.config.get("onnx_model_path", "model/yolo11n.onnx")
        else:
            model_path = self.config.get("model_path", "model/yolo11n.pt")

        print(f"[Detector] Loading model: {model_path}")
        self.model = YOLO(model_path)
        
        # Handle case where model.names is None (e.g., some ONNX models)
        self._class_names = self.model.names
        if self._class_names is None:
            # Default COCO class names for YOLO
            self._class_names = {
                0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle',
                4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck',
                8: 'boat', 9: 'traffic light', 10: 'fire hydrant',
                11: 'stop sign', 12: 'parking meter', 13: 'bench',
                14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse',
                18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear',
                22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella',
                26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
                30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite',
                34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard',
                37: 'surfboard', 38: 'tennis racket', 39: 'bottle',
                40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife',
                44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple',
                48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot',
                52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake',
                56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed',
                60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop',
                64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone',
                68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink',
                72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase',
                76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'
            }
            print("[Detector] Using default COCO class names")
        
        print(f"[Detector] Model loaded. Classes: {len(self._class_names)}")

    def _classify_category(self, class_id: int) -> str:
        """Map COCO class ID to driving-relevant category."""
        if class_id in self._vehicle_ids:
            return "vehicle"
        elif class_id in self._pedestrian_ids:
            return "pedestrian"
        elif class_id in self._cyclist_ids:
            return "cyclist"
        elif class_id in self._traffic_sign_ids:
            return "traffic_sign"
        else:
            return "other"

    def detect(self, image: np.ndarray) -> List[DetectedObject]:
        """
        Run YOLO detection on an image.

        Args:
            image: RGB numpy array (H, W, 3)

        Returns:
            List of DetectedObject instances
        """
        t0 = time.perf_counter()

        results = self.model(image, verbose=False, imgsz=self.imgsz,
                             conf=self.conf_threshold)
        result = results[0]

        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                category = self._classify_category(cls_id)
                class_name = self._class_names.get(cls_id, f"class_{cls_id}")

                det = DetectedObject(
                    class_id=cls_id,
                    class_name=class_name,
                    confidence=conf,
                    bbox=(x1, y1, x2, y2),
                    category=category,
                )
                detections.append(det)

        dt = (time.perf_counter() - t0) * 1000
        self._inference_times.append(dt)
        if len(self._inference_times) > 100:
            self._inference_times = self._inference_times[-100:]

        return detections

    def get_obstacles(self, detections: List[DetectedObject]) -> List[DetectedObject]:
        """
        Filter detections to only those relevant for obstacle avoidance.
        Returns vehicles, pedestrians, and cyclists (things that need to be avoided).
        """
        obstacle_categories = {"vehicle", "pedestrian", "cyclist"}
        return [d for d in detections if d.category in obstacle_categories]

    def get_detection_summary(self, detections: List[DetectedObject]) -> dict:
        """Summarize detections by category."""
        summary = {"vehicle": 0, "pedestrian": 0, "cyclist": 0,
                   "traffic_sign": 0, "other": 0}
        for d in detections:
            summary[d.category] = summary.get(d.category, 0) + 1
        return summary

    @property
    def avg_inference_ms(self) -> float:
        if not self._inference_times:
            return 0.0
        return sum(self._inference_times) / len(self._inference_times)
