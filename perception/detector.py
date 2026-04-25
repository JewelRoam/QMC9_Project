"""
YOLO-based Object Detector for obstacle detection.
Supports PyTorch, ONNX, and OpenVINO backends.
Includes a lightweight IOU-based tracker for velocity estimation and temporal consistency.
"""
import numpy as np
import time
import os
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict


@dataclass
class DetectedObject:
    """Represents a single detected object in the scene."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2 in pixels
    category: str  # "vehicle", "pedestrian", "cyclist", "traffic_sign", "other"
    track_id: int = -1
    distance: float = -1.0  # meters, -1 means unknown
    world_position: Optional[Tuple[float, float, float]] = None  # x, y, z in world coords
    velocity: Tuple[float, float] = (0.0, 0.0)  # vx, vy in m/s
    is_stale: bool = False  # True if extrapolated from history rather than detected

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


class SimpleIOUTracker:
    """
    Lightweight tracker to maintain object identity across frames.
    Essential for velocity estimation and filtering transient false negatives.
    """
    def __init__(self, iou_threshold: float = 0.3, max_age: int = 5):
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.next_id = 1
        self.tracks: Dict[int, Dict] = {}  # id -> {last_bbox, age, last_world_pos, velocity, class_info}

    def _calculate_iou(self, boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        interArea = max(0, xB - xA) * max(0, yB - yA)
        boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
        boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
        return interArea / float(boxAArea + boxBArea - interArea + 1e-6)

    def update(self, detections: List[DetectedObject], dt: float) -> List[DetectedObject]:
        matched_indices = set()
        
        # 1. Match current detections with existing tracks
        for det in detections:
            best_iou = -1
            best_id = -1
            for tid, track in self.tracks.items():
                iou = self._calculate_iou(det.bbox, track['last_bbox'])
                if iou > best_iou and iou > self.iou_threshold:
                    best_iou = iou
                    best_id = tid
            
            if best_id != -1:
                det.track_id = best_id
                matched_indices.add(best_id)
                # Update velocity if we have world position
                if det.world_position and self.tracks[best_id]['last_world_pos']:
                    p1 = np.array(self.tracks[best_id]['last_world_pos'][:2])
                    p2 = np.array(det.world_position[:2])
                    det.velocity = tuple((p2 - p1) / (dt + 1e-6))
                
                self.tracks[best_id].update({
                    'last_bbox': det.bbox,
                    'last_world_pos': det.world_position,
                    'age': 0,
                    'velocity': det.velocity
                })
            else:
                det.track_id = self.next_id
                self.tracks[self.next_id] = {
                    'last_bbox': det.bbox,
                    'last_world_pos': det.world_position,
                    'age': 0,
                    'velocity': (0.0, 0.0),
                    'class_id': det.class_id,
                    'class_name': det.class_name,
                    'category': det.category
                }
                self.next_id += 1

        # 2. Handle unmatched tracks (aging and removal)
        to_remove = []
        for tid in self.tracks:
            if tid not in matched_indices:
                self.tracks[tid]['age'] += 1
                if self.tracks[tid]['age'] > self.max_age:
                    to_remove.append(tid)
        
        for tid in to_remove:
            del self.tracks[tid]

        return detections


class YOLODetector:
    """
    High-performance YOLO detector with OpenVINO support and object tracking.
    """
    def __init__(self, config: dict):
        self.config = config
        self.imgsz = config.get("imgsz", 416)
        self.conf_threshold = config.get("confidence_threshold", 0.4)
        self.use_openvino = config.get("use_openvino", False)
        self.use_onnx = config.get("use_onnx", True)

        self._vehicle_ids = set(config.get("vehicle_classes", [2, 3, 5, 7]))
        self._pedestrian_ids = set(config.get("pedestrian_classes", [0]))
        self._cyclist_ids = set(config.get("cyclist_classes", [1]))
        self._traffic_sign_ids = set(config.get("traffic_sign_classes", [9, 11, 12]))

        self.tracker = SimpleIOUTracker()
        self._load_model()
        self._inference_times = []
        self._last_time = time.perf_counter()

    def _load_model(self):
        """Load model using preferred backend."""
        from ultralytics import YOLO
        
        if self.use_openvino:
            model_path = self.config.get("openvino_model_path", "model/yolo11n_openvino")
            print(f"[Detector] Loading OpenVINO backend: {model_path}")
        elif self.use_onnx:
            model_path = self.config.get("onnx_model_path", "model/yolo11n.onnx")
            print(f"[Detector] Loading ONNX backend: {model_path}")
        else:
            model_path = self.config.get("model_path", "model/yolo11n.pt")
            print(f"[Detector] Loading PyTorch backend: {model_path}")

        self.model = YOLO(model_path, task='detect')
        self._class_names = self.model.names

    def _classify_category(self, class_id: int) -> str:
        if class_id in self._vehicle_ids: return "vehicle"
        if class_id in self._pedestrian_ids: return "pedestrian"
        if class_id in self._cyclist_ids: return "cyclist"
        if class_id in self._traffic_sign_ids: return "traffic_sign"
        return "other"

    def detect(self, image: np.ndarray) -> List[DetectedObject]:
        current_time = time.perf_counter()
        dt = current_time - self._last_time
        self._last_time = current_time

        t0 = time.perf_counter()
        results = self.model(image, verbose=False, imgsz=self.imgsz, conf=self.conf_threshold)
        dt_inf = (time.perf_counter() - t0) * 1000
        self._inference_times.append(dt_inf)
        
        detections = []
        result = results[0]
        if result.boxes is not None:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                category = self._classify_category(cls_id)
                
                det = DetectedObject(
                    class_id=cls_id,
                    class_name=self._class_names.get(cls_id, "unknown"),
                    confidence=conf,
                    bbox=(x1, y1, x2, y2),
                    category=category
                )
                detections.append(det)

        # Apply temporal tracking to smooth detections and estimate velocity
        tracked_detections = self.tracker.update(detections, dt)
        
        if len(self._inference_times) > 100: self._inference_times.pop(0)
        return tracked_detections

    def get_obstacles(self, detections: List[DetectedObject]) -> List[DetectedObject]:
        return [d for d in detections if d.category in {"vehicle", "pedestrian", "cyclist"}]

    @property
    def avg_inference_ms(self) -> float:
        return sum(self._inference_times) / len(self._inference_times) if self._inference_times else 0.0
