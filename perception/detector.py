"""
NexusPilot: YOLO-based Object Detector
Optimized for PyTorch, ONNX, and OpenVINO with robust metadata handling.
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
    distance: float = -1.0
    world_position: Optional[Tuple[float, float, float]] = None
    velocity: Tuple[float, float] = (0.0, 0.0)
    is_stale: bool = False

    @property
    def center(self) -> Tuple[int, int]:
        return ((self.bbox[0] + self.bbox[2]) // 2,
                (self.bbox[1] + self.bbox[3]) // 2)

    @property
    def height(self) -> int:
        return self.bbox[3] - self.bbox[1]


class SimpleIOUTracker:
    """
    Lightweight tracker to maintain object identity across frames.
    """
    def __init__(self, iou_threshold: float = 0.3, max_age: int = 5):
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.next_id = 1
        self.tracks: Dict[int, Dict] = {}

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
                self.tracks[best_id].update({
                    'last_bbox': det.bbox,
                    'age': 0,
                })
            else:
                det.track_id = self.next_id
                self.tracks[self.next_id] = {'last_bbox': det.bbox, 'age': 0}
                self.next_id += 1

        to_remove = []
        for tid in self.tracks:
            if tid not in matched_indices:
                self.tracks[tid]['age'] += 1
                if self.tracks[tid]['age'] > self.max_age: to_remove.append(tid)
        for tid in to_remove: del self.tracks[tid]
        return detections


class YOLODetector:
    """
    High-performance YOLO detector with metadata fail-safes.
    """
    def __init__(self, config: dict):
        self.config = config
        self.imgsz = config.get("imgsz", 320)
        self.conf_threshold = config.get("confidence_threshold", 0.4)
        
        # Mapping from KITTI/COCO classes to internal categories
        self._vehicle_ids = set(config.get("vehicle_classes", [2, 3, 5, 7]))
        self._pedestrian_ids = set(config.get("pedestrian_classes", [0]))
        self._cyclist_ids = set(config.get("cyclist_classes", [1]))
        
        # Fallback class names if model metadata is missing
        self._default_names = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}

        self.tracker = SimpleIOUTracker()
        self._load_model()
        self._inference_times = []
        self._last_time = time.perf_counter()

    def _load_model(self):
        """Loads model and ensures metadata integrity."""
        from ultralytics import YOLO
        
        model_path = self.config.get("onnx_model_path", "model/yolo11n.onnx")
        print(f"[Detector] Initializing Backend: {model_path}")
        
        self.model = YOLO(model_path, task='detect')
        
        # CRITICAL FIX: Safeguard against NoneType in model.names
        raw_names = getattr(self.model, 'names', None)
        if raw_names and isinstance(raw_names, dict):
            self._class_names = raw_names
        else:
            print("[WARNING] Model metadata (names) missing or corrupted. Using fallback map.")
            self._class_names = self._default_names

    def _classify_category(self, class_id: int) -> str:
        if class_id in self._vehicle_ids: return "vehicle"
        if class_id in self._pedestrian_ids: return "pedestrian"
        if class_id in self._cyclist_ids: return "cyclist"
        return "other"

    def detect(self, image: np.ndarray) -> List[DetectedObject]:
        current_time = time.perf_counter()
        dt = current_time - self._last_time
        self._last_time = current_time

        t0 = time.perf_counter()
        # Suppress ultralytics verbose logging to keep terminal clean
        results = self.model(image, verbose=False, imgsz=self.imgsz, conf=self.conf_threshold)
        self._inference_times.append((time.perf_counter() - t0) * 1000)
        
        detections = []
        if not results or results[0].boxes is None:
            return []

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            det = DetectedObject(
                class_id=cls_id,
                class_name=self._class_names.get(cls_id, f"class_{cls_id}"),
                confidence=conf,
                bbox=(x1, y1, x2, y2),
                category=self._classify_category(cls_id)
            )
            detections.append(det)

        if len(self._inference_times) > 100: self._inference_times.pop(0)
        return self.tracker.update(detections, dt)

    def get_obstacles(self, detections: List[DetectedObject]) -> List[DetectedObject]:
        """Early return filter for relevant planning obstacles."""
        return [d for d in detections if d.category in {"vehicle", "pedestrian", "cyclist"}]

    @property
    def avg_inference_ms(self) -> float:
        if not self._inference_times: return 0.0
        return sum(self._inference_times) / len(self._inference_times)
