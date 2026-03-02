"""
Performance Metrics Collector for Autonomous Driving Evaluation.
Collects and analyzes driving performance data:
  - Collision detection and classification
  - Lane keeping accuracy (deviation from center)
  - Speed profiles and efficiency
  - Obstacle avoidance success rate
  - End-to-end latency breakdown
  - V2V cooperative gain metrics

Usage:
  from testing.metrics_collector import MetricsCollector
  collector = MetricsCollector()
  collector.start_recording()
  # ... run simulation ...
  report = collector.generate_report()
"""
import time
import math
import json
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, field, asdict
from collections import deque


@dataclass
class FrameMetrics:
    """Metrics for a single simulation frame."""
    timestamp: float
    frame_id: int

    # Position and kinematics
    ego_x: float = 0.0
    ego_y: float = 0.0
    ego_yaw: float = 0.0
    ego_speed: float = 0.0  # km/h

    # Control outputs
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0

    # Planning info
    target_speed: float = 0.0
    planner_status: str = "normal"
    nearest_obstacle_dist: float = float('inf')

    # Detection info
    num_detections: int = 0
    num_obstacles: int = 0

    # Timing
    perception_time_ms: float = 0.0
    planning_time_ms: float = 0.0
    control_time_ms: float = 0.0
    total_frame_time_ms: float = 0.0

    # V2V (if enabled)
    v2v_messages_received: int = 0
    cooperative_obstacles: int = 0


@dataclass
class CollisionEvent:
    """Records a collision event."""
    timestamp: float
    frame_id: int
    other_actor_type: str
    impact_speed: float  # km/h at collision
    location: Tuple[float, float, float]
    severity: str = "minor"  # minor/moderate/severe


@dataclass
class TestRunSummary:
    """Summary of a complete test run."""
    scenario_name: str
    duration_seconds: float
    total_frames: int
    average_fps: float

    # Safety metrics
    collision_count: int = 0
    collision_events: List[CollisionEvent] = field(default_factory=list)
    near_misses: int = 0  # Close calls (< 3m to obstacle)

    # Performance metrics
    average_speed: float = 0.0
    max_speed: float = 0.0
    min_speed: float = 0.0
    speed_variance: float = 0.0

    # Lane keeping (if lane info available)
    avg_lane_deviation: float = 0.0
    max_lane_deviation: float = 0.0
    time_out_of_lane: float = 0.0  # seconds

    # Obstacle avoidance
    obstacles_encountered: int = 0
    successful_avoidances: int = 0
    min_distance_to_obstacle: float = float('inf')

    # Efficiency
    total_distance_m: float = 0.0
    time_stopped: float = 0.0  # seconds with speed < 1 km/h
    time_in_emergency: float = 0.0

    # Latency
    avg_perception_latency_ms: float = 0.0
    avg_planning_latency_ms: float = 0.0
    avg_control_latency_ms: float = 0.0
    avg_total_latency_ms: float = 0.0

    # V2V cooperation
    v2v_enabled: bool = False
    cooperative_gain: float = 0.0  # % improvement with V2V

    # Success criteria
    passed: bool = True
    failure_reasons: List[str] = field(default_factory=list)


class MetricsCollector:
    """
    Collects and analyzes performance metrics during simulation runs.
    """

    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.frame_history: deque = deque(maxlen=window_size)
        self.collision_events: List[CollisionEvent] = []
        self.near_miss_count = 0

        self.recording = False
        self.start_time = None
        self.frame_count = 0

        # For distance calculation
        self.last_position = None
        self.total_distance = 0.0

        # Lane deviation tracking
        self.lane_deviations = []

    def start_recording(self):
        """Start recording metrics."""
        self.recording = True
        self.start_time = time.time()
        self.frame_count = 0
        self.frame_history.clear()
        self.collision_events.clear()
        self.near_miss_count = 0
        self.total_distance = 0.0
        self.lane_deviations = []
        self.last_position = None

    def stop_recording(self):
        """Stop recording and return summary."""
        self.recording = False
        return self._compute_summary()

    def record_frame(self, metrics: FrameMetrics):
        """Record metrics for one frame."""
        if not self.recording:
            return

        self.frame_count += 1
        metrics.frame_id = self.frame_count
        metrics.timestamp = time.time() - self.start_time

        # Calculate distance traveled
        if self.last_position is not None:
            dx = metrics.ego_x - self.last_position[0]
            dy = metrics.ego_y - self.last_position[1]
            self.total_distance += math.sqrt(dx*dx + dy*dy)

        self.last_position = (metrics.ego_x, metrics.ego_y)

        # Check for near misses
        if 0 < metrics.nearest_obstacle_dist < 3.0:
            self.near_miss_count += 1

        self.frame_history.append(metrics)

    def record_collision(self, other_actor_type: str, impact_speed: float,
                         location: Tuple[float, float, float]):
        """Record a collision event."""
        if not self.recording:
            return

        # Determine severity
        if impact_speed < 10:
            severity = "minor"
        elif impact_speed < 30:
            severity = "moderate"
        else:
            severity = "severe"

        event = CollisionEvent(
            timestamp=time.time() - self.start_time,
            frame_id=self.frame_count,
            other_actor_type=other_actor_type,
            impact_speed=impact_speed,
            location=location,
            severity=severity
        )
        self.collision_events.append(event)

    def record_lane_deviation(self, deviation_meters: float):
        """Record lane center deviation."""
        if self.recording:
            self.lane_deviations.append(deviation_meters)

    def _compute_summary(self) -> TestRunSummary:
        """Compute summary statistics from recorded frames."""
        if not self.frame_history:
            return TestRunSummary(
                scenario_name="unknown",
                duration_seconds=0,
                total_frames=0,
                average_fps=0
            )

        frames = list(self.frame_history)
        duration = frames[-1].timestamp - frames[0].timestamp if len(frames) > 1 else 0

        # Speed statistics
        speeds = [f.ego_speed for f in frames]
        avg_speed = np.mean(speeds) if speeds else 0
        max_speed = max(speeds) if speeds else 0
        min_speed = min(speeds) if speeds else 0
        speed_var = np.var(speeds) if speeds else 0

        # Latency statistics
        perc_latencies = [f.perception_time_ms for f in frames]
        plan_latencies = [f.planning_time_ms for f in frames]
        ctrl_latencies = [f.control_time_ms for f in frames]
        total_latencies = [f.total_frame_time_ms for f in frames]

        # Time calculations
        stopped_frames = sum(1 for f in frames if f.ego_speed < 1.0)
        emergency_frames = sum(1 for f in frames if f.planner_status == "emergency")

        # Lane deviation
        avg_lane_dev = np.mean(self.lane_deviations) if self.lane_deviations else 0
        max_lane_dev = max(self.lane_deviations) if self.lane_deviations else 0
        out_of_lane_time = sum(1 for d in self.lane_deviations if d > 1.0) * (duration / len(frames)) if frames else 0

        # Nearest obstacle distances
        min_obs_dist = min((f.nearest_obstacle_dist for f in frames if f.nearest_obstacle_dist > 0), default=float('inf'))

        # Determine pass/fail
        failure_reasons = []
        if self.collision_events:
            failure_reasons.append(f"Collisions: {len(self.collision_events)}")
        if avg_speed < 5.0 and duration > 10:
            failure_reasons.append("Vehicle stuck (avg speed too low)")
        if out_of_lane_time > duration * 0.3:
            failure_reasons.append("Excessive time out of lane")

        return TestRunSummary(
            scenario_name="test_run",
            duration_seconds=duration,
            total_frames=self.frame_count,
            average_fps=self.frame_count / duration if duration > 0 else 0,
            collision_count=len(self.collision_events),
            collision_events=self.collision_events,
            near_misses=self.near_miss_count,
            average_speed=avg_speed,
            max_speed=max_speed,
            min_speed=min_speed,
            speed_variance=speed_var,
            avg_lane_deviation=avg_lane_dev,
            max_lane_deviation=max_lane_dev,
            time_out_of_lane=out_of_lane_time,
            min_distance_to_obstacle=min_obs_dist,
            total_distance_m=self.total_distance,
            time_stopped=stopped_frames * (duration / len(frames)) if frames else 0,
            time_in_emergency=emergency_frames * (duration / len(frames)) if frames else 0,
            avg_perception_latency_ms=np.mean(perc_latencies) if perc_latencies else 0,
            avg_planning_latency_ms=np.mean(plan_latencies) if plan_latencies else 0,
            avg_control_latency_ms=np.mean(ctrl_latencies) if ctrl_latencies else 0,
            avg_total_latency_ms=np.mean(total_latencies) if total_latencies else 0,
            passed=len(failure_reasons) == 0,
            failure_reasons=failure_reasons
        )

    def save_run_data(self, filepath: str):
        """Save all frame data to JSON for detailed analysis."""
        data = {
            'frames': [asdict(f) for f in self.frame_history],
            'collisions': [asdict(c) for c in self.collision_events],
            'summary': asdict(self._compute_summary())
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    def get_realtime_stats(self) -> Dict:
        """Get current runtime statistics (for HUD display)."""
        if not self.frame_history:
            return {}

        recent = list(self.frame_history)[-30:]  # Last 30 frames
        return {
            'fps': len(recent) / (recent[-1].timestamp - recent[0].timestamp) if len(recent) > 1 else 0,
            'avg_speed': np.mean([f.ego_speed for f in recent]),
            'collision_count': len(self.collision_events),
            'nearest_obstacle': min((f.nearest_obstacle_dist for f in recent if f.nearest_obstacle_dist > 0), default=999),
            'total_distance': self.total_distance,
        }


class ComparativeAnalyzer:
    """
    Compare performance across multiple test runs or configurations.
    """

    def __init__(self):
        self.results: List[TestRunSummary] = []

    def add_result(self, summary: TestRunSummary):
        """Add a test run result."""
        self.results.append(summary)

    def compare_v2v_gain(self) -> Dict:
        """
        Compare runs with and without V2V to compute cooperative gain.
        Returns percentage improvements.
        """
        with_v2v = [r for r in self.results if r.v2v_enabled]
        without_v2v = [r for r in self.results if not r.v2v_enabled]

        if not with_v2v or not without_v2v:
            return {"error": "Need both V2V and non-V2V runs for comparison"}

        # Average metrics
        def avg(values):
            return np.mean(values) if values else 0

        collisions_without = avg([r.collision_count for r in without_v2v])
        collisions_with = avg([r.collision_count for r in with_v2v])

        near_misses_without = avg([r.near_misses for r in without_v2v])
        near_misses_with = avg([r.near_misses for r in with_v2v])

        min_dist_without = avg([r.min_distance_to_obstacle for r in without_v2v])
        min_dist_with = avg([r.min_distance_to_obstacle for r in with_v2v])

        return {
            "collision_reduction_pct": (collisions_without - collisions_with) / max(collisions_without, 1) * 100,
            "near_miss_reduction_pct": (near_misses_without - near_misses_with) / max(near_misses_without, 1) * 100,
            "min_distance_improvement_pct": (min_dist_with - min_dist_without) / max(min_dist_without, 0.1) * 100,
            "sample_size_with_v2v": len(with_v2v),
            "sample_size_without_v2v": len(without_v2v),
        }

    def generate_comparison_table(self) -> str:
        """Generate a markdown table comparing all runs."""
        lines = [
            "| Scenario | Collisions | Near Misses | Avg Speed | Min Dist | Passed |",
            "|----------|-----------|-------------|-----------|----------|--------|"
        ]

        for r in self.results:
            lines.append(
                f"| {r.scenario_name} | {r.collision_count} | {r.near_misses} | "
                f"{r.average_speed:.1f} | {r.min_distance_to_obstacle:.2f} | "
                f"{'✓' if r.passed else '✗'} |"
            )

        return "\n".join(lines)

    def find_best_configuration(self, metric: str = "collision_count") -> Optional[TestRunSummary]:
        """Find the best performing configuration based on a metric."""
        if not self.results:
            return None

        # Lower is better for these metrics
        lower_is_better = ["collision_count", "near_misses", "avg_lane_deviation", "time_in_emergency"]

        if metric in lower_is_better:
            return min(self.results, key=lambda r: getattr(r, metric, float('inf')))
        else:
            return max(self.results, key=lambda r: getattr(r, metric, 0))
