"""
Unified logging and performance metrics for QMC9 Project.
"""
import logging
import time
import json
import os
from collections import defaultdict
from datetime import datetime


class PerformanceMetrics:
    """Track and record performance metrics for evaluation."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._timers = {}
        self._counters = defaultdict(int)
        self._values = defaultdict(list)
        self._start_time = time.time()

    def start_timer(self, name: str):
        self._timers[name] = time.perf_counter()

    def stop_timer(self, name: str) -> float:
        if name in self._timers:
            elapsed = time.perf_counter() - self._timers[name]
            self._values[f"{name}_ms"].append(elapsed * 1000)
            return elapsed * 1000
        return 0.0

    def increment(self, name: str, value: int = 1):
        self._counters[name] += value

    def record(self, name: str, value: float):
        self._values[name].append(value)

    def get_average(self, name: str) -> float:
        vals = self._values.get(name, [])
        return sum(vals) / len(vals) if vals else 0.0

    def get_counter(self, name: str) -> int:
        return self._counters.get(name, 0)

    def get_summary(self) -> dict:
        summary = {
            "duration_seconds": time.time() - self._start_time,
            "counters": dict(self._counters),
            "averages": {},
        }
        for k, v in self._values.items():
            if v:
                summary["averages"][k] = {
                    "mean": sum(v) / len(v),
                    "min": min(v),
                    "max": max(v),
                    "count": len(v),
                }
        return summary

    def save(self, filepath: str):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(self.get_summary(), f, indent=2)


class FPSCounter:
    """Simple FPS counter."""

    def __init__(self, window_size: int = 30):
        self._window_size = window_size
        self._timestamps = []

    def tick(self):
        now = time.perf_counter()
        self._timestamps.append(now)
        # Keep only recent timestamps
        if len(self._timestamps) > self._window_size:
            self._timestamps = self._timestamps[-self._window_size:]

    @property
    def fps(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0
        dt = self._timestamps[-1] - self._timestamps[0]
        if dt <= 0:
            return 0.0
        return (len(self._timestamps) - 1) / dt


def setup_logger(name: str = "QMC9", level: str = "INFO", log_file: str = None) -> logging.Logger:
    """Create a configured logger instance."""
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))

    # Avoid duplicate handlers
    if logger.handlers:
        return logger

    formatter = logging.Formatter(
        "[%(asctime)s] %(levelname)-8s %(name)-12s | %(message)s",
        datefmt="%H:%M:%S",
    )

    # Console handler
    ch = logging.StreamHandler()
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    # File handler (optional)
    if log_file:
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        fh = logging.FileHandler(log_file)
        fh.setFormatter(formatter)
        logger.addHandler(fh)

    return logger
