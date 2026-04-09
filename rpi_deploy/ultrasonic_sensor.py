#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派小车超声波传感器模块
Ultrasonic Sensor Module for Raspberry Pi Robot Car

HC-SR04超声波测距模块驱动
优先使用gpiozero.DistanceSensor, 与makerobo_code案例6/7一致

引脚定义:
  trigger = 20, echo = 21 (与makerobo_code一致)
"""

import time
from typing import Optional, List, Tuple
from dataclasses import dataclass
from statistics import median

# 尝试导入gpiozero DistanceSensor
try:
    from gpiozero import DistanceSensor as GpiozeroDistanceSensor
    DISTANCE_SENSOR_AVAILABLE = True
except ImportError:
    DISTANCE_SENSOR_AVAILABLE = False
    GpiozeroDistanceSensor = None
    print("[WARNING] gpiozero.DistanceSensor not available, running in simulation mode")

from .hardware_config import hardware_config


@dataclass
class DistanceReading:
    """距离测量结果"""
    distance_cm: float
    timestamp: float
    angle: Optional[float] = None  # 测量时的舵机角度
    valid: bool = True


class UltrasonicSensor:
    """
    HC-SR04超声波传感器驱动类

    使用gpiozero.DistanceSensor, 与makerobo_code案例一致:
      DistanceSensor(echo=21, trigger=20, max_distance=3, threshold_distance=0.2)

    gpiozero返回的距离单位为米, 内部自动转换为厘米.
    """

    def __init__(self, trigger_pin: int = None, echo_pin: int = None):
        self.config = hardware_config.ultrasonic
        self.trigger_pin = trigger_pin or self.config.trigger_pin
        self.echo_pin = echo_pin or self.config.echo_pin

        # 测量参数
        self.max_distance_m = 3.0       # 最大距离3米
        self.threshold_distance_m = 0.2 # 阈值距离0.2米
        self.min_distance_cm = 2.0      # 最小有效距离2cm
        self.max_distance_cm = 400.0    # 最大有效距离400cm

        # 滤波参数
        self.history_size = 5
        self.distance_history: List[float] = []

        # gpiozero sensor instance
        self._sensor = None

        if DISTANCE_SENSOR_AVAILABLE:
            self._init_gpio()
        else:
            print(f"[SIM] Ultrasonic sensor simulated on TRIG={self.trigger_pin}, ECHO={self.echo_pin}")

    def _init_gpio(self):
        """Initialize gpiozero.DistanceSensor with makerobo pin mapping."""
        try:
            self._sensor = GpiozeroDistanceSensor(
                echo=self.echo_pin,
                trigger=self.trigger_pin,
                max_distance=self.max_distance_m,
                threshold_distance=self.threshold_distance_m,
            )
            print(f"[INFO] Ultrasonic sensor initialized: TRIG=GPIO{self.trigger_pin}, ECHO=GPIO{self.echo_pin}")
        except Exception as e:
            print(f"[ERROR] Failed to initialize ultrasonic sensor: {e}")
            self._sensor = None

    def _simulate_distance(self) -> float:
        """模拟距离读数（用于测试）"""
        import random
        return random.uniform(30.0, 100.0)

    def measure_once(self) -> DistanceReading:
        """
        执行单次距离测量

        Returns:
            DistanceReading对象包含距离(cm)和时间戳
        """
        if not DISTANCE_SENSOR_AVAILABLE or self._sensor is None:
            distance = self._simulate_distance()
            return DistanceReading(
                distance_cm=distance,
                timestamp=time.time(),
                valid=True
            )

        try:
            # gpiozero returns distance in meters, convert to cm
            distance_m = self._sensor.distance
            distance_cm = distance_m * 100.0

            # gpiozero may return 0 or max_distance when out of range
            if distance_m <= 0:
                return DistanceReading(
                    distance_cm=float('inf'),
                    timestamp=time.time(),
                    valid=False
                )

            valid = self.min_distance_cm <= distance_cm <= self.max_distance_cm
            return DistanceReading(
                distance_cm=distance_cm,
                timestamp=time.time(),
                valid=valid
            )
        except Exception as e:
            print(f"[ERROR] Measurement failed: {e}")
            return DistanceReading(
                distance_cm=float('inf'),
                timestamp=time.time(),
                valid=False
            )

    def measure_average(self, samples: int = 3, delay: float = 0.05) -> DistanceReading:
        """
        多次测量取中值滤波

        Args:
            samples: 采样次数
            delay: 每次测量间隔

        Returns:
            滤波后的距离读数
        """
        readings = []

        for _ in range(samples):
            reading = self.measure_once()
            if reading.valid:
                readings.append(reading.distance_cm)
            time.sleep(delay)

        if not readings:
            return DistanceReading(
                distance_cm=float('inf'),
                timestamp=time.time(),
                valid=False
            )

        # 使用中值滤波
        filtered_distance = median(readings)

        # 更新历史记录
        self.distance_history.append(filtered_distance)
        if len(self.distance_history) > self.history_size:
            self.distance_history.pop(0)

        return DistanceReading(
            distance_cm=filtered_distance,
            timestamp=time.time(),
            valid=True
        )

    def get_filtered_distance(self) -> float:
        """
        获取历史滤波后的距离

        Returns:
            基于历史记录的平滑距离值(cm)
        """
        if len(self.distance_history) < 2:
            reading = self.measure_average()
            return reading.distance_cm if reading.valid else float('inf')

        recent = self.distance_history[-3:] if len(self.distance_history) >= 3 else self.distance_history
        return median(recent)

    def is_obstacle_detected(self, threshold_cm: float = None) -> bool:
        """
        检测是否有障碍物

        Args:
            threshold_cm: 检测阈值(cm)，默认使用配置中的安全距离

        Returns:
            如果检测到障碍物返回True
        """
        if threshold_cm is None:
            threshold_cm = hardware_config.control.safe_distance_cm

        distance = self.get_filtered_distance()
        return distance < threshold_cm

    def scan_with_servo(self, servo_controller,
                        angles: Tuple[int, ...] = None,
                        delay: float = 0.3) -> List[DistanceReading]:
        """
        使用舵机云台进行多角度扫描

        Args:
            servo_controller: 舵机控制器实例
            angles: 扫描角度列表
            delay: 每个角度的测量延迟

        Returns:
            各角度的距离读数列表
        """
        if angles is None:
            angles = hardware_config.control.scan_angles

        readings = []

        for angle in angles:
            servo_controller.set_ultrasonic_angle(angle)
            time.sleep(delay)

            reading = self.measure_average(samples=2)
            reading.angle = angle
            readings.append(reading)

            status = "ok" if reading.valid else "FAIL"
            print(f"  [{status}] Angle {angle}: {reading.distance_cm:.1f}cm")

        # 归中
        servo_controller.center_ultrasonic()

        return readings

    def find_clear_direction(self, servo_controller,
                             threshold_cm: float = None) -> Optional[int]:
        """
        扫描并找到最畅通的方向

        Args:
            servo_controller: 舵机控制器实例
            threshold_cm: 安全距离阈值(cm)

        Returns:
            最畅通方向的角度，如果没有则返回None
        """
        if threshold_cm is None:
            threshold_cm = hardware_config.control.safe_distance_cm

        print(f"\n[SCAN] Finding clear direction (threshold: {threshold_cm}cm)...")
        readings = self.scan_with_servo(servo_controller)

        valid_readings = [r for r in readings if r.valid and r.distance_cm >= threshold_cm]

        if not valid_readings:
            print("[WARNING] No clear direction found!")
            return None

        best_reading = max(valid_readings, key=lambda r: r.distance_cm)
        print(f"[OK] Best direction: {best_reading.angle} ({best_reading.distance_cm:.1f}cm)")

        return best_reading.angle

    def test_sensor(self, samples: int = 10):
        """测试传感器"""
        print("\n=== Ultrasonic Sensor Test ===")
        print(f"Testing with {samples} samples...")

        for i in range(samples):
            reading = self.measure_average()
            status = "ok" if reading.valid else "FAIL"
            print(f"  [{status}] Sample {i+1}: {reading.distance_cm:.1f}cm")
            time.sleep(0.5)

        print("=== Test Complete ===")

    def cleanup(self):
        """清理资源"""
        if self._sensor:
            self._sensor.close()
            self._sensor = None
        print("[INFO] Ultrasonic sensor cleaned up")


# Convenience function
def create_ultrasonic_sensor() -> UltrasonicSensor:
    """创建超声波传感器实例"""
    return UltrasonicSensor()


if __name__ == "__main__":
    print("Testing Ultrasonic Sensor...")
    sensor = create_ultrasonic_sensor()

    try:
        sensor.test_sensor(samples=10)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        sensor.cleanup()