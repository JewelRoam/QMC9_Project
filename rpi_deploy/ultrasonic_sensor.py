#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派小车超声波传感器模块
Ultrasonic Sensor Module for Raspberry Pi Robot Car

HC-SR04超声波测距模块驱动
支持舵机云台扫描功能
"""

import time
from typing import Optional, List, Tuple
from dataclasses import dataclass
from statistics import median

# 尝试导入GPIO库
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("[WARNING] RPi.GPIO not available, running in simulation mode")

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
    
    连接方式:
    - VCC -> 5V
    - GND -> GND
    - Trig -> GPIO4 (默认)
    - Echo -> GPIO17 (默认，需通过电平转换或分压电阻降至3.3V)
    """
    
    def __init__(self, trigger_pin: int = None, echo_pin: int = None):
        self.config = hardware_config.ultrasonic
        self.trigger_pin = trigger_pin or self.config.trigger_pin
        self.echo_pin = echo_pin or self.config.echo_pin
        
        # 速度声速 (cm/us)，在20°C时约为343m/s = 0.0343 cm/us
        self.sound_speed = 0.0343
        
        # 测量参数
        self.timeout_us = 25000  # 最大等待时间25ms（约4米）
        self.min_distance = 2.0   # 最小有效距离2cm
        self.max_distance = 400.0 # 最大有效距离400cm
        
        # 滤波参数
        self.history_size = 5
        self.distance_history: List[float] = []
        
        if GPIO_AVAILABLE:
            self._init_gpio()
        else:
            print(f"[SIM] Ultrasonic sensor simulated on TRIG={self.trigger_pin}, ECHO={self.echo_pin}")
    
    def _init_gpio(self):
        """初始化GPIO引脚"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            GPIO.output(self.trigger_pin, False)
            
            # 等待传感器稳定
            time.sleep(0.1)
            print(f"[INFO] Ultrasonic sensor initialized: TRIG=GPIO{self.trigger_pin}, ECHO=GPIO{self.echo_pin}")
        except Exception as e:
            print(f"[ERROR] Failed to initialize ultrasonic sensor: {e}")
    
    def _simulate_distance(self) -> float:
        """模拟距离读数（用于测试）"""
        import random
        # 模拟30-100cm的随机距离
        return random.uniform(30.0, 100.0)
    
    def measure_once(self) -> DistanceReading:
        """
        执行单次距离测量
        
        Returns:
            DistanceReading对象包含距离和时间戳
        """
        if not GPIO_AVAILABLE:
            # 模拟模式
            distance = self._simulate_distance()
            return DistanceReading(
                distance_cm=distance,
                timestamp=time.time(),
                valid=True
            )
        
        try:
            # 发送触发信号（10us高电平）
            GPIO.output(self.trigger_pin, True)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, False)
            
            # 记录发送时刻
            start_time = time.time()
            timeout = start_time + self.timeout_us / 1e6
            
            while GPIO.input(self.echo_pin) == 0:
                start_time = time.time()
                if start_time > timeout:
                    return DistanceReading(
                        distance_cm=float('inf'),
                        timestamp=time.time(),
                        valid=False
                    )
            
            # 记录接收时刻
            end_time = time.time()
            timeout = end_time + self.timeout_us / 1e6
            
            while GPIO.input(self.echo_pin) == 1:
                end_time = time.time()
                if end_time > timeout:
                    return DistanceReading(
                        distance_cm=float('inf'),
                        timestamp=time.time(),
                        valid=False
                    )
            
            # 计算距离
            duration = end_time - start_time
            distance = (duration * 34300) / 2  # 往返距离除以2
            
            # 有效性检查
            valid = self.min_distance <= distance <= self.max_distance
            
            return DistanceReading(
                distance_cm=distance,
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
            基于历史记录的平滑距离值
        """
        if len(self.distance_history) < 2:
            reading = self.measure_average()
            return reading.distance_cm if reading.valid else float('inf')
        
        # 使用最近的历史记录进行中值滤波
        recent = self.distance_history[-3:] if len(self.distance_history) >= 3 else self.distance_history
        return median(recent)
    
    def is_obstacle_detected(self, threshold_cm: float = None) -> bool:
        """
        检测是否有障碍物
        
        Args:
            threshold_cm: 检测阈值，默认使用配置中的安全距离
        
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
            # 设置舵机角度
            servo_controller.set_ultrasonic_angle(angle)
            time.sleep(delay)  # 等待舵机到位
            
            # 测量距离
            reading = self.measure_average(samples=2)
            reading.angle = angle
            readings.append(reading)
            
            status = "✓" if reading.valid else "✗"
            print(f"  [{status}] Angle {angle}°: {reading.distance_cm:.1f}cm")
        
        # 归中
        servo_controller.center_ultrasonic()
        
        return readings
    
    def find_clear_direction(self, servo_controller,
                            threshold_cm: float = None) -> Optional[int]:
        """
        扫描并找到最畅通的方向
        
        Args:
            servo_controller: 舵机控制器实例
            threshold_cm: 安全距离阈值
        
        Returns:
            最畅通方向的角度，如果没有则返回None
        """
        if threshold_cm is None:
            threshold_cm = hardware_config.control.safe_distance_cm
        
        print(f"\n[SCAN] Finding clear direction (threshold: {threshold_cm}cm)...")
        readings = self.scan_with_servo(servo_controller)
        
        # 找到超过阈值的最大的距离
        valid_readings = [r for r in readings if r.valid and r.distance_cm >= threshold_cm]
        
        if not valid_readings:
            print("[WARNING] No clear direction found!")
            return None
        
        best_reading = max(valid_readings, key=lambda r: r.distance_cm)
        print(f"[OK] Best direction: {best_reading.angle}° ({best_reading.distance_cm:.1f}cm)")
        
        return best_reading.angle
    
    def test_sensor(self, samples: int = 10):
        """测试传感器"""
        print("\n=== Ultrasonic Sensor Test ===")
        print(f"Testing with {samples} samples...")
        
        for i in range(samples):
            reading = self.measure_average()
            status = "✓" if reading.valid else "✗"
            print(f"  [{status}] Sample {i+1}: {reading.distance_cm:.1f}cm")
            time.sleep(0.5)
        
        print("=== Test Complete ===")
    
    def cleanup(self):
        """清理资源"""
        if GPIO_AVAILABLE:
            GPIO.cleanup([self.trigger_pin, self.echo_pin])
            print("[INFO] Ultrasonic sensor cleaned up")


# 便捷函数
def create_ultrasonic_sensor() -> UltrasonicSensor:
    """创建超声波传感器实例"""
    return UltrasonicSensor()


if __name__ == "__main__":
    # 独立测试
    print("Testing Ultrasonic Sensor...")
    sensor = create_ultrasonic_sensor()
    
    try:
        sensor.test_sensor(samples=10)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        sensor.cleanup()
