#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派小车电机驱动模块
Motor Driver Module for Raspberry Pi Robot Car

基于创乐博LOBOROBOT扩展板的电机控制
使用gpiozero库实现PWM速度控制
"""

import time
from enum import Enum
from typing import Optional, Tuple
from dataclasses import dataclass

# 尝试导入gpiozero，如果失败则使用模拟模式
try:
    from gpiozero import PWMOutputDevice, DigitalOutputDevice
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("[WARNING] gpiozero not available, running in simulation mode")

from .hardware_config import hardware_config, MotorPins


class Direction(Enum):
    """运动方向枚举"""
    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    ROTATE_LEFT = 5
    ROTATE_RIGHT = 6


@dataclass
class MotorState:
    """电机状态"""
    speed: float = 0.0  # -100 to 100
    direction: Direction = Direction.STOP


class SingleMotor:
    """单个电机控制类"""
    
    def __init__(self, pwm_pin: int, dir1_pin: int, dir2_pin: int, name: str = "Motor"):
        self.name = name
        self.pwm_pin = pwm_pin
        self.dir1_pin = dir1_pin
        self.dir2_pin = dir2_pin
        
        self._pwm: Optional[PWMOutputDevice] = None
        self._dir1: Optional[DigitalOutputDevice] = None
        self._dir2: Optional[DigitalOutputDevice] = None
        self._state = MotorState()
        
        if GPIO_AVAILABLE:
            self._init_gpio()
    
    def _init_gpio(self):
        """初始化GPIO引脚"""
        try:
            self._pwm = PWMOutputDevice(self.pwm_pin)
            self._dir1 = DigitalOutputDevice(self.dir1_pin)
            self._dir2 = DigitalOutputDevice(self.dir2_pin)
            print(f"[INFO] {self.name} initialized on pins PWM={self.pwm_pin}, "
                  f"DIR1={self.dir1_pin}, DIR2={self.dir2_pin}")
        except Exception as e:
            print(f"[ERROR] Failed to initialize {self.name}: {e}")
            self._pwm = None
    
    def set_speed(self, speed: float):
        """
        设置电机速度
        
        Args:
            speed: 速度值 (-100.0 to 100.0)
                   正值为前进，负值为后退，0为停止
        """
        speed = max(-100.0, min(100.0, speed))
        self._state.speed = speed
        
        if abs(speed) < hardware_config.control.min_speed:
            self.stop()
            return
        
        if not GPIO_AVAILABLE or self._pwm is None:
            # 模拟模式
            direction = "FORWARD" if speed > 0 else "BACKWARD"
            print(f"[SIM] {self.name}: {direction} at {abs(speed):.1f}%")
            return
        
        # 设置方向
        if speed > 0:
            self._dir1.on()
            self._dir2.off()
        else:
            self._dir1.off()
            self._dir2.on()
        
        # 设置PWM占空比 (0-1)
        duty_cycle = abs(speed) / 100.0
        self._pwm.value = duty_cycle
    
    def stop(self):
        """停止电机"""
        self._state.speed = 0.0
        self._state.direction = Direction.STOP
        
        if not GPIO_AVAILABLE or self._pwm is None:
            print(f"[SIM] {self.name}: STOP")
            return
        
        self._pwm.off()
        self._dir1.off()
        self._dir2.off()
    
    def get_state(self) -> MotorState:
        """获取当前状态"""
        return self._state
    
    def cleanup(self):
        """清理资源"""
        self.stop()
        if GPIO_AVAILABLE and self._pwm:
            self._pwm.close()
            self._dir1.close()
            self._dir2.close()


class MotorController:
    """
    4WD小车电机控制器
    管理四个电机的协调运动
    """
    
    def __init__(self):
        self.config = hardware_config.motor
        
        # 初始化四个电机
        self.left_front = SingleMotor(
            self.config.left_front_pwm,
            self.config.left_front_dir1,
            self.config.left_front_dir2,
            "LeftFront"
        )
        self.right_front = SingleMotor(
            self.config.right_front_pwm,
            self.config.right_front_dir1,
            self.config.right_front_dir2,
            "RightFront"
        )
        self.left_rear = SingleMotor(
            self.config.left_rear_pwm,
            self.config.left_rear_dir1,
            self.config.left_rear_dir2,
            "LeftRear"
        )
        self.right_rear = SingleMotor(
            self.config.right_rear_pwm,
            self.config.right_rear_dir1,
            self.config.right_rear_dir2,
            "RightRear"
        )
        
        self._current_direction = Direction.STOP
        self._current_speed = 0.0
    
    def move_forward(self, speed: float = 50.0):
        """前进"""
        self._current_direction = Direction.FORWARD
        self._current_speed = speed
        
        self.left_front.set_speed(speed)
        self.right_front.set_speed(speed)
        self.left_rear.set_speed(speed)
        self.right_rear.set_speed(speed)
    
    def move_backward(self, speed: float = 50.0):
        """后退"""
        self._current_direction = Direction.BACKWARD
        self._current_speed = speed
        
        self.left_front.set_speed(-speed)
        self.right_front.set_speed(-speed)
        self.left_rear.set_speed(-speed)
        self.right_rear.set_speed(-speed)
    
    def turn_left(self, speed: float = 50.0, turn_rate: float = 0.5):
        """
        左转（差速转向）
        
        Args:
            speed: 基础速度
            turn_rate: 转向速率 (0-1)，越大转弯越急
        """
        self._current_direction = Direction.LEFT
        self._current_speed = speed
        
        left_speed = speed * (1 - turn_rate)
        right_speed = speed
        
        self.left_front.set_speed(left_speed)
        self.right_front.set_speed(right_speed)
        self.left_rear.set_speed(left_speed)
        self.right_rear.set_speed(right_speed)
    
    def turn_right(self, speed: float = 50.0, turn_rate: float = 0.5):
        """
        右转（差速转向）
        
        Args:
            speed: 基础速度
            turn_rate: 转向速率 (0-1)，越大转弯越急
        """
        self._current_direction = Direction.RIGHT
        self._current_speed = speed
        
        left_speed = speed
        right_speed = speed * (1 - turn_rate)
        
        self.left_front.set_speed(left_speed)
        self.right_front.set_speed(right_speed)
        self.left_rear.set_speed(left_speed)
        self.right_rear.set_speed(right_speed)
    
    def rotate_left(self, speed: float = 50.0):
        """原地左转"""
        self._current_direction = Direction.ROTATE_LEFT
        self._current_speed = speed
        
        self.left_front.set_speed(-speed)
        self.right_front.set_speed(speed)
        self.left_rear.set_speed(-speed)
        self.right_rear.set_speed(speed)
    
    def rotate_right(self, speed: float = 50.0):
        """原地右转"""
        self._current_direction = Direction.ROTATE_RIGHT
        self._current_speed = speed
        
        self.left_front.set_speed(speed)
        self.right_front.set_speed(-speed)
        self.left_rear.set_speed(speed)
        self.right_rear.set_speed(-speed)
    
    def curve_move(self, linear_speed: float, angular_rate: float):
        """
        曲线运动（用于路径规划）
        
        Args:
            linear_speed: 线速度 (-100 to 100)
            angular_rate: 角速度比率 (-1 to 1)，负值左转，正值右转
        """
        # 计算左右轮速差
        speed_diff = abs(linear_speed) * angular_rate * hardware_config.control.turn_factor
        
        left_speed = linear_speed - speed_diff
        right_speed = linear_speed + speed_diff
        
        # 限制在有效范围内
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        self.left_front.set_speed(left_speed)
        self.right_front.set_speed(right_speed)
        self.left_rear.set_speed(left_speed)
        self.right_rear.set_speed(right_speed)
        
        self._current_speed = (abs(left_speed) + abs(right_speed)) / 2
    
    def stop(self):
        """停止所有电机"""
        self._current_direction = Direction.STOP
        self._current_speed = 0.0
        
        self.left_front.stop()
        self.right_front.stop()
        self.left_rear.stop()
        self.right_rear.stop()
    
    def emergency_stop(self):
        """紧急停止"""
        print("[EMERGENCY] Stopping all motors!")
        self.stop()
    
    def get_status(self) -> dict:
        """获取控制器状态"""
        return {
            'direction': self._current_direction.name,
            'speed': self._current_speed,
            'motors': {
                'left_front': self.left_front.get_state().__dict__,
                'right_front': self.right_front.get_state().__dict__,
                'left_rear': self.left_rear.get_state().__dict__,
                'right_rear': self.right_rear.get_state().__dict__
            }
        }
    
    def test_sequence(self):
        """测试序列：依次测试各种运动"""
        print("\n=== Motor Test Sequence ===")
        
        test_moves = [
            ("Forward", lambda: self.move_forward(30)),
            ("Backward", lambda: self.move_backward(30)),
            ("Turn Left", lambda: self.turn_left(40, 0.5)),
            ("Turn Right", lambda: self.turn_right(40, 0.5)),
            ("Rotate Left", lambda: self.rotate_left(30)),
            ("Rotate Right", lambda: self.rotate_right(30)),
        ]
        
        for name, move_func in test_moves:
            print(f"\nTesting: {name}")
            move_func()
            time.sleep(2)
            self.stop()
            time.sleep(0.5)
        
        print("\n=== Test Complete ===")
    
    def cleanup(self):
        """清理所有资源"""
        self.stop()
        self.left_front.cleanup()
        self.right_front.cleanup()
        self.left_rear.cleanup()
        self.right_rear.cleanup()


# 便捷函数
def create_motor_controller() -> MotorController:
    """创建电机控制器实例"""
    return MotorController()


if __name__ == "__main__":
    # 独立测试
    print("Testing Motor Controller...")
    controller = create_motor_controller()
    
    try:
        controller.test_sequence()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.cleanup()
