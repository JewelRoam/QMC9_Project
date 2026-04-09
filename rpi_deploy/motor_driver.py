#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派小车电机驱动模块
Motor Driver Module for Raspberry Pi Robot Car

基于创乐博LOBOROBOT扩展板的电机控制
使用gpiozero.Robot实现双电机差速驱动

引脚定义与makerobo_code案例一致:
  left  = Motor(forward=22, backward=27, enable=18)
  right = Motor(forward=25, backward=24, enable=23)

Usage:
  from rpi_deploy.motor_driver import MotorController
  ctrl = MotorController()
  ctrl.move_forward(0.3)   # 0.0-1.0 speed
  ctrl.rotate_left(0.3)
  ctrl.stop()
"""

import time
from enum import Enum
from typing import Optional

# 尝试导入gpiozero，如果失败则使用模拟模式
try:
    from gpiozero import Robot, Motor
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    Robot = None
    Motor = None
    print("[WARNING] gpiozero not available, running in simulation mode")

from .hardware_config import hardware_config


class Direction(Enum):
    """运动方向枚举"""
    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    ROTATE_LEFT = 5
    ROTATE_RIGHT = 6


class MotorController:
    """
    4WD小车电机控制器
    使用gpiozero.Robot双电机差速驱动

    左侧前后轮并联为一组, 右侧前后轮并联为一组,
    与makerobo_code案例3/6/7完全一致.
    """

    def __init__(self):
        self.config = hardware_config.motor
        self._robot: Optional[Robot] = None
        self._current_direction = Direction.STOP
        self._current_speed = 0.0

        if GPIO_AVAILABLE:
            self._init_gpio()
        else:
            print("[SIM] Motor controller in simulation mode")

    def _init_gpio(self):
        """Initialize gpiozero.Robot with makerobo pin mapping."""
        try:
            self._robot = Robot(
                left=Motor(
                    forward=self.config.left_forward,
                    backward=self.config.left_backward,
                    enable=self.config.left_enable,
                ),
                right=Motor(
                    forward=self.config.right_forward,
                    backward=self.config.right_backward,
                    enable=self.config.right_enable,
                ),
            )
            print(f"[INFO] Robot initialized: L(fwd={self.config.left_forward},bwd={self.config.left_backward},en={self.config.left_enable}) "
                  f"R(fwd={self.config.right_forward},bwd={self.config.right_backward},en={self.config.right_enable})")
        except Exception as e:
            print(f"[ERROR] Failed to initialize Robot: {e}")
            self._robot = None

    # -- Basic movement (speed: 0.0 - 1.0) --------------------------------

    def move_forward(self, speed: float = 0.3):
        """
        前进

        Args:
            speed: 速度 0.0-1.0 (gpiozero convention)
        """
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.FORWARD
        self._current_speed = speed

        if self._robot:
            self._robot.forward(speed)
        else:
            print(f"[SIM] FORWARD at {speed:.2f}")

    def move_backward(self, speed: float = 0.3):
        """
        后退

        Args:
            speed: 速度 0.0-1.0
        """
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.BACKWARD
        self._current_speed = speed

        if self._robot:
            self._robot.backward(speed)
        else:
            print(f"[SIM] BACKWARD at {speed:.2f}")

    def turn_left(self, speed: float = 0.3):
        """
        左转（差速转向，左轮慢右轮快）

        Args:
            speed: 速度 0.0-1.0
        """
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.LEFT
        self._current_speed = speed

        if self._robot:
            self._robot.left(speed)
        else:
            print(f"[SIM] LEFT at {speed:.2f}")

    def turn_right(self, speed: float = 0.3):
        """
        右转（差速转向，左轮快右轮慢）

        Args:
            speed: 速度 0.0-1.0
        """
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.RIGHT
        self._current_speed = speed

        if self._robot:
            self._robot.right(speed)
        else:
            print(f"[SIM] RIGHT at {speed:.2f}")

    def rotate_left(self, speed: float = 0.3):
        """
        原地左转（左轮后退、右轮前进）
        gpiozero Robot.left() = reverse(left), forward(right) = pivot left
        """
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.ROTATE_LEFT
        self._current_speed = speed

        if self._robot:
            self._robot.left(speed)
        else:
            print(f"[SIM] ROTATE_LEFT at {speed:.2f}")

    def rotate_right(self, speed: float = 0.3):
        """
        原地右转（左轮前进、右轮后退）
        gpiozero Robot.right() = forward(left), reverse(right) = pivot right
        """
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.ROTATE_RIGHT
        self._current_speed = speed

        if self._robot:
            self._robot.right(speed)
        else:
            print(f"[SIM] ROTATE_RIGHT at {speed:.2f}")

    def curve_move(self, linear_speed: float, angular_rate: float):
        """
        曲线运动

        Args:
            linear_speed: 线速度 (-1.0 to 1.0)
            angular_rate: 角速度比率 (-1.0 to 1.0)
        """
        # Map to left/right motor speeds
        left_speed = max(-1.0, min(1.0, linear_speed + angular_rate * hardware_config.control.turn_factor))
        right_speed = max(-1.0, min(1.0, linear_speed - angular_rate * hardware_config.control.turn_factor))

        if self._robot:
            # gpiozero Robot doesn't have a direct curve method, use motor control
            self._robot.left_motor.forward(max(0, left_speed))
            self._robot.left_motor.backward(max(0, -left_speed))
            self._robot.right_motor.forward(max(0, right_speed))
            self._robot.right_motor.backward(max(0, -right_speed))
        else:
            print(f"[SIM] CURVE L={left_speed:.2f} R={right_speed:.2f}")

        self._current_speed = (abs(left_speed) + abs(right_speed)) / 2

    def stop(self):
        """停止所有电机"""
        self._current_direction = Direction.STOP
        self._current_speed = 0.0

        if self._robot:
            self._robot.stop()
        else:
            print("[SIM] STOP")

    def emergency_stop(self):
        """紧急停止"""
        print("[EMERGENCY] Stopping all motors!")
        self.stop()

    # -- Status & cleanup ---------------------------------------------------

    def get_status(self) -> dict:
        """获取控制器状态"""
        return {
            'direction': self._current_direction.name,
            'speed': self._current_speed,
            'gpio_available': GPIO_AVAILABLE,
            'robot_initialized': self._robot is not None,
        }

    def test_sequence(self):
        """测试序列：依次测试各种运动"""
        print("\n=== Motor Test Sequence ===")
        print("Ensure the robot has space to move!\n")

        test_moves = [
            ("Forward",  lambda: self.move_forward(0.3)),
            ("Backward", lambda: self.move_backward(0.3)),
            ("Turn Left", lambda: self.turn_left(0.3)),
            ("Turn Right", lambda: self.turn_right(0.3)),
            ("Rotate Left", lambda: self.rotate_left(0.3)),
            ("Rotate Right", lambda: self.rotate_right(0.3)),
        ]

        for name, move_func in test_moves:
            print(f"Testing: {name}")
            move_func()
            time.sleep(1.5)
            self.stop()
            time.sleep(0.5)

        print("\n=== Test Complete ===")

    def cleanup(self):
        """清理所有资源"""
        self.stop()
        if self._robot:
            self._robot.close()
            self._robot = None
        print("[INFO] Motor controller cleaned up")


# Convenience function
def create_motor_controller() -> MotorController:
    """创建电机控制器实例"""
    return MotorController()


if __name__ == "__main__":
    print("Testing Motor Controller...")
    controller = create_motor_controller()

    try:
        controller.test_sequence()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.cleanup()