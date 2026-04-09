#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机驱动测试脚本
Motor Driver Test Script

基于gpiozero.Robot双电机差速驱动架构
Pin mapping matches makerobo_code:
  left  = Motor(forward=22, backward=27, enable=18)
  right = Motor(forward=25, backward=24, enable=23)

测试内容：
1. 前进/后退
2. 左转/右转
3. 原地旋转
4. 曲线运动
5. 紧急停止

使用方法：
    python test_motor.py [--duration SECONDS]
"""

import sys
import os
import time
import argparse

# Dynamically resolve project root
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

try:
    from rpi_deploy.motor_driver import MotorController, Direction
    from rpi_deploy.hardware_config import hardware_config
except ImportError as e:
    print(f"[ERROR] Import failed: {e}")
    print("Make sure you're running this on Raspberry Pi with the project installed.")
    sys.exit(1)


def test_forward_backward(controller: MotorController, duration: float = 2.0):
    """测试前进和后退"""
    print("\n" + "=" * 50)
    print("TEST 1: Forward/Backward Movement")
    print("=" * 50)

    # Speed values for gpiozero.Robot are 0.0-1.0
    speeds = [0.2, 0.4, 0.6]

    for speed in speeds:
        print(f"\nForward at speed {speed:.1f}...")
        controller.move_forward(speed)
        time.sleep(duration)
        controller.stop()
        time.sleep(1.0)

        print(f"Backward at speed {speed:.1f}...")
        controller.move_backward(speed)
        time.sleep(duration)
        controller.stop()
        time.sleep(1.0)

    print("\n[OK] Forward/backward test complete")


def test_turning(controller: MotorController, duration: float = 2.0):
    """测试转向（差速转向）"""
    print("\n" + "=" * 50)
    print("TEST 2: Turning Test (differential steering)")
    print("=" * 50)

    # Left turn
    print("\nTurning left at speed 0.3...")
    controller.turn_left(0.3)
    time.sleep(duration)
    controller.stop()
    time.sleep(1.0)

    # Right turn
    print("Turning right at speed 0.3...")
    controller.turn_right(0.3)
    time.sleep(duration)
    controller.stop()
    time.sleep(1.0)

    print("\n[OK] Turning test complete")


def test_rotation(controller: MotorController, duration: float = 2.0):
    """测试原地旋转"""
    print("\n" + "=" * 50)
    print("TEST 3: Rotation Test (pivot)")
    print("=" * 50)

    print("\nPivot left at speed 0.3...")
    controller.rotate_left(0.3)
    time.sleep(duration)
    controller.stop()
    time.sleep(1.0)

    print("Pivot right at speed 0.3...")
    controller.rotate_right(0.3)
    time.sleep(duration)
    controller.stop()
    time.sleep(1.0)

    print("\n[OK] Rotation test complete")


def test_curve_movement(controller: MotorController):
    """测试曲线运动"""
    print("\n" + "=" * 50)
    print("TEST 4: Curve Movement Test")
    print("=" * 50)

    movements = [
        (0.4, -0.3, "Gentle left curve"),
        (0.4, 0.3, "Gentle right curve"),
        (0.3, -0.6, "Sharp left curve"),
        (0.3, 0.6, "Sharp right curve"),
    ]

    for linear, angular, description in movements:
        print(f"\n{description} (linear={linear}, angular={angular})...")
        controller.curve_move(linear, angular)
        time.sleep(3.0)
        controller.stop()
        time.sleep(1.0)

    print("\n[OK] Curve movement test complete")


def test_emergency_stop(controller: MotorController):
    """测试紧急停止"""
    print("\n" + "=" * 50)
    print("TEST 5: Emergency Stop Test")
    print("=" * 50)

    print("\nMoving forward at speed 0.5...")
    controller.move_forward(0.5)
    time.sleep(1.0)

    print("EMERGENCY STOP!")
    controller.emergency_stop()
    time.sleep(2.0)

    print("\n[OK] Emergency stop test complete")


def test_status(controller: MotorController):
    """测试状态查询"""
    print("\n" + "=" * 50)
    print("TEST 6: Status Check")
    print("=" * 50)

    controller.move_forward(0.3)
    status = controller.get_status()
    print(f"Status after forward: {status}")

    time.sleep(0.5)

    controller.stop()
    status = controller.get_status()
    print(f"Status after stop: {status}")

    print("\n[OK] Status check complete")


def main():
    parser = argparse.ArgumentParser(description='Motor Driver Test Script')
    parser.add_argument('--duration', type=float, default=2.0,
                        help='Duration for each test movement (seconds)')
    parser.add_argument('--test', type=str, default='all',
                        choices=['all', 'forward', 'turn', 'rotate', 'curve', 'emergency', 'status'],
                        help='Which test to run')
    args = parser.parse_args()

    # Print pin configuration
    cfg = hardware_config.motor
    print("\n" + "=" * 60)
    print("Raspberry Pi Car - Motor Driver Test")
    print("=" * 60)
    print(f"Pin config (makerobo):")
    print(f"  Left  : fwd=GPIO{cfg.left_forward}, bwd=GPIO{cfg.left_backward}, en=GPIO{cfg.left_enable}")
    print(f"  Right : fwd=GPIO{cfg.right_forward}, bwd=GPIO{cfg.right_backward}, en=GPIO{cfg.right_enable}")
    print(f"Test duration per movement: {args.duration}s")
    print("WARNING: Make sure the car is on a safe surface!")
    print("Press Ctrl+C to stop at any time")
    print("=" * 60)

    input("\nPress Enter to start testing...")

    controller = MotorController()

    try:
        if args.test in ['all', 'forward']:
            test_forward_backward(controller, args.duration)

        if args.test in ['all', 'turn']:
            test_turning(controller, args.duration)

        if args.test in ['all', 'rotate']:
            test_rotation(controller, args.duration)

        if args.test in ['all', 'curve']:
            test_curve_movement(controller)

        if args.test in ['all', 'emergency']:
            test_emergency_stop(controller)

        if args.test in ['all', 'status']:
            test_status(controller)

        print("\n" + "=" * 60)
        print("ALL TESTS COMPLETED SUCCESSFULLY!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Stopping all motors...")
        controller.emergency_stop()
    finally:
        controller.cleanup()
        print("[OK] Cleanup complete")


if __name__ == "__main__":
    main()