#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机驱动测试脚本
Motor Driver Test Script

测试内容：
1. 单个电机正转/反转
2. 4WD前进/后退
3. 差速转向
4. 原地旋转
5. 紧急停止

使用方法：
    python test_motor.py [--duration SECONDS]
"""

import sys
import os
import time
import argparse

# Dynamically resolve project root (works on any machine)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

try:
    from rpi_deploy.motor_driver import MotorController, Direction
    from rpi_deploy.hardware_config import hardware_config
except ImportError as e:
    print(f"[ERROR] Import failed: {e}")
    print("Make sure you're running this on Raspberry Pi with the project installed.")
    sys.exit(1)


def test_single_motors(controller: MotorController, duration: float = 2.0):
    """测试单个电机"""
    print("\n" + "="*50)
    print("TEST 1: Single Motor Test")
    print("="*50)
    
    motors = [
        ("Left Front", controller.left_front),
        ("Right Front", controller.right_front),
        ("Left Rear", controller.left_rear),
        ("Right Rear", controller.right_rear),
    ]
    
    for name, motor in motors:
        print(f"\nTesting {name} motor...")
        
        # 正转
        print(f"  Forward at 30%...")
        motor.set_speed(30)
        time.sleep(duration)
        motor.stop()
        time.sleep(0.5)
        
        # 反转
        print(f"  Backward at 30%...")
        motor.set_speed(-30)
        time.sleep(duration)
        motor.stop()
        time.sleep(0.5)
    
    print("\n[OK] Single motor test complete")


def test_forward_backward(controller: MotorController, duration: float = 3.0):
    """测试前进和后退"""
    print("\n" + "="*50)
    print("TEST 2: Forward/Backward Movement")
    print("="*50)
    
    speeds = [30, 50, 70]
    
    for speed in speeds:
        print(f"\nForward at {speed}%...")
        controller.move_forward(speed)
        time.sleep(duration)
        controller.stop()
        time.sleep(1.0)
        
        print(f"Backward at {speed}%...")
        controller.move_backward(speed)
        time.sleep(duration)
        controller.stop()
        time.sleep(1.0)
    
    print("\n[OK] Forward/backward test complete")


def test_turning(controller: MotorController, duration: float = 3.0):
    """测试转向"""
    print("\n" + "="*50)
    print("TEST 3: Turning Test")
    print("="*50)
    
    # 左转
    print("\nTurning left at 40% speed, 50% turn rate...")
    controller.turn_left(speed=40, turn_rate=0.5)
    time.sleep(duration)
    controller.stop()
    time.sleep(1.0)
    
    # 右转
    print("Turning right at 40% speed, 50% turn rate...")
    controller.turn_right(speed=40, turn_rate=0.5)
    time.sleep(duration)
    controller.stop()
    time.sleep(1.0)
    
    # 急转弯
    print("Sharp left turn (80% turn rate)...")
    controller.turn_left(speed=30, turn_rate=0.8)
    time.sleep(duration)
    controller.stop()
    time.sleep(1.0)
    
    print("\n[OK] Turning test complete")


def test_rotation(controller: MotorController, duration: float = 2.0):
    """测试原地旋转"""
    print("\n" + "="*50)
    print("TEST 4: Rotation Test")
    print("="*50)
    
    print("\nRotating left (in place)...")
    controller.rotate_left(speed=40)
    time.sleep(duration)
    controller.stop()
    time.sleep(1.0)
    
    print("Rotating right (in place)...")
    controller.rotate_right(speed=40)
    time.sleep(duration)
    controller.stop()
    time.sleep(1.0)
    
    print("\n[OK] Rotation test complete")


def test_curve_movement(controller: MotorController):
    """测试曲线运动"""
    print("\n" + "="*50)
    print("TEST 5: Curve Movement Test")
    print("="*50)
    
    movements = [
        (50, -0.3, "Gentle left curve"),
        (50, 0.3, "Gentle right curve"),
        (30, -0.7, "Sharp left curve"),
        (30, 0.7, "Sharp right curve"),
    ]
    
    for speed, angular, description in movements:
        print(f"\n{description} (speed={speed}, angular={angular})...")
        controller.curve_move(speed, angular)
        time.sleep(3.0)
        controller.stop()
        time.sleep(1.0)
    
    print("\n[OK] Curve movement test complete")


def test_emergency_stop(controller: MotorController):
    """测试紧急停止"""
    print("\n" + "="*50)
    print("TEST 6: Emergency Stop Test")
    print("="*50)
    
    print("\nMoving forward at 60%...")
    controller.move_forward(60)
    time.sleep(1.0)
    
    print("EMERGENCY STOP!")
    controller.emergency_stop()
    time.sleep(2.0)
    
    print("\n[OK] Emergency stop test complete")


def main():
    parser = argparse.ArgumentParser(description='Motor Driver Test Script')
    parser.add_argument('--duration', type=float, default=2.0,
                       help='Duration for each test movement (seconds)')
    parser.add_argument('--test', type=str, default='all',
                       choices=['all', 'single', 'forward', 'turn', 'rotate', 'curve', 'emergency'],
                       help='Which test to run')
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("Raspberry Pi Car - Motor Driver Test")
    print("="*60)
    print(f"Test duration per movement: {args.duration}s")
    print("WARNING: Make sure the car is on a safe surface!")
    print("Press Ctrl+C to stop at any time")
    print("="*60)
    
    input("\nPress Enter to start testing...")
    
    controller = MotorController()
    
    try:
        if args.test in ['all', 'single']:
            test_single_motors(controller, args.duration)
        
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
        
        print("\n" + "="*60)
        print("ALL TESTS COMPLETED SUCCESSFULLY!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Stopping all motors...")
        controller.emergency_stop()
    finally:
        controller.cleanup()
        print("[OK] Cleanup complete")


if __name__ == "__main__":
    main()
