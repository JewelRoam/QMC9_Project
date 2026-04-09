#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机控制测试脚本
Servo Controller Test Script

测试内容：
1. 超声波云台舵机运动范围
2. 摄像头云台水平和俯仰运动
3. 扫描序列测试
4. 同时多舵机控制

使用方法：
    python test_servo.py [--type all|ultrasonic|camera]
"""

import sys
import os
import time
import argparse

# Dynamically resolve project root (works on any machine)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

try:
    from rpi_deploy.servo_controller import ServoController
    from rpi_deploy.hardware_config import hardware_config
except ImportError as e:
    print(f"[ERROR] Import failed: {e}")
    sys.exit(1)


def test_ultrasonic_servo(controller: ServoController):
    """测试超声波云台舵机"""
    print("\n" + "="*50)
    print("TEST 1: Ultrasonic Servo Test")
    print("="*50)
    
    config = hardware_config.ultrasonic
    angles = [config.center_angle, 45, 0, 45, 90, 135, 180, 135, 90]
    
    print(f"\nServo range: {config.min_angle}° to {config.max_angle}°")
    print(f"Center position: {config.center_angle}°")
    
    for angle in angles:
        print(f"  Setting angle to {angle}°...")
        controller.set_ultrasonic_angle(angle)
        time.sleep(0.5)
    
    print("\n[OK] Ultrasonic servo test complete")


def test_camera_servos(controller: ServoController):
    """测试摄像头云台舵机"""
    print("\n" + "="*50)
    print("TEST 2: Camera Servo Test")
    print("="*50)
    
    config = hardware_config.camera
    
    print(f"\nPan range: {config.pan_min}° to {config.pan_max}°")
    print(f"Tilt range: {config.tilt_min}° to {config.tilt_max}°")
    
    # 水平扫描
    print("\nPan sweep test...")
    for pan in [90, 45, 0, 45, 90, 135, 180, 135, 90]:
        print(f"  Pan: {pan}°")
        controller.set_camera_pan(pan)
        time.sleep(0.3)
    
    # 俯仰扫描
    print("\nTilt sweep test...")
    for tilt in [45, 20, -10, 20, 45, 70, 90, 70, 45]:
        print(f"  Tilt: {tilt}°")
        controller.set_camera_tilt(tilt)
        time.sleep(0.3)
    
    # 组合位置测试
    print("\nCombined position test...")
    positions = [
        (90, 45),   # Center
        (45, 30),   # Left-down
        (135, 30),  # Right-down
        (135, 70),  # Right-up
        (45, 70),   # Left-up
        (90, 45),   # Back to center
    ]
    
    for pan, tilt in positions:
        print(f"  Position: pan={pan}°, tilt={tilt}°")
        controller.set_camera_position(pan, tilt)
        time.sleep(0.5)
    
    print("\n[OK] Camera servo test complete")


def test_scan_sequence(controller: ServoController):
    """测试扫描序列"""
    print("\n" + "="*50)
    print("TEST 3: Scan Sequence Test")
    print("="*50)
    
    print("\nStandard scan sequence...")
    angles = hardware_config.control.scan_angles
    print(f"Scan angles: {angles}")
    
    for angle in angles:
        controller.set_ultrasonic_angle(angle)
        print(f"  Scanned at {angle}°")
        time.sleep(0.3)
    
    controller.center_ultrasonic()
    print("\n[OK] Scan sequence test complete")


def test_smooth_movement(controller: ServoController):
    """测试平滑运动"""
    print("\n" + "="*50)
    print("TEST 4: Smooth Movement Test")
    print("="*50)
    
    print("\nSmooth pan movement...")
    for angle in range(0, 181, 5):
        controller.set_ultrasonic_angle(angle)
        time.sleep(0.05)
    
    time.sleep(0.5)
    
    print("Smooth return...")
    for angle in range(180, -1, -5):
        controller.set_ultrasonic_angle(angle)
        time.sleep(0.05)
    
    controller.center_ultrasonic()
    print("\n[OK] Smooth movement test complete")


def main():
    parser = argparse.ArgumentParser(description='Servo Controller Test Script')
    parser.add_argument('--type', type=str, default='all',
                       choices=['all', 'ultrasonic', 'camera', 'scan', 'smooth'],
                       help='Which test to run')
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("Raspberry Pi Car - Servo Controller Test")
    print("="*60)
    print("WARNING: Make sure servos have clear movement range!")
    print("Press Ctrl+C to stop at any time")
    print("="*60)
    
    input("\nPress Enter to start testing...")
    
    controller = ServoController()
    
    try:
        if args.type in ['all', 'ultrasonic']:
            test_ultrasonic_servo(controller)
            time.sleep(1.0)
        
        if args.type in ['all', 'camera']:
            test_camera_servos(controller)
            time.sleep(1.0)
        
        if args.type in ['all', 'scan']:
            test_scan_sequence(controller)
            time.sleep(1.0)
        
        if args.type in ['all', 'smooth']:
            test_smooth_movement(controller)
        
        print("\n" + "="*60)
        print("ALL TESTS COMPLETED SUCCESSFULLY!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Stopping...")
    finally:
        controller.cleanup()
        print("[OK] Cleanup complete")


if __name__ == "__main__":
    main()
