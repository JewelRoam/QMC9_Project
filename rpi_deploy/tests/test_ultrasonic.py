#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
超声波传感器测试脚本
Ultrasonic Sensor Test Script

测试内容：
1. 基本距离测量
2. 多次采样滤波
3. 舵机云台扫描
4. 障碍物检测

使用方法：
    python test_ultrasonic.py [--samples N] [--scan]
"""

import sys
import time
import argparse

sys.path.insert(0, '/home/pi/QMC9_Project')

try:
    from rpi_deploy.ultrasonic_sensor import UltrasonicSensor, create_ultrasonic_sensor
    from rpi_deploy.servo_controller import ServoController
    from rpi_deploy.hardware_config import hardware_config
except ImportError as e:
    print(f"[ERROR] Import failed: {e}")
    sys.exit(1)


def test_basic_measurement(sensor: UltrasonicSensor, samples: int = 10):
    """测试基本测量功能"""
    print("\n" + "="*50)
    print("TEST 1: Basic Distance Measurement")
    print("="*50)
    
    print(f"\nTaking {samples} measurements...")
    print("Place an object at different distances and observe readings.")
    
    for i in range(samples):
        reading = sensor.measure_once()
        status = "✓" if reading.valid else "✗"
        print(f"  [{status}] Sample {i+1}: {reading.distance_cm:.1f} cm")
        time.sleep(0.5)
    
    print("\n[OK] Basic measurement test complete")


def test_filtered_measurement(sensor: UltrasonicSensor, samples: int = 10):
    """测试滤波测量"""
    print("\n" + "="*50)
    print("TEST 2: Filtered Measurement (Median Filter)")
    print("="*50)
    
    print(f"\nTaking {samples} filtered measurements...")
    print("Each reading is the median of 3 samples.")
    
    for i in range(samples):
        reading = sensor.measure_average(samples=3)
        status = "✓" if reading.valid else "✗"
        print(f"  [{status}] Filtered {i+1}: {reading.distance_cm:.1f} cm")
        time.sleep(0.3)
    
    print("\n[OK] Filtered measurement test complete")


def test_obstacle_detection(sensor: UltrasonicSensor, threshold: float = None):
    """测试障碍物检测"""
    print("\n" + "="*50)
    print("TEST 3: Obstacle Detection")
    print("="*50)
    
    if threshold is None:
        threshold = hardware_config.control.safe_distance_cm
    
    print(f"\nObstacle detection threshold: {threshold} cm")
    print("Move an object closer/farther than the threshold...")
    print("Press Ctrl+C to stop this test\n")
    
    try:
        while True:
            distance = sensor.get_filtered_distance()
            detected = sensor.is_obstacle_detected(threshold)
            
            status = "⚠️ OBSTACLE!" if detected else "✓ Clear"
            print(f"  Distance: {distance:.1f} cm - {status}", end='\r')
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n")
    
    print("\n[OK] Obstacle detection test complete")


def test_scan_with_servo(sensor: UltrasonicSensor, servo: ServoController):
    """测试舵机云台扫描"""
    print("\n" + "="*50)
    print("TEST 4: Servo Scanning")
    print("="*50)
    
    print("\nScanning with servo at multiple angles...")
    print("Place objects at different positions and observe readings.\n")
    
    # 执行扫描
    readings = sensor.scan_with_servo(servo)
    
    print("\n--- Scan Results ---")
    valid_readings = [r for r in readings if r.valid]
    if valid_readings:
        avg_distance = sum(r.distance_cm for r in valid_readings) / len(valid_readings)
        print(f"Average distance: {avg_distance:.1f} cm")
        print(f"Valid readings: {len(valid_readings)}/{len(readings)}")
    else:
        print("No valid readings obtained!")
    
    print("\n[OK] Servo scanning test complete")


def test_find_clear_direction(sensor: UltrasonicSensor, servo: ServoController):
    """测试寻找畅通方向"""
    print("\n" + "="*50)
    print("TEST 5: Find Clear Direction")
    print("="*50)
    
    print("\nSearching for clearest direction...")
    print("Place obstacles in some directions to test.\n")
    
    best_angle = sensor.find_clear_direction(servo)
    
    if best_angle is not None:
        print(f"\n[OK] Clearest direction found at {best_angle}°")
    else:
        print("\n[WARNING] No clear direction found - obstacles in all directions!")


def main():
    parser = argparse.ArgumentParser(description='Ultrasonic Sensor Test Script')
    parser.add_argument('--samples', type=int, default=10,
                       help='Number of measurement samples')
    parser.add_argument('--scan', action='store_true',
                       help='Include servo scanning tests')
    parser.add_argument('--threshold', type=float, default=None,
                       help='Obstacle detection threshold (cm)')
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("Raspberry Pi Car - Ultrasonic Sensor Test")
    print("="*60)
    print("WARNING: Ensure HC-SR04 is properly connected!")
    print("  TRIG -> GPIO4, ECHO -> GPIO17 (with level shifter)")
    print("Press Ctrl+C to stop at any time")
    print("="*60)
    
    input("\nPress Enter to start testing...")
    
    sensor = create_ultrasonic_sensor()
    servo = None
    
    if args.scan:
        print("\nInitializing servo controller for scanning...")
        servo = ServoController()
    
    try:
        # 基础测试
        test_basic_measurement(sensor, args.samples)
        test_filtered_measurement(sensor, args.samples)
        test_obstacle_detection(sensor, args.threshold)
        
        # 扫描测试（需要舵机）
        if args.scan and servo:
            test_scan_with_servo(sensor, servo)
            test_find_clear_direction(sensor, servo)
        
        print("\n" + "="*60)
        print("ALL TESTS COMPLETED SUCCESSFULLY!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Stopping...")
    finally:
        sensor.cleanup()
        if servo:
            servo.cleanup()
        print("[OK] Cleanup complete")


if __name__ == "__main__":
    main()
