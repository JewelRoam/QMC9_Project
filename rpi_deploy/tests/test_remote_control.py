#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
远程控制测试脚本
Remote Control Test Script

测试内容：
1. 服务器启动和客户端连接
2. 基本运动命令（前进、后退、转向、停止）
3. 舵机控制命令
4. 状态查询
5. 视频流服务

使用方法：
    # 在树莓派上启动服务器
    python test_remote_control.py --server
    
    # 在PC上运行客户端测试
    python test_remote_control.py --client --host 192.168.137.33
"""

import sys
import time
import argparse
import threading

sys.path.insert(0, '/home/pi/QMC9_Project')

try:
    from rpi_deploy.remote_control import (
        RemoteControlServer, RemoteControlClient,
        create_remote_server, create_remote_client,
        VehicleStatus
    )
    from rpi_deploy.motor_driver import MotorController
    from rpi_deploy.servo_controller import ServoController
    from rpi_deploy.ultrasonic_sensor import UltrasonicSensor
    from rpi_deploy.camera_driver import CameraDriver, VideoStreamServer
    from rpi_deploy.hardware_config import hardware_config
except ImportError as e:
    print(f"[ERROR] Import failed: {e}")
    sys.exit(1)


# 全局硬件实例（用于服务器端）
motor_controller = None
servo_controller = None
ultrasonic_sensor = None
camera_driver = None
video_server = None


def init_hardware():
    """初始化硬件"""
    global motor_controller, servo_controller, ultrasonic_sensor, camera_driver
    
    print("[INFO] Initializing hardware...")
    
    try:
        motor_controller = MotorController()
        print("  ✓ Motor controller initialized")
    except Exception as e:
        print(f"  ✗ Motor controller failed: {e}")
    
    try:
        servo_controller = ServoController()
        print("  ✓ Servo controller initialized")
    except Exception as e:
        print(f"  ✗ Servo controller failed: {e}")
    
    try:
        ultrasonic_sensor = UltrasonicSensor()
        print("  ✓ Ultrasonic sensor initialized")
    except Exception as e:
        print(f"  ✗ Ultrasonic sensor failed: {e}")
    
    try:
        camera_driver = CameraDriver()
        if camera_driver.open():
            camera_driver.start_capture()
            print("  ✓ Camera initialized")
        else:
            print("  ✗ Camera failed to open")
            camera_driver = None
    except Exception as e:
        print(f"  ✗ Camera failed: {e}")
        camera_driver = None


def cleanup_hardware():
    """清理硬件资源"""
    global motor_controller, servo_controller, ultrasonic_sensor, camera_driver, video_server
    
    print("\n[INFO] Cleaning up hardware...")
    
    if video_server:
        video_server.stop()
    
    if motor_controller:
        motor_controller.cleanup()
    
    if servo_controller:
        servo_controller.cleanup()
    
    if ultrasonic_sensor:
        ultrasonic_sensor.cleanup()
    
    if camera_driver:
        camera_driver.close()
    
    print("[OK] Cleanup complete")


def get_vehicle_status() -> VehicleStatus:
    """获取车辆状态"""
    status = VehicleStatus()
    status.timestamp = time.time()
    
    if motor_controller:
        motor_status = motor_controller.get_status()
        status.motor_speed = motor_status['speed']
        status.direction = motor_status['direction']
    
    if servo_controller:
        servo_status = servo_controller.get_status()
        status.ultrasonic_angle = servo_status['ultrasonic_angle']
        status.camera_pan = servo_status['camera_pan']
        status.camera_tilt = servo_status['camera_tilt']
    
    if ultrasonic_sensor:
        reading = ultrasonic_sensor.measure_once()
        if reading.valid:
            status.obstacle_distance = reading.distance_cm
    
    return status


def handle_move(params):
    """处理移动命令"""
    direction = params.get('direction', 'stop')
    speed = params.get('speed', 50.0)
    
    if not motor_controller:
        return {"error": "Motor controller not available"}
    
    print(f"[CMD] Move: {direction} at {speed}%")
    
    if direction == 'forward':
        motor_controller.move_forward(speed)
    elif direction == 'backward':
        motor_controller.move_backward(speed)
    elif direction == 'left':
        motor_controller.turn_left(speed)
    elif direction == 'right':
        motor_controller.turn_right(speed)
    else:
        motor_controller.stop()
    
    return {"status": f"Moving {direction}"}


def handle_stop(params):
    """处理停止命令"""
    if motor_controller:
        motor_controller.stop()
        print("[CMD] Stop")
        return {"status": "Stopped"}
    return {"error": "Motor controller not available"}


def handle_servo(params):
    """处理舵机命令"""
    if not servo_controller:
        return {"error": "Servo controller not available"}
    
    servo_type = params.get('type', '')
    angle = params.get('angle', 90)
    
    print(f"[CMD] Servo: {servo_type} to {angle}°")
    
    if servo_type == 'ultrasonic':
        servo_controller.set_ultrasonic_angle(angle)
    elif servo_type == 'pan':
        servo_controller.set_camera_pan(angle)
    elif servo_type == 'tilt':
        servo_controller.set_camera_tilt(angle)
    elif servo_type == 'center':
        servo_controller.center_all()
    else:
        return {"error": f"Unknown servo type: {servo_type}"}
    
    return {"status": f"Servo {servo_type} set to {angle}°"}


def handle_scan(params):
    """处理扫描命令"""
    if not ultrasonic_sensor or not servo_controller:
        return {"error": "Required hardware not available"}
    
    print("[CMD] Scanning with ultrasonic...")
    readings = ultrasonic_sensor.scan_with_servo(servo_controller)
    
    results = []
    for r in readings:
        results.append({
            'angle': r.angle,
            'distance': r.distance_cm,
            'valid': r.valid
        })
    
    return {"scan_results": results}


def run_server():
    """运行远程控制服务器"""
    print("\n" + "="*60)
    print("Raspberry Pi Car - Remote Control Server")
    print("="*60)
    
    # 初始化硬件
    init_hardware()
    
    # 创建服务器
    server = create_remote_server()
    
    # 注册命令处理器
    server.register_handler('move', handle_move)
    server.register_handler('stop', handle_stop)
    server.register_handler('servo', handle_servo)
    server.register_handler('scan', handle_scan)
    server.register_status_provider(get_vehicle_status)
    
    # 启动服务器
    server.start()
    
    # 启动视频流服务（如果摄像头可用）
    global video_server
    if camera_driver:
        video_server = VideoStreamServer(camera_driver)
        video_server.start()
    
    print("\n" + "="*60)
    print("Server is running!")
    print(f"Control port: {hardware_config.network.control_port}")
    if video_server:
        print(f"Video stream: http://<pi-ip>:{hardware_config.network.video_port}")
    print("Press Ctrl+C to stop")
    print("="*60)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nStopping server...")
    finally:
        server.stop()
        cleanup_hardware()


def test_client_connection(client: RemoteControlClient):
    """测试客户端连接"""
    print("\n" + "="*50)
    print("TEST 1: Connection Test")
    print("="*50)
    
    print("\nTesting heartbeat...")
    if client.heartbeat():
        print("  ✓ Heartbeat OK")
    else:
        print("  ✗ Heartbeat failed")
        return False
    
    print("\n[OK] Connection test passed")
    return True


def test_movement_commands(client: RemoteControlClient):
    """测试运动命令"""
    print("\n" + "="*50)
    print("TEST 2: Movement Commands")
    print("="*50)
    
    commands = [
        ('forward', 30),
        ('stop', 0),
        ('backward', 30),
        ('stop', 0),
        ('left', 40),
        ('stop', 0),
        ('right', 40),
        ('stop', 0),
    ]
    
    for direction, speed in commands:
        print(f"\nSending: {direction} (speed={speed})")
        response = client.send_command('move', {
            'direction': direction,
            'speed': speed
        })
        print(f"  Response: {response}")
        time.sleep(1.5)
    
    print("\n[OK] Movement commands test complete")


def test_servo_commands(client: RemoteControlClient):
    """测试舵机命令"""
    print("\n" + "="*50)
    print("TEST 3: Servo Commands")
    print("="*50)
    
    # 超声波舵机
    for angle in [90, 45, 135, 90]:
        print(f"\nSetting ultrasonic servo to {angle}°")
        response = client.set_servo('ultrasonic', angle)
        print(f"  Response: {response}")
        time.sleep(1.0)
    
    # 摄像头舵机
    positions = [(90, 45), (45, 30), (135, 60), (90, 45)]
    for pan, tilt in positions:
        print(f"\nSetting camera to pan={pan}°, tilt={tilt}°")
        client.set_servo('pan', pan)
        client.set_servo('tilt', tilt)
        time.sleep(1.0)
    
    print("\n[OK] Servo commands test complete")


def test_status_query(client: RemoteControlClient):
    """测试状态查询"""
    print("\n" + "="*50)
    print("TEST 4: Status Query")
    print("="*50)
    
    for i in range(3):
        print(f"\nQuery {i+1}:")
        status = client.get_status()
        if status:
            print(f"  Motor speed: {status.motor_speed:.1f}%")
            print(f"  Direction: {status.direction}")
            print(f"  Obstacle distance: {status.obstacle_distance:.1f}cm")
            print(f"  Ultrasonic angle: {status.ultrasonic_angle:.1f}°")
        else:
            print("  Failed to get status")
        time.sleep(1.0)
    
    print("\n[OK] Status query test complete")


def run_client(host: str):
    """运行客户端测试"""
    print("\n" + "="*60)
    print("Raspberry Pi Car - Remote Control Client Test")
    print("="*60)
    print(f"Connecting to: {host}:{hardware_config.network.control_port}")
    print("="*60)
    
    client = create_remote_client(host)
    
    if not client.connect():
        print("[ERROR] Failed to connect to server!")
        return
    
    try:
        # 连接测试
        if not test_client_connection(client):
            return
        
        input("\nPress Enter to start movement tests (ensure car is safe)...")
        
        # 运动命令测试
        test_movement_commands(client)
        
        # 舵机命令测试
        test_servo_commands(client)
        
        # 状态查询测试
        test_status_query(client)
        
        print("\n" + "="*60)
        print("ALL CLIENT TESTS COMPLETED SUCCESSFULLY!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        # 确保停止
        client.stop()
        client.disconnect()
        print("[OK] Disconnected")


def main():
    parser = argparse.ArgumentParser(description='Remote Control Test Script')
    parser.add_argument('--server', action='store_true',
                       help='Run as server (on Raspberry Pi)')
    parser.add_argument('--client', action='store_true',
                       help='Run as client (on PC)')
    parser.add_argument('--host', type=str, default='192.168.137.33',
                       help='Server IP address (for client mode)')
    args = parser.parse_args()
    
    if args.server:
        run_server()
    elif args.client:
        run_client(args.host)
    else:
        print("Please specify --server or --client mode")
        print("\nUsage examples:")
        print("  On Raspberry Pi: python test_remote_control.py --server")
        print("  On PC:           python test_remote_control.py --client --host 192.168.137.33")


if __name__ == "__main__":
    main()
