#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头测试脚本
Camera Test Script

测试内容：
1. 摄像头基本捕获
2. 分辨率设置
3. 视频流预览
4. 图像保存

使用方法：
    python test_camera.py [--duration SECONDS] [--save]
"""

import sys
import os
import time
import argparse

# Dynamically resolve project root (works on any machine)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

try:
    from rpi_deploy.camera_driver import CameraDriver, create_camera_driver
    from rpi_deploy.hardware_config import hardware_config
except ImportError as e:
    print(f"[ERROR] Import failed: {e}")
    sys.exit(1)


def test_basic_capture(camera: CameraDriver, duration: float = 5.0):
    """测试基本捕获功能"""
    print("\n" + "="*50)
    print("TEST 1: Basic Capture Test")
    print("="*50)
    
    if not camera.open():
        print("[ERROR] Failed to open camera!")
        return False
    
    print(f"\nCapturing frames for {duration} seconds...")
    
    start_time = time.time()
    frame_count = 0
    
    while time.time() - start_time < duration:
        frame = camera.get_frame()
        if frame is not None:
            frame_count += 1
            if frame_count % 30 == 0:
                print(f"  Captured {frame_count} frames...")
        time.sleep(0.033)  # ~30fps
    
    elapsed = time.time() - start_time
    actual_fps = frame_count / elapsed if elapsed > 0 else 0
    
    print(f"\n[OK] Captured {frame_count} frames in {elapsed:.1f}s")
    print(f"     Actual FPS: {actual_fps:.1f}")
    
    return True


def test_resolution(camera: CameraDriver):
    """测试不同分辨率"""
    print("\n" + "="*50)
    print("TEST 2: Resolution Test")
    print("="*50)
    
    resolutions = [
        (640, 480),
        (800, 600),
        (1280, 720),
    ]
    
    for width, height in resolutions:
        print(f"\nTesting resolution: {width}x{height}")
        
        camera.close()
        camera.config.width = width
        camera.config.height = height
        
        if camera.open():
            # 读取几帧测试
            for _ in range(5):
                frame = camera.get_frame()
                if frame is not None:
                    actual_h, actual_w = frame.shape[:2]
                    print(f"  Actual resolution: {actual_w}x{actual_h}")
                    break
                time.sleep(0.1)
        else:
            print(f"  [WARNING] Failed to set resolution {width}x{height}")
    
    print("\n[OK] Resolution test complete")


def test_preview_window(camera: CameraDriver, duration: float = 10.0):
    """测试预览窗口（需要显示器）"""
    print("\n" + "="*50)
    print("TEST 3: Preview Window Test")
    print("="*50)
    
    try:
        import cv2
    except ImportError:
        print("[ERROR] OpenCV not available for preview")
        return
    
    print(f"\nOpening preview window for {duration} seconds...")
    print("Press 'q' to exit early, 's' to save snapshot")
    
    if not camera.open():
        print("[ERROR] Failed to open camera!")
        return
    
    camera.start_capture()
    
    start_time = time.time()
    frame_count = 0
    snapshot_count = 0
    
    try:
        while time.time() - start_time < duration:
            frame = camera.get_current_frame()
            if frame is not None:
                frame_count += 1
                
                # 添加信息文本
                fps = frame_count / (time.time() - start_time)
                info_text = f"FPS: {fps:.1f} | Frame: {frame_count} | Press 'q' to quit, 's' to save"
                cv2.putText(frame, info_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                cv2.imshow('Camera Test', frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\nUser quit preview")
                    break
                elif key == ord('s'):
                    snapshot_count += 1
                    filename = f"/home/pi/snapshot_{snapshot_count:03d}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"  Snapshot saved: {filename}")
            else:
                time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nPreview interrupted")
    finally:
        cv2.destroyAllWindows()
        camera.stop_capture()
    
    elapsed = time.time() - start_time
    actual_fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"\n[OK] Preview complete: {frame_count} frames, {actual_fps:.1f} FPS")


def test_image_save(camera: CameraDriver):
    """测试图像保存"""
    print("\n" + "="*50)
    print("TEST 4: Image Save Test")
    print("="*50)
    
    if not camera.open():
        print("[ERROR] Failed to open camera!")
        return
    
    print("\nCapturing and saving images...")
    
    for i in range(3):
        filename = f"/home/pi/test_image_{i+1:02d}.jpg"
        frame = camera.capture_image(filename)
        if frame is not None:
            print(f"  Saved: {filename} ({frame.shape[1]}x{frame.shape[0]})")
        time.sleep(0.5)
    
    print("\n[OK] Image save test complete")
    print("     Images saved to /home/pi/")


def test_status_info(camera: CameraDriver):
    """测试状态信息获取"""
    print("\n" + "="*50)
    print("TEST 5: Status Information")
    print("="*50)
    
    if not camera.open():
        print("[ERROR] Failed to open camera!")
        return
    
    status = camera.get_status()
    
    print("\nCamera Status:")
    for key, value in status.items():
        print(f"  {key}: {value}")
    
    print("\n[OK] Status info test complete")


def main():
    parser = argparse.ArgumentParser(description='Camera Test Script')
    parser.add_argument('--duration', type=float, default=5.0,
                       help='Test duration in seconds')
    parser.add_argument('--preview', action='store_true',
                       help='Show preview window (requires display)')
    parser.add_argument('--save', action='store_true',
                       help='Save test images')
    parser.add_argument('--device', type=int, default=0,
                       help='Camera device ID (default: 0)')
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("Raspberry Pi Car - Camera Test")
    print("="*60)
    print(f"Device ID: {args.device}")
    print("Press Ctrl+C to stop at any time")
    print("="*60)
    
    input("\nPress Enter to start testing...")
    
    camera = create_camera_driver(args.device)
    
    try:
        # 基础捕获测试
        if not test_basic_capture(camera, args.duration):
            print("\n[ERROR] Basic capture test failed!")
            return
        
        # 分辨率测试
        test_resolution(camera)
        
        # 预览窗口测试
        if args.preview:
            test_preview_window(camera, args.duration)
        
        # 图像保存测试
        if args.save:
            test_image_save(camera)
        
        # 状态信息
        test_status_info(camera)
        
        print("\n" + "="*60)
        print("ALL TESTS COMPLETED SUCCESSFULLY!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Stopping...")
    finally:
        camera.close()
        print("[OK] Cleanup complete")


if __name__ == "__main__":
    main()
