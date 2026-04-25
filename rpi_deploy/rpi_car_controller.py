"""
Defensive RPi Car Controller - Deployment Grade.
Fixes for: Motor Deadband, Camera Offsets, and Loop Jitter.
"""
import os
import sys
import time
import argparse
import threading
import cv2
import numpy as np

# Path setup
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from perception.detector import YOLODetector
from planning.apf_planner import APFPlanner, Obstacle
from rpi_deploy.motor_driver import MotorController
from rpi_deploy.ultrasonic_sensor import UltrasonicSensor

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pc-host", default="192.168.1.50")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    # --- Constants & Calibration ---
    BASE_PWM = 0.30           # Minimum power to overcome friction
    CAMERA_OFFSET_X = 0.12    # Camera is 12cm ahead of rotation center
    MAX_FRAME_DELAY = 0.4     # 400ms lag trigger safety stop
    
    # 1. Hardware Init
    motor = MotorController()
    ultrasonic = UltrasonicSensor()
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 2. Software Init
    detector = YOLODetector({"use_onnx": True, "imgsz": 320, "confidence_threshold": 0.4})
    planner = APFPlanner({
        "k_attractive": 1.2,
        "k_repulsive": 160.0,
        "d0": 2.2,
        "emergency_distance": 0.35, # 35cm
        "max_speed": 12.0
    })

    print("\n[SECURITY] Watchdog Active. System Ready.")
    dist_buffer = {}

    try:
        last_loop_time = time.perf_counter()
        while True:
            loop_start = time.perf_counter()
            dt = loop_start - last_loop_time
            last_loop_time = loop_start

            # --- Safety Layer 0: Frame Delay Watchdog ---
            if dt > MAX_FRAME_DELAY:
                print(f"[WATCHDOG] Critical Lag Detected ({dt*1000:.0f}ms). Stopping.")
                motor.stop()
                # Continue to next frame to recover

            ret, frame = cap.read()
            if not ret: continue
            
            # --- Safety Layer 1: Hardware Ultrasonic (Direct Priority) ---
            u_reading = ultrasonic.measure_once()
            if u_reading.valid and u_reading.distance_cm < 20.0:
                motor.emergency_stop()
                print(f"!!! HARD STOP: {u_reading.distance_cm:.1f}cm")
                continue

            # --- Perception ---
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detections = detector.detect(rgb)
            obstacles = detector.get_obstacles(detections)
            
            apf_obs_list = []
            for det in obstacles:
                # Calibrated for standard 30cm box
                raw_dist = 480.0 / (det.height + 1e-6) * 0.32 
                
                # EMA Smoothing
                tid = det.track_id
                if tid not in dist_buffer: dist_buffer[tid] = raw_dist
                dist_buffer[tid] = 0.7 * dist_buffer[tid] + 0.3 * raw_dist
                
                # Coordinate Correction: Move origin from camera to axle
                # X is forward, Y is lateral
                corrected_x = dist_buffer[tid] + CAMERA_OFFSET_X
                lateral = (det.center[0] - 320) / 320.0 * corrected_x * 0.55
                
                apf_obs_list.append(Obstacle(x=corrected_x, y=lateral, distance=corrected_x, category=det.category))

            # --- Planning ---
            # Goal is 2.5m ahead relative to axle
            out = planner.compute(0, 0, 0, 8.0, 2.5, 0, apf_obs_list)

            # --- Execution with Deadband Compensation ---
            if out.emergency_brake or out.status == "emergency":
                motor.stop()
            elif out.status == "recovering":
                motor.move_backward(0.4)
            else:
                # Map target_speed (0-12) to PWM (0-1)
                # Ensure it never drops below BASE_PWM unless stopping
                speed_req = out.target_speed / 12.0
                clamped_speed = speed_req * (1.0 - BASE_PWM) + BASE_PWM
                
                if out.target_speed < 0.1: motor.stop()
                else: motor.curve_move(clamped_speed, out.target_steering)

            if not args.headless:
                cv2.imshow("Security Monitor", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break

    except KeyboardInterrupt:
        pass
    finally:
        motor.cleanup()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
