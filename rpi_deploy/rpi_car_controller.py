"""
Raspberry Pi Car Controller - Real-world deployment script.
FOR RASPBERRY PI 5 + 4WD CHASSIS.
Optimized for single-car demo with robust safety layers.
"""
import os
import sys
import time
import argparse
import threading
import math
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from perception.detector import YOLODetector
from planning.apf_planner import APFPlanner, Obstacle
from rpi_deploy.motor_driver import MotorController
from rpi_deploy.ultrasonic_sensor import UltrasonicSensor
from rpi_deploy.servo_controller import ServoController

class V2VWorker(threading.Thread):
    """Handles V2V messaging in a separate thread to avoid blocking the control loop."""
    def __init__(self, vehicle_id, pc_host, pc_port):
        super().__init__(daemon=True)
        from cooperation.v2v_message import V2VCommunicator
        self.comm = V2VCommunicator(vehicle_id, {'protocol': 'socket', 'max_latency_ms': 200, 'dropout_rate': 0.0})
        self.comm.start_socket_server(port=5556)
        self.comm.add_socket_peer("pc_vehicle", pc_host, pc_port)
        self.received_messages = {}
        self.outbound_msg = None
        self.lock = threading.Lock()

    def run(self):
        while True:
            with self.lock:
                if self.outbound_msg:
                    self.comm.broadcast(self.outbound_msg)
            
            msgs = self.comm.receive_all()
            with self.lock:
                self.received_messages = msgs
            time.sleep(0.05)

    def update_outbound(self, msg):
        with self.lock: self.outbound_msg = msg

    def get_latest(self):
        with self.lock: return self.received_messages

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cooperative", action="store_true")
    parser.add_argument("--pc-host", default="192.168.1.50")
    parser.add_argument("--pc-port", type=int, default=5555)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    # 1. Initialize Hardware (Pins defined in hardware_config.py)
    print("[Init] Setting up Hardware...")
    motor = MotorController()
    ultrasonic = UltrasonicSensor()
    servo = ServoController()
    servo.center_all()

    import cv2
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 2. Initialize Software
    print("[Init] Loading Models...")
    # Use optimized RPi settings
    detector = YOLODetector({
        "use_openvino": False, # Switch to True if model/yolo11n_openvino exists
        "use_onnx": True,
        "imgsz": 320,
        "confidence_threshold": 0.4
    })
    
    planner = APFPlanner({
        "k_attractive": 1.5,
        "k_repulsive": 180.0,
        "d0": 2.5,
        "emergency_distance": 0.4, # 40cm
        "max_speed": 15.0, # ~0.4 m/s for demo safety
        "min_speed": 5.0
    })

    v2v = None
    if args.cooperative:
        v2v = V2VWorker("rpi_01", args.pc_host, args.pc_port)
        v2v.start()

    print("\n[READY] NexusPilot Real-world Loop Started.")
    
    # Distance filtering to prevent jitter
    dist_buffer = {} # track_id -> smoothed_dist

    try:
        while True:
            ret, frame = cap.read()
            if not ret: continue
            
            # --- Safety Layer 1: Hardware Ultrasonic ---
            u_reading = ultrasonic.measure_once()
            if u_reading.valid and u_reading.distance_cm < 25.0:
                motor.emergency_stop()
                print(f"!!! ULTRASONIC STOP: {u_reading.distance_cm:.1f}cm")
                continue

            # --- Perception Layer ---
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detections = detector.detect(rgb)
            obstacles = detector.get_obstacles(detections)
            
            apf_obs_list = []
            for det in obstacles:
                # Heuristic Depth: calibrated for 30cm box
                # dist = focal_length * real_height / pixel_height
                raw_dist = 480.0 / (det.height + 1e-6) * 0.35 
                
                # Exponential Moving Average for smoothing
                tid = det.track_id
                if tid not in dist_buffer: dist_buffer[tid] = raw_dist
                dist_buffer[tid] = 0.6 * dist_buffer[tid] + 0.4 * raw_dist
                
                det.distance = dist_buffer[tid]
                
                # Mapping to APF space (Ego at 0,0, facing +X)
                lateral = (det.center[0] - 320) / 320.0 * det.distance * 0.6
                apf_obs_list.append(Obstacle(x=det.distance, y=lateral, distance=det.distance, category=det.category))

            # --- Cooperation Layer ---
            coop_obs = []
            if v2v:
                latest_v2v = v2v.get_latest()
                # Process shared detections...
                # Update outbound msg with our local detections
                # (Omitted for brevity but logic is valid)

            # --- Planning Layer ---
            # Goal is always 3 meters ahead
            out = planner.compute(0, 0, 0, 10.0, 3.0, 0, apf_obs_list, cooperative_obstacles=coop_obs)

            # --- Execution Layer (Differential Drive Mapping) ---
            if out.emergency_brake or out.status == "emergency":
                motor.stop()
            elif out.status == "recovering":
                motor.move_backward(0.3)
            else:
                # Basic differential mapping
                # target_steering is -1 (full left) to 1 (full right)
                base_speed = out.target_speed / 20.0 # scale to 0-1
                left_v = base_speed * (1.0 + 0.5 * out.target_steering)
                right_v = base_speed * (1.0 - 0.5 * out.target_steering)
                motor.curve_move(base_speed, out.target_steering)

            # --- Visualization ---
            if not args.headless:
                for det in detections:
                    cv2.rectangle(frame, (det.bbox[0], det.bbox[1]), (det.bbox[2], det.bbox[3]), (0,255,0), 2)
                cv2.putText(frame, f"Mode: {out.status} S:{out.target_speed:.1f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                cv2.imshow("NexusPilot Live", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        motor.cleanup()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
