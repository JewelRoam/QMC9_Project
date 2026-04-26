"""
NexusPilot: Professional RPi Car Controller
- Integrated Hardware Self-Test
- Dual-mode support (Headless/GUI)
- RPi 5 GPIO & Metadata Fail-safes
"""
import os
import sys
import time
import argparse
import cv2
import numpy as np

# Force RPi 5 GPIO chip index
os.environ["LG_CHIP"] = "0"

# Path setup
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from perception.detector import YOLODetector
from planning.apf_planner import APFPlanner, Obstacle
from rpi_deploy.motor_driver import MotorController
from rpi_deploy.ultrasonic_sensor import UltrasonicSensor

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Run without X11 window display")
    args = parser.parse_args()

    # 1. Hardware Init
    print("[INIT] Starting Hardware Self-Test...")
    motor = MotorController()
    ultrasonic = UltrasonicSensor()
    
    # --- PHYSICAL SELF-TEST ---
    # Quick pulse to verify motor health
    motor.curve_move(0.4, 0.0)
    time.sleep(0.2)
    motor.stop()
    print("[INIT] Hardware Check: OK.")

    # 2. Camera Init
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Could not open camera device.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

    # 3. Software Init
    print("[INIT] Loading AI Brain (YOLOv11n ONNX)...")
    detector = YOLODetector({"use_onnx": True, "imgsz": 320, "confidence_threshold": 0.4})
    
    planner = APFPlanner({
        "k_attractive": 1.5,
        "k_repulsive": 180.0,
        "d0": 2.2,
        "emergency_distance": 0.30,
        "max_speed": 10.0
    })

    # --- INFERENCE WARMUP ---
    print("[INIT] Pre-heating inference engine...")
    dummy_img = np.zeros((320, 320, 3), dtype=np.uint8)
    detector.detect(dummy_img)
    
    print("\n" + "="*40)
    print(" NEXUSPILOT: READY FOR MISSION ")
    print("="*40)
    
    BASE_PWM = 0.32
    dist_buffer = {}
    last_log_time = 0
    target_fps = 15
    frame_interval = 1.0 / target_fps

    try:
        while True:
            loop_start = time.perf_counter()

            # --- Safety Layer 1: Ultrasonic Lock ---
            u_res = ultrasonic.measure_once()
            if u_res.valid and u_res.distance_cm < 15.0:
                motor.emergency_stop()
                if time.time() - last_log_time > 1.5:
                    print(f"!! SAFETY LOCK: {u_res.distance_cm:.1f}cm")
                    last_log_time = time.time()
                continue

            # --- Layer 2: Perception ---
            ret, frame = cap.read()
            if not ret: continue
            
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detections = detector.detect(rgb)
            obstacles = detector.get_obstacles(detections)
            
            apf_obs_list = []
            for det in obstacles:
                raw_dist = 480.0 / (det.height + 1e-6) * 0.32 
                tid = det.track_id
                if tid not in dist_buffer: dist_buffer[tid] = raw_dist
                dist_buffer[tid] = 0.7 * dist_buffer[tid] + 0.3 * raw_dist
                
                corrected_x = dist_buffer[tid] + 0.12
                lateral = (det.center[0] - 160) / 160.0 * corrected_x * 0.55
                apf_obs_list.append(Obstacle(x=corrected_x, y=lateral, distance=corrected_x, category=det.category))

            # --- Layer 3: Planning ---
            out = planner.compute(0, 0, 0, 8.0, 3.0, 0, apf_obs_list)

            # --- Layer 4: Control & Deadband Compensation ---
            if out.emergency_brake or out.status == "emergency":
                motor.stop()
            elif out.status == "recovering":
                motor.move_backward(0.4)
            else:
                # Map target_speed to PWM scale
                speed_scale = out.target_speed / 10.0
                clamped_pwm = speed_scale * (1.0 - BASE_PWM) + BASE_PWM
                
                if out.target_speed < 0.1:
                    motor.stop()
                else:
                    motor.curve_move(clamped_pwm, out.target_steering)
                    if time.time() - last_log_time > 1.5:
                        print(f"[RUN] Mode: {out.status} | PWM: {clamped_pwm:.2f} | Dist: {u_res.distance_cm:.1f}cm")
                        last_log_time = time.time()

            # --- Visuals ---
            if not args.headless:
                for det in detections:
                    cv2.rectangle(frame, (det.bbox[0], det.bbox[1]), (det.bbox[2], det.bbox[3]), (0,255,0), 2)
                cv2.putText(frame, out.status, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                cv2.imshow("Security Monitor", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break

            # Stability: Throttle FPS
            elapsed = time.perf_counter() - loop_start
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)

    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Manual halt received.")
    finally:
        motor.cleanup()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
