"""
NexusPilot: RPi Car Controller v2
- YOLO vision (throttled to 3 FPS) + servo-enhanced ultrasonic scanning
- State-machine based obstacle avoidance (inspired by makerobo Case 7)
- CPU-optimized for Raspberry Pi 5
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
from rpi_deploy.servo_controller import ServoController

# ---- Constants ----

# Ultrasonic thresholds (cm)
EMERGENCY_DIST = 10.0      # Hard stop
OBSTACLE_DIST = 50.0       # Trigger servo scan + avoidance
SIDE_CLEAR_DIST = 55.0     # Side distance to confirm obstacle is passed

# Speed settings (0.0-1.0 gpiozero scale)
CRUISE_SPEED = 0.2
AVOID_BACKUP_SPEED = 0.3
AVOID_CURVE_SPEED = 0.2    # Forward speed during avoidance turn
AVOID_TURN_STEER = 0.5     # Steering intensity for avoidance turn
RECOVER_SPEED = 0.2        # Forward speed during recovery
RECOVERY_BACKUP_DURATION = 0.3
AVOID_CLEAR_TIME = 0.5     # Front must stay clear this long before side-check
RECOVER_MAX_DURATION = 2.0 # Safety timeout: max seconds in RECOVER

# Servo settle time (seconds)
SERVO_SETTLE = 0.25
SERVO_SIDE_CHECK_INTERVAL = 0.4  # Check side every N seconds in AVOID/RECOVER

# Camera
CAM_WIDTH = 320
CAM_HEIGHT = 320
CAM_CENTER_X = CAM_WIDTH / 2.0

# YOLO inference throttling: run detection every N frames
YOLO_FRAME_INTERVAL = 2    # ~6 FPS at 12 Hz main loop
YOLO_CONFIDENCE_THRESHOLD = 0.3  # Lower threshold for better detection rate

# Main loop target: 12 Hz (ultrasonic-only frames are ~80 µs, YOLO frames are ~200 ms)
MAIN_LOOP_INTERVAL = 0.083  # ~12 FPS for ultrasonic safety checks

# State machine
STATE_CRUISE = "CRUISE"
STATE_BLOCKED = "BLOCKED"
STATE_AVOID = "AVOID"
STATE_RECOVER = "RECOVER"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true",
                        help="Run without X11 window display (saves ~10% CPU)")
    args = parser.parse_args()

    # ---- Hardware Init ----
    print("[INIT] Starting Hardware...")
    motor = MotorController()
    ultrasonic = UltrasonicSensor()
    servo = ServoController()
    servo.center_ultrasonic()
    time.sleep(0.5)
    print("[INIT] Hardware OK.")

    # ---- Camera Init ----
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Could not open camera device.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    # Reduce camera FPS to save CPU
    cap.set(cv2.CAP_PROP_FPS, 15)

    # ---- AI Init ----
    print("[INIT] Loading YOLO11n ONNX...")
    detector = YOLODetector({
        "use_onnx": True,
        "imgsz": 640,
        "confidence_threshold": YOLO_CONFIDENCE_THRESHOLD,
    })
    planner = APFPlanner({
        "k_attractive": 1.5,
        "k_repulsive": 180.0,
        "d0": 2.2,
        "emergency_distance": 0.30,
        "max_speed": 10.0,
    })

    # Warmup
    print("[INIT] Warming up inference...")
    detector.detect(np.zeros((320, 320, 3), dtype=np.uint8))

    print("\n" + "=" * 40)
    print(" NEXUSPILOT: READY FOR MISSION ")
    print("=" * 40)

    # ---- Runtime State ----
    state = STATE_CRUISE
    dist_buffer = {}
    last_log_time = 0
    avoid_action = None
    avoid_start_time = 0
    avoid_clear_start = 0     # When front first became clear during AVOID
    recover_start_time = 0
    side_cleared = False      # Has the avoidance-side been confirmed clear?
    last_side_check_time = 0  # For periodic side-checking in AVOID/RECOVER
    consecutive_blocked = 0

    # Cached detections (reused between YOLO frames)
    cached_detections = []
    cached_apf_obs = []
    yolo_frame_count = 0

    try:
        while True:
            loop_start = time.perf_counter()

            # ---- Safety: Emergency ultrasonic check ----
            u_res = ultrasonic.measure_once()
            if u_res.valid and u_res.distance_cm < EMERGENCY_DIST:
                motor.emergency_stop()
                state = STATE_BLOCKED
                consecutive_blocked += 1
                avoid_clear_start = 0
                side_cleared = False
                if time.time() - last_log_time > 1.0:
                    print(f"!! EMERGENCY STOP: {u_res.distance_cm:.1f}cm")
                    last_log_time = time.time()
                # Flush stale camera frame
                cap.read()
                # Sleep briefly to avoid tight spin loop
                time.sleep(0.05)
                continue

            # ---- Read camera (only on YOLO frames or GUI frames) ----
            frame = None
            run_yolo = (yolo_frame_count % YOLO_FRAME_INTERVAL == 0) and state not in (STATE_AVOID, STATE_BLOCKED)

            if run_yolo or not args.headless:
                ret, frame = cap.read()
                if not ret:
                    frame = None

            if run_yolo and frame is not None:
                # ---- YOLO Perception (throttled to ~3 FPS) ----
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                cached_detections = detector.detect(rgb)
                obstacles = detector.get_obstacles(cached_detections)

                # Rebuild APF obstacle list (cached until next YOLO frame)
                cached_apf_obs = []
                for det in obstacles:
                    raw_dist = 480.0 / (det.height + 1e-6) * 0.32
                    tid = det.track_id
                    if tid not in dist_buffer:
                        dist_buffer[tid] = raw_dist
                    dist_buffer[tid] = 0.7 * dist_buffer[tid] + 0.3 * raw_dist
                    corrected_x = dist_buffer[tid] + 0.12
                    lateral = (det.center[0] - CAM_CENTER_X) / CAM_CENTER_X * corrected_x * 0.55
                    cached_apf_obs.append(Obstacle(
                        x=corrected_x, y=lateral,
                        distance=corrected_x, category=det.category
                    ))

            yolo_frame_count += 1

            # Prune stale track IDs from dist_buffer (tracker already cleaned them)
            if yolo_frame_count % 30 == 0:
                active_ids = {det.track_id for det in cached_detections if det.track_id > 0}
                dist_buffer = {tid: v for tid, v in dist_buffer.items() if tid in active_ids}

            # ---- State Machine ----
            front_dist = u_res.distance_cm if u_res.valid else 999.0

            if state == STATE_CRUISE:
                if front_dist < OBSTACLE_DIST:
                    motor.stop()
                    state = STATE_BLOCKED
                    consecutive_blocked += 1
                    avoid_clear_start = 0
                    side_cleared = False
                    if time.time() - last_log_time > 1.0:
                        print(f"[BLOCKED] Front: {front_dist:.1f}cm")
                        last_log_time = time.time()

                else:
                    # Clear -> forward with APF steering + screen-side bias
                    out = planner.compute(0, 0, 0, 8.0, 3.0, 0, cached_apf_obs)
                    steer = out.target_steering * 0.4

                    if abs(steer) < 0.05 and cached_apf_obs:
                        closest = min(cached_apf_obs, key=lambda o: o.distance)
                        if closest.y > 0.05:
                            steer = -0.3  # obstacle on right -> steer left
                        elif closest.y < -0.05:
                            steer = 0.3   # obstacle on left -> steer right

                    if not out.emergency_brake and out.target_speed > 0.1:
                        motor.curve_move(CRUISE_SPEED, steer)
                    else:
                        motor.stop()

                    if time.time() - last_log_time > 3.0:
                        print(f"[CRUISE] Front: {front_dist:.1f}cm | Det: {len(cached_apf_obs)} | steer={steer:.2f}")
                        last_log_time = time.time()

            elif state == STATE_BLOCKED:
                # Step 1: Back up to create clearance
                motor.move_backward(AVOID_BACKUP_SPEED)
                time.sleep(RECOVERY_BACKUP_DURATION)
                motor.stop()
                time.sleep(0.2)

                # Step 2: Scan left and right with ultrasonic servo
                dis_left = _scan_left(ultrasonic, servo)
                dis_right = _scan_right(ultrasonic, servo)

                # Re-center servo
                servo.center_ultrasonic()
                time.sleep(SERVO_SETTLE)

                # Step 3: Decide direction
                if dis_left < OBSTACLE_DIST and dis_right < OBSTACLE_DIST:
                    # Both sides blocked -> spin out as last resort
                    side_cleared = False
                    if consecutive_blocked >= 3:
                        print("[BLOCKED] Both blocked x3, spinning out")
                        motor.rotate_left(AVOID_CURVE_SPEED)
                        time.sleep(1.5)
                        motor.stop()
                        consecutive_blocked = 0
                        state = STATE_CRUISE
                    else:
                        print(f"[BLOCKED] Both blocked L={dis_left:.0f} R={dis_right:.0f}, spin")
                        motor.rotate_left(AVOID_CURVE_SPEED)
                        time.sleep(0.8)
                        motor.stop()
                        state = STATE_CRUISE
                elif dis_left > dis_right:
                    # More room on the left -> forward-left curve
                    avoid_action = "left"
                    print(f"[AVOID] Curve LEFT (L={dis_left:.0f} > R={dis_right:.0f})")
                    state = STATE_AVOID
                    avoid_start_time = time.time()
                    avoid_clear_start = 0
                    side_cleared = False
                else:
                    # More room on the right -> forward-right curve
                    avoid_action = "right"
                    print(f"[AVOID] Curve RIGHT (R={dis_right:.0f} > L={dis_left:.0f})")
                    state = STATE_AVOID
                    avoid_start_time = time.time()
                    avoid_clear_start = 0
                    side_cleared = False

            elif state == STATE_AVOID:
                # Forward curve in avoidance direction to gain lateral displacement
                if avoid_action == "left":
                    motor.curve_move(AVOID_CURVE_SPEED, -AVOID_TURN_STEER)
                else:
                    motor.curve_move(AVOID_CURVE_SPEED, AVOID_TURN_STEER)

                front_now = ultrasonic.measure_once()
                if front_now.valid and front_now.distance_cm > OBSTACLE_DIST:
                    if avoid_clear_start == 0:
                        avoid_clear_start = time.time()
                    elif time.time() - avoid_clear_start >= AVOID_CLEAR_TIME:
                        # Front clear — check side; if clear too, go straight to CRUISE
                        side_dist = _check_side_distance(ultrasonic, servo, avoid_action)
                        if side_dist > SIDE_CLEAR_DIST:
                            side_cleared = True
                            servo.center_ultrasonic()
                            motor.stop()
                            consecutive_blocked = max(0, consecutive_blocked - 1)
                            state = STATE_CRUISE
                            avoid_elapsed = time.time() - avoid_start_time
                            print(f"[CRUISE] Obstacle cleared in {avoid_elapsed:.1f}s (side={side_dist:.0f}cm)")
                        else:
                            # Front clear but side still blocked — enter RECOVER
                            servo.center_ultrasonic()
                            motor.stop()
                            state = STATE_RECOVER
                            recover_start_time = time.time()
                            avoid_elapsed = time.time() - avoid_start_time
                            print(f"[RECOVER] Front clear, side={side_dist:.0f}cm — continuing")
                else:
                    avoid_clear_start = 0

                # Safety timeout
                if time.time() - avoid_start_time >= 6.0:
                    motor.stop()
                    servo.center_ultrasonic()
                    side_cleared = False
                    print("[AVOID] Timeout, forcing CRUISE")
                    consecutive_blocked = 0
                    state = STATE_CRUISE

            elif state == STATE_RECOVER:
                # Drive forward with APF steering; periodically check side clearance
                out = planner.compute(0, 0, 0, 8.0, 3.0, 0, cached_apf_obs)
                steer = out.target_steering * 0.4
                if abs(steer) < 0.05 and cached_apf_obs:
                    closest = min(cached_apf_obs, key=lambda o: o.distance)
                    if closest.y > 0.05:
                        steer = -0.3
                    elif closest.y < -0.05:
                        steer = 0.3
                if not out.emergency_brake and out.target_speed > 0.1:
                    motor.curve_move(RECOVER_SPEED, steer)
                else:
                    motor.curve_move(RECOVER_SPEED, 0.0)

                # Periodic side check
                now = time.time()
                if now - last_side_check_time >= SERVO_SIDE_CHECK_INTERVAL:
                    last_side_check_time = now
                    side_dist = _check_side_distance(ultrasonic, servo, avoid_action)
                    if side_dist > SIDE_CLEAR_DIST:
                        side_cleared = True
                    servo.center_ultrasonic()

                elapsed = time.time() - recover_start_time
                if side_cleared or elapsed >= RECOVER_MAX_DURATION:
                    consecutive_blocked = max(0, consecutive_blocked - 1)
                    state = STATE_CRUISE
                    front_check = ultrasonic.measure_once()
                    dist_str = f"{front_check.distance_cm:.1f}cm" if front_check.valid else "---"
                    reason = "side clear" if side_cleared else "timeout"
                    print(f"[CRUISE] Resumed ({reason}, front={dist_str})")
                else:
                    front_check = ultrasonic.measure_once()
                    if front_check.valid and front_check.distance_cm < OBSTACLE_DIST:
                        motor.stop()
                        side_cleared = False
                        state = STATE_BLOCKED
                        consecutive_blocked += 1
            # ---- Visual Overlay (only on frames we already read) ----
            if not args.headless and frame is not None:
                _draw_overlay(frame, cached_detections, state, u_res)
                cv2.imshow("NexusPilot Monitor", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # FPS throttle
            elapsed = time.perf_counter() - loop_start
            if elapsed < MAIN_LOOP_INTERVAL:
                time.sleep(MAIN_LOOP_INTERVAL - elapsed)

    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Manual halt received.")
    finally:
        motor.cleanup()
        servo.cleanup()
        cap.release()
        cv2.destroyAllWindows()


# ---- Servo Scan Helpers ----

def _scan_left(sensor: UltrasonicSensor, servo: ServoController) -> float:
    """Scan left (~155 degrees) and return distance in cm."""
    servo.set_ultrasonic_angle(155)
    time.sleep(SERVO_SETTLE)
    reading = sensor.measure_once()
    return reading.distance_cm if reading.valid else 999.0


def _scan_right(sensor: UltrasonicSensor, servo: ServoController) -> float:
    """Scan right (~85 degrees) and return distance in cm."""
    servo.set_ultrasonic_angle(85)
    time.sleep(SERVO_SETTLE)
    reading = sensor.measure_once()
    return reading.distance_cm if reading.valid else 999.0


def _check_side_distance(sensor: UltrasonicSensor, servo: ServoController,
                         avoid_action: str) -> float:
    """Check distance on the avoidance side to verify the obstacle is passed."""
    # Avoidance side: if we curved left, obstacle is on right; check right side
    side = "right" if avoid_action == "left" else "left"
    if side == "left":
        return _scan_left(sensor, servo)
    return _scan_right(sensor, servo)

# ---- Overlay Drawing ----

def _draw_overlay(frame, detections, state, u_res):
    """Draw detection boxes and status on frame."""
    for det in detections:
        cv2.rectangle(frame,
                      (det.bbox[0], det.bbox[1]),
                      (det.bbox[2], det.bbox[3]),
                      (0, 255, 0), 2)
        label = f"{det.category} {det.confidence:.1f}"
        cv2.putText(frame, label, (det.bbox[0], det.bbox[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    color = {
        STATE_CRUISE: (0, 255, 0),
        STATE_BLOCKED: (0, 0, 255),
        STATE_AVOID: (0, 165, 255),
        STATE_RECOVER: (255, 255, 0),
    }.get(state, (255, 255, 255))
    dist_str = f"{u_res.distance_cm:.0f}cm" if u_res.valid else "---"
    cv2.putText(frame, f"{state} | {dist_str}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


if __name__ == "__main__":
    main()
