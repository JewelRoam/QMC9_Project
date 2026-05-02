"""
NexusPilot: RPi Car Controller v3
- Default: YOLO+APF only (no ultrasonic required)
- --enable-ultrasonic: full state machine with ultrasonic scanning and side-verify
- CPU-optimized for Raspberry Pi 5
"""
import os
import sys
import time
import argparse
import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Optional

# Force RPi 5 GPIO chip index
os.environ["LG_CHIP"] = "0"

# Path setup
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from perception.detector import YOLODetector
from planning.apf_planner import APFPlanner, Obstacle
from rpi_deploy.motor_driver import MotorController

# ---- Constants ----

# Ultrasonic thresholds (cm)
EMERGENCY_DIST = 10.0      # Hard stop
OBSTACLE_DIST = 50.0       # Trigger servo scan + avoidance
SIDE_CLEAR_DIST = 55.0     # Side distance to confirm obstacle is passed

# YOLO-only distance thresholds (meters, estimated from bbox height)
YOLO_EMERGENCY_DIST = 0.35   # Hard stop + reverse
YOLO_AVOID_DIST = 0.80       # Strong avoidance steering
YOLO_CAUTION_DIST = 1.50     # Moderate avoidance (boost APF repulsion)

# Speed settings (0.0-1.0 gpiozero scale)
CRUISE_SPEED = 0.2
AVOID_BACKUP_SPEED = 0.3
AVOID_CURVE_SPEED = 0.2
AVOID_TURN_STEER = 0.5
RECOVER_SPEED = 0.2
RECOVERY_BACKUP_DURATION = 0.3
AVOID_CLEAR_TIME = 0.5
RECOVER_MAX_DURATION = 2.0

# Servo settle time (seconds)
SERVO_SETTLE = 0.25
SERVO_SIDE_CHECK_INTERVAL = 0.4

# Camera
CAM_WIDTH = 320
CAM_HEIGHT = 320
CAM_CENTER_X = CAM_WIDTH / 2.0
FRAME_AREA = CAM_WIDTH * CAM_HEIGHT

# YOLO inference throttling
YOLO_FRAME_INTERVAL = 2
YOLO_CONFIDENCE_THRESHOLD = 0.3

# Main loop target: 12 Hz
MAIN_LOOP_INTERVAL = 0.083

# State machine
STATE_CRUISE = "CRUISE"
STATE_BLOCKED = "BLOCKED"
STATE_AVOID = "AVOID"
STATE_RECOVER = "RECOVER"


@dataclass
class RuntimeState:
    """Shared mutable state for the main loop."""
    state: str = STATE_CRUISE
    dist_buffer: dict = field(default_factory=dict)
    last_log_time: float = 0.0
    avoid_action: Optional[str] = None
    avoid_start_time: float = 0.0
    avoid_clear_start: float = 0.0
    recover_start_time: float = 0.0
    side_cleared: bool = False
    last_side_check_time: float = 0.0
    consecutive_blocked: int = 0
    cached_detections: list = field(default_factory=list)
    cached_apf_obs: list = field(default_factory=list)
    yolo_frame_count: int = 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true",
                        help="Run without X11 window display (saves ~10% CPU)")
    parser.add_argument("--enable-ultrasonic", action="store_true",
                        help="Enable ultrasonic sensor for obstacle detection and servo scanning")
    args = parser.parse_args()

    # ---- Hardware Init ----
    print("[INIT] Starting hardware...")
    motor = MotorController()

    ultrasonic = None
    servo = None
    if args.enable_ultrasonic:
        from rpi_deploy.ultrasonic_sensor import UltrasonicSensor
        from rpi_deploy.servo_controller import ServoController
        ultrasonic = UltrasonicSensor()
        servo = ServoController()
        servo.center_ultrasonic()
        time.sleep(0.5)
        print("[INIT] Ultrasonic + Servo enabled.")
    else:
        print("[INIT] YOLO-only mode (no ultrasonic).")

    # ---- Camera Init ----
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Could not open camera device.")
        motor.cleanup()
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 15)

    # ---- AI Init ----
    print("[INIT] Loading YOLO11n ONNX...")
    try:
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
    except Exception as e:
        print(f"[ERROR] AI init failed: {e}")
        motor.cleanup()
        cap.release()
        return

    print("\n" + "=" * 40)
    mode_str = "ULTRASONIC" if args.enable_ultrasonic else "YOLO-ONLY"
    print(f" NEXUSPILOT: READY ({mode_str}) ")
    print("=" * 40)

    rt = RuntimeState()

    # Pre-computed: states where YOLO is skipped (ultrasonic mode only)
    skip_yolo_states = (STATE_AVOID, STATE_BLOCKED) if args.enable_ultrasonic else ()

    try:
        while True:
            loop_start = time.perf_counter()

            # ---- Always drain camera buffer to avoid stale frames ----
            ret, frame = cap.read()
            if not ret:
                frame = None

            # ---- Ultrasonic emergency check ----
            u_res = None
            front_dist = 999.0
            if ultrasonic is not None:
                u_res = ultrasonic.measure_once()
                front_dist = u_res.distance_cm if u_res.valid else 999.0

                if u_res.valid and u_res.distance_cm < EMERGENCY_DIST:
                    motor.emergency_stop()
                    rt.state = STATE_BLOCKED
                    rt.consecutive_blocked += 1
                    rt.avoid_clear_start = 0
                    rt.side_cleared = False
                    if time.time() - rt.last_log_time > 1.0:
                        print(f"!! EMERGENCY STOP: {u_res.distance_cm:.1f}cm")
                        rt.last_log_time = time.time()
                    time.sleep(0.05)
                    continue

            # ---- YOLO Perception ----
            run_yolo = (rt.yolo_frame_count % YOLO_FRAME_INTERVAL == 0) and rt.state not in skip_yolo_states

            if run_yolo and frame is not None:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rt.cached_detections = detector.detect(rgb)
                obstacles = detector.get_obstacles(rt.cached_detections)

                rt.cached_apf_obs = []
                for det in obstacles:
                    raw_dist = 480.0 / (det.height + 1e-6) * 0.32
                    tid = det.track_id
                    if tid not in rt.dist_buffer:
                        rt.dist_buffer[tid] = raw_dist
                    rt.dist_buffer[tid] = 0.7 * rt.dist_buffer[tid] + 0.3 * raw_dist
                    corrected_x = rt.dist_buffer[tid] + 0.12
                    lateral = (det.center[0] - CAM_CENTER_X) / CAM_CENTER_X * corrected_x * 0.55
                    rt.cached_apf_obs.append(Obstacle(
                        x=corrected_x, y=lateral,
                        distance=corrected_x, category=det.category
                    ))

            rt.yolo_frame_count += 1

            # Prune stale track IDs
            if rt.yolo_frame_count % 30 == 0:
                active_ids = {d.track_id for d in rt.cached_detections if d.track_id > 0}
                rt.dist_buffer = {tid: v for tid, v in rt.dist_buffer.items() if tid in active_ids}

            # ---- State Machine ----
            if args.enable_ultrasonic:
                _tick_ultrasonic(rt, motor, ultrasonic, servo, planner, front_dist)
            else:
                _tick_yolo(rt, motor, planner)

            # ---- Visual Overlay ----
            if not args.headless and frame is not None:
                _draw_overlay(frame, rt.cached_detections, rt.state, u_res)
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
        if servo is not None:
            servo.cleanup()
        cap.release()
        cv2.destroyAllWindows()


# ================================================================
# YOLO-only state machine
# ================================================================

def _tick_yolo(rt: RuntimeState, motor, planner):
    """Pure YOLO+APF cruise with distance-based avoidance.
    All detections contribute — any obstacle close enough triggers response
    regardless of category or screen position."""

    # Find the closest threat among all detections
    closest_dist = 999.0
    closest_lateral = 0.0   # negative = left of frame, positive = right
    for det in rt.cached_detections:
        raw_dist = 480.0 / (det.height + 1e-6) * 0.32
        tid = det.track_id
        smoothed = rt.dist_buffer.get(tid, raw_dist)
        corrected_x = smoothed + 0.12
        lateral = (det.center[0] - CAM_CENTER_X) / CAM_CENTER_X

        if corrected_x < closest_dist:
            closest_dist = corrected_x
            closest_lateral = lateral

    if rt.state == STATE_CRUISE:
        if closest_dist < YOLO_EMERGENCY_DIST:
            # Too close — stop and reverse
            motor.stop()
            rt.state = STATE_BLOCKED
            rt.consecutive_blocked += 1
            if time.time() - rt.last_log_time > 1.0:
                print(f"[BLOCKED] Emergency: {closest_dist:.2f}m")
                rt.last_log_time = time.time()

        elif closest_dist < YOLO_AVOID_DIST:
            # Close — aggressive avoidance steering, bypass normal APF
            # Steer away from the obstacle: lateral > 0 (right side) -> steer left (negative)
            steer = -closest_lateral * 0.8
            steer = max(-1.0, min(1.0, steer))

            # Slow down proportionally to closeness
            speed_factor = max(0.3, (closest_dist - YOLO_EMERGENCY_DIST) /
                               (YOLO_AVOID_DIST - YOLO_EMERGENCY_DIST))
            motor.curve_move(CRUISE_SPEED * speed_factor, steer)

            if time.time() - rt.last_log_time > 1.0:
                print(f"[AVOID] {closest_dist:.2f}m steer={steer:.2f}")
                rt.last_log_time = time.time()

        elif closest_dist < YOLO_CAUTION_DIST:
            # Moderate — APF with boosted repulsion
            _apf_cruise(rt, motor, planner, CRUISE_SPEED)

        else:
            # Clear — normal APF cruise
            _apf_cruise(rt, motor, planner, CRUISE_SPEED)

    elif rt.state == STATE_BLOCKED:
        motor.move_backward(AVOID_BACKUP_SPEED)
        time.sleep(RECOVERY_BACKUP_DURATION)
        motor.stop()
        time.sleep(0.2)
        rt.state = STATE_CRUISE
        rt.consecutive_blocked = max(0, rt.consecutive_blocked - 1)
        print(f"[BLOCKED] Reversed from {closest_dist:.2f}m, resuming")


# ================================================================
# Ultrasonic-enabled state machine
# ================================================================

def _tick_ultrasonic(rt: RuntimeState, motor, ultrasonic, servo, planner, front_dist):
    """Full state machine with ultrasonic scanning, servo avoidance,
    side verification, and APF-assisted recovery."""

    if rt.state == STATE_CRUISE:
        if front_dist < OBSTACLE_DIST:
            motor.stop()
            rt.state = STATE_BLOCKED
            rt.consecutive_blocked += 1
            rt.avoid_clear_start = 0
            rt.side_cleared = False
            if time.time() - rt.last_log_time > 1.0:
                print(f"[BLOCKED] Front: {front_dist:.1f}cm")
                rt.last_log_time = time.time()
        else:
            _apf_cruise(rt, motor, planner, CRUISE_SPEED)

    elif rt.state == STATE_BLOCKED:
        motor.move_backward(AVOID_BACKUP_SPEED)
        time.sleep(RECOVERY_BACKUP_DURATION)
        motor.stop()
        time.sleep(0.2)

        dis_left = _scan_left(ultrasonic, servo)
        dis_right = _scan_right(ultrasonic, servo)
        servo.center_ultrasonic()
        time.sleep(SERVO_SETTLE)

        if dis_left < OBSTACLE_DIST and dis_right < OBSTACLE_DIST:
            rt.side_cleared = False
            if rt.consecutive_blocked >= 3:
                print("[BLOCKED] Both blocked x3, spinning out")
                motor.rotate_left(AVOID_CURVE_SPEED)
                time.sleep(1.5)
                motor.stop()
                rt.consecutive_blocked = 0
                rt.state = STATE_CRUISE
            else:
                print(f"[BLOCKED] Both blocked L={dis_left:.0f} R={dis_right:.0f}, spin")
                motor.rotate_left(AVOID_CURVE_SPEED)
                time.sleep(0.8)
                motor.stop()
                rt.state = STATE_CRUISE
        elif dis_left > dis_right:
            rt.avoid_action = "left"
            print(f"[AVOID] Curve LEFT (L={dis_left:.0f} > R={dis_right:.0f})")
            rt.state = STATE_AVOID
            rt.avoid_start_time = time.time()
            rt.avoid_clear_start = 0
            rt.side_cleared = False
        else:
            rt.avoid_action = "right"
            print(f"[AVOID] Curve RIGHT (R={dis_right:.0f} > L={dis_left:.0f})")
            rt.state = STATE_AVOID
            rt.avoid_start_time = time.time()
            rt.avoid_clear_start = 0
            rt.side_cleared = False

    elif rt.state == STATE_AVOID:
        if rt.avoid_action == "left":
            motor.curve_move(AVOID_CURVE_SPEED, -AVOID_TURN_STEER)
        else:
            motor.curve_move(AVOID_CURVE_SPEED, AVOID_TURN_STEER)

        front_now = ultrasonic.measure_once()
        if front_now.valid and front_now.distance_cm > OBSTACLE_DIST:
            if rt.avoid_clear_start == 0:
                rt.avoid_clear_start = time.time()
            elif time.time() - rt.avoid_clear_start >= AVOID_CLEAR_TIME:
                side_dist = _check_side_distance(ultrasonic, servo, rt.avoid_action)
                if side_dist > SIDE_CLEAR_DIST:
                    rt.side_cleared = True
                    servo.center_ultrasonic()
                    motor.stop()
                    rt.consecutive_blocked = max(0, rt.consecutive_blocked - 1)
                    rt.state = STATE_CRUISE
                    elapsed = time.time() - rt.avoid_start_time
                    print(f"[CRUISE] Cleared in {elapsed:.1f}s (side={side_dist:.0f}cm)")
                else:
                    servo.center_ultrasonic()
                    motor.stop()
                    rt.state = STATE_RECOVER
                    rt.recover_start_time = time.time()
                    elapsed = time.time() - rt.avoid_start_time
                    print(f"[RECOVER] Front clear, side={side_dist:.0f}cm")
        else:
            rt.avoid_clear_start = 0

        if time.time() - rt.avoid_start_time >= 6.0:
            motor.stop()
            servo.center_ultrasonic()
            rt.side_cleared = False
            print("[AVOID] Timeout, forcing CRUISE")
            rt.consecutive_blocked = 0
            rt.state = STATE_CRUISE

    elif rt.state == STATE_RECOVER:
        _apf_cruise(rt, motor, planner, RECOVER_SPEED)

        now = time.time()
        if now - rt.last_side_check_time >= SERVO_SIDE_CHECK_INTERVAL:
            rt.last_side_check_time = now
            side_dist = _check_side_distance(ultrasonic, servo, rt.avoid_action)
            if side_dist > SIDE_CLEAR_DIST:
                rt.side_cleared = True
            servo.center_ultrasonic()

        elapsed = time.time() - rt.recover_start_time
        if rt.side_cleared or elapsed >= RECOVER_MAX_DURATION:
            rt.consecutive_blocked = max(0, rt.consecutive_blocked - 1)
            rt.state = STATE_CRUISE
            fc = ultrasonic.measure_once()
            ds = f"{fc.distance_cm:.1f}cm" if fc.valid else "---"
            reason = "side clear" if rt.side_cleared else "timeout"
            print(f"[CRUISE] Resumed ({reason}, front={ds})")
        else:
            fc = ultrasonic.measure_once()
            if fc.valid and fc.distance_cm < OBSTACLE_DIST:
                motor.stop()
                rt.side_cleared = False
                rt.state = STATE_BLOCKED
                rt.consecutive_blocked += 1


# ================================================================
# Shared helpers
# ================================================================

def _apf_cruise(rt: RuntimeState, motor, planner, speed: float):
    """Run APF planning and apply steering. Shared by both modes."""
    out = planner.compute(0, 0, 0, 8.0, 3.0, 0, rt.cached_apf_obs)
    steer = out.target_steering * 0.4

    if abs(steer) < 0.05 and rt.cached_apf_obs:
        closest = min(rt.cached_apf_obs, key=lambda o: o.distance)
        if closest.y > 0.05:
            steer = -0.3
        elif closest.y < -0.05:
            steer = 0.3

    if not out.emergency_brake and out.target_speed > 0.1:
        motor.curve_move(speed, steer)
    else:
        motor.stop()

    if time.time() - rt.last_log_time > 3.0:
        print(f"[CRUISE] Det:{len(rt.cached_apf_obs)} steer={steer:.2f}")
        rt.last_log_time = time.time()


# ---- Servo Scan Helpers ----

def _scan_left(sensor, servo):
    """Scan left (~155 degrees) and return distance in cm."""
    servo.set_ultrasonic_angle(155)
    time.sleep(SERVO_SETTLE)
    reading = sensor.measure_once()
    return reading.distance_cm if reading.valid else 999.0


def _scan_right(sensor, servo):
    """Scan right (~85 degrees) and return distance in cm."""
    servo.set_ultrasonic_angle(85)
    time.sleep(SERVO_SETTLE)
    reading = sensor.measure_once()
    return reading.distance_cm if reading.valid else 999.0


def _check_side_distance(sensor, servo, avoid_action):
    """Check distance on the avoidance side to verify the obstacle is passed."""
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

    if u_res is not None:
        dist_str = f"{u_res.distance_cm:.0f}cm" if u_res.valid else "---"
    else:
        dist_str = "N/A"
    cv2.putText(frame, f"{state} | {dist_str}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


if __name__ == "__main__":
    main()
