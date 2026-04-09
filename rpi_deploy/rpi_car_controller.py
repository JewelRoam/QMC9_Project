"""
Raspberry Pi Car Controller - Real-world deployment script.
Runs on the Raspberry Pi 5 with 4WD car platform.

Pipeline: Camera → YOLO (ONNX) → APF Planning → Motor Control
Optional: V2V communication with PC (Edge Cloud) via WiFi/Socket

Hardware:
  - Raspberry Pi 5
  - USB Camera or Pi Camera
  - HC-SR04 Ultrasonic sensor (safety layer)
  - 4WD motor driver via expansion board (PWM)

Usage (on Raspberry Pi):
  python rpi_car_controller.py
  python rpi_car_controller.py --cooperative  # Enable V2V with PC
"""
import os
import sys
import time
import argparse
import threading

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import yaml
except ImportError:
    yaml = None
    print("[WARNING] pyyaml not installed, config loading disabled. Install: pip install pyyaml")

try:
    import numpy as np
except ImportError:
    np = None
    print("[WARNING] numpy not installed, camera-based mode disabled. Install: pip install numpy")


class UltrasonicSensor:
    """
    HC-SR04 Ultrasonic distance sensor for emergency safety layer.
    Falls back to dummy mode if GPIO is unavailable (e.g., on PC for testing).
    """

    def __init__(self, trigger_pin: int = 23, echo_pin: int = 24):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self._gpio_available = False
        self._last_distance = 999.0

        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(trigger_pin, GPIO.OUT)
            GPIO.setup(echo_pin, GPIO.IN)
            GPIO.output(trigger_pin, False)
            time.sleep(0.1)
            self._gpio_available = True
            print("[Ultrasonic] GPIO initialized")
        except (ImportError, RuntimeError):
            print("[Ultrasonic] GPIO not available, using dummy mode")

    def measure_distance(self) -> float:
        """Measure distance in meters. Returns 999.0 if unavailable."""
        if not self._gpio_available:
            return 999.0

        try:
            GPIO = self.GPIO
            GPIO.output(self.trigger_pin, True)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, False)

            pulse_start = time.time()
            timeout = pulse_start + 0.04  # 40ms timeout

            while GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return 999.0

            pulse_end = time.time()
            timeout = pulse_end + 0.04

            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return 999.0

            distance = (pulse_end - pulse_start) * 34300 / 2 / 100  # meters
            self._last_distance = distance
            return distance
        except Exception:
            return self._last_distance

    def cleanup(self):
        if self._gpio_available:
            self.GPIO.cleanup()


class MotorDriver:
    """
    4WD Motor driver for Raspberry Pi expansion board.
    Controls left and right motor groups via PWM.
    Falls back to console output if GPIO unavailable.
    """

    # Default pin configuration for the 4WD robot expansion board
    LEFT_FORWARD_PIN = 20
    LEFT_BACKWARD_PIN = 21
    RIGHT_FORWARD_PIN = 19
    RIGHT_BACKWARD_PIN = 26
    LEFT_PWM_PIN = 16
    RIGHT_PWM_PIN = 13

    def __init__(self):
        self._gpio_available = False
        self._left_pwm = None
        self._right_pwm = None

        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)

            # Setup direction pins
            for pin in [self.LEFT_FORWARD_PIN, self.LEFT_BACKWARD_PIN,
                        self.RIGHT_FORWARD_PIN, self.RIGHT_BACKWARD_PIN]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, False)

            # Setup PWM pins
            GPIO.setup(self.LEFT_PWM_PIN, GPIO.OUT)
            GPIO.setup(self.RIGHT_PWM_PIN, GPIO.OUT)
            self._left_pwm = GPIO.PWM(self.LEFT_PWM_PIN, 1000)
            self._right_pwm = GPIO.PWM(self.RIGHT_PWM_PIN, 1000)
            self._left_pwm.start(0)
            self._right_pwm.start(0)

            self._gpio_available = True
            print("[Motor] GPIO motor driver initialized")
        except (ImportError, RuntimeError):
            print("[Motor] GPIO not available, using console output mode")

    def drive(self, left_pwm: int, right_pwm: int, direction: str = "forward"):
        """
        Drive the motors.

        Args:
            left_pwm: 0-100 PWM duty cycle for left motors
            right_pwm: 0-100 PWM duty cycle for right motors
            direction: "forward", "backward", or "stop"
        """
        left_pwm = max(0, min(100, left_pwm))
        right_pwm = max(0, min(100, right_pwm))

        if not self._gpio_available:
            if direction != "stop" and (left_pwm > 0 or right_pwm > 0):
                print(f"[Motor] {direction} L={left_pwm} R={right_pwm}")
            return

        GPIO = self.GPIO

        if direction == "stop":
            GPIO.output(self.LEFT_FORWARD_PIN, False)
            GPIO.output(self.LEFT_BACKWARD_PIN, False)
            GPIO.output(self.RIGHT_FORWARD_PIN, False)
            GPIO.output(self.RIGHT_BACKWARD_PIN, False)
            self._left_pwm.ChangeDutyCycle(0)
            self._right_pwm.ChangeDutyCycle(0)

        elif direction == "forward":
            GPIO.output(self.LEFT_FORWARD_PIN, True)
            GPIO.output(self.LEFT_BACKWARD_PIN, False)
            GPIO.output(self.RIGHT_FORWARD_PIN, True)
            GPIO.output(self.RIGHT_BACKWARD_PIN, False)
            self._left_pwm.ChangeDutyCycle(left_pwm)
            self._right_pwm.ChangeDutyCycle(right_pwm)

        elif direction == "backward":
            GPIO.output(self.LEFT_FORWARD_PIN, False)
            GPIO.output(self.LEFT_BACKWARD_PIN, True)
            GPIO.output(self.RIGHT_FORWARD_PIN, False)
            GPIO.output(self.RIGHT_BACKWARD_PIN, True)
            self._left_pwm.ChangeDutyCycle(left_pwm)
            self._right_pwm.ChangeDutyCycle(right_pwm)

    def stop(self):
        self.drive(0, 0, "stop")

    def cleanup(self):
        self.stop()
        if self._gpio_available:
            if self._left_pwm:
                self._left_pwm.stop()
            if self._right_pwm:
                self._right_pwm.stop()
            self.GPIO.cleanup()


class CameraCapture:
    """Camera capture for Raspberry Pi (USB or Pi Camera)."""

    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480):
        import cv2
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_index}")

        print(f"[Camera] Opened camera {camera_index} at {width}x{height}")

    def read(self):
        """Read a frame. Returns RGB numpy array or None."""
        import cv2
        ret, frame = self.cap.read()
        if ret:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return None

    def release(self):
        self.cap.release()


def load_config(config_path: str = "config/config.yaml") -> dict:
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="RPi Car Controller")
    parser.add_argument("--mode", type=str, default="camera",
                        choices=["camera", "obstacle_avoidance"],
                        help="Run mode: 'camera' (YOLO+APF pipeline) or 'obstacle_avoidance' (ultrasonic only)")
    parser.add_argument("--cooperative", action="store_true",
                        help="Enable V2V cooperative mode with PC")
    parser.add_argument("--pc-host", default="192.168.1.50",
                        help="PC (Edge Cloud) IP address")
    parser.add_argument("--pc-port", type=int, default=5555,
                        help="PC V2V communication port")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index")
    parser.add_argument("--headless", action="store_true",
                        help="Run without display (for SSH)")
    parser.add_argument("--config", default="config/config.yaml",
                        help="Config file path")
    args = parser.parse_args()

    # Delegate to obstacle_avoidance module if requested
    if args.mode == "obstacle_avoidance":
        from rpi_deploy.obstacle_avoidance import main as obstacle_main
        # Strip our --mode arg so obstacle_avoidance's argparse works correctly
        remaining = [a for a in sys.argv[1:] if a not in ("--mode", "obstacle_avoidance")]
        obstacle_main(remaining if remaining else None)
        return

    if yaml is None:
        print("[ERROR] pyyaml is required for camera mode. Install: pip install pyyaml")
        sys.exit(1)

    config = load_config(args.config)
    print("=" * 50)
    print("  RPi Autonomous Car Controller")
    print("=" * 50)

    # Import modules (after path setup)
    from perception.detector import YOLODetector
    from planning.apf_planner import APFPlanner, Obstacle
    from control.vehicle_controller import VehicleController

    # Initialize perception (use ONNX for RPi efficiency)
    rpi_perception_cfg = dict(config['perception'])
    rpi_perception_cfg['use_onnx'] = True  # Prefer ONNX on RPi
    rpi_perception_cfg['imgsz'] = 320      # Smaller for RPi speed
    rpi_perception_cfg['confidence_threshold'] = 0.35

    print("[Init] Loading YOLO model...")
    detector = YOLODetector(rpi_perception_cfg)

    # Initialize planner (with more conservative parameters for real-world)
    rpi_planner_cfg = dict(config['planning']['apf'])
    rpi_planner_cfg['max_speed'] = 15.0       # Slower for safety
    rpi_planner_cfg['emergency_distance'] = 0.3  # 30cm emergency
    rpi_planner_cfg['d0'] = 2.0               # 2m influence range (real scale)
    planner = APFPlanner(rpi_planner_cfg)

    # Initialize controller
    controller = VehicleController(config['control'], platform="raspberry_pi")

    # Initialize hardware
    print("[Init] Setting up hardware...")
    ultrasonic = UltrasonicSensor(
        trigger_pin=config.get('raspberry_pi', {}).get('ultrasonic', {}).get('trigger_pin', 23),
        echo_pin=config.get('raspberry_pi', {}).get('ultrasonic', {}).get('echo_pin', 24),
    )
    emergency_dist = config.get('raspberry_pi', {}).get('ultrasonic', {}).get('emergency_distance', 0.15)

    motor = MotorDriver()
    camera = CameraCapture(args.camera, width=640, height=480)

    # V2V Cooperative mode
    v2v_comm = None
    if args.cooperative:
        from cooperation.v2v_message import V2VCommunicator
        comm_config = {'protocol': 'socket', 'max_latency_ms': 200, 'dropout_rate': 0.0}
        v2v_comm = V2VCommunicator("rpi_vehicle_1", comm_config)
        v2v_comm.start_socket_server(port=5556)
        v2v_comm.add_socket_peer("pc_vehicle", args.pc_host, args.pc_port)
        print(f"[V2V] Cooperative mode: PC at {args.pc_host}:{args.pc_port}")

    # Display setup
    show_display = not args.headless
    if show_display:
        import cv2
        cv2.namedWindow('RPi Car View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RPi Car View', 640, 480)

    print("\n[Ready] Starting autonomous driving loop...")
    print("[Ready] Press Ctrl+C to stop\n")

    # Simple goal: always drive forward (in real scenario, use waypoints or line following)
    # For RPi car, the "goal" is a point ahead of the car
    goal_distance_ahead = 2.0  # meters ahead

    frame_count = 0
    try:
        while True:
            t0 = time.perf_counter()

            # 1. Read camera
            rgb_image = camera.read()
            if rgb_image is None:
                continue

            # 2. Ultrasonic safety check (highest priority)
            ultra_dist = ultrasonic.measure_distance()
            if ultra_dist < emergency_dist:
                motor.stop()
                print(f"[EMERGENCY] Ultrasonic: {ultra_dist:.3f}m - STOPPED")
                time.sleep(0.5)
                continue

            # 3. YOLO detection
            detections = detector.detect(rgb_image)
            obstacles = detector.get_obstacles(detections)

            # 4. Estimate distances from bbox size (simple heuristic for real camera)
            # Without depth camera: use bbox height as proxy for distance
            for det in obstacles:
                # Heuristic: larger bbox = closer object
                # Rough calibration: bbox_height=480 → ~0.5m, bbox_height=50 → ~5m
                bbox_h = det.height
                if bbox_h > 0:
                    det.distance = max(0.3, 480.0 / bbox_h * 0.3)

            # 5. Convert to APF obstacles (ego at origin, goal ahead)
            ego_x, ego_y, ego_yaw = 0.0, 0.0, 0.0
            goal_x, goal_y = goal_distance_ahead, 0.0

            apf_obstacles = []
            img_cx = 320  # half of 640
            for det in obstacles:
                if det.distance > 0:
                    # Map bbox center to lateral offset
                    cx, _ = det.center
                    lateral = (cx - img_cx) / img_cx * det.distance * 0.5
                    apf_obstacles.append(Obstacle(
                        x=det.distance,
                        y=lateral,
                        distance=det.distance,
                        category=det.category,
                        confidence=det.confidence,
                    ))

            # 6. V2V cooperation (if enabled)
            coop_obstacles = []
            if v2v_comm:
                # Broadcast our detections
                shared_dets = v2v_comm.detections_to_shared(obstacles, ego_x, ego_y)
                msg = v2v_comm.create_message(
                    position=(0, 0, 0), velocity=(0, 0),
                    detections=shared_dets, intent="cruising",
                )
                v2v_comm.broadcast(msg)

                # Receive from PC
                v2v_msgs = v2v_comm.receive_all()
                for vid, vmsg in v2v_msgs.items():
                    for sd in vmsg.detections:
                        coop_obstacles.append(Obstacle(
                            x=sd.world_x, y=sd.world_y,
                            distance=sd.distance_from_sender,
                            category=sd.category,
                            confidence=sd.confidence * 0.8,
                            source="v2v",
                        ))

            # 7. APF planning
            planner_output = planner.compute(
                ego_x, ego_y, ego_yaw, 0.0,
                goal_x, goal_y, apf_obstacles,
                cooperative_obstacles=coop_obstacles if coop_obstacles else None,
            )

            # 8. Motor control
            control = controller.compute_control(planner_output, 0.0)
            motor.drive(
                left_pwm=control['left_pwm'],
                right_pwm=control['right_pwm'],
                direction=control['direction'],
            )

            # 9. Display (if available)
            if show_display:
                import cv2
                display = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                for det in detections:
                    x1, y1, x2, y2 = det.bbox
                    color = {"vehicle": (0,255,0), "pedestrian": (0,0,255),
                             "cyclist": (255,165,0)}.get(det.category, (128,128,128))
                    cv2.rectangle(display, (x1,y1), (x2,y2), color, 2)
                    lbl = f"{det.class_name} {det.confidence:.2f}"
                    if det.distance > 0:
                        lbl += f" {det.distance:.1f}m"
                    cv2.putText(display, lbl, (x1, y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                # HUD
                cv2.putText(display, f"Status: {planner_output.status}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display, f"Ultra: {ultra_dist:.2f}m", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(display, f"L={control['left_pwm']} R={control['right_pwm']} {control['direction']}",
                            (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                cv2.imshow('RPi Car View', display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # Timing
            dt = (time.perf_counter() - t0) * 1000
            frame_count += 1
            if frame_count % 30 == 0:
                print(f"[Loop] Frame {frame_count} | {dt:.0f}ms | "
                      f"Det: {len(detections)} | Status: {planner_output.status} | "
                      f"Ultra: {ultra_dist:.2f}m | "
                      f"Motor: L={control['left_pwm']} R={control['right_pwm']}")

    except KeyboardInterrupt:
        print("\n[Stopped] User interrupt")
    finally:
        motor.cleanup()
        camera.release()
        ultrasonic.cleanup()
        if show_display:
            import cv2
            cv2.destroyAllWindows()
        print("[Cleanup] Done")


if __name__ == '__main__':
    main()
