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


def _run_remote_mode(args):
    """
    Remote control mode – start TCP server, accept commands from PC client.
    Motor, servo, and ultrasonic are controlled via network commands.
    """
    from rpi_deploy.motor_driver import MotorController
    from rpi_deploy.servo_controller import ServoController
    from rpi_deploy.ultrasonic_sensor import UltrasonicSensor
    from rpi_deploy.remote_control import RemoteControlServer, VehicleStatus

    print("=" * 50)
    print("  Mode: REMOTE control (TCP server)")
    print("=" * 50)

    motor = MotorController()
    servo = ServoController()
    sensor = UltrasonicSensor()
    server = RemoteControlServer()

    # --- Command handlers ---
    def handle_move(params):
        direction = params.get("direction", "stop")
        speed = params.get("speed", 50) / 100.0  # percentage → 0-1
        speed = max(0.0, min(1.0, speed))
        if direction == "forward":
            motor.move_forward(speed)
        elif direction == "backward":
            motor.move_backward(speed)
        elif direction == "left":
            motor.rotate_left(speed)
        elif direction == "right":
            motor.rotate_right(speed)
        else:
            motor.stop()
        return f"Moved {direction} at {speed:.2f}"

    def handle_stop(params):
        motor.stop()
        return "Stopped"

    def handle_servo(params):
        servo_type = params.get("type", "ultrasonic")
        angle = params.get("angle", 90)
        if servo_type == "ultrasonic":
            servo.set_ultrasonic_angle(angle)
        elif servo_type == "camera_pan":
            servo.set_camera_pan(angle)
        elif servo_type == "camera_tilt":
            servo.set_camera_tilt(angle)
        return f"Servo {servo_type} set to {angle}"

    def handle_scan(params):
        reading = sensor.measure_average(samples=3)
        return {"distance_cm": reading.distance_cm, "valid": reading.valid}

    def handle_auto(params):
        """Switch to autonomous obstacle avoidance mode."""
        from rpi_deploy.obstacle_avoidance import ObstacleAvoidanceController, mode_servo
        ctrl = ObstacleAvoidanceController()
        ctrl.start()
        # Run avoidance in a background thread
        avoidance_thread = threading.Thread(target=mode_servo, args=(ctrl,), daemon=True)
        avoidance_thread.start()
        return "Auto mode started"

    server.register_handler("move", handle_move)
    server.register_handler("stop", handle_stop)
    server.register_handler("servo", handle_servo)
    server.register_handler("scan", handle_scan)
    server.register_handler("auto", handle_auto)

    # --- Status provider ---
    _status_state = {"direction": "STOP"}

    def _update_direction(name: str):
        _status_state["direction"] = name

    # Override handle_move to also track direction
    _orig_handle_move = handle_move

    def handle_move_with_status(params):
        direction = params.get("direction", "stop")
        _update_direction(direction.upper() if direction != "stop" else "STOP")
        return _orig_handle_move(params)

    def handle_stop_with_status(params):
        _update_direction("STOP")
        return handle_stop(params)

    # Re-register with direction tracking
    server.register_handler("move", handle_move_with_status)
    server.register_handler("stop", handle_stop_with_status)

    def get_status() -> VehicleStatus:
        reading = sensor.measure_once()
        return VehicleStatus(
            direction=_status_state["direction"],
            obstacle_distance=reading.distance_cm if reading.valid else 999.0,
            ultrasonic_angle=servo._ultrasonic_angle,
            camera_pan=servo._camera_pan,
            camera_tilt=servo._camera_tilt,
            timestamp=time.time(),
        )

    server.register_status_provider(get_status)
    server.start()

    print("[Remote] Server running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[Remote] Interrupted")
    finally:
        motor.cleanup()
        servo.cleanup()
        sensor.cleanup()
        server.stop()
        print("[Remote] Cleanup done")


def _run_v2v_mode(args):
    """
    V2V cooperative avoidance mode – ultrasonic avoidance + V2V communication.
    Runs servo-enhanced avoidance while broadcasting/receiving V2V messages
    with a PC peer for cooperative perception.
    """
    from rpi_deploy.obstacle_avoidance import ObstacleAvoidanceController
    from rpi_deploy.motor_driver import MotorController
    from rpi_deploy.ultrasonic_sensor import UltrasonicSensor
    from rpi_deploy.servo_controller import ServoController
    from cooperation.v2v_message import V2VCommunicator, SharedDetection

    print("=" * 50)
    print("  Mode: V2V cooperative avoidance")
    print(f"  PC peer: {args.pc_host}:{args.pc_port}")
    print("=" * 50)

    ctrl = ObstacleAvoidanceController()
    motor = ctrl.motor
    sensor = ctrl.sensor
    servo = ctrl.servo

    # V2V communicator
    comm_config = {"protocol": "socket", "max_latency_ms": 200, "dropout_rate": 0.0}
    v2v = V2VCommunicator("rpi_vehicle_1", comm_config)
    v2v.start_socket_server(port=args.pc_port + 1)
    v2v.add_socket_peer("pc_vehicle", args.pc_host, args.pc_port)
    print(f"[V2V] Socket server on port {args.pc_port + 1}, peer at {args.pc_host}:{args.pc_port}")

    # Obstacle thresholds
    OBSTACLE_CM = 50.0
    SAFE_CM = 30.0
    SERVO_SETTLE = 0.4

    ctrl.wait_for_start()

    try:
        while ctrl.is_running:
            # Scan three directions
            dist_front = ctrl.scan_front_cm()
            dist_left = ctrl.scan_left_cm()
            dist_right = ctrl.scan_right_cm()

            # V2V: broadcast our status
            our_detections = []
            if dist_front < OBSTACLE_CM:
                our_detections.append(SharedDetection(
                    class_name="ultrasonic_obstacle", category="obstacle",
                    confidence=1.0, world_x=dist_front / 100.0,
                    world_y=0.0, distance_from_sender=dist_front / 100.0))
            msg = v2v.create_message(
                position=(0, 0, 0), velocity=(0, 0),
                detections=our_detections,
                intent="cruising" if dist_front > OBSTACLE_CM else "yielding",
            )
            v2v.broadcast(msg)

            # V2V: receive peer detections
            v2v_msgs = v2v.receive_all()
            v2v_obstacle = False
            for vid, vmsg in v2v_msgs.items():
                for sd in vmsg.detections:
                    if sd.distance_from_sender < SAFE_CM / 100.0:
                        v2v_obstacle = True

            # Decision: local + V2V
            if dist_front < OBSTACLE_CM or v2v_obstacle:
                ctrl.stop()
                time.sleep(0.2)
                if v2v_obstacle:
                    print("[V2V] Peer reports nearby obstacle – yielding")
                ctrl.backward(0.3)
                time.sleep(0.5)
                ctrl.stop()

                # Choose direction
                if dist_left > dist_right and dist_left > OBSTACLE_CM:
                    ctrl.left(0.3)
                elif dist_right > OBSTACLE_CM:
                    ctrl.right(0.3)
                else:
                    ctrl.left(0.3)  # default pivot
                time.sleep(0.5)
                ctrl.stop()
                ctrl.scan_front_cm()
            else:
                ctrl.forward(0.15)

            print(f"[V2V] F={dist_front:.0f} L={dist_left:.0f} R={dist_right:.0f} cm | "
                  f"Peers={len(v2v_msgs)} | V2V_obs={v2v_obstacle}")
            time.sleep(0.3)

    except KeyboardInterrupt:
        pass
    finally:
        ctrl.cleanup()
        print("[V2V] Cleanup done")


def load_config(config_path: str = "config/config.yaml") -> dict:
    if yaml is None:
        raise RuntimeError("pyyaml is required. Install: pip install pyyaml")
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="RPi Car Controller")
    parser.add_argument("--mode", type=str, default="camera",
                        choices=["camera", "obstacle_avoidance", "remote", "v2v"],
                        help="Run mode: 'camera' (YOLO+APF), 'obstacle_avoidance' (ultrasonic), "
                             "'remote' (network control), 'v2v' (cooperative avoidance)")
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
        # Always pass a list (even empty) so argparse uses it instead of sys.argv
        remaining = [a for a in sys.argv[1:] if a not in ("--mode", "obstacle_avoidance")]
        obstacle_main(remaining)
        return

    # Delegate to remote control mode
    if args.mode == "remote":
        _run_remote_mode(args)
        return

    # Delegate to V2V cooperative avoidance mode
    if args.mode == "v2v":
        _run_v2v_mode(args)
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

    # Initialize hardware (using modular drivers)
    print("[Init] Setting up hardware...")
    from rpi_deploy.motor_driver import MotorController
    from rpi_deploy.ultrasonic_sensor import UltrasonicSensor
    from rpi_deploy.hardware_config import hardware_config

    ultrasonic = UltrasonicSensor()
    motor = MotorController()
    emergency_dist = config.get('raspberry_pi', {}).get('ultrasonic', {}).get(
        'emergency_distance', 0.15)
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
            ultra_dist = ultrasonic.measure_once().distance_cm / 100.0  # cm → m
            if ultra_dist < emergency_dist:
                motor.emergency_stop()
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
            # Map planner output to MotorController API
            direction = control.get('direction', 'forward')
            speed = max(control.get('left_pwm', 0), control.get('right_pwm', 0)) / 100.0
            speed = max(0.0, min(1.0, speed))
            if direction == "forward":
                motor.move_forward(speed)
            elif direction == "backward":
                motor.move_backward(speed)
            else:
                motor.stop()

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
