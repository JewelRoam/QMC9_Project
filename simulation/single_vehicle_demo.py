"""
Single Vehicle Demo - Full Closed-Loop Pipeline in CARLA.
Demonstrates: Perception (YOLO) → Depth → APF Planning → PID Control

This is the FIRST milestone: prove the full pipeline works on PC
before deploying to Raspberry Pi.

Usage:
  1. Start CARLA server: CARLA_0.9.13/WindowsNoEditor/CarlaUE4.exe
  2. Run: python -m simulation.single_vehicle_demo
"""
import os
import sys
import time
import yaml
import cv2
import numpy as np

# Ensure project root is in path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation.carla_env import CarlaEnv
from perception.detector import YOLODetector
from perception.depth_estimator import DepthEstimator
from planning.apf_planner import APFPlanner
from control.vehicle_controller import VehicleController
from utils.logger import setup_logger, FPSCounter, PerformanceMetrics


def load_config(config_path: str = "config/config.yaml") -> dict:
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def draw_hud(image: np.ndarray, fps: float, planner_output, control_dict: dict,
             detection_summary: dict, vehicle_speed: float) -> np.ndarray:
    """Draw comprehensive HUD overlay on the image."""
    h, w = image.shape[:2]

    # --- Left panel: Detection info ---
    panel_w, panel_h = 300, 200
    overlay = image.copy()
    cv2.rectangle(overlay, (10, 10), (10 + panel_w, 10 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
    cv2.rectangle(image, (10, 10), (10 + panel_w, 10 + panel_h), (0, 255, 0), 2)

    y = 35
    cv2.putText(image, "=== Perception-Planning-Control ===", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    y += 25
    cv2.putText(image, f"FPS: {fps:.1f}", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y += 20
    cv2.putText(image, f"Speed: {vehicle_speed:.1f} km/h", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y += 20
    cv2.putText(image, f"Vehicles: {detection_summary.get('vehicle', 0)}", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    y += 20
    cv2.putText(image, f"Pedestrians: {detection_summary.get('pedestrian', 0)}", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    y += 20
    cv2.putText(image, f"Cyclists: {detection_summary.get('cyclist', 0)}", (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1)

    # --- Right panel: Planning & Control ---
    if planner_output:
        y += 25
        status_color = {
            "normal": (0, 255, 0),
            "avoiding": (0, 165, 255),
            "emergency": (0, 0, 255),
            "goal_reached": (255, 255, 0),
        }.get(planner_output.status, (255, 255, 255))

        cv2.putText(image, f"Status: {planner_output.status.upper()}", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
        y += 20
        cv2.putText(image, f"Nearest Obs: {planner_output.nearest_obstacle_dist:.1f}m", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    if control_dict:
        y += 20
        cv2.putText(image, f"Throttle: {control_dict.get('throttle', 0):.2f}  "
                    f"Brake: {control_dict.get('brake', 0):.2f}", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y += 18
        cv2.putText(image, f"Steer: {control_dict.get('steer', 0):.2f}", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    return image


def draw_detections(image: np.ndarray, detections, depth_estimator) -> np.ndarray:
    """Draw bounding boxes with distance info."""
    for det in detections:
        x1, y1, x2, y2 = det.bbox
        color = {
            "vehicle": (0, 255, 0),
            "pedestrian": (0, 0, 255),
            "cyclist": (255, 165, 0),
            "traffic_sign": (0, 165, 255),
        }.get(det.category, (128, 128, 128))

        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        label = f"{det.class_name} {det.confidence:.2f}"
        if det.distance > 0:
            label += f" {det.distance:.1f}m"

        cv2.putText(image, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    return image


def main():
    config = load_config()
    logger = setup_logger("SingleVehicle", config.get("logging", {}).get("level", "INFO"))
    metrics = PerformanceMetrics()
    fps_counter = FPSCounter()

    # Initialize modules
    logger.info("Initializing perception module...")
    detector = YOLODetector(config['perception'])
    depth_estimator = DepthEstimator(config['depth'], config['sensors']['rgb_camera'])
    logger.info("Initializing planning module...")
    planner = APFPlanner(config['planning']['apf'])
    logger.info("Initializing control module...")
    controller = VehicleController(config['control'], platform="carla")

    # Initialize CARLA
    logger.info("Connecting to CARLA...")
    env = CarlaEnv(config)
    env.connect()

    try:
        # Spawn ego vehicle
        ego_cfg = config.get('ego_vehicle', {})
        ego = env.spawn_vehicle(
            "ego_vehicle",
            blueprint_filter=ego_cfg.get('blueprint_filter'),
            spawn_index=ego_cfg.get('spawn_index'),
        )

        # Attach sensors
        sensor_cfg = config.get('sensors', {})
        ego.attach_rgb_camera(sensor_cfg.get('rgb_camera', {}))
        ego.attach_depth_camera(sensor_cfg.get('depth_camera', {}))

        # Generate route using CARLA's GlobalRoutePlanner
        env.generate_route(ego, sampling_resolution=2.0)

        # Spawn traffic (walkers disabled - can cause hangs in CARLA 0.9.13)
        env.spawn_traffic(num_vehicles=15, num_walkers=0)

        # Wait for first sensor data
        logger.info("Waiting for sensor data...")
        for _ in range(20):
            env.tick()
            time.sleep(0.05)

        # Setup display
        cv2.namedWindow('Single Vehicle Demo', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Single Vehicle Demo', 1280, 720)

        logger.info("=== Starting single-vehicle closed-loop demo ===")
        logger.info("Press 'q' or close window to stop.")

        planner_output = None
        control_dict = {}
        frame_count = 0

        while True:
            metrics.start_timer("total_frame")

            # 1. Tick simulation
            env.tick()

            # 2. Get sensor data
            rgb_image = ego.get_rgb_image()
            depth_data = ego.get_depth_data()

            if rgb_image is None:
                continue

            # 3. Update depth
            if depth_data is not None:
                depth_estimator.update_depth_image(depth_data)

            # 4. PERCEPTION: Detect obstacles
            metrics.start_timer("perception")
            detections = detector.detect(rgb_image)
            obstacles_det = detector.get_obstacles(detections)
            metrics.stop_timer("perception")

            # 5. DEPTH: Enrich detections with distance
            metrics.start_timer("depth")
            depth_estimator.enrich_detections(obstacles_det, ego.get_transform())
            metrics.stop_timer("depth")

            # 6. Get ego state
            ego_x, ego_y, _ = ego.get_location()
            ego_yaw = ego.get_yaw()
            ego_speed = ego.get_speed_kmh()

            # 7. Advance waypoint if close enough
            # Use a larger threshold so we always have a look-ahead goal
            ego.advance_waypoint(threshold=8.0)
            goal = ego.get_next_waypoint()

            if goal:
                goal_x, goal_y = goal
                
                # Debug: print vehicle state and goal
                if frame_count % 30 == 0:
                    logger.info(f"Ego: ({ego_x:.1f}, {ego_y:.1f}) yaw={ego_yaw:.1f}° | Goal: ({goal_x:.1f}, {goal_y:.1f}) | Speed: {ego_speed:.1f}km/h")

                # 8. PLANNING: APF obstacle avoidance
                metrics.start_timer("planning")
                apf_obstacles = planner.detections_to_obstacles(
                    obstacles_det, ego_x, ego_y, ego_yaw
                )
                planner_output = planner.compute(
                    ego_x, ego_y, ego_yaw, ego_speed,
                    goal_x, goal_y, apf_obstacles
                )
                metrics.stop_timer("planning")

                # 9. CONTROL: Compute and apply vehicle control
                metrics.start_timer("control")
                control_dict = controller.compute_control(
                    planner_output, ego_speed
                )
                controller.apply_carla_control(ego.actor, control_dict)
                metrics.stop_timer("control")

            # 10. VISUALIZATION
            fps_counter.tick()
            display = rgb_image.copy()
            display = draw_detections(display, detections, depth_estimator)

            summary = detector.get_detection_summary(detections)
            display = draw_hud(display, fps_counter.fps, planner_output,
                             control_dict, summary, ego_speed)

            # Convert RGB to BGR for OpenCV display
            cv2.imshow('Single Vehicle Demo', cv2.cvtColor(display, cv2.COLOR_RGB2BGR))

            metrics.stop_timer("total_frame")
            metrics.increment("frames")

            # Log periodically
            frame_count += 1
            if frame_count % 100 == 0:
                avg_total = metrics.get_average("total_frame_ms")
                avg_perc = metrics.get_average("perception_ms")
                avg_plan = metrics.get_average("planning_ms")
                logger.info(f"Frame {frame_count} | FPS: {fps_counter.fps:.1f} | "
                           f"Total: {avg_total:.1f}ms | Perception: {avg_perc:.1f}ms | "
                           f"Planning: {avg_plan:.1f}ms | Speed: {ego_speed:.1f}km/h | "
                           f"Status: {planner_output.status if planner_output else 'N/A'}")

            # Check exit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if cv2.getWindowProperty('Single Vehicle Demo', cv2.WND_PROP_VISIBLE) < 1:
                break

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        cv2.destroyAllWindows()
        env.cleanup()

        # Save metrics
        metrics_path = os.path.join(config.get("logging", {}).get("output_dir", "output"),
                                    "single_vehicle_metrics.json")
        metrics.save(metrics_path)
        logger.info(f"Metrics saved to {metrics_path}")
        logger.info("Single vehicle demo finished.")


if __name__ == '__main__':
    main()
