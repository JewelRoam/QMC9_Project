"""
CARLA Environment Wrapper.
Provides a clean interface for:
  - Synchronous mode simulation
  - Multi-vehicle spawning and management
  - Sensor attachment (RGB camera, depth camera)
  - Waypoint-based route generation
  - Traffic generation
"""
import glob
import os
import sys
import math
import random
import time
import numpy as np
from typing import List, Tuple, Optional, Dict

# Add CARLA egg to path
def find_carla_egg():
    """Find CARLA egg file using multiple search strategies."""
    egg_pattern = 'carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'
    )
    
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Get project root (parent of simulation/)
    project_root = os.path.dirname(script_dir)
    # Get workspace root (parent of QMC9_Project/)
    workspace_root = os.path.dirname(project_root)
    
    search_paths = [
        # From project root (e.g., e:/QMC9_Project/QMC9_Project/)
        os.path.join(project_root, 'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI', 'carla', 'dist', egg_pattern),
        # From workspace root (e.g., e:/QMC9_Project/)
        os.path.join(workspace_root, 'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI', 'carla', 'dist', egg_pattern),
        # Current working directory
        os.path.join(os.getcwd(), 'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI', 'carla', 'dist', egg_pattern),
        # Parent of current working directory
        os.path.join(os.path.dirname(os.getcwd()), 'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI', 'carla', 'dist', egg_pattern),
    ]
    
    for path_pattern in search_paths:
        matches = glob.glob(path_pattern)
        if matches:
            return matches[0]
    
    return None

carla_egg_path = find_carla_egg()
if carla_egg_path and carla_egg_path not in sys.path:
    sys.path.append(carla_egg_path)
elif not carla_egg_path:
    print(f"[WARNING] CARLA egg file not found for Python {sys.version_info.major}.{sys.version_info.minor}")

import carla


class ManagedVehicle:
    """Represents a managed vehicle in the CARLA world with attached sensors."""

    def __init__(self, vehicle_id: str, actor, world, spawn_transform=None):
        self.vehicle_id = vehicle_id
        self.actor = actor
        self.world = world
        self.spawn_transform = spawn_transform  # Original spawn location
        self.sensors = {}
        self._rgb_data = None
        self._depth_data = None
        self._route_waypoints = []
        self._current_wp_index = 0

    def attach_rgb_camera(self, config: dict) -> None:
        """Attach RGB camera sensor."""
        bp_library = self.world.get_blueprint_library()
        cam_bp = bp_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(config.get('image_size_x', 1280)))
        cam_bp.set_attribute('image_size_y', str(config.get('image_size_y', 720)))
        cam_bp.set_attribute('fov', str(config.get('fov', 110)))

        pos = config.get('position', [1.5, 0.0, 2.4])
        cam_transform = carla.Transform(carla.Location(x=pos[0], y=pos[1], z=pos[2]))
        camera = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.actor)
        camera.listen(lambda img: self._on_rgb_image(img))
        self.sensors['rgb_camera'] = camera

    def attach_depth_camera(self, config: dict) -> None:
        """Attach depth camera sensor."""
        bp_library = self.world.get_blueprint_library()
        cam_bp = bp_library.find('sensor.camera.depth')
        cam_bp.set_attribute('image_size_x', str(config.get('image_size_x', 1280)))
        cam_bp.set_attribute('image_size_y', str(config.get('image_size_y', 720)))
        cam_bp.set_attribute('fov', str(config.get('fov', 110)))

        pos = config.get('position', [1.5, 0.0, 2.4])
        cam_transform = carla.Transform(carla.Location(x=pos[0], y=pos[1], z=pos[2]))
        camera = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.actor)
        camera.listen(lambda img: self._on_depth_image(img))
        self.sensors['depth_camera'] = camera

    def _on_rgb_image(self, image):
        self._rgb_data = image

    def _on_depth_image(self, image):
        self._depth_data = image

    def get_rgb_image(self) -> Optional[np.ndarray]:
        """Get latest RGB image as numpy array (RGB format)."""
        if self._rgb_data is None:
            return None
        arr = np.frombuffer(self._rgb_data.raw_data, dtype=np.uint8)
        arr = arr.reshape((self._rgb_data.height, self._rgb_data.width, 4))
        return arr[:, :, :3][:, :, ::-1].copy()  # BGRA -> RGB

    def get_depth_data(self):
        """Get latest raw depth sensor data."""
        return self._depth_data

    def get_transform(self) -> carla.Transform:
        return self.actor.get_transform()

    def get_location(self) -> Tuple[float, float, float]:
        loc = self.actor.get_location()
        return (loc.x, loc.y, loc.z)

    def get_yaw(self) -> float:
        return self.actor.get_transform().rotation.yaw

    def get_speed_kmh(self) -> float:
        vel = self.actor.get_velocity()
        speed_ms = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        return speed_ms * 3.6

    def set_route(self, waypoints: List[carla.Waypoint]):
        """Set a route of waypoints to follow."""
        self._route_waypoints = waypoints
        self._current_wp_index = 0

    def get_next_waypoint(self) -> Optional[Tuple[float, float]]:
        """Get the next waypoint on the route as (x, y)."""
        if not self._route_waypoints:
            return None
        if self._current_wp_index >= len(self._route_waypoints):
            self._current_wp_index = 0  # Loop

        wp = self._route_waypoints[self._current_wp_index]
        loc = wp.transform.location
        return (loc.x, loc.y)

    def advance_waypoint(self, threshold: float = 5.0) -> bool:
        """Advance to next waypoint if close enough. Returns True if advanced."""
        if not self._route_waypoints:
            return False

        wp = self._route_waypoints[self._current_wp_index]
        my_loc = self.actor.get_location()
        dist = math.sqrt(
            (wp.transform.location.x - my_loc.x) ** 2 +
            (wp.transform.location.y - my_loc.y) ** 2
        )
        if dist < threshold:
            self._current_wp_index += 1
            if self._current_wp_index >= len(self._route_waypoints):
                self._current_wp_index = 0
            return True
        return False

    def destroy(self):
        """Destroy vehicle and all sensors."""
        for name, sensor in self.sensors.items():
            if sensor.is_listening:
                sensor.stop()
            sensor.destroy()
        self.sensors.clear()
        self.actor.destroy()


class CarlaEnv:
    """
    CARLA Environment manager.
    Handles synchronous mode, vehicle spawning, sensor management,
    traffic generation, and route planning.
    """

    def __init__(self, config: dict):
        """
        Args:
            config: full config dict from config.yaml
        """
        self.config = config
        carla_cfg = config.get('carla', {})

        self.host = carla_cfg.get('host', 'localhost')
        self.port = carla_cfg.get('port', 2000)
        self.timeout = carla_cfg.get('timeout', 10.0)
        self.sync_mode = carla_cfg.get('synchronous_mode', True)
        self.delta_seconds = carla_cfg.get('fixed_delta_seconds', 0.05)
        self.target_map = carla_cfg.get('map', 'Town03')

        self.client = None
        self.world = None
        self.map = None
        self.vehicles: Dict[str, ManagedVehicle] = {}
        self._traffic_actors = []
        self._original_settings = None

    def connect(self):
        """Connect to CARLA server and setup world."""
        print(f"[CARLA] Connecting to {self.host}:{self.port}...")
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)

        # Load target map if different
        self.world = self.client.get_world()
        current_map = self.world.get_map().name.split('/')[-1]
        if current_map != self.target_map:
            print(f"[CARLA] Loading map {self.target_map}...")
            self.world = self.client.load_world(self.target_map)
            time.sleep(2)  # Wait for map to load

        self.map = self.world.get_map()

        # Save original settings
        self._original_settings = self.world.get_settings()

        # Set synchronous mode
        if self.sync_mode:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = self.delta_seconds
            self.world.apply_settings(settings)
            print(f"[CARLA] Synchronous mode enabled (dt={self.delta_seconds}s)")

        print(f"[CARLA] Connected. Map: {self.map.name}")

    def spawn_vehicle(self, vehicle_id: str, blueprint_filter: str = None,
                      spawn_index: int = None) -> ManagedVehicle:
        """
        Spawn a managed vehicle with retry logic for collision avoidance.

        Args:
            vehicle_id: unique ID for this vehicle
            blueprint_filter: vehicle blueprint filter (e.g. 'vehicle.tesla.model3')
            spawn_index: specific spawn point index, or None for random

        Returns:
            ManagedVehicle instance
        """
        bp_library = self.world.get_blueprint_library()

        if blueprint_filter:
            bp_list = bp_library.filter(blueprint_filter)
            bp = random.choice(bp_list) if bp_list else random.choice(bp_library.filter('vehicle'))
        else:
            bp = random.choice(bp_library.filter('vehicle'))

        if bp.has_attribute('color'):
            bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))

        spawn_points = self.map.get_spawn_points()
        
        # Try to spawn with collision detection
        max_retries = 10
        for attempt in range(max_retries):
            if spawn_index is not None and spawn_index < len(spawn_points):
                spawn_point = spawn_points[spawn_index]
            else:
                spawn_point = random.choice(spawn_points)

            try:
                # Try to spawn with collision check
                actor = self.world.try_spawn_actor(bp, spawn_point)
                if actor is not None:
                    managed = ManagedVehicle(vehicle_id, actor, self.world, spawn_transform=spawn_point)
                    self.vehicles[vehicle_id] = managed
                    print(f"[CARLA] Spawned {vehicle_id}: {actor.type_id} at "
                          f"({spawn_point.location.x:.1f}, {spawn_point.location.y:.1f})")
                    return managed
                else:
                    print(f"[CARLA] Spawn attempt {attempt+1}/{max_retries} failed for {vehicle_id}, retrying...")
                    # Tick to update world state
                    if self.sync_mode:
                        self.world.tick()
            except Exception as e:
                print(f"[CARLA] Spawn error on attempt {attempt+1}: {e}")
                if self.sync_mode:
                    self.world.tick()
        
        raise RuntimeError(f"Failed to spawn {vehicle_id} after {max_retries} attempts")

    def generate_route(self, vehicle: ManagedVehicle, end_location=None, 
                       sampling_resolution: float = 2.0) -> List:
        """Generate route using CARLA's GlobalRoutePlanner from vehicle's spawn point."""
        # Add CARLA PythonAPI paths for agents module
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        workspace_root = os.path.dirname(project_root)
        
        carla_api_paths = [
            # From project root
            os.path.join(project_root, 'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI', 'carla'),
            # From workspace root
            os.path.join(workspace_root, 'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI', 'carla'),
            # From current working directory
            os.path.join(os.getcwd(), 'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI', 'carla'),
            # Parent of current working directory
            os.path.join(os.path.dirname(os.getcwd()), 'CARLA_0.9.13', 'WindowsNoEditor', 'PythonAPI', 'carla'),
        ]
        for carla_api_path in carla_api_paths:
            if os.path.exists(carla_api_path) and carla_api_path not in sys.path:
                sys.path.insert(0, carla_api_path)
                break
        from agents.navigation.global_route_planner import GlobalRoutePlanner
        
        spawn_points = self.map.get_spawn_points()
        
        # Use vehicle's original spawn location as route start
        if vehicle.spawn_transform:
            start_loc = vehicle.spawn_transform.location
        else:
            # Fallback: use current location
            start_loc = vehicle.actor.get_location()
        
        # Find destination (furthest spawn point from start)
        if end_location is None:
            end_idx = max(range(len(spawn_points)),
                         key=lambda i: (spawn_points[i].location.x - start_loc.x)**2 +
                                      (spawn_points[i].location.y - start_loc.y)**2)
            end_location = spawn_points[end_idx].location
        
        # Plan route
        grp = GlobalRoutePlanner(self.map, sampling_resolution)
        route_trace = grp.trace_route(start_loc, end_location)
        waypoints = [wp for wp, _ in route_trace]
        
        # Skip first few waypoints if they're too close to start (within 20m)
        # This helps the vehicle get a better initial heading
        skip_count = 0
        for i, wp in enumerate(waypoints):
            dist = math.sqrt((wp.transform.location.x - start_loc.x)**2 + 
                           (wp.transform.location.y - start_loc.y)**2)
            if dist < 20.0 and i < len(waypoints) - 5:
                skip_count = i + 1
            else:
                break
        
        if skip_count > 0:
            waypoints = waypoints[skip_count:]
            print(f"[CARLA] Skipped {skip_count} initial waypoints (too close)")
        
        vehicle.set_route(waypoints)
        print(f"[CARLA] Route: {len(waypoints)} waypoints from "
              f"({start_loc.x:.1f}, {start_loc.y:.1f}) to ({end_location.x:.1f}, {end_location.y:.1f})")
        return waypoints
    
    def _generate_simple_route(self, vehicle: ManagedVehicle, num_waypoints: int = 50,
                                spacing: float = 5.0) -> List:
        """Fallback simple route generation using waypoint.next()"""
        loc = vehicle.actor.get_location()
        current_wp = self.map.get_waypoint(loc, project_to_road=True)
        
        waypoints = [current_wp]
        for _ in range(num_waypoints - 1):
            next_wps = current_wp.next(spacing)
            if next_wps:
                current_wp = random.choice(next_wps)
                waypoints.append(current_wp)
            else:
                break
        
        vehicle.set_route(waypoints)
        print(f"[CARLA] Generated simple route with {len(waypoints)} waypoints (fallback)")
        return waypoints

    def spawn_traffic(self, num_vehicles: int = 20, num_walkers: int = 10):
        """Spawn NPC traffic vehicles and pedestrians."""
        bp_library = self.world.get_blueprint_library()
        spawn_points = self.map.get_spawn_points()

        # Spawn NPC vehicles
        vehicle_bps = bp_library.filter('vehicle')
        random.shuffle(spawn_points)
        count = 0
        for i, sp in enumerate(spawn_points[:num_vehicles]):
            bp = random.choice(vehicle_bps)
            if bp.has_attribute('color'):
                bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
            try:
                actor = self.world.spawn_actor(bp, sp)
                actor.set_autopilot(True)
                self._traffic_actors.append(actor)
                count += 1
            except Exception:
                pass

        print(f"[CARLA] Spawned {count} NPC traffic vehicles")

        # Spawn walkers (pedestrians) - with timeout protection
        walker_count = 0
        if num_walkers > 0:
            try:
                walker_bps = bp_library.filter('walker.pedestrian.*')
                walker_start = time.time()
                walker_timeout = 5.0  # Max 5 seconds for walker spawning

                for _ in range(num_walkers):
                    if time.time() - walker_start > walker_timeout:
                        print(f"[CARLA] Walker spawning timed out after {walker_timeout}s")
                        break

                    bp = random.choice(walker_bps)
                    try:
                        loc = self.world.get_random_location_from_navigation()
                    except Exception:
                        break

                    if loc:
                        spawn_point = carla.Transform(loc)
                        try:
                            walker = self.world.spawn_actor(bp, spawn_point)
                            self._traffic_actors.append(walker)

                            ctrl_bp = bp_library.find('controller.ai.walker')
                            controller = self.world.spawn_actor(ctrl_bp, carla.Transform(), attach_to=walker)
                            controller.start()
                            dest = self.world.get_random_location_from_navigation()
                            if dest:
                                controller.go_to_location(dest)
                            controller.set_max_speed(1.0 + random.random())
                            self._traffic_actors.append(controller)
                            walker_count += 1
                        except Exception:
                            pass
            except Exception as e:
                print(f"[CARLA] Walker spawning error: {e}")

        print(f"[CARLA] Spawned {walker_count} pedestrians")

    def tick(self):
        """Advance simulation by one tick (synchronous mode)."""
        if self.sync_mode:
            self.world.tick()

    def cleanup(self):
        """Destroy all actors and restore settings."""
        print("[CARLA] Cleaning up...")

        # Destroy managed vehicles
        for vid, vehicle in self.vehicles.items():
            try:
                vehicle.destroy()
            except Exception:
                pass
        self.vehicles.clear()

        # Destroy traffic
        for actor in self._traffic_actors:
            try:
                if actor.is_alive:
                    actor.destroy()
            except Exception:
                pass
        self._traffic_actors.clear()

        # Restore original settings
        if self._original_settings and self.world:
            try:
                self.world.apply_settings(self._original_settings)
            except Exception:
                pass

        print("[CARLA] Cleanup complete")
