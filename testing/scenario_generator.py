"""
Automated Scenario Generator for CARLA.
Generates diverse test scenarios programmatically:
  - Intersection scenarios (with/without traffic)
  - Obstacle arrangements (static/dynamic)
  - Congestion levels (light/medium/heavy)
  - Pedestrian crossings
  - Lane change scenarios

Usage:
  from testing.scenario_generator import ScenarioGenerator
  gen = ScenarioGenerator(env)
  scenario = gen.generate_intersection_scenario(difficulty='medium')
"""
import random
import math
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class ScenarioConfig:
    """Configuration for a test scenario."""
    name: str
    description: str
    spawn_points: List[Tuple[float, float, float]]  # x, y, yaw
    obstacle_positions: List[Dict] = field(default_factory=list)
    npc_vehicles: int = 0
    pedestrians: int = 0
    weather: str = "ClearNoon"
    duration_seconds: float = 60.0
    success_criteria: Dict = field(default_factory=dict)


class ScenarioGenerator:
    """
    Generates predefined and randomized test scenarios for autonomous driving evaluation.
    """

    def __init__(self, carla_env):
        self.env = carla_env
        self.world = carla_env.world
        self.map = carla_env.map

    # ==================== Predefined Scenarios ====================

    def generate_straight_road_scenario(self, difficulty: str = "easy") -> ScenarioConfig:
        """
        Simple straight road with optional obstacles.
        Good for basic lane keeping and speed control testing.
        """
        spawn_points = self.map.get_spawn_points()
        # Find a straight road spawn point
        ego_spawn = random.choice(spawn_points[:10])  # First 10 are usually on straight roads

        obstacles = []
        npc_count = 0

        if difficulty == "medium":
            # Add a static obstacle ahead
            obstacles.append({
                'type': 'vehicle',
                'position': (ego_spawn.location.x + 30, ego_spawn.location.y, 0),
                'static': True
            })
            npc_count = 2
        elif difficulty == "hard":
            # Multiple obstacles and moving NPCs
            for i in range(3):
                obstacles.append({
                    'type': 'vehicle',
                    'position': (ego_spawn.location.x + 20 + i*15, ego_spawn.location.y + random.uniform(-2, 2), 0),
                    'static': random.choice([True, False])
                })
            npc_count = 5

        return ScenarioConfig(
            name=f"straight_road_{difficulty}",
            description=f"Straight road scenario with {difficulty} difficulty",
            spawn_points=[(ego_spawn.location.x, ego_spawn.location.y, ego_spawn.rotation.yaw)],
            obstacle_positions=obstacles,
            npc_vehicles=npc_count,
            pedestrians=0,
            success_criteria={
                'max_lane_deviation': 1.0,  # meters
                'min_speed': 10.0,  # km/h average
                'no_collision': True
            }
        )

    def generate_intersection_scenario(self, difficulty: str = "medium") -> ScenarioConfig:
        """
        Intersection with crossing traffic.
        Tests right-of-way handling and collision avoidance.
        """
        # Town03 has good intersections
        # Use specific spawn points known to be near intersections
        intersection_spawns = [
            (88.6, -12.5, 90),   # Main intersection
            (44.3, -42.1, 0),    # Another intersection
            (-7.5, -65.2, 180),  # Third intersection
        ]

        ego_pos = random.choice(intersection_spawns)

        obstacles = []
        npc_count = 0

        if difficulty == "easy":
            # Single crossing vehicle with clear right-of-way
            obstacles.append({
                'type': 'vehicle',
                'position': (ego_pos[0] + 20, ego_pos[1] - 10, 90),
                'velocity': (0, 5, 0),  # Crossing from right
                'crossing': True
            })
            npc_count = 1

        elif difficulty == "medium":
            # Multiple crossing vehicles, need to negotiate
            obstacles.append({
                'type': 'vehicle',
                'position': (ego_pos[0] + 15, ego_pos[1] - 8, 90),
                'velocity': (0, 8, 0),
                'crossing': True
            })
            obstacles.append({
                'type': 'vehicle',
                'position': (ego_pos[0] - 15, ego_pos[1] + 8, 270),
                'velocity': (0, -6, 0),
                'crossing': True
            })
            npc_count = 3

        elif difficulty == "hard":
            # Dense intersection with pedestrians
            for i in range(4):
                angle = i * 90
                rad = math.radians(angle)
                obstacles.append({
                    'type': 'vehicle',
                    'position': (
                        ego_pos[0] + 15 * math.cos(rad),
                        ego_pos[1] + 15 * math.sin(rad),
                        angle + 90
                    ),
                    'velocity': (5 * math.cos(rad + math.pi/2), 5 * math.sin(rad + math.pi/2), 0),
                    'crossing': True
                })
            npc_count = 6

        return ScenarioConfig(
            name=f"intersection_{difficulty}",
            description=f"Intersection scenario with crossing traffic ({difficulty})",
            spawn_points=[ego_pos],
            obstacle_positions=obstacles,
            npc_vehicles=npc_count,
            pedestrians=2 if difficulty == "hard" else 0,
            success_criteria={
                'no_collision': True,
                'max_stop_time': 10.0,  # Don't get stuck too long
                'proper_right_of_way': True
            }
        )

    def generate_occlusion_scenario(self) -> ScenarioConfig:
        """
        Scenario where an obstacle is hidden behind another vehicle.
        Tests V2V cooperative perception (occlusion compensation).
        """
        spawn_points = self.map.get_spawn_points()
        ego_spawn = random.choice(spawn_points[:20])

        # Create a line of vehicles where the last one is hidden
        obstacles = [
            {
                'type': 'vehicle',
                'position': (ego_spawn.location.x + 25, ego_spawn.location.y, 0),
                'static': False,
                'velocity': (8, 0, 0),
                'role': 'blocker'  # This blocks view of the next vehicle
            },
            {
                'type': 'vehicle',
                'position': (ego_spawn.location.x + 45, ego_spawn.location.y, 0),
                'static': False,
                'velocity': (0, 0, 0),  # Stopped - hidden behind blocker
                'role': 'hidden_obstacle'
            },
        ]

        return ScenarioConfig(
            name="occlusion_v2v_test",
            description="Occlusion scenario - hidden obstacle behind leading vehicle",
            spawn_points=[(ego_spawn.location.x, ego_spawn.location.y, ego_spawn.rotation.yaw)],
            obstacle_positions=obstacles,
            npc_vehicles=2,
            pedestrians=0,
            success_criteria={
                'no_collision': True,
                'detect_hidden_obstacle': True,  # Need V2V to detect this
                'safe_braking_distance': 10.0
            }
        )

    def generate_pedestrian_crossing_scenario(self, density: str = "low") -> ScenarioConfig:
        """
        Pedestrian crossing scenario with VRU priority testing.
        """
        spawn_points = self.map.get_spawn_points()
        ego_spawn = random.choice(spawn_points[:15])

        ped_count = {"low": 1, "medium": 3, "high": 6}.get(density, 1)

        obstacles = []
        for i in range(ped_count):
            obstacles.append({
                'type': 'pedestrian',
                'position': (
                    ego_spawn.location.x + 20 + i * 3,
                    ego_spawn.location.y + random.uniform(-5, 5),
                    0
                ),
                'crossing_direction': 'left_to_right',
                'speed': random.uniform(1.0, 1.5)
            })

        return ScenarioConfig(
            name=f"pedestrian_crossing_{density}",
            description=f"Pedestrian crossing with {density} density",
            spawn_points=[(ego_spawn.location.x, ego_spawn.location.y, ego_spawn.rotation.yaw)],
            obstacle_positions=obstacles,
            npc_vehicles=0,
            pedestrians=ped_count,
            success_criteria={
                'no_collision': True,
                'pedestrian_priority_respected': True,
                'yield_distance': 5.0  # Stop at least 5m before pedestrian
            }
        )

    def generate_randomized_scenario(self, seed: Optional[int] = None) -> ScenarioConfig:
        """
        Generate a completely random scenario for robustness testing.
        """
        if seed is not None:
            random.seed(seed)

        spawn_points = self.map.get_spawn_points()
        ego_spawn = random.choice(spawn_points)

        # Random number of obstacles
        num_obstacles = random.randint(0, 5)
        obstacles = []

        for _ in range(num_obstacles):
            obs_type = random.choice(['vehicle', 'pedestrian'])
            offset_x = random.uniform(10, 50)
            offset_y = random.uniform(-5, 5)

            obstacles.append({
                'type': obs_type,
                'position': (
                    ego_spawn.location.x + offset_x,
                    ego_spawn.location.y + offset_y,
                    random.uniform(0, 360)
                ),
                'static': random.choice([True, False]),
                'velocity': (random.uniform(0, 10), 0, 0) if not random.choice([True, False]) else (0, 0, 0)
            })

        return ScenarioConfig(
            name=f"random_seed_{seed or 'auto'}",
            description="Randomly generated scenario",
            spawn_points=[(ego_spawn.location.x, ego_spawn.location.y, ego_spawn.rotation.yaw)],
            obstacle_positions=obstacles,
            npc_vehicles=random.randint(0, 8),
            pedestrians=random.randint(0, 4),
            weather=random.choice(["ClearNoon", "CloudyNoon", "WetNoon"]),
            success_criteria={'no_collision': True}
        )

    # ==================== Batch Generation ====================

    def generate_test_suite(self, suite_name: str = "full") -> List[ScenarioConfig]:
        """
        Generate a complete test suite with multiple scenarios.

        Args:
            suite_name: "quick" (5 scenarios), "standard" (15), or "full" (30)
        """
        scenarios = []

        # Always include basic scenarios
        scenarios.extend([
            self.generate_straight_road_scenario("easy"),
            self.generate_straight_road_scenario("medium"),
            self.generate_intersection_scenario("easy"),
            self.generate_intersection_scenario("medium"),
            self.generate_pedestrian_crossing_scenario("low"),
        ])

        if suite_name in ["standard", "full"]:
            scenarios.extend([
                self.generate_straight_road_scenario("hard"),
                self.generate_intersection_scenario("hard"),
                self.generate_occlusion_scenario(),
                self.generate_pedestrian_crossing_scenario("medium"),
                self.generate_pedestrian_crossing_scenario("high"),
            ])

        if suite_name == "full":
            # Add randomized scenarios
            for i in range(10):
                scenarios.append(self.generate_randomized_scenario(seed=1000 + i))

        return scenarios

    def save_scenario(self, scenario: ScenarioConfig, filepath: str):
        """Save scenario config to JSON file."""
        import json
        with open(filepath, 'w') as f:
            json.dump(scenario.__dict__, f, indent=2)

    @staticmethod
    def load_scenario(filepath: str) -> ScenarioConfig:
        """Load scenario config from JSON file."""
        import json
        with open(filepath, 'r') as f:
            data = json.load(f)
        return ScenarioConfig(**data)
