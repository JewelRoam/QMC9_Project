# Testing Module

This directory contains test scripts and utilities for the autonomous vehicle project.

## Test Scripts

### Unit Tests

- **`test_distance_coordination.py`** - Tests the distance-aware coordination system
  ```bash
  python -m testing.test_distance_coordination
  ```
  Tests coordination zones (inactive/monitoring/active/warning/critical) and TTI-based priority.

### CARLA Simulation Tests

- **`test_intersection_scenario.py`** - Tests intersection coordination in CARLA
  ```bash
  # First start CARLA server
  .\CARLA_0.9.13\WindowsNoEditor\CarlaUE4.exe
  
  # Then run test
  python -m testing.test_intersection_scenario
  ```
  Creates a controlled intersection scenario with two vehicles approaching from perpendicular directions.

### Existing Test Tools

- **`scenario_generator.py`** - Generates test scenarios
- **`metrics_collector.py`** - Collects performance metrics
- **`parameter_tuner.py`** - APF parameter tuning
- **`automated_runner.py`** - Automated test execution
- **`report_generator.py`** - Generate test reports

## Running Tests

### Quick Unit Test (No CARLA required)
```bash
python -m testing.test_distance_coordination
```

### Full Integration Test (Requires CARLA)
```bash
# Terminal 1: Start CARLA
.\CARLA_0.9.13\WindowsNoEditor\CarlaUE4.exe

# Terminal 2: Run multi-vehicle demo
python -m simulation.multi_vehicle_demo

# Or run intersection test
python -m testing.test_intersection_scenario
```

## Test Output

Test results are saved to:
- `output/multi_vehicle_metrics.json` - Performance metrics
- `output/test_report.html` - HTML report
- Console output - Real-time status

## Adding New Tests

1. Create a new file in `testing/` directory
2. Import necessary modules from parent directory
3. Use the existing test scripts as templates
4. Update this README with usage instructions
