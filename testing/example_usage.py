"""
Example Usage of the Testing Framework.

This script demonstrates how to use the automated testing framework
to evaluate and tune your autonomous driving system.

Run this after starting CARLA server:
    CARLA_0.9.13/WindowsNoEditor/CarlaUE4.exe

Then:
    python testing/example_usage.py
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from testing.automated_runner import AutomatedTestRunner
from testing.report_generator import ReportGenerator


def example_1_quick_test():
    """
    Example 1: Run a quick test suite to verify basic functionality.
    This runs 5 scenarios and generates a report.
    """
    print("="*70)
    print("EXAMPLE 1: Quick Test Suite")
    print("="*70)

    runner = AutomatedTestRunner("config/config.yaml")

    try:
        runner.connect()

        # Run quick test suite (5 scenarios)
        results = runner.run_test_suite("quick")

        # Generate report
        runner.generate_report("output/example1_quick")

        # Print summary
        passed = sum(1 for r in results if r.passed)
        print(f"\nResults: {passed}/{len(results)} scenarios passed")

    finally:
        runner.disconnect()


def example_2_parameter_tuning():
    """
    Example 2: Auto-tune APF parameters.
    This tries different parameter combinations and finds the best one.
    """
    print("\n" + "="*70)
    print("EXAMPLE 2: Parameter Tuning")
    print("="*70)

    runner = AutomatedTestRunner("config/config.yaml")

    try:
        runner.connect()

        # Tune parameters with 15 iterations
        best_config = runner.tune_parameters(iterations=15)

        print("\nBest configuration found:")
        print(f"  k_attractive: {best_config.k_attractive:.2f}")
        print(f"  k_repulsive: {best_config.k_repulsive:.2f}")
        print(f"  d0: {best_config.d0:.2f}")
        print(f"  max_speed: {best_config.max_speed:.1f}")

        # Save best config to file
        import json
        with open("output/best_apf_config.json", "w") as f:
            json.dump(best_config.to_planner_config(), f, indent=2)
        print("\nSaved to: output/best_apf_config.json")

    finally:
        runner.disconnect()


def example_3_compare_configs():
    """
    Example 3: Compare two different configurations.
    Useful for A/B testing algorithm changes.
    """
    print("\n" + "="*70)
    print("EXAMPLE 3: Compare Configurations")
    print("="*70)

    runner = AutomatedTestRunner("config/config.yaml")

    # Configuration A: Conservative (high safety margin)
    config_a = {
        "k_attractive": 1.0,
        "k_repulsive": 200.0,
        "d0": 20.0,
        "max_speed": 25.0,
        "emergency_distance": 4.0,
    }

    # Configuration B: Aggressive (higher speed)
    config_b = {
        "k_attractive": 1.5,
        "k_repulsive": 100.0,
        "d0": 12.0,
        "max_speed": 35.0,
        "emergency_distance": 2.5,
    }

    try:
        runner.connect()

        # Run tests with config A
        print("\nTesting Configuration A (Conservative)...")
        results_a = runner.run_test_suite("quick", planner_config=config_a)

        # Clear results and run with config B
        runner.results = []
        print("\nTesting Configuration B (Aggressive)...")
        results_b = runner.run_test_suite("quick", planner_config=config_b)

        # Compare
        print("\n" + "-"*70)
        print("COMPARISON RESULTS")
        print("-"*70)

        avg_collisions_a = sum(r.collision_count for r in results_a) / len(results_a)
        avg_collisions_b = sum(r.collision_count for r in results_b) / len(results_b)

        avg_speed_a = sum(r.average_speed for r in results_a) / len(results_a)
        avg_speed_b = sum(r.average_speed for r in results_b) / len(results_b)

        print(f"{'Metric':<25} {'Config A':>15} {'Config B':>15}")
        print("-"*60)
        print(f"{'Avg Collisions':<25} {avg_collisions_a:>15.2f} {avg_collisions_b:>15.2f}")
        print(f"{'Avg Speed (km/h)':<25} {avg_speed_a:>15.1f} {avg_speed_b:>15.1f}")
        print(f"{'Passed Scenarios':<25} {sum(r.passed for r in results_a):>15} {sum(r.passed for r in results_b):>15}")

        # Generate comparison report
        gen = ReportGenerator(title="Configuration Comparison: A vs B")
        for r in results_a:
            gen.add_run(r, "Config A (Conservative)")
        for r in results_b:
            gen.add_run(r, "Config B (Aggressive)")
        gen.generate_html("output/comparison_report.html")

    finally:
        runner.disconnect()


def example_4_single_scenario_debug():
    """
    Example 4: Run a single scenario for debugging.
    Useful when you need to investigate a specific situation.
    """
    print("\n" + "="*70)
    print("EXAMPLE 4: Single Scenario Debug")
    print("="*70)

    from testing.scenario_generator import ScenarioGenerator

    runner = AutomatedTestRunner("config/config.yaml")

    try:
        runner.connect()

        # Create a specific scenario
        gen = ScenarioGenerator(runner.env)
        scenario = gen.generate_intersection_scenario(difficulty="medium")

        print(f"Running scenario: {scenario.name}")
        print(f"Description: {scenario.description}")

        # Run just this scenario
        result = runner.run_scenario(scenario, duration_seconds=30.0)

        print("\nResults:")
        print(f"  Passed: {result.passed}")
        print(f"  Collisions: {result.collision_count}")
        print(f"  Near Misses: {result.near_misses}")
        print(f"  Avg Speed: {result.average_speed:.1f} km/h")
        print(f"  Min Obstacle Distance: {result.min_distance_to_obstacle:.2f} m")

        if result.failure_reasons:
            print("\nFailure reasons:")
            for reason in result.failure_reasons:
                print(f"  - {reason}")

    finally:
        runner.disconnect()


def main():
    """Run all examples."""
    print("\n" + "="*70)
    print("AUTONOMOUS DRIVING TESTING FRAMEWORK - EXAMPLES")
    print("="*70)
    print("\nMake sure CARLA server is running before executing!")
    print("Command: CARLA_0.9.13/WindowsNoEditor/CarlaUE4.exe")
    print("\nPress Ctrl+C at any time to stop.")
    print("="*70)

    import argparse
    parser = argparse.ArgumentParser(description="Testing Framework Examples")
    parser.add_argument("--example", type=int, choices=[1, 2, 3, 4], default=1,
                       help="Which example to run (1-4)")
    args = parser.parse_args()

    examples = {
        1: example_1_quick_test,
        2: example_2_parameter_tuning,
        3: example_3_compare_configs,
        4: example_4_single_scenario_debug,
    }

    try:
        examples[args.example]()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "="*70)
    print("Done!")
    print("="*70)


if __name__ == '__main__':
    main()
