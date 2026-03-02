"""
Simplified QMC9 ROS 2 Simulation Runner
Runs all nodes without requiring full ROS 2 package structure.

Usage:
    # Terminal 1: Start CARLA
    cd CARLA_0.9.13 && ./CarlaUE4.sh
    
    # Terminal 2: Run simulation stack
    python ros2_nodes/run_simulation.py --mode simulation
    
    # Or run algorithm-only mode (no CARLA)
    python ros2_nodes/run_simulation.py --mode algorithm_only
"""
import sys
import os
import argparse
import signal
import threading
from typing import List, Optional

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

# Check ROS 2 availability
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    ROS2_AVAILABLE = True
except ImportError:
    print("[ERROR] ROS 2 not available. Please source ROS 2 environment.")
    print("  Example: source /opt/ros/humble/setup.bash")
    ROS2_AVAILABLE = False
    sys.exit(1)

# Import nodes
try:
    from ros2_nodes.carla_ros_interface import CarlaRosInterface
    CARLA_NODE_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] CarlaRosInterface not available: {e}")
    CARLA_NODE_AVAILABLE = False

try:
    from ros2_nodes.perception_node import PerceptionNode
    PERCEPTION_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] PerceptionNode not available: {e}")
    PERCEPTION_AVAILABLE = False

try:
    from ros2_nodes.planning_node import PlanningNode
    PLANNING_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] PlanningNode not available: {e}")
    PLANNING_AVAILABLE = False

try:
    from ros2_nodes.control_node import ControlNode
    CONTROL_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] ControlNode not available: {e}")
    CONTROL_AVAILABLE = False

try:
    from ros2_nodes.cooperation_node import CooperationNode
    COOPERATION_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] CooperationNode not available: {e}")
    COOPERATION_AVAILABLE = False


class SimulationRunner:
    """Manages the lifecycle of all ROS 2 nodes."""
    
    def __init__(self):
        self.nodes: List = []
        self.executor: Optional[MultiThreadedExecutor] = None
        self.running = False
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        print(f"\n[INFO] Received signal {signum}, shutting down...")
        self.stop()
    
    def start_simulation_mode(self, carla_host='localhost', carla_port=2000, 
                              enable_cooperation=True):
        """Start full simulation stack with CARLA."""
        print("=" * 60)
        print("Starting QMC9 Simulation Stack (with CARLA)")
        print("=" * 60)
        
        if not CARLA_NODE_AVAILABLE:
            print("[ERROR] CarlaRosInterface not available, cannot run simulation mode")
            return False
        
        # Initialize ROS 2
        rclpy.init()
        
        # Create executor
        self.executor = MultiThreadedExecutor(num_threads=8)
        
        # Add CARLA interface node
        try:
            carla_node = CarlaRosInterface()
            self.executor.add_node(carla_node)
            self.nodes.append(carla_node)
            print("[OK] CarlaRosInterface started")
        except Exception as e:
            print(f"[ERROR] Failed to start CarlaRosInterface: {e}")
            return False
        
        # Add algorithm nodes
        self._add_algorithm_nodes(enable_cooperation)
        
        self.running = True
        return True
    
    def start_algorithm_only_mode(self, enable_cooperation=True):
        """Start only algorithm nodes (for testing without CARLA)."""
        print("=" * 60)
        print("Starting QMC9 Algorithm Stack (CARLA-free)")
        print("=" * 60)
        
        # Initialize ROS 2
        rclpy.init()
        
        # Create executor
        self.executor = MultiThreadedExecutor(num_threads=4)
        
        # Add algorithm nodes only
        self._add_algorithm_nodes(enable_cooperation)
        
        print("\n[NOTE] Algorithm-only mode requires sensor data from:")
        print("  - ROS 2 bag file: ros2 bag play <bag_file>")
        print("  - Mock publisher: ros2 topic pub /sensors/camera/rgb/image_raw ...")
        
        self.running = True
        return True
    
    def _add_algorithm_nodes(self, enable_cooperation):
        """Add perception, planning, and control nodes."""
        
        # Add perception node
        if PERCEPTION_AVAILABLE:
            try:
                perception_node = PerceptionNode()
                self.executor.add_node(perception_node)
                self.nodes.append(perception_node)
                print("[OK] PerceptionNode started")
            except Exception as e:
                print(f"[WARNING] Failed to start PerceptionNode: {e}")
        
        # Add planning node
        if PLANNING_AVAILABLE:
            try:
                planning_node = PlanningNode()
                self.executor.add_node(planning_node)
                self.nodes.append(planning_node)
                print("[OK] PlanningNode started")
            except Exception as e:
                print(f"[WARNING] Failed to start PlanningNode: {e}")
        
        # Add control node
        if CONTROL_AVAILABLE:
            try:
                control_node = ControlNode()
                self.executor.add_node(control_node)
                self.nodes.append(control_node)
                print("[OK] ControlNode started")
            except Exception as e:
                print(f"[WARNING] Failed to start ControlNode: {e}")
        
        # Add cooperation node
        if enable_cooperation and COOPERATION_AVAILABLE:
            try:
                cooperation_node = CooperationNode()
                self.executor.add_node(cooperation_node)
                self.nodes.append(cooperation_node)
                print("[OK] CooperationNode started")
            except Exception as e:
                print(f"[WARNING] Failed to start CooperationNode: {e}")
    
    def run(self):
        """Run the simulation until stopped."""
        if not self.running or self.executor is None:
            print("[ERROR] Not initialized, call start_*_mode first")
            return
        
        print("\n" + "=" * 60)
        print("Simulation running. Press Ctrl+C to stop.")
        print("=" * 60 + "\n")
        
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            print("\n[INFO] Keyboard interrupt received")
        finally:
            self.stop()
    
    def stop(self):
        """Stop all nodes and cleanup."""
        if not self.running:
            return
        
        print("\n[INFO] Stopping all nodes...")
        
        # Stop executor
        if self.executor:
            self.executor.shutdown()
        
        # Destroy nodes
        for node in self.nodes:
            try:
                node.destroy_node()
                print(f"[OK] {node.get_name()} stopped")
            except Exception as e:
                print(f"[WARNING] Error stopping {node}: {e}")
        
        # Shutdown ROS 2
        if rclpy.ok():
            rclpy.shutdown()
        
        self.running = False
        print("[OK] All nodes stopped")


def main():
    parser = argparse.ArgumentParser(
        description='QMC9 ROS 2 Simulation Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Full simulation with CARLA
  python run_simulation.py --mode simulation

  # Algorithm-only mode (for testing)
  python run_simulation.py --mode algorithm_only

  # Disable multi-vehicle cooperation
  python run_simulation.py --mode simulation --no-cooperation

  # Connect to remote CARLA server
  python run_simulation.py --mode simulation --carla-host 192.168.1.100
        """
    )
    
    parser.add_argument(
        '--mode', '-m',
        choices=['simulation', 'algorithm_only'],
        default='simulation',
        help='Running mode (default: simulation)'
    )
    
    parser.add_argument(
        '--carla-host',
        default='localhost',
        help='CARLA server host (default: localhost)'
    )
    
    parser.add_argument(
        '--carla-port',
        type=int,
        default=2000,
        help='CARLA server port (default: 2000)'
    )
    
    parser.add_argument(
        '--no-cooperation',
        action='store_true',
        help='Disable multi-vehicle cooperation'
    )
    
    args = parser.parse_args()
    
    # Create runner
    runner = SimulationRunner()
    
    # Start based on mode
    success = False
    if args.mode == 'simulation':
        success = runner.start_simulation_mode(
            carla_host=args.carla_host,
            carla_port=args.carla_port,
            enable_cooperation=not args.no_cooperation
        )
    else:
        success = runner.start_algorithm_only_mode(
            enable_cooperation=not args.no_cooperation
        )
    
    if success:
        runner.run()
    else:
        print("[ERROR] Failed to start simulation")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
