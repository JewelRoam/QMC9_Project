"""
CARLA Bridge for Python 3.7
Bridges CARLA simulator to ZeroMQ for cross-environment communication.
"""

from .carla_zmq_bridge import CarlaZmqBridge, main

__all__ = ['CarlaZmqBridge', 'main']
