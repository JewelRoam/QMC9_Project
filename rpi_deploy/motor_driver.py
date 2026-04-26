"""
NexusPilot: Robust Motor Driver for RPi 5
Explicitly manages L298N Enable pins (GPIO 18/23) for LOBOROBOT board.
"""
import time
import sys
import os
from enum import Enum
from typing import Optional

try:
    from gpiozero import PWMOutputDevice, DigitalOutputDevice
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

# Force RPi 5 GPIO chip index
os.environ["LG_CHIP"] = "0"

# Path setup
if __name__ == "__main__" or __package__ is None:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from rpi_deploy.hardware_config import hardware_config
else:
    from .hardware_config import hardware_config

class MotorController:
    """
    Direct L298N Driver for 4WD Chassis.
    Explicitly drives Forward, Backward, and PWM Enable pins.
    """
    def __init__(self):
        self.config = hardware_config.motor
        self._initialized = False
        
        if GPIO_AVAILABLE:
            try:
                # Left Motor Pins (GPIO 22, 27, 18)
                self.left_fwd = DigitalOutputDevice(self.config.left_forward)
                self.left_bwd = DigitalOutputDevice(self.config.left_backward)
                self.left_en  = PWMOutputDevice(self.config.left_enable)
                
                # Right Motor Pins (GPIO 25, 24, 23)
                self.right_fwd = DigitalOutputDevice(self.config.right_forward)
                self.right_bwd = DigitalOutputDevice(self.config.right_backward)
                self.right_en  = PWMOutputDevice(self.config.right_enable)
                
                self._initialized = True
                print(f"[Motor] L298N initialized. Pins: L({self.config.left_forward},{self.config.left_backward},{self.config.left_enable}) R({self.config.right_forward},{self.config.right_backward},{self.config.right_enable})")
            except Exception as e:
                print(f"[Motor] Initialization Failed: {e}")

    def _drive(self, left_speed: float, right_speed: float):
        """Internal raw PWM driving logic."""
        if not self._initialized: return

        # Left Motor Control
        self.left_fwd.value = 1 if left_speed > 0 else 0
        self.left_bwd.value = 1 if left_speed < 0 else 0
        self.left_en.value = abs(left_speed)

        # Right Motor Control
        self.right_fwd.value = 1 if right_speed > 0 else 0
        self.right_bwd.value = 1 if right_speed < 0 else 0
        self.right_en.value = abs(right_speed)

    def curve_move(self, linear_speed: float, angular_rate: float):
        """
        Combined motion for APF steering.
        linear_speed: 0 to 1.0
        angular_rate: -1.0 (left) to 1.0 (right)
        """
        turn_gain = 0.6
        left_speed = linear_speed + (angular_rate * turn_gain)
        right_speed = linear_speed - (angular_rate * turn_gain)
        
        # Normalize to keep speed in range
        max_v = max(abs(left_speed), abs(right_speed), 1.0)
        self._drive(left_speed / max_v * abs(linear_speed), 
                   right_speed / max_v * abs(linear_speed))

    def stop(self):
        self._drive(0, 0)

    def emergency_stop(self):
        self._drive(0, 0)

    def move_backward(self, speed: float):
        self._drive(-speed, -speed)

    def cleanup(self):
        self.stop()
        if self._initialized:
            self.left_fwd.close()
            self.left_bwd.close()
            self.left_en.close()
            self.right_fwd.close()
            self.right_bwd.close()
            self.right_en.close()

if __name__ == "__main__":
    ctrl = MotorController()
    print("Testing hardware drive...")
    ctrl._drive(0.4, 0.4)
    time.sleep(1.0)
    ctrl.stop()
    ctrl.cleanup()
