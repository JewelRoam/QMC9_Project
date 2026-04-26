"""
NexusPilot: Motor Driver Module
Controls the 4WD chassis using differential drive logic through gpiozero.
"""
import time
import sys
import os
from enum import Enum
from typing import Optional

try:
    from gpiozero import Robot, Motor
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    Robot = None
    Motor = None

# Add root directory to sys.path for direct script execution
if __name__ == "__main__" or __package__ is None:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from rpi_deploy.hardware_config import hardware_config
else:
    from .hardware_config import hardware_config


class Direction(Enum):
    """Enumeration of movement directions."""
    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    ROTATE_LEFT = 5
    ROTATE_RIGHT = 6


class MotorController:
    """
    Dual-motor differential drive controller.
    Compatible with LOBOROBOT expansion boards.
    """
    def __init__(self):
        self.config = hardware_config.motor
        self._robot: Optional[Robot] = None
        self._current_direction = Direction.STOP
        self._current_speed = 0.0

        if GPIO_AVAILABLE:
            self._init_gpio()

    def _init_gpio(self):
        """Initializes gpiozero.Robot with pin mapping from config."""
        try:
            self._robot = Robot(
                left=Motor(
                    forward=self.config.left_forward,
                    backward=self.config.left_backward,
                    enable=self.config.left_enable,
                ),
                right=Motor(
                    forward=self.config.right_forward,
                    backward=self.config.right_backward,
                    enable=self.config.right_enable,
                ),
            )
        except Exception as e:
            print(f"[Motor] Init Error: {e}")
            self._robot = None

    def move_forward(self, speed: float = 0.3):
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.FORWARD
        self._current_speed = speed
        if self._robot: self._robot.forward(speed)

    def move_backward(self, speed: float = 0.3):
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.BACKWARD
        self._current_speed = speed
        if self._robot: self._robot.backward(speed)

    def turn_left(self, speed: float = 0.3):
        """Turn left (forward-left differential steering)."""
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.LEFT
        self._current_speed = speed
        if self._robot:
            self._robot.left_motor.forward(speed * 0.3)
            self._robot.right_motor.forward(speed)

    def turn_right(self, speed: float = 0.3):
        """Turn right (forward-right differential steering)."""
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.RIGHT
        self._current_speed = speed
        if self._robot:
            self._robot.left_motor.forward(speed)
            self._robot.right_motor.forward(speed * 0.3)

    def rotate_left(self, speed: float = 0.3):
        """Spin counter-clockwise in place (left motor backward, right motor forward)."""
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.ROTATE_LEFT
        self._current_speed = speed
        if self._robot:
            self._robot.left_motor.backward(speed)
            self._robot.right_motor.forward(speed)

    def rotate_right(self, speed: float = 0.3):
        """Spin clockwise in place (left motor forward, right motor backward)."""
        speed = max(0.0, min(1.0, speed))
        self._current_direction = Direction.ROTATE_RIGHT
        self._current_speed = speed
        if self._robot:
            self._robot.left_motor.forward(speed)
            self._robot.right_motor.backward(speed)

    def get_status(self) -> dict:
        """Return current motor controller status."""
        return {
            'direction': self._current_direction.name,
            'speed': self._current_speed,
            'gpio_available': GPIO_AVAILABLE and self._robot is not None,
        }

    def curve_move(self, linear_speed: float, angular_rate: float):
        """
        Executes combined linear and angular motion.
        Used for smooth APF trajectory following.
        """
        turn_factor = 0.5
        left_speed = max(-1.0, min(1.0, linear_speed + angular_rate * turn_factor))
        right_speed = max(-1.0, min(1.0, linear_speed - angular_rate * turn_factor))

        if self._robot:
            self._apply_motor_speed(self._robot.left_motor, left_speed)
            self._apply_motor_speed(self._robot.right_motor, right_speed)

    @staticmethod
    def _apply_motor_speed(motor: "Motor", speed: float):
        """Set a single motor speed without pin conflict. Only calls forward() or backward(), never both."""
        if speed > 0:
            motor.forward(speed)
        elif speed < 0:
            motor.backward(-speed)
        else:
            motor.stop()

    def stop(self):
        self._current_direction = Direction.STOP
        self._current_speed = 0.0
        if self._robot: self._robot.stop()

    def emergency_stop(self):
        self.stop()

    def test_sequence(self):
        """Basic hardware verification sequence."""
        print("[Motor] Starting test sequence...")
        moves = [("FWD", self.move_forward), ("BWD", self.move_backward)]
        for name, func in moves:
            print(f"  Action: {name}")
            func(0.3)
            time.sleep(1.0)
            self.stop()
            time.sleep(0.5)

    def cleanup(self):
        self.stop()
        if self._robot:
            self._robot.close()
            self._robot = None

def create_motor_controller() -> MotorController:
    return MotorController()

if __name__ == "__main__":
    controller = create_motor_controller()
    try:
        controller.test_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
