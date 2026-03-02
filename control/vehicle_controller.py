"""
Vehicle Controller - converts APF planner output to vehicle control commands.
Supports:
  - CARLA simulation (throttle/steer/brake via carla.VehicleControl)
  - Raspberry Pi (PWM motor commands via abstraction layer)
  - Abstract interface for future STM32/Micro-ROS integration
"""
import time
import math
import numpy as np
from typing import Optional, Tuple


class PIDController:
    """Simple PID controller for speed and steering regulation."""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1,
                 output_min: float = -1.0, output_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, setpoint: float, measurement: float) -> float:
        """Compute PID output."""
        now = time.perf_counter()
        error = setpoint - measurement

        if self._prev_time is None:
            dt = 0.05  # default 50ms
        else:
            dt = now - self._prev_time
            dt = max(dt, 0.001)  # prevent division by zero

        self._integral += error * dt
        # Anti-windup
        self._integral = np.clip(self._integral, self.output_min * 10, self.output_max * 10)

        derivative = (error - self._prev_error) / dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = np.clip(output, self.output_min, self.output_max)

        self._prev_error = error
        self._prev_time = now

        return float(output)

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None


class VehicleController:
    """
    Unified vehicle controller that translates planner output
    to platform-specific commands (CARLA / RPi / STM32).
    """

    def __init__(self, config: dict, platform: str = "carla"):
        """
        Args:
            config: control section of config.yaml
            platform: "carla", "raspberry_pi", or "stm32"
        """
        self.platform = platform
        self.config = config

        pid_cfg = config.get("pid", {})
        steer_cfg = pid_cfg.get("steering", {})
        throttle_cfg = pid_cfg.get("throttle", {})

        self.steering_pid = PIDController(
            kp=steer_cfg.get("kp", 1.0),
            ki=steer_cfg.get("ki", 0.0),
            kd=steer_cfg.get("kd", 0.1),
            output_min=-1.0, output_max=1.0,
        )
        self.speed_pid = PIDController(
            kp=throttle_cfg.get("kp", 0.8),
            ki=throttle_cfg.get("ki", 0.05),
            kd=throttle_cfg.get("kd", 0.01),
            output_min=-1.0, output_max=1.0,
        )

        self.max_steering_angle = config.get("max_steering_angle", 70.0)
        self.emergency_brake_distance = config.get("emergency_brake_distance", 3.0)

        # State tracking
        self._last_control = None
        self._emergency_active = False

    def compute_control(self, planner_output, current_speed_kmh: float,
                        current_steering: float = 0.0) -> dict:
        """
        Compute control commands from planner output.

        Args:
            planner_output: PlannerOutput from APF planner
            current_speed_kmh: current vehicle speed in km/h
            current_steering: current steering angle (normalized -1 to 1)

        Returns:
            dict with control values:
                For CARLA: {"throttle", "steer", "brake", "hand_brake", "reverse"}
                For RPi:   {"left_pwm", "right_pwm", "direction"}
        """
        if planner_output.emergency_brake:
            return self._emergency_brake()

        self._emergency_active = False

        if self.platform == "carla":
            target_speed = planner_output.target_speed

            # Handle reverse driving (recovery mode)
            if target_speed < 0:
                # Reverse driving
                speed_error = abs(target_speed) - abs(current_speed_kmh)
                if speed_error > 0:
                    # Need to accelerate in reverse
                    throttle = np.clip(speed_error / 10.0, 0.1, 0.5)
                else:
                    # Already going fast enough in reverse
                    throttle = 0.1

                # When reversing, invert steering for intuitive control
                # (turning left while reversing means rear goes left = car rotates right)
                steer = float(np.clip(-planner_output.target_steering, -1.0, 1.0))

                control = {
                    "throttle": float(throttle),
                    "steer": steer,
                    "brake": 0.0,
                    "hand_brake": False,
                    "reverse": True,
                }
                self._last_control = control
                return control

            # Forward driving (normal case)
            speed_error = target_speed - current_speed_kmh
            if speed_error > 0:
                # Need to accelerate
                throttle = np.clip(speed_error / 15.0, 0.1, 0.8)  # Proportional
                brake = 0.0
            elif speed_error < -5:
                # Need to brake significantly
                throttle = 0.0
                brake = np.clip(abs(speed_error) / 30.0, 0.1, 1.0)
            else:
                # Maintain speed - gentle throttle
                throttle = 0.15
                brake = 0.0

            # Use planner's steering directly (already normalized -1 to 1)
            steer = float(np.clip(planner_output.target_steering, -1.0, 1.0))

            control = {
                "throttle": float(throttle),
                "steer": steer,
                "brake": float(brake),
                "hand_brake": False,
                "reverse": False,
            }
            self._last_control = control
            return control

        # For non-CARLA platforms, use PID
        speed_output = self.speed_pid.compute(
            planner_output.target_speed, current_speed_kmh
        )
        target_steer = planner_output.target_steering
        steer_output = self.steering_pid.compute(target_steer, current_steering)

        if self.platform == "raspberry_pi":
            return self._to_rpi_control(speed_output, steer_output)
        else:
            return self._to_generic_control(speed_output, steer_output)

    def _compute_control_old(self, planner_output, current_speed_kmh, current_steering=0.0):
        """Legacy PID-based control (kept for RPi)."""
        speed_output = self.speed_pid.compute(
            planner_output.target_speed, current_speed_kmh
        )
        target_steer = planner_output.target_steering
        steer_output = self.steering_pid.compute(target_steer, current_steering)

        if self.platform == "carla":
            return self._to_carla_control(speed_output, steer_output)
        elif self.platform == "raspberry_pi":
            return self._to_rpi_control(speed_output, steer_output)
        else:
            return self._to_generic_control(speed_output, steer_output)

    def _emergency_brake(self) -> dict:
        """Emergency stop control."""
        self._emergency_active = True
        self.speed_pid.reset()

        if self.platform == "carla":
            return {
                "throttle": 0.0,
                "steer": 0.0,
                "brake": 1.0,
                "hand_brake": False,
                "reverse": False,
            }
        elif self.platform == "raspberry_pi":
            return {
                "left_pwm": 0,
                "right_pwm": 0,
                "direction": "stop",
            }
        else:
            return {"speed": 0.0, "steering": 0.0, "brake": True}

    def _to_carla_control(self, speed_output: float, steer_output: float) -> dict:
        """Convert to CARLA VehicleControl format."""
        if speed_output >= 0:
            throttle = min(speed_output, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(abs(speed_output), 1.0)

        control = {
            "throttle": float(throttle),
            "steer": float(np.clip(steer_output, -1.0, 1.0)),
            "brake": float(brake),
            "hand_brake": False,
            "reverse": False,
        }
        self._last_control = control
        return control

    def _to_rpi_control(self, speed_output: float, steer_output: float) -> dict:
        """
        Convert to Raspberry Pi 4WD motor commands.
        Differential drive: left/right PWM values determine turning.
        """
        max_pwm = self.config.get("max_speed_pwm", 100)

        # Base speed from speed_output (0-1 range)
        base_speed = max(0, min(speed_output, 1.0)) * max_pwm

        # Differential steering
        # steer_output: -1 (full left) to +1 (full right)
        steer_factor = np.clip(steer_output, -1.0, 1.0)

        if speed_output <= 0:
            direction = "stop" if speed_output == 0 else "backward"
            left_pwm = 0
            right_pwm = 0
        else:
            direction = "forward"
            if steer_factor >= 0:
                # Turn right: slow down right wheels
                left_pwm = int(base_speed)
                right_pwm = int(base_speed * (1.0 - abs(steer_factor) * 0.8))
            else:
                # Turn left: slow down left wheels
                left_pwm = int(base_speed * (1.0 - abs(steer_factor) * 0.8))
                right_pwm = int(base_speed)

        control = {
            "left_pwm": left_pwm,
            "right_pwm": right_pwm,
            "direction": direction,
        }
        self._last_control = control
        return control

    def _to_generic_control(self, speed_output: float, steer_output: float) -> dict:
        """Generic control output for abstract interfaces."""
        control = {
            "speed": float(speed_output),
            "steering": float(steer_output),
            "brake": speed_output < 0,
        }
        self._last_control = control
        return control

    def apply_carla_control(self, vehicle, control_dict: dict):
        """
        Apply control to a CARLA vehicle actor.

        Args:
            vehicle: carla.Vehicle actor
            control_dict: output from compute_control()
        """
        import carla
        ctrl = carla.VehicleControl()
        ctrl.throttle = control_dict["throttle"]
        ctrl.steer = control_dict["steer"]
        ctrl.brake = control_dict["brake"]
        ctrl.hand_brake = control_dict.get("hand_brake", False)
        ctrl.reverse = control_dict.get("reverse", False)
        vehicle.apply_control(ctrl)

    @property
    def is_emergency(self) -> bool:
        return self._emergency_active

    @property
    def last_control(self) -> Optional[dict]:
        return self._last_control
