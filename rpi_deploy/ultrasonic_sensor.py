"""
NexusPilot: Ultrasonic Sensor Module
Handles distance measurement with HC-SR04 using hard timeouts for real-time safety.
"""
import time
import sys
import os
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

# Add root directory to sys.path for direct script execution
if __name__ == "__main__" or __package__ is None:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from rpi_deploy.hardware_config import hardware_config
else:
    from .hardware_config import hardware_config

@dataclass
class DistanceReading:
    """Standardized distance measurement result."""
    distance_cm: float
    valid: bool
    timestamp: float = field(default_factory=time.time)

class UltrasonicSensor:
    """
    HC-SR04 Driver with software-level safety timeouts.
    """
    def __init__(self, trig: Optional[int] = None, echo: Optional[int] = None):
        self.config = hardware_config.ultrasonic
        self.trig = trig if trig is not None else self.config.trigger_pin
        self.echo = echo if echo is not None else self.config.echo_pin
        
        if GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trig, GPIO.OUT)
            GPIO.setup(self.echo, GPIO.IN)
            print(f"[Ultrasonic] Initialized on Trig:{self.trig}, Echo:{self.echo}")

    def measure_once(self) -> DistanceReading:
        """
        Perform a single distance measurement.
        Includes a 30ms timeout to prevent blocking the main control loop.
        """
        if not GPIO:
            return DistanceReading(999.0, False)
        
        GPIO.output(self.trig, False)
        time.sleep(0.01)
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        start_wait = time.time()
        pulse_start = time.time()
        
        # Wait for ECHO to go HIGH
        while GPIO.input(self.echo) == 0:
            pulse_start = time.time()
            if pulse_start - start_wait > 0.03: # Hard timeout
                return DistanceReading(999.0, False)

        # Wait for ECHO to go LOW
        pulse_end = time.time()
        while GPIO.input(self.echo) == 1:
            pulse_end = time.time()
            if pulse_end - pulse_start > 0.03: # Hard timeout
                break

        duration = pulse_end - pulse_start
        distance = (duration * 34300) / 2
        
        is_valid = 2.0 < distance < 400.0
        return DistanceReading(distance if is_valid else 999.0, is_valid)

    def measure_average(self, count: int = 5) -> DistanceReading:
        """
        Returns the median of multiple measurements to filter out noise.
        """
        readings = []
        for _ in range(count):
            r = self.measure_once()
            if r.valid:
                readings.append(r.distance_cm)
            time.sleep(0.01)

        if not readings:
            return DistanceReading(999.0, False)
        return DistanceReading(float(np.median(readings)), True)

    def get_filtered_distance(self, count: int = 5) -> float:
        """Return a noise-filtered distance value (cm). Convenience wrapper."""
        reading = self.measure_average(count=count)
        return reading.distance_cm if reading.valid else 999.0

    def is_obstacle_detected(self, threshold: float = 30.0) -> bool:
        """Check if an obstacle is within the threshold distance (cm)."""
        return self.get_filtered_distance(count=3) < threshold

    def scan_with_servo(self, servo) -> List[dict]:
        """
        Scan at multiple servo angles and return distance readings.

        Args:
            servo: ServoController instance

        Returns:
            List of dicts with keys: angle, distance_cm, valid
        """
        angles = hardware_config.control.scan_angles
        delay = hardware_config.control.scan_delay
        results = []
        for angle in angles:
            servo.set_ultrasonic_angle(angle)
            time.sleep(delay)
            reading = self.measure_once()
            results.append({
                'angle': angle,
                'distance_cm': reading.distance_cm,
                'valid': reading.valid,
            })
        servo.center_ultrasonic()
        time.sleep(delay)
        return results

    def find_clear_direction(self, servo) -> float:
        """
        Scan with servo and return the angle with the most clearance.

        Args:
            servo: ServoController instance

        Returns:
            Angle (degrees) with the largest valid distance reading.
        """
        readings = self.scan_with_servo(servo)
        valid = [r for r in readings if r['valid']]
        if not valid:
            return hardware_config.ultrasonic.center_angle
        best = max(valid, key=lambda r: r['distance_cm'])
        return float(best['angle'])

    def cleanup(self):
        print("[Ultrasonic] Cleanup complete")


def create_ultrasonic_sensor() -> UltrasonicSensor:
    """Factory function to create an ultrasonic sensor instance."""
    return UltrasonicSensor()


if __name__ == "__main__":
    print("Testing Ultrasonic Sensor (Direct Execution)...")
    sensor = UltrasonicSensor()
    try:
        while True:
            res = sensor.measure_once()
            status = "[VALID]" if res.valid else "[INVALID]"
            print(f"Distance: {res.distance_cm:.1f} cm {status}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        sensor.cleanup()
