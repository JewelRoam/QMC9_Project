#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Servo Controller Module for Raspberry Pi Robot Car

Based on PCA9685 I2C PWM controller for servo driving.
Compatible with Chuang Le Bo LOBOROBOT expansion board.
"""

import time
import math
import sys
import os
from typing import Optional, Tuple

# Attempt to import smbus, if failed then use simulation mode
try:
    import smbus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False
    print("[WARNING] smbus not available, running in simulation mode")

# Add root directory to sys.path for direct script execution
if __name__ == "__main__" or __package__ is None:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from rpi_deploy.hardware_config import hardware_config
else:
    from .hardware_config import hardware_config


class PCA9685:
    """
    PCA9685 I2C PWM controller driver class
    16-channel PWM output, 12-bit resolution
    """
    
    # PCA9685 register addresses
    __MODE1 = 0x00
    __MODE2 = 0x01
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALL_LED_ON_L = 0xFA
    __ALL_LED_ON_H = 0xFB
    __ALL_LED_OFF_L = 0xFC
    __ALL_LED_OFF_H = 0xFD
    
    # Bit definitions
    __RESTART = 0x80
    __SLEEP = 0x10
    __ALLCALL = 0x01
    __INVRT = 0x10
    __OUTDRV = 0x04
    
    def __init__(self, address: int = 0x40, bus_number: int = 1):
        self.address = address
        self.bus_number = bus_number
        self._bus: Optional[smbus.SMBus] = None
        
        if SMBUS_AVAILABLE:
            try:
                self._bus = smbus.SMBus(bus_number)
                self._init_device()
                print(f"[INFO] PCA9685 initialized at address 0x{address:02X}")
            except Exception as e:
                print(f"[ERROR] Failed to initialize PCA9685: {e}")
                self._bus = None
        else:
            print(f"[SIM] PCA9685 simulated at address 0x{address:02X}")
    
    def _init_device(self):
        """Initialize PCA9685 device"""
        if self._bus is None:
            return
        
        # Reset device
        self._write_byte(self.__MODE1, self.__ALLCALL)
        time.sleep(0.005)  # Wait for oscillator to stabilize
        
        # Set mode
        mode1 = self._read_byte(self.__MODE1)
        mode1 = mode1 & ~self.__SLEEP  # Exit sleep mode
        self._write_byte(self.__MODE1, mode1)
        time.sleep(0.005)
        
        # Set output mode
        self._write_byte(self.__MODE2, self.__OUTDRV)
    
    def _write_byte(self, reg: int, value: int):
        """Write a single byte"""
        if self._bus:
            self._bus.write_byte_data(self.address, reg, value)
    
    def _read_byte(self, reg: int) -> int:
        """Read a single byte"""
        if self._bus:
            return self._bus.read_byte_data(self.address, reg)
        return 0
    
    def set_pwm_freq(self, freq_hz: float):
        """
        Set PWM frequency
        
        Args:
            freq_hz: Frequency (Hz), servos usually use 50Hz (20ms period)
        """
        if self._bus is None:
            print(f"[SIM] Set PWM frequency to {freq_hz}Hz")
            return
        
        # Calculate prescale value
        prescaleval = 25000000.0  # 25MHz internal clock
        prescaleval /= 4096.0     # 12-bit resolution
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        
        # Enter sleep mode to set prescale
        old_mode = self._read_byte(self.__MODE1)
        new_mode = (old_mode & 0x7F) | self.__SLEEP
        self._write_byte(self.__MODE1, new_mode)
        self._write_byte(self.__PRESCALE, prescale)
        self._write_byte(self.__MODE1, old_mode)
        time.sleep(0.005)
        self._write_byte(self.__MODE1, old_mode | self.__RESTART)
        
        print(f"[INFO] PWM frequency set to {freq_hz}Hz (prescale={prescale})")
    
    def set_pwm(self, channel: int, on: int, off: int):
        """
        Set PWM output for a specific channel
        
        Args:
            channel: Channel number (0-15)
            on: Turn-on tick value (0-4095)
            off: Turn-off tick value (0-4095)
        """
        if self._bus is None:
            return
        
        self._write_byte(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self._write_byte(self.__LED0_ON_H + 4 * channel, on >> 8)
        self._write_byte(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self._write_byte(self.__LED0_OFF_H + 4 * channel, off >> 8)
    
    def set_duty_cycle(self, channel: int, pulse: float):
        """
        Set duty cycle percentage
        
        Args:
            channel: Channel number
            pulse: Duty cycle (0-100)
        """
        pulse = max(0, min(100, pulse))
        off_value = int(pulse * 4096 / 100)
        self.set_pwm(channel, 0, off_value)
    
    def set_level(self, channel: int, value: int):
        """
        Set digital level output
        
        Args:
            channel: Channel number
            value: 0=low level, 1=high level
        """
        if value == 0:
            self.set_pwm(channel, 0, 0)
        else:
            self.set_pwm(channel, 0, 4095)
    
    def cleanup(self):
        """Clean up resources, turn off all outputs"""
        if self._bus:
            for channel in range(16):
                self.set_pwm(channel, 0, 0)
            self._bus.close()
            print("[INFO] PCA9685 cleaned up")


class ServoController:
    """
    Servo controller
    Manages servos for ultrasonic gimbal and camera gimbal
    """
    
    # Servo pulse width range (microseconds)
    SERVO_MIN_PULSE = 500   # 0.5ms = 0 degrees
    SERVO_MAX_PULSE = 2500  # 2.5ms = 180 degrees
    
    def __init__(self):
        self.config = hardware_config.i2c
        self.ultrasonic_config = hardware_config.ultrasonic
        self.camera_config = hardware_config.camera
        
        # Initialize PCA9685
        self._pca = PCA9685(
            address=self.config.pca9685_address,
            bus_number=self.config.bus_number
        )
        
        # Set PWM frequency
        self._pca.set_pwm_freq(self.config.pwm_frequency)
        time.sleep(0.1)  # Allow oscillator to stabilize before driving servos
        
        # Current angle state
        self._ultrasonic_angle = self.ultrasonic_config.center_angle
        self._camera_pan = 90
        self._camera_tilt = 60
        
        # Initialize to center position
        self.center_all()
    
    def _angle_to_pulse(self, angle: float) -> int:
        """
        Convert angle to PCA9685 pulse tick value
        
        Formula: 4096 * ((angle * 11) + 500) / 20000
        Where 11 is the pulse increment per degree (2000us/180 degrees ≈ 11.11)
        500 is the offset corresponding to 0.5ms
        """
        # Limit angle range
        angle = max(0, min(180, angle))
        
        # Chuang Le Bo LOBOROBOT formula
        pulse = int(4096 * ((angle * 11) + 500) / 20000)
        return max(0, min(4095, pulse))
    
    def set_servo_angle(self, channel: int, angle: float):
        """
        Set servo angle
        
        Args:
            channel: PCA9685 channel number
            angle: Angle (0-180 degrees)
        """
        pulse = self._angle_to_pulse(angle)
        self._pca.set_pwm(channel, 0, pulse)
        
        # Update state
        if channel == self.ultrasonic_config.servo_channel:
            self._ultrasonic_angle = angle
        elif channel == self.camera_config.pan_servo_channel:
            self._camera_pan = angle
        elif channel == self.camera_config.tilt_servo_channel:
            self._camera_tilt = angle
    
    # ========== Ultrasonic Gimbal Control ==========
    
    def set_ultrasonic_angle(self, angle: float):
        """Set ultrasonic sensor angle"""
        angle = max(self.ultrasonic_config.min_angle,
                   min(self.ultrasonic_config.max_angle, angle))
        self.set_servo_angle(self.ultrasonic_config.servo_channel, angle)
    
    def center_ultrasonic(self):
        """Center ultrasonic sensor"""
        self.set_ultrasonic_angle(self.ultrasonic_config.center_angle)
    
    def scan_ultrasonic(self, angles: Tuple[int, ...] = None, 
                       delay: float = None) -> dict:
        """
        Scan a specified sequence of angles
        
        Args:
            angles: List of scanning angles, defaults to sequence in config
            delay: Delay after each movement
        
        Returns:
            Dictionary of angle positions {angle: timestamp}
        """
        if angles is None:
            angles = hardware_config.control.scan_angles
        if delay is None:
            delay = hardware_config.control.scan_delay
        
        positions = {}
        for angle in angles:
            self.set_ultrasonic_angle(angle)
            positions[angle] = time.time()
            time.sleep(delay)
        
        return positions
    
    # ========== Camera Gimbal Control ==========
    
    def set_camera_pan(self, angle: float):
        """Set camera pan angle"""
        angle = max(self.camera_config.pan_min,
                   min(self.camera_config.pan_max, angle))
        self.set_servo_angle(self.camera_config.pan_servo_channel, angle)
    
    def set_camera_tilt(self, angle: float):
        """Set camera tilt angle. Clamped to physical servo range (0 to tilt_max)."""
        angle = max(0, min(self.camera_config.tilt_max, angle))
        self.set_servo_angle(self.camera_config.tilt_servo_channel, angle)
    
    def set_camera_position(self, pan: float, tilt: float):
        """Set camera pan and tilt angles simultaneously"""
        self.set_camera_pan(pan)
        self.set_camera_tilt(tilt)
    
    def center_camera(self):
        """Center camera to a safe position (pan=90, tilt=60)."""
        self.set_camera_position(90, 60)
    
    def track_object(self, delta_x: float, delta_y: float, 
                    speed: float = 2.0):
        """
        Adjust camera tracking based on object offset
        
        Args:
            delta_x: Horizontal offset (-1 to 1, negative is left)
            delta_y: Vertical offset (-1 to 1, negative is up)
            speed: Tracking speed multiplier
        """
        new_pan = self._camera_pan - delta_x * speed * 5
        new_tilt = self._camera_tilt + delta_y * speed * 5
        self.set_camera_position(new_pan, new_tilt)
    
    def center_all(self):
        """Center all servos"""
        self.center_ultrasonic()
        self.center_camera()
    
    def get_status(self) -> dict:
        """Get servo status"""
        return {
            'ultrasonic_angle': self._ultrasonic_angle,
            'camera_pan': self._camera_pan,
            'camera_tilt': self._camera_tilt,
            'pca9685_address': f"0x{self.config.pca9685_address:02X}",
            'pwm_frequency': self.config.pwm_frequency
        }
    
    def test_sequence(self):
        """Test sequence"""
        print("\n=== Servo Test Sequence ===")
        
        # Test ultrasonic gimbal
        print("\nTesting ultrasonic servo...")
        for angle in [90, 45, 0, 45, 90, 135, 180, 135, 90]:
            print(f"  Setting ultrasonic angle to {angle}°")
            self.set_ultrasonic_angle(angle)
            time.sleep(0.5)
        
        # Test camera gimbal
        print("\nTesting camera servos...")
        positions = [
            (90, 45),   # Center
            (45, 30),   # Bottom left
            (135, 30),  # Bottom right
            (135, 60),  # Top right
            (45, 60),   # Top left
            (90, 45),   # Back to center
        ]
        for pan, tilt in positions:
            print(f"  Setting camera to pan={pan}°, tilt={tilt}°")
            self.set_camera_position(pan, tilt)
            time.sleep(0.5)
        
        print("\n=== Test Complete ===")
    
    def cleanup(self):
        """Clean up resources"""
        self.center_all()
        time.sleep(0.5)
        self._pca.cleanup()


# Convenience functions
def create_servo_controller() -> ServoController:
    """Create a servo controller instance"""
    return ServoController()


if __name__ == "__main__":
    # Standalone test
    print("Testing Servo Controller...")
    controller = create_servo_controller()
    
    try:
        controller.test_sequence()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.cleanup()
