"""
Robust Ultrasonic Sensor module with hard timeouts to prevent loop lockup.
"""
import time
try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

class UltrasonicSensor:
    def __init__(self, trig=20, echo=21):
        self.trig = trig
        self.echo = echo
        if GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trig, GPIO.OUT)
            GPIO.setup(self.echo, GPIO.IN)

    def measure_once(self):
        if not GPIO: return type('obj', (object,), {'distance_cm': 999.0, 'valid': False})
        
        GPIO.output(self.trig, False)
        time.sleep(0.01)
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        # Hard timeout for ECHO signal (approx 30ms = 5 meters)
        start_wait = time.time()
        pulse_start = time.time()
        while GPIO.input(self.echo) == 0:
            pulse_start = time.time()
            if pulse_start - start_wait > 0.03:
                return type('obj', (object,), {'distance_cm': 999.0, 'valid': False})

        pulse_end = time.time()
        while GPIO.input(self.echo) == 1:
            pulse_end = time.time()
            if pulse_end - pulse_start > 0.03:
                break

        duration = pulse_end - pulse_start
        distance = (duration * 34300) / 2
        return type('obj', (object,), {'distance_cm': distance, 'valid': True})

    def cleanup(self):
        pass
