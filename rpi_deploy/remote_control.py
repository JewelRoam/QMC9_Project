#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raspberry Pi Car Remote Control Module
- TCP server mode (default): accept commands from PC client
- Local mode (--local): direct WASD keyboard control on the RPi itself

Usage:
    # TCP server mode (RPi listens for PC commands)
    python -m rpi_deploy.remote_control

    # Local mode (direct WASD on RPi, no network needed)
    python rpi_deploy/remote_control.py --local
    python rpi_deploy/remote_control.py --local --port 5000
"""

import socket
import threading
import json
import time
import sys
import argparse
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass, asdict
from enum import Enum

# ---- Fix imports: work both as package module and as standalone script ----
try:
    from .hardware_config import hardware_config
except ImportError:
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from hardware_config import hardware_config


class CommandType(Enum):
    MOVE = "move"
    STOP = "stop"
    SERVO = "servo"
    CAMERA = "camera"
    GET_STATUS = "status"
    SCAN = "scan"
    AUTO_MODE = "auto"
    HEARTBEAT = "heartbeat"


@dataclass
class ControlCommand:
    cmd_type: str
    params: Dict[str, Any]
    timestamp: float

    def to_json(self) -> str:
        return json.dumps({
            'type': self.cmd_type,
            'params': self.params,
            'timestamp': self.timestamp
        })

    @classmethod
    def from_json(cls, data: str) -> 'ControlCommand':
        obj = json.loads(data)
        return cls(
            cmd_type=obj.get('type', 'unknown'),
            params=obj.get('params', {}),
            timestamp=obj.get('timestamp', time.time())
        )


@dataclass
class VehicleStatus:
    motor_speed: float = 0.0
    direction: str = "STOP"
    ultrasonic_angle: float = 90.0
    camera_pan: float = 90.0
    camera_tilt: float = 45.0
    obstacle_distance: float = 999.0
    auto_mode: bool = False
    timestamp: float = 0.0

    def to_json(self) -> str:
        return json.dumps(asdict(self))


# ================================================================
# TCP Remote Control Server
# ================================================================

class RemoteControlServer:
    """Accepts TCP commands from a PC client and dispatches to handlers."""

    def __init__(self, host: str = None, port: int = None):
        self.config = hardware_config.network
        self.host = host or self.config.control_host
        self.port = port or self.config.control_port

        self._socket: Optional[socket.socket] = None
        self._running = False
        self._server_thread: Optional[threading.Thread] = None
        self._clients: list = []

        self._command_handlers: Dict[str, Callable] = {}
        self._status_provider: Optional[Callable[[], VehicleStatus]] = None

        print(f"[INFO] Remote control server configured: {self.host}:{self.port}")

    def register_handler(self, cmd_type: str, handler: Callable[[Dict], Any]):
        self._command_handlers[cmd_type] = handler

    def register_status_provider(self, provider: Callable[[], VehicleStatus]):
        self._status_provider = provider

    def start(self):
        if self._running:
            return
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind((self.host, self.port))
            self._socket.listen(5)

            self._running = True
            self._server_thread = threading.Thread(target=self._server_loop, daemon=True)
            self._server_thread.start()
            print(f"[OK] Remote control server started on {self.host}:{self.port}")
        except Exception as e:
            print(f"[ERROR] Failed to start server: {e}")

    def stop(self):
        self._running = False
        for client in self._clients:
            try:
                client.close()
            except Exception:
                pass
        self._clients.clear()
        if self._socket:
            self._socket.close()
            self._socket = None
        print("[INFO] Remote control server stopped")

    def _server_loop(self):
        while self._running:
            try:
                self._socket.settimeout(1.0)
                client_socket, address = self._socket.accept()
                print(f"[INFO] Client connected: {address}")
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, address),
                    daemon=True
                )
                client_thread.start()
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"[ERROR] Server error: {e}")

    def _handle_client(self, client_socket: socket.socket, address):
        self._clients.append(client_socket)
        buffer = ""
        try:
            while self._running:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    response = self._process_command(line.strip())
                    if response:
                        client_socket.send((response + '\n').encode('utf-8'))
        except ConnectionResetError:
            print(f"[INFO] Client disconnected: {address}")
        except Exception as e:
            print(f"[ERROR] Client error {address}: {e}")
        finally:
            client_socket.close()
            if client_socket in self._clients:
                self._clients.remove(client_socket)

    def _process_command(self, data: str) -> Optional[str]:
        try:
            command = ControlCommand.from_json(data)
            cmd_type = command.cmd_type

            if cmd_type in self._command_handlers:
                result = self._command_handlers[cmd_type](command.params)
                return json.dumps({'success': True, 'result': result})
            elif cmd_type == CommandType.GET_STATUS.value:
                if self._status_provider:
                    return self._status_provider().to_json()
                return json.dumps({'error': 'Status provider not registered'})
            elif cmd_type == CommandType.HEARTBEAT.value:
                return json.dumps({'alive': True, 'timestamp': time.time()})
            else:
                return json.dumps({'error': f'Unknown command: {cmd_type}'})
        except json.JSONDecodeError:
            return json.dumps({'error': 'Invalid JSON format'})
        except Exception as e:
            return json.dumps({'error': str(e)})


# ================================================================
# TCP Remote Control Client
# ================================================================

class RemoteControlClient:
    """Client for sending commands to the RPi server from a PC."""

    def __init__(self, host: str, port: int = None):
        self.host = host
        self.port = port or hardware_config.network.control_port
        self._socket: Optional[socket.socket] = None
        self._connected = False

    def connect(self) -> bool:
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.connect((self.host, self.port))
            self._connected = True
            print(f"[OK] Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
            return False

    def disconnect(self):
        if self._socket:
            self._socket.close()
            self._socket = None
        self._connected = False

    def send_command(self, cmd_type: str, params: Dict = None) -> Dict:
        if not self._connected or not self._socket:
            return {'error': 'Not connected'}
        command = ControlCommand(
            cmd_type=cmd_type,
            params=params or {},
            timestamp=time.time()
        )
        try:
            self._socket.send((command.to_json() + '\n').encode('utf-8'))
            self._socket.settimeout(5.0)
            response = self._socket.recv(4096).decode('utf-8').strip()
            return json.loads(response)
        except socket.timeout:
            return {'error': 'Response timeout'}
        except Exception as e:
            return {'error': str(e)}

    def move_forward(self, speed: float = 50.0):
        return self.send_command('move', {'direction': 'forward', 'speed': speed})

    def move_backward(self, speed: float = 50.0):
        return self.send_command('move', {'direction': 'backward', 'speed': speed})

    def turn_left(self, speed: float = 50.0):
        return self.send_command('move', {'direction': 'left', 'speed': speed})

    def turn_right(self, speed: float = 50.0):
        return self.send_command('move', {'direction': 'right', 'speed': speed})

    def stop(self):
        return self.send_command('stop')

    def set_servo(self, servo_type: str, angle: float):
        return self.send_command('servo', {'type': servo_type, 'angle': angle})

    def scan_ultrasonic(self):
        return self.send_command('scan')

    def get_status(self) -> VehicleStatus:
        response = self.send_command('status')
        if 'error' not in response:
            return VehicleStatus(**response)
        return None

    def heartbeat(self) -> bool:
        response = self.send_command('heartbeat')
        return 'alive' in response and response['alive']


# ================================================================
# Local WASD Keyboard Control (runs directly on RPi)
# ================================================================

SPEED_LEVELS = [0.2, 0.4, 0.6, 0.8, 1.0]
SPEED_LABELS = ["20%", "40%", "60%", "80%", "100%"]


def _get_key():
    """Cross-platform non-blocking single keypress."""
    if sys.platform == 'win32':
        import msvcrt
        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ('\x00', '\xe0'):
                ch2 = msvcrt.getwch()
                mapping = {'H': 'up', 'P': 'down', 'K': 'left', 'M': 'right'}
                return mapping.get(ch2, ch2)
            return ch
        return None
    else:
        import select
        import tty
        import termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        ch3 = sys.stdin.read(1)
                        mapping = {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}
                        return mapping.get(ch3, ch3)
                    return ch2
                return ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return None


def run_local():
    """Direct WASD keyboard control on the RPi — drives motors/servos locally."""
    # Import hardware drivers
    try:
        import os
        os.environ["LG_CHIP"] = "0"
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from rpi_deploy.motor_driver import MotorController
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from motor_driver import MotorController

    servo = None
    ultrasonic = None
    try:
        from rpi_deploy.servo_controller import ServoController
        from rpi_deploy.ultrasonic_sensor import UltrasonicSensor
        servo = ServoController()
        ultrasonic = UltrasonicSensor()
        servo.center_ultrasonic()
        has_servo = True
    except Exception:
        has_servo = False

    motor = MotorController()
    speed_idx = 1  # 40%
    us_angle = 120.0  # hardware center

    print(f"""
{'='*50}
  Local WASD Control — Raspberry Pi Direct
{'='*50}
  W / ↑  Forward         S / ↓  Backward
  A / ←  Turn Left       D / →  Turn Right
  Space  Stop            Q      Quit

  1-5    Speed ({', '.join(SPEED_LABELS)})

  U/J    Ultrasonic servo left/right
  R      Read ultrasonic distance
  C      Center servo
{'='*50}
  Speed: {SPEED_LABELS[speed_idx]}
{'='*50}
""")

    try:
        while True:
            key = _get_key()
            if key is None:
                time.sleep(0.02)
                continue

            speed = SPEED_LEVELS[speed_idx]

            if key in ('w', 'W', 'up'):
                motor.move_forward(speed)
                print(f"  Forward @ {SPEED_LABELS[speed_idx]}")
            elif key in ('s', 'S', 'down'):
                motor.move_backward(speed)
                print(f"  Backward @ {SPEED_LABELS[speed_idx]}")
            elif key in ('a', 'A', 'left'):
                motor.turn_left(speed)
                print(f"  Turn Left @ {SPEED_LABELS[speed_idx]}")
            elif key in ('d', 'D', 'right'):
                motor.turn_right(speed)
                print(f"  Turn Right @ {SPEED_LABELS[speed_idx]}")
            elif key == ' ':
                motor.stop()
                print("  Stop")

            elif key in '12345':
                speed_idx = int(key) - 1
                print(f"  Speed → {SPEED_LABELS[speed_idx]}")

            elif key in ('u', 'U') and has_servo:
                us_angle = max(30, us_angle - 20)
                servo.set_ultrasonic_angle(us_angle)
                print(f"  US Servo → {us_angle:.0f}°")
            elif key in ('j', 'J') and has_servo:
                us_angle = min(180, us_angle + 20)
                servo.set_ultrasonic_angle(us_angle)
                print(f"  US Servo → {us_angle:.0f}°")
            elif key in ('r', 'R') and ultrasonic is not None:
                reading = ultrasonic.measure_once()
                if reading.valid:
                    print(f"  Distance: {reading.distance_cm:.1f} cm")
                else:
                    print("  Distance: ---")
            elif key in ('c', 'C') and has_servo:
                us_angle = 120.0
                servo.center_ultrasonic()
                print("  Servo centered")

            elif key in ('q', 'Q', '\x1b'):
                print("\n[Quit] Stopping and exiting...")
                break

    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        motor.stop()
        motor.cleanup()
        if servo is not None:
            servo.cleanup()


# ================================================================
# Entry point
# ================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Raspberry Pi Car Remote Control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Modes:
  (default)    TCP server — accept commands from PC client
  --local      Direct WASD control on RPi (no network needed)

Examples:
  python -m rpi_deploy.remote_control                 # TCP server mode
  python rpi_deploy/remote_control.py --local          # Local WASD on RPi
  python rpi_deploy/remote_control.py --local --port 5001
        """)
    parser.add_argument("--local", action="store_true",
                        help="Run local WASD keyboard control (no network)")
    parser.add_argument("--port", type=int, default=None,
                        help="Control port (default: from hardware_config)")
    args = parser.parse_args()

    if args.local:
        run_local()
    else:
        # TCP server mode — register hardware handlers
        try:
            import os
            os.environ["LG_CHIP"] = "0"
            sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            from rpi_deploy.motor_driver import MotorController
            from rpi_deploy.servo_controller import ServoController
            from rpi_deploy.ultrasonic_sensor import UltrasonicSensor
        except ImportError:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from motor_driver import MotorController
            from servo_controller import ServoController
            from ultrasonic_sensor import UltrasonicSensor

        motor = MotorController()
        servo = ServoController()
        ultrasonic = UltrasonicSensor()
        servo.center_ultrasonic()

        server = RemoteControlServer(port=args.port)

        # Register hardware command handlers
        def handle_move(params):
            direction = params.get('direction', 'forward')
            speed_pct = params.get('speed', 50.0)
            speed = speed_pct / 100.0
            actions = {
                'forward': lambda: motor.move_forward(speed),
                'backward': lambda: motor.move_backward(speed),
                'left': lambda: motor.turn_left(speed),
                'right': lambda: motor.turn_right(speed),
            }
            action = actions.get(direction)
            if action:
                action()
                return f"{direction} @ {speed_pct}%"
            return "Unknown direction"

        def handle_stop(params):
            motor.stop()
            return "Stopped"

        def handle_servo(params):
            s_type = params.get('type', 'ultrasonic')
            angle = params.get('angle', 90.0)
            if s_type == 'ultrasonic':
                servo.set_ultrasonic_angle(angle)
            elif s_type == 'camera_pan':
                servo.set_camera_pan(angle)
            elif s_type == 'camera_tilt':
                servo.set_camera_tilt(angle)
            return f"{s_type} → {angle}°"

        def handle_scan(params):
            servo.center_ultrasonic()
            time.sleep(0.25)
            angles = (30, 75, 120, 165, 180)
            results = {}
            for a in angles:
                servo.set_ultrasonic_angle(a)
                time.sleep(0.25)
                r = ultrasonic.measure_once()
                results[a] = r.distance_cm if r.valid else 999.0
            servo.center_ultrasonic()
            return results

        def handle_status():
            r = ultrasonic.measure_once()
            return VehicleStatus(
                motor_speed=0.0,
                direction="STOP",
                obstacle_distance=r.distance_cm if r.valid else 999.0,
                timestamp=time.time()
            )

        server.register_handler('move', handle_move)
        server.register_handler('stop', handle_stop)
        server.register_handler('servo', handle_servo)
        server.register_handler('scan', handle_scan)
        server.register_status_provider(handle_status)

        server.start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[Shutdown]")
        finally:
            server.stop()
            motor.stop()
            motor.cleanup()
            servo.cleanup()


if __name__ == "__main__":
    main()
