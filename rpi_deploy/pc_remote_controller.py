#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PC端远程遥控客户端
PC-side Remote Control Client for Raspberry Pi Robot Car

Interactive keyboard control for the RPi car via TCP socket.
Supports: movement, servo, ultrasonic scan, status query, mode switch.

Usage (on PC):
    python -m rpi_deploy.pc_remote_controller --host 192.168.137.33
    python -m rpi_deploy.pc_remote_controller --host 192.168.137.33 --port 5000

Controls:
    W / ↑  : Forward        S / ↓  : Backward
    A / ←  : Turn Left      D / →  : Turn Right
    Space  : Stop           Q / Esc : Quit
    1-5    : Speed levels (20%, 40%, 60%, 80%, 100%)
    U / J  : Ultrasonic servo left / right
    P      : Pan camera servo scan
    R      : Query vehicle status
    M      : Switch to auto (obstacle avoidance) mode
"""

import sys
import os
import time
import argparse
import threading

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from typing import Optional
from rpi_deploy.remote_control import RemoteControlClient, VehicleStatus


# ---- Keyboard input (cross-platform) ----

def _get_key_windows():
    """Read a single keypress on Windows (non-blocking)."""
    import msvcrt
    if msvcrt.kbhit():
        ch = msvcrt.getwch()
        # Arrow keys send 0xe0 prefix
        if ch in ('\x00', '\xe0'):
            ch2 = msvcrt.getwch()
            mapping = {'H': 'up', 'P': 'down', 'K': 'left', 'M': 'right'}
            return mapping.get(ch2, ch2)
        return ch
    return None


def _get_key_unix():
    """Read a single keypress on Unix (non-blocking via select)."""
    import select
    import tty
    import termios
    if select.select([sys.stdin], [], [], 0.05)[0]:
        ch = sys.stdin.read(1)
        # Escape sequences for arrow keys
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                mapping = {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}
                return mapping.get(ch3, ch3)
            return ch2
        return ch
    return None


def get_key():
    """Read a single keypress (cross-platform, non-blocking)."""
    if sys.platform == 'win32':
        return _get_key_windows()
    return _get_key_unix()


# ---- Remote Controller ----

class PCRemoteController:
    """
    Interactive PC remote controller for RPi car.
    Connects via TCP and allows keyboard-based real-time control.
    """

    SPEED_LEVELS = [0.2, 0.4, 0.6, 0.8, 1.0]
    SPEED_LABELS = ["20%", "40%", "60%", "80%", "100%"]

    def __init__(self, host: str, port: int = 5000):
        self.host = host
        self.port = port
        self.client = RemoteControlClient(host, port)
        self.speed_idx = 1  # Default 40%
        self.connected = False
        self._status_thread = None
        self._running = False
        self._last_status: Optional[VehicleStatus] = None

    def connect(self) -> bool:
        """Connect to the RPi car server."""
        print(f"[Connecting] {self.host}:{self.port} ...")
        self.connected = self.client.connect()
        if self.connected:
            print("[OK] Connected to RPi car!")
        else:
            print("[FAIL] Connection failed. Check IP and port.")
        return self.connected

    def disconnect(self):
        """Disconnect from the RPi car."""
        self._running = False
        if self._status_thread:
            self._status_thread.join(timeout=2)
        self.client.disconnect()
        self.connected = False
        print("[Disconnected]")

    @property
    def current_speed(self) -> float:
        return self.SPEED_LEVELS[self.speed_idx]

    @property
    def current_speed_label(self) -> str:
        return self.SPEED_LABELS[self.speed_idx]

    def send_move(self, direction: str):
        """Send a movement command."""
        speed_pct = self.current_speed * 100
        response = self.client.send_command('move', {
            'direction': direction,
            'speed': speed_pct
        })
        self._print_response(f"Move {direction} @ {self.current_speed_label}", response)

    def send_stop(self):
        """Send stop command."""
        response = self.client.send_command('stop')
        self._print_response("Stop", response)

    def send_servo(self, servo_type: str, angle: float):
        """Send servo command."""
        response = self.client.send_command('servo', {
            'type': servo_type,
            'angle': angle
        })
        self._print_response(f"Servo {servo_type} → {angle}°", response)

    def send_scan(self):
        """Trigger ultrasonic scan."""
        response = self.client.send_command('scan')
        self._print_response("Scan", response)

    def send_auto(self):
        """Switch to auto obstacle avoidance mode."""
        response = self.client.send_command('auto')
        self._print_response("Auto mode", response)

    def query_status(self):
        """Query and display vehicle status."""
        status = self.client.get_status()
        if status:
            self._last_status = status
            print(f"\n{'='*40}")
            print(f"  Vehicle Status")
            print(f"{'='*40}")
            print(f"  Direction:     {status.direction}")
            print(f"  Motor Speed:   {status.motor_speed:.0f}%")
            print(f"  Obstacle:      {status.obstacle_distance:.1f} cm")
            print(f"  US Servo:      {status.ultrasonic_angle:.0f}°")
            print(f"  Camera Pan:    {status.camera_pan:.0f}°")
            print(f"  Camera Tilt:   {status.camera_tilt:.0f}°")
            print(f"  Auto Mode:     {'ON' if status.auto_mode else 'OFF'}")
            print(f"{'='*40}\n")
        else:
            print("[WARN] Failed to get status")

    def start_status_monitor(self):
        """Start background status monitoring thread."""
        self._running = True
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self._status_thread.start()

    def _status_loop(self):
        """Background loop to poll status periodically."""
        while self._running:
            try:
                status = self.client.get_status()
                if status:
                    self._last_status = status
                    # Compact status line
                    dist_str = f"{status.obstacle_distance:.0f}cm" if status.obstacle_distance < 900 else "∞"
                    sys.stdout.write(
                        f"\r[Status] {status.direction} | Speed:{status.motor_speed:.0f}% | "
                        f"Dist:{dist_str} | US:{status.ultrasonic_angle:.0f}°  "
                    )
                    sys.stdout.flush()
            except Exception:
                pass
            time.sleep(2.0)

    def _print_response(self, action: str, response: dict):
        """Print command response."""
        if response.get('success'):
            print(f"  ✓ {action}: {response.get('result', 'OK')}")
        elif response.get('error'):
            print(f"  ✗ {action}: ERROR - {response['error']}")
        else:
            print(f"  ? {action}: {response}")

    def print_help(self):
        """Print control help."""
        print(f"""
{'='*55}
  RPi Car Remote Controller - Keyboard Controls
{'='*55}
  Movement:
    W / ↑    Forward          S / ↓    Backward
    A / ←    Turn Left        D / →    Turn Right
    Space    Stop             Q / Esc  Quit

  Speed:
    1-5      Set speed ({', '.join(self.SPEED_LABELS)})
    + / =    Increase speed   - / _    Decrease speed

  Servo:
    U        Ultrasonic ←     J        Ultrasonic →
    I        Camera pan ←     O        Camera pan →
    K        Camera tilt ↑    L        Camera tilt ↓
    C        Center all servos

  Other:
    R        Query status     P        Ultrasonic scan
    M        Auto mode        H        This help
{'='*55}
  Current speed: {self.current_speed_label}
{'='*55}
""")


def run_interactive(host: str, port: int):
    """Run the interactive remote controller."""
    ctrl = PCRemoteController(host, port)

    if not ctrl.connect():
        return

    ctrl.print_help()

    # Ultrasonic servo tracking angle
    us_angle = 90.0
    cam_pan = 90.0
    cam_tilt = 0.0
    SERVO_STEP = 15.0

    try:
        while True:
            key = get_key()
            if key is None:
                time.sleep(0.02)
                continue

            # ---- Movement ----
            if key in ('w', 'W', 'up'):
                ctrl.send_move('forward')
            elif key in ('s', 'S', 'down'):
                ctrl.send_move('backward')
            elif key in ('a', 'A', 'left'):
                ctrl.send_move('left')
            elif key in ('d', 'D', 'right'):
                ctrl.send_move('right')
            elif key == ' ':
                ctrl.send_stop()

            # ---- Speed ----
            elif key in '12345':
                ctrl.speed_idx = int(key) - 1
                print(f"  Speed → {ctrl.current_speed_label}")
            elif key in ('+', '='):
                ctrl.speed_idx = min(len(ctrl.SPEED_LEVELS) - 1, ctrl.speed_idx + 1)
                print(f"  Speed → {ctrl.current_speed_label}")
            elif key in ('-', '_'):
                ctrl.speed_idx = max(0, ctrl.speed_idx - 1)
                print(f"  Speed → {ctrl.current_speed_label}")

            # ---- Servo ----
            elif key in ('u', 'U'):
                us_angle = max(0, us_angle - SERVO_STEP)
                ctrl.send_servo('ultrasonic', us_angle)
            elif key in ('j', 'J'):
                us_angle = min(180, us_angle + SERVO_STEP)
                ctrl.send_servo('ultrasonic', us_angle)
            elif key in ('i', 'I'):
                cam_pan = max(0, cam_pan - SERVO_STEP)
                ctrl.send_servo('camera_pan', cam_pan)
            elif key in ('o', 'O'):
                cam_pan = min(180, cam_pan + SERVO_STEP)
                ctrl.send_servo('camera_pan', cam_pan)
            elif key in ('k', 'K'):
                cam_tilt = min(90, cam_tilt + SERVO_STEP)
                ctrl.send_servo('camera_tilt', cam_tilt)
            elif key in ('l', 'L'):
                cam_tilt = max(0, cam_tilt - SERVO_STEP)
                ctrl.send_servo('camera_tilt', cam_tilt)
            elif key in ('c', 'C'):
                us_angle, cam_pan, cam_tilt = 90, 90, 0
                ctrl.send_servo('ultrasonic', us_angle)
                ctrl.send_servo('camera_pan', cam_pan)
                ctrl.send_servo('camera_tilt', cam_tilt)

            # ---- Other ----
            elif key in ('r', 'R'):
                ctrl.query_status()
            elif key in ('p', 'P'):
                ctrl.send_scan()
            elif key in ('m', 'M'):
                ctrl.send_auto()
            elif key in ('h', 'H'):
                ctrl.print_help()

            # ---- Quit ----
            elif key in ('q', 'Q', '\x1b'):
                print("\n[Quit] Exiting...")
                break

    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        ctrl.send_stop()
        ctrl.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description="PC Remote Controller for RPi Car",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m rpi_deploy.pc_remote_controller --host 192.168.137.33
  python -m rpi_deploy.pc_remote_controller --host 192.168.1.10 --port 5000
        """)
    parser.add_argument("--host", required=True,
                        help="Raspberry Pi IP address")
    parser.add_argument("--port", type=int, default=5000,
                        help="Control port (default: 5000)")
    args = parser.parse_args()

    run_interactive(args.host, args.port)


if __name__ == "__main__":
    main()