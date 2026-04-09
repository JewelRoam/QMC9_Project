#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PC端V2V协作协调器
PC-side V2V Cooperative Coordinator for Raspberry Pi Robot Car

Receives obstacle detections from RPi vehicle via UDP,
shares PC-side detections (from YOLO/camera), and provides
cooperative planning guidance back to the RPi.

Usage (on PC):
    python -m rpi_deploy.pc_v2v_coordinator --rpi-host 192.168.137.33
    python -m rpi_deploy.pc_v2v_coordinator --rpi-host 192.168.1.10 --port 5555

Architecture:
    RPi (v2v mode) ←──UDP──→ PC (pc_v2v_coordinator)
    - RPi broadcasts: ultrasonic detections, intent, position
    - PC broadcasts: camera/YOLO detections, planning advice
    - Both use V2VCommunicator socket mode
"""

import sys
import os
import time
import argparse
import threading
from typing import Optional
import json

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cooperation.v2v_message import V2VCommunicator, V2VMessage, SharedDetection


class PCV2VCoordinator:
    """
    PC-side V2V coordinator.
    
    Receives V2V messages from RPi vehicle, processes them,
    and broadcasts PC-side detections/planning back.
    """

    def __init__(self, vehicle_id: str, rpi_host: str, listen_port: int, send_port: int):
        self.vehicle_id = vehicle_id
        self.rpi_host = rpi_host
        self.listen_port = listen_port
        self.send_port = send_port

        # V2V communicator
        config = {
            "protocol": "socket",
            "max_latency_ms": 200,
            "dropout_rate": 0.0,
        }
        self.v2v = V2VCommunicator(vehicle_id, config)
        
        # Received messages tracking
        self._rpi_messages: dict = {}
        self._running = False
        self._monitor_thread = None

        # Statistics
        self._stats = {
            "messages_received": 0,
            "messages_sent": 0,
            "obstacles_shared": 0,
            "yield_events": 0,
        }

    def start(self):
        """Start the V2V coordinator."""
        # Start UDP listener (RPi sends to this port)
        self.v2v.start_socket_server(host="0.0.0.0", port=self.listen_port)
        # Register RPi as a peer (send to RPi's listener)
        self.v2v.add_socket_peer("rpi_vehicle_1", self.rpi_host, self.send_port)

        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()

        print(f"[V2V] Coordinator started: {self.vehicle_id}")
        print(f"[V2V] Listening on UDP port {self.listen_port}")
        print(f"[V2V] Sending to {self.rpi_host}:{self.send_port}")

    def stop(self):
        """Stop the V2V coordinator."""
        self._running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=3)
        print(f"[V2V] Coordinator stopped. Stats: {self._stats}")

    def _monitor_loop(self):
        """Background loop: receive and process V2V messages from RPi."""
        while self._running:
            try:
                messages = self.v2v.receive_all()
                for vid, msg in messages.items():
                    self._rpi_messages[vid] = msg
                    self._stats["messages_received"] += 1
                    self._print_rpi_message(msg)
            except Exception as e:
                if self._running:
                    print(f"[V2V ERROR] Receive: {e}")
            time.sleep(0.1)

    def _print_rpi_message(self, msg: V2VMessage):
        """Print a received RPi message in readable format."""
        age = msg.age_ms
        det_count = len(msg.detections)
        det_str = ""
        for d in msg.detections:
            det_str += f"\n    └─ {d.class_name} ({d.category}) @ {d.distance_from_sender:.2f}m, conf={d.confidence:.2f}"

        print(f"\n[V2V ← RPi] {msg.vehicle_id} | age={age:.0f}ms | intent={msg.intent}")
        print(f"    pos=({msg.position[0]:.2f}, {msg.position[1]:.2f}, {msg.position[2]:.0f}°)")
        print(f"    vel=({msg.velocity[0]:.1f} km/h, {msg.velocity[1]:.0f}°)")
        if det_count > 0:
            print(f"    detections ({det_count}):{det_str}")
        if msg.cooperation_request:
            print(f"    cooperation_request: {msg.cooperation_request}")

    def broadcast_detections(self, detections: list, intent: str = "cruising",
                              position: tuple = (0, 0, 0),
                              velocity: tuple = (0, 0),
                              cooperation_request: Optional[str] = None):
        """
        Broadcast PC-side detections to RPi.
        
        Args:
            detections: List of SharedDetection objects
            intent: Current intent string
            position: (x, y, yaw) position
            velocity: (speed, heading) velocity
            cooperation_request: Optional cooperation request
        """
        msg = self.v2v.create_message(
            position=position,
            velocity=velocity,
            detections=detections,
            intent=intent,
            cooperation_request=cooperation_request,
        )
        self.v2v.broadcast(msg)
        self._stats["messages_sent"] += 1
        self._stats["obstacles_shared"] += len(detections)

    def get_rpi_status(self) -> dict:
        """Get latest status from all connected RPi vehicles."""
        return dict(self._rpi_messages)

    def should_yield(self) -> bool:
        """Check if any RPi vehicle is requesting yield."""
        for vid, msg in self._rpi_messages.items():
            if msg.intent in ("yielding", "emergency_brake"):
                return True
            if msg.cooperation_request == "request_yield":
                return True
        return False

    def print_stats(self):
        """Print coordination statistics."""
        print(f"\n{'='*50}")
        print(f"  V2V Coordination Statistics")
        print(f"{'='*50}")
        print(f"  Messages received:  {self._stats['messages_received']}")
        print(f"  Messages sent:      {self._stats['messages_sent']}")
        print(f"  Obstacles shared:   {self._stats['obstacles_shared']}")
        print(f"  Yield events:       {self._stats['yield_events']}")
        print(f"  Connected vehicles: {list(self._rpi_messages.keys())}")
        print(f"{'='*50}")


def run_standalone(rpi_host: str, port: int):
    """
    Run the V2V coordinator in standalone mode.
    Monitors RPi vehicle status and provides basic coordination.
    """
    coordinator = PCV2VCoordinator(
        vehicle_id="pc_coordinator",
        rpi_host=rpi_host,
        listen_port=port,        # PC listens on this port
        send_port=port + 1,      # RPi listens on port+1
    )

    print(f"\n{'='*55}")
    print(f"  PC V2V Cooperative Coordinator")
    print(f"  RPi target: {rpi_host}")
    print(f"  Ports: listen={port}, send={port+1}")
    print(f"{'='*55}\n")

    coordinator.start()

    # Simulate periodic PC-side awareness broadcast
    # In production, this would be fed by YOLO detection on PC
    broadcast_interval = 2.0  # seconds
    last_broadcast = 0

    try:
        while True:
            now = time.time()

            # Periodically broadcast PC presence (heartbeat-like)
            if now - last_broadcast >= broadcast_interval:
                # Check if any RPi needs yielding
                rpi_status = coordinator.get_rpi_status()
                
                intent = "cruising"
                coop_req = None
                for vid, msg in rpi_status.items():
                    if msg.intent == "yielding":
                        # Acknowledge yield request
                        intent = "cooperative_monitoring"
                        coop_req = "request_pass"
                        coordinator._stats["yield_events"] += 1

                # Broadcast PC status (no detections in standalone mode)
                coordinator.broadcast_detections(
                    detections=[],
                    intent=intent,
                    cooperation_request=coop_req,
                )
                last_broadcast = now

                # Compact status line
                n_vehicles = len(rpi_status)
                rpi_intents = [m.intent for m in rpi_status.values()]
                sys.stdout.write(
                    f"\r[V2V] Vehicles: {n_vehicles} | "
                    f"RPi intents: {rpi_intents} | "
                    f"Rx: {coordinator._stats['messages_received']} "
                    f"Tx: {coordinator._stats['messages_sent']}  "
                )
                sys.stdout.flush()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n[V2V] Interrupted")
    finally:
        coordinator.print_stats()
        coordinator.stop()


def run_with_camera(rpi_host: str, port: int, camera_index: int = 0):
    """
    Run V2V coordinator with camera-based detection on PC.
    Uses YOLO detector to find obstacles and shares them with RPi.
    """
    try:
        from perception.detector import YOLODetector
    except ImportError:
        print("[ERROR] YOLO detector not available. Run in standalone mode instead.")
        return

    try:
        import cv2
    except ImportError:
        print("[ERROR] OpenCV not available. Install: pip install opencv-python")
        return

    coordinator = PCV2VCoordinator(
        vehicle_id="pc_coordinator",
        rpi_host=rpi_host,
        listen_port=port,
        send_port=port + 1,
    )

    # Load YOLO detector
    detector = YOLODetector({"use_onnx": True, "imgsz": 640, "confidence_threshold": 0.3})

    # Open camera
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {camera_index}")
        return

    print(f"\n{'='*55}")
    print(f"  PC V2V Coordinator + Camera Detection")
    print(f"  RPi target: {rpi_host}")
    print(f"  Camera: index {camera_index}")
    print(f"{'='*55}\n")

    coordinator.start()

    try:
        while True:
            # Read camera frame
            ret, frame = cap.read()
            if not ret:
                continue

            # YOLO detection
            detections = detector.detect(frame)

            # Convert to SharedDetection
            shared_dets = []
            for det in detections:
                if hasattr(det, 'distance') and det.distance > 0:
                    shared_dets.append(SharedDetection(
                        class_name=det.class_name,
                        category=det.category,
                        confidence=det.confidence,
                        world_x=det.distance,
                        world_y=0.0,
                        distance_from_sender=det.distance,
                    ))

            # Broadcast to RPi
            coordinator.broadcast_detections(
                detections=shared_dets,
                intent="cruising",
            )

            # Display
            display = frame.copy()
            for det in detections:
                x1, y1, x2, y2 = det.bbox
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(display, f"{det.class_name} {det.confidence:.2f}",
                           (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow("PC V2V Coordinator", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[V2V] Interrupted")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        coordinator.print_stats()
        coordinator.stop()


def main():
    parser = argparse.ArgumentParser(
        description="PC V2V Cooperative Coordinator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Standalone monitoring mode
  python -m rpi_deploy.pc_v2v_coordinator --rpi-host 192.168.137.33

  # With camera detection
  python -m rpi_deploy.pc_v2v_coordinator --rpi-host 192.168.137.33 --camera

  # Custom ports
  python -m rpi_deploy.pc_v2v_coordinator --rpi-host 192.168.1.10 --port 5555
        """)
    parser.add_argument("--rpi-host", required=True,
                        help="Raspberry Pi IP address")
    parser.add_argument("--port", type=int, default=5555,
                        help="V2V communication port (default: 5555)")
    parser.add_argument("--camera", action="store_true",
                        help="Enable camera-based detection on PC")
    parser.add_argument("--camera-index", type=int, default=0,
                        help="Camera device index (default: 0)")
    args = parser.parse_args()

    if args.camera:
        run_with_camera(args.rpi_host, args.port, args.camera_index)
    else:
        run_standalone(args.rpi_host, args.port)


if __name__ == "__main__":
    main()