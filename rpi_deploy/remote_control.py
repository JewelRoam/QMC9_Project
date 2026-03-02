#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派小车远程控制服务器模块
Remote Control Server Module for Raspberry Pi Robot Car

基于TCP Socket的网络遥控接口
支持键盘/手柄控制和视频流传输
"""

import socket
import threading
import json
import time
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass, asdict
from enum import Enum

from .hardware_config import hardware_config


class CommandType(Enum):
    """命令类型枚举"""
    MOVE = "move"           # 运动控制
    STOP = "stop"           # 停止
    SERVO = "servo"         # 舵机控制
    CAMERA = "camera"       # 摄像头控制
    GET_STATUS = "status"   # 获取状态
    SCAN = "scan"           # 超声波扫描
    AUTO_MODE = "auto"      # 自动模式切换
    HEARTBEAT = "heartbeat" # 心跳


@dataclass
class ControlCommand:
    """控制命令数据结构"""
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
    """车辆状态数据结构"""
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


class RemoteControlServer:
    """
    远程控制服务器
    通过TCP Socket接收控制命令并执行
    """
    
    def __init__(self, host: str = None, port: int = None):
        self.config = hardware_config.network
        self.host = host or self.config.control_host
        self.port = port or self.config.control_port
        
        self._socket: Optional[socket.socket] = None
        self._running = False
        self._server_thread: Optional[threading.Thread] = None
        self._clients: list = []
        
        # 回调函数注册表
        self._command_handlers: Dict[str, Callable] = {}
        self._status_provider: Optional[Callable[[], VehicleStatus]] = None
        
        print(f"[INFO] Remote control server configured: {self.host}:{self.port}")
    
    def register_handler(self, cmd_type: str, handler: Callable[[Dict], Any]):
        """注册命令处理器"""
        self._command_handlers[cmd_type] = handler
        print(f"[INFO] Registered handler for '{cmd_type}'")
    
    def register_status_provider(self, provider: Callable[[], VehicleStatus]):
        """注册状态提供器"""
        self._status_provider = provider
    
    def start(self):
        """启动服务器"""
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
        """停止服务器"""
        self._running = False
        
        # 关闭所有客户端连接
        for client in self._clients:
            try:
                client.close()
            except:
                pass
        self._clients.clear()
        
        if self._socket:
            self._socket.close()
            self._socket = None
        
        print("[INFO] Remote control server stopped")
    
    def _server_loop(self):
        """服务器主循环"""
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
        """处理客户端连接"""
        self._clients.append(client_socket)
        buffer = ""
        
        try:
            while self._running:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break
                
                buffer += data
                
                # 处理完整的消息（以换行分隔）
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
        """处理单个命令"""
        try:
            command = ControlCommand.from_json(data)
            cmd_type = command.cmd_type
            
            if cmd_type in self._command_handlers:
                result = self._command_handlers[cmd_type](command.params)
                return json.dumps({'success': True, 'result': result})
            
            elif cmd_type == CommandType.GET_STATUS.value:
                if self._status_provider:
                    status = self._status_provider()
                    return status.to_json()
                return json.dumps({'error': 'Status provider not registered'})
            
            elif cmd_type == CommandType.HEARTBEAT.value:
                return json.dumps({'alive': True, 'timestamp': time.time()})
            
            else:
                return json.dumps({'error': f'Unknown command: {cmd_type}'})
        
        except json.JSONDecodeError:
            return json.dumps({'error': 'Invalid JSON format'})
        except Exception as e:
            return json.dumps({'error': str(e)})


class RemoteControlClient:
    """
    远程控制客户端
    用于PC端发送控制命令到树莓派
    """
    
    def __init__(self, host: str, port: int = None):
        self.host = host
        self.port = port or hardware_config.network.control_port
        self._socket: Optional[socket.socket] = None
        self._connected = False
    
    def connect(self) -> bool:
        """连接到服务器"""
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
        """断开连接"""
        if self._socket:
            self._socket.close()
            self._socket = None
        self._connected = False
        print("[INFO] Disconnected")
    
    def send_command(self, cmd_type: str, params: Dict = None) -> Dict:
        """发送命令并接收响应"""
        if not self._connected or not self._socket:
            return {'error': 'Not connected'}
        
        command = ControlCommand(
            cmd_type=cmd_type,
            params=params or {},
            timestamp=time.time()
        )
        
        try:
            self._socket.send((command.to_json() + '\n').encode('utf-8'))
            
            # 等待响应
            self._socket.settimeout(5.0)
            response = self._socket.recv(4096).decode('utf-8').strip()
            return json.loads(response)
        
        except socket.timeout:
            return {'error': 'Response timeout'}
        except Exception as e:
            return {'error': str(e)}
    
    # ===== 便捷控制方法 =====
    
    def move_forward(self, speed: float = 50.0):
        """前进"""
        return self.send_command('move', {'direction': 'forward', 'speed': speed})
    
    def move_backward(self, speed: float = 50.0):
        """后退"""
        return self.send_command('move', {'direction': 'backward', 'speed': speed})
    
    def turn_left(self, speed: float = 50.0):
        """左转"""
        return self.send_command('move', {'direction': 'left', 'speed': speed})
    
    def turn_right(self, speed: float = 50.0):
        """右转"""
        return self.send_command('move', {'direction': 'right', 'speed': speed})
    
    def stop(self):
        """停止"""
        return self.send_command('stop')
    
    def set_servo(self, servo_type: str, angle: float):
        """设置舵机角度"""
        return self.send_command('servo', {'type': servo_type, 'angle': angle})
    
    def scan_ultrasonic(self):
        """触发超声波扫描"""
        return self.send_command('scan')
    
    def get_status(self) -> VehicleStatus:
        """获取车辆状态"""
        response = self.send_command('status')
        if 'error' not in response:
            return VehicleStatus(**response)
        return None
    
    def heartbeat(self) -> bool:
        """检查连接状态"""
        response = self.send_command('heartbeat')
        return 'alive' in response and response['alive']


def create_remote_server(host: str = None, port: int = None) -> RemoteControlServer:
    """创建远程控制服务器实例"""
    return RemoteControlServer(host, port)


def create_remote_client(host: str, port: int = None) -> RemoteControlClient:
    """创建远程控制客户端实例"""
    return RemoteControlClient(host, port)


if __name__ == "__main__":
    # 测试服务器
    print("Testing Remote Control Server...")
    server = create_remote_server()
    
    # 注册示例处理器
    def handle_move(params):
        print(f"Move command: {params}")
        return "Moving"
    
    def handle_stop(params):
        print("Stop command")
        return "Stopped"
    
    server.register_handler('move', handle_move)
    server.register_handler('stop', handle_stop)
    
    server.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.stop()
