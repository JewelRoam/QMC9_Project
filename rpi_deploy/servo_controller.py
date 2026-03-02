#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派小车舵机控制模块
Servo Controller Module for Raspberry Pi Robot Car

基于PCA9685 I2C PWM控制器的舵机驱动
兼容创乐博LOBOROBOT扩展板
"""

import time
import math
from typing import Optional, Tuple

# 尝试导入smbus，如果失败则使用模拟模式
try:
    import smbus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False
    print("[WARNING] smbus not available, running in simulation mode")

from .hardware_config import hardware_config


class PCA9685:
    """
    PCA9685 I2C PWM控制器驱动类
    16通道PWM输出，12位分辨率
    """
    
    # PCA9685寄存器地址
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
    
    # 位定义
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
        """初始化PCA9685设备"""
        if self._bus is None:
            return
        
        # 复位设备
        self._write_byte(self.__MODE1, self.__ALLCALL)
        time.sleep(0.005)  # 等待振荡器稳定
        
        # 设置模式
        mode1 = self._read_byte(self.__MODE1)
        mode1 = mode1 & ~self.__SLEEP  # 退出睡眠模式
        self._write_byte(self.__MODE1, mode1)
        time.sleep(0.005)
        
        # 设置输出模式
        self._write_byte(self.__MODE2, self.__OUTDRV)
    
    def _write_byte(self, reg: int, value: int):
        """写入单个字节"""
        if self._bus:
            self._bus.write_byte_data(self.address, reg, value)
    
    def _read_byte(self, reg: int) -> int:
        """读取单个字节"""
        if self._bus:
            return self._bus.read_byte_data(self.address, reg)
        return 0
    
    def set_pwm_freq(self, freq_hz: float):
        """
        设置PWM频率
        
        Args:
            freq_hz: 频率(Hz)，舵机通常使用50Hz (20ms周期)
        """
        if self._bus is None:
            print(f"[SIM] Set PWM frequency to {freq_hz}Hz")
            return
        
        # 计算预分频值
        prescaleval = 25000000.0  # 25MHz内部时钟
        prescaleval /= 4096.0     # 12位分辨率
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        
        # 进入睡眠模式以设置预分频
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
        设置指定通道的PWM输出
        
        Args:
            channel: 通道号 (0-15)
            on: 开启计数值 (0-4095)
            off: 关闭计数值 (0-4095)
        """
        if self._bus is None:
            return
        
        self._write_byte(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self._write_byte(self.__LED0_ON_H + 4 * channel, on >> 8)
        self._write_byte(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self._write_byte(self.__LED0_OFF_H + 4 * channel, off >> 8)
    
    def set_duty_cycle(self, channel: int, pulse: float):
        """
        设置占空比百分比
        
        Args:
            channel: 通道号
            pulse: 占空比 (0-100)
        """
        pulse = max(0, min(100, pulse))
        off_value = int(pulse * 4096 / 100)
        self.set_pwm(channel, 0, off_value)
    
    def set_level(self, channel: int, value: int):
        """
        设置数字电平输出
        
        Args:
            channel: 通道号
            value: 0=低电平, 1=高电平
        """
        if value == 0:
            self.set_pwm(channel, 0, 0)
        else:
            self.set_pwm(channel, 0, 4095)
    
    def cleanup(self):
        """清理资源，关闭所有输出"""
        if self._bus:
            for channel in range(16):
                self.set_pwm(channel, 0, 0)
            self._bus.close()
            print("[INFO] PCA9685 cleaned up")


class ServoController:
    """
    舵机控制器
    管理超声波云台和摄像头云台的舵机
    """
    
    # 舵机脉冲宽度范围 (微秒)
    SERVO_MIN_PULSE = 500   # 0.5ms = 0度
    SERVO_MAX_PULSE = 2500  # 2.5ms = 180度
    
    def __init__(self):
        self.config = hardware_config.i2c
        self.ultrasonic_config = hardware_config.ultrasonic
        self.camera_config = hardware_config.camera
        
        # 初始化PCA9685
        self._pca = PCA9685(
            address=self.config.pca9685_address,
            bus_number=self.config.bus_number
        )
        
        # 设置PWM频率
        self._pca.set_pwm_freq(self.config.pwm_frequency)
        
        # 当前角度状态
        self._ultrasonic_angle = self.ultrasonic_config.center_angle
        self._camera_pan = 90
        self._camera_tilt = 45
        
        # 初始化到中心位置
        self.center_all()
    
    def _angle_to_pulse(self, angle: float) -> int:
        """
        将角度转换为PCA9685脉冲计数值
        
        公式: 4096 * ((angle * 11) + 500) / 20000
        其中11是每度的脉冲增量 (2000us/180度 ≈ 11.11)
        500是0.5ms对应的偏移量
        """
        # 限制角度范围
        angle = max(0, min(180, angle))
        
        # 创乐博LOBOROBOT公式
        pulse = int(4096 * ((angle * 11) + 500) / 20000)
        return max(0, min(4095, pulse))
    
    def set_servo_angle(self, channel: int, angle: float):
        """
        设置舵机角度
        
        Args:
            channel: PCA9685通道号
            angle: 角度 (0-180度)
        """
        pulse = self._angle_to_pulse(angle)
        self._pca.set_pwm(channel, 0, pulse)
        
        # 更新状态
        if channel == self.ultrasonic_config.servo_channel:
            self._ultrasonic_angle = angle
        elif channel == self.camera_config.pan_servo_channel:
            self._camera_pan = angle
        elif channel == self.camera_config.tilt_servo_channel:
            self._camera_tilt = angle
    
    # ========== 超声波云台控制 ==========
    
    def set_ultrasonic_angle(self, angle: float):
        """设置超声波传感器角度"""
        angle = max(self.ultrasonic_config.min_angle,
                   min(self.ultrasonic_config.max_angle, angle))
        self.set_servo_angle(self.ultrasonic_config.servo_channel, angle)
    
    def center_ultrasonic(self):
        """超声波传感器归中"""
        self.set_ultrasonic_angle(self.ultrasonic_config.center_angle)
    
    def scan_ultrasonic(self, angles: Tuple[int, ...] = None, 
                       delay: float = None) -> dict:
        """
        扫描指定角度序列
        
        Args:
            angles: 扫描角度列表，默认使用配置中的序列
            delay: 每次移动后的延迟
        
        Returns:
            角度位置字典 {angle: timestamp}
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
    
    # ========== 摄像头云台控制 ==========
    
    def set_camera_pan(self, angle: float):
        """设置摄像头水平角度"""
        angle = max(self.camera_config.pan_min,
                   min(self.camera_config.pan_max, angle))
        self.set_servo_angle(self.camera_config.pan_servo_channel, angle)
    
    def set_camera_tilt(self, angle: float):
        """设置摄像头俯仰角度"""
        angle = max(self.camera_config.tilt_min,
                   min(self.camera_config.tilt_max, angle))
        self.set_servo_angle(self.camera_config.tilt_servo_channel, angle)
    
    def set_camera_position(self, pan: float, tilt: float):
        """同时设置摄像头水平和俯仰角度"""
        self.set_camera_pan(pan)
        self.set_camera_tilt(tilt)
    
    def center_camera(self):
        """摄像头归中"""
        self.set_camera_position(90, 45)
    
    def track_object(self, delta_x: float, delta_y: float, 
                    speed: float = 2.0):
        """
        根据目标偏移量调整摄像头跟踪
        
        Args:
            delta_x: 水平方向偏移 (-1 to 1, 负值向左)
            delta_y: 垂直方向偏移 (-1 to 1, 负值向上)
            speed: 跟踪速度系数
        """
        new_pan = self._camera_pan - delta_x * speed * 5
        new_tilt = self._camera_tilt + delta_y * speed * 5
        self.set_camera_position(new_pan, new_tilt)
    
    def center_all(self):
        """所有舵机归中"""
        self.center_ultrasonic()
        self.center_camera()
    
    def get_status(self) -> dict:
        """获取舵机状态"""
        return {
            'ultrasonic_angle': self._ultrasonic_angle,
            'camera_pan': self._camera_pan,
            'camera_tilt': self._camera_tilt,
            'pca9685_address': f"0x{self.config.pca9685_address:02X}",
            'pwm_frequency': self.config.pwm_frequency
        }
    
    def test_sequence(self):
        """测试序列"""
        print("\n=== Servo Test Sequence ===")
        
        # 测试超声波云台
        print("\nTesting ultrasonic servo...")
        for angle in [90, 45, 0, 45, 90, 135, 180, 135, 90]:
            print(f"  Setting ultrasonic angle to {angle}°")
            self.set_ultrasonic_angle(angle)
            time.sleep(0.5)
        
        # 测试摄像头云台
        print("\nTesting camera servos...")
        positions = [
            (90, 45),   # 中心
            (45, 30),   # 左下
            (135, 30),  # 右下
            (135, 60),  # 右上
            (45, 60),   # 左上
            (90, 45),   # 回到中心
        ]
        for pan, tilt in positions:
            print(f"  Setting camera to pan={pan}°, tilt={tilt}°")
            self.set_camera_position(pan, tilt)
            time.sleep(0.5)
        
        print("\n=== Test Complete ===")
    
    def cleanup(self):
        """清理资源"""
        self.center_all()
        time.sleep(0.5)
        self._pca.cleanup()


# 便捷函数
def create_servo_controller() -> ServoController:
    """创建舵机控制器实例"""
    return ServoController()


if __name__ == "__main__":
    # 独立测试
    print("Testing Servo Controller...")
    controller = create_servo_controller()
    
    try:
        controller.test_sequence()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.cleanup()
