# NexusPilot: Raspberry Pi 5 Hardware Guide

This guide provides the essential hardware specifications, pin mappings, and troubleshooting steps for deploying the NexusPilot system onto the physical 4WD robot car.

## 🛠️ Hardware Specifications

- **Compute**: Raspberry Pi 5 (8GB Recommended)
- **Expansion Board**: LOBOROBOT Smart Robot Expansion Board V3.0
- **PWM Driver**: PCA9685 I2C (Address: `0x40`)
- **Distance Sensing**: HC-SR04 Ultrasonic Sensor
- **Vision**: Standard USB UVC Camera ($640 \times 480$)

## 📌 Pin Mapping

| Component | Function | GPIO Pins (BCM) |
|-----------|----------|-----------------|
| **Left Motors** | Forward, Backward, Enable | 22, 27, 18 |
| **Right Motors**| Forward, Backward, Enable | 25, 24, 23 |
| **Ultrasonic**  | Trigger, Echo | 20, 21 |
| **Status LED**  | Green, Red | 5, 6 |
| **User Button** | Start/Stop Trigger | 19 |
| **I2C Bus**     | SDA, SCL | 2, 3 |

**Servo Channels (PCA9685):**
- `Ch 0`: Ultrasonic Pan-Tilt (0° - 180°)
- `Ch 1`: Camera Horizontal Pan
- `Ch 2`: Camera Vertical Tilt

## 📦 Environment Setup

1. **Enable I2C:**
   ```bash
   sudo raspi-config # Interface Options -> I2C -> Enable
   ```

2. **Install Dependencies:**
   ```bash
   sudo apt-get update && sudo apt-get install -y python3-opencv i2c-tools
   pip3 install gpiozero smbus2 numpy pyyaml
   ```

3. **Check I2C Connectivity:**
   ```bash
   sudo i2cdetect -y 1 # Should show 0x40
   ```

## 🔧 Troubleshooting

- **Motor Deadband:** If motors hum but don't spin, increase `base_pwm` in `config/config.yaml`.
- **Ultrasonic Lockup:** Ensure Trig/Echo wires are secure. The software includes a 30ms hard timeout safety.
- **Camera Lag:** Use a high-speed USB port. Ensure resolution is set to $320 \times 320$ or $640 \times 480$ for optimal FPS.
- **Permissions:** If GPIO access is denied, run `sudo usermod -a -G gpio $USER`.

---
*Refer to the main project README for autonomous navigation logic and evaluation results.*
