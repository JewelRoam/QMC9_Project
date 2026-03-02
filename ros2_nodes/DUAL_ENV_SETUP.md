# QMC9 双环境架构设置指南

## 问题背景

- **CARLA 0.9.13**: 需要 Python 3.7
- **ROS 2 Jazzy**: 需要 Python 3.12 (通过 pixi 环境)
- **解决方案**: 使用 ZeroMQ 进程间通信(IPC)将两个环境解耦

## 关键发现 (Windows ROS 2)

> ⚠️ **重要**: Windows 上的 ROS 2 Jazzy 使用 pixi 包管理器自带的 Python 3.12
> - Python 路径: `C:\pixi_ws\.pixi\envs\default\python.exe`
> - **不要**创建额外的虚拟环境给 ROS 2 使用
> - 必须先调用 `setup.bat` 才能正确加载 rclpy

## 架构设计

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Python 3.7 环境 (CARLA)                              │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                    carla_bridge/carla_zmq_bridge.py                   │  │
│  │  • 连接 CARLA 模拟器                                                   │  │
│  │  • 发布传感器数据到 ZeroMQ (端口 5555-5558)                            │  │
│  │  • 接收控制命令                                                        │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────┬─────────────────────────────────────────┘
                                    │ ZeroMQ IPC
                                    │ (跨Python版本通信)
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        Python 3.12 环境 (ROS 2 Jazzy)                        │
│                                                                              │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────────────┐  │
│  │   zmq_to_ros    │    │ perception_node │    │      planning_node      │  │
│  │    _node.py     │◄───│                 │◄───│                         │  │
│  │                 │    │ • YOLO检测       │    │  • APF路径规划           │  │
│  │ • 接收ZMQ消息    │    │ • 深度估计       │    │  • 避障决策              │  │
│  │ • 转为ROS话题    │    │ • 发布检测结果   │    │  • 发布路径              │  │
│  └────────┬────────┘    └─────────────────┘    └────────────┬────────────┘  │
│           │                                                  │               │
│           └──────────────────────────────────────────────────┘               │
│                              │                                               │
│                     ROS 2 Topics (标准接口)                                   │
├──────────────────────────────┼───────────────────────────────────────────────┤
│                      RViz可视化 / rosbag记录 / 真实车辆控制                     │
└──────────────────────────────┴───────────────────────────────────────────────┘
```

## 目录结构

```
ros2_nodes/
├── README.md                           # 主要文档
├── ARCHITECTURE.md                     # 详细架构说明
├── DUAL_ENV_SETUP.md                   # 本文件
│
├── run_carla_bridge.bat                # 启动CARLA桥接 (Python 3.7)
├── run_ros_stack.bat                   # 启动ROS 2栈 (Python 3.12)
│
├── carla_bridge/                       # Python 3.7 环境 (CARLA端)
│   ├── __init__.py
│   ├── carla_zmq_bridge.py             # CARLA ↔ ZeroMQ桥接
│   └── requirements.txt                # Python 3.7依赖
│
├── zmq_to_ros_node.py                  # ZeroMQ → ROS 2转换
├── perception_node.py                  # 感知节点
├── planning_node.py                    # 规划节点
├── control_node.py                     # 控制节点
├── cooperation_node.py                 # 协同节点
├── run_simulation.py                   # 主运行脚本
│
└── launch/
    └── simulation.launch.py            # ROS 2启动文件
```

## 环境配置

### 1. CARLA 环境 (Python 3.7)

**运行自动设置脚本 (推荐):**
```batch
# 这将创建 venv_carla 虚拟环境并安装所有依赖
setup_carla_env.bat
```

**手动配置 (如果已有 Python 3.7):**
```batch
# 创建虚拟环境
python -m venv venv_carla

# 激活并安装依赖
venv_carla\Scripts\activate
pip install pyzmq numpy opencv-python pyyaml
```

### 2. ROS 2 环境 (Python 3.12)

**无需手动配置!**

ROS 2 Jazzy 已自带 Python 3.12:
- 位置: `C:\pixi_ws\.pixi\envs\default\python.exe`
- `run_ros_stack.bat` 会自动处理环境设置

**注意**: 不要为 ROS 2 创建虚拟环境，直接使用 pixi 环境的 Python。

## 使用方法

### 快速启动 (推荐)

**终端 1 - 启动 CARLA:**
```batch
cd CARLA_0.9.13\WindowsNoEditor
CarlaUE4.exe -RenderOffScreen
```

**终端 2 - 启动 CARLA 桥接:**
```batch
ros2_nodes\run_carla_bridge.bat
```

**终端 3 - 启动 ROS 2 栈:**
```batch
ros2_nodes\run_ros_stack.bat
```

### 手动启动 (调试用途)

**CARLA 端 (Python 3.7):**
```batch
venv_carla\Scripts\activate
python ros2_nodes/carla_bridge/carla_zmq_bridge.py
```

**ROS 2 端 (Python 3.12):**
```batch
# 必须先 source ROS 2 环境
call C:\pixi_ws\ros2-windows\setup.bat

# 使用 pixi 的 Python 运行
C:\pixi_ws\.pixi\envs\default\python.exe ros2_nodes/run_simulation.py --mode algorithm_only
```

## 通信协议 (ZeroMQ)

### 端口分配

| 端口 | 方向 | 数据类型 | 说明 |
|------|------|----------|------|
| 5555 | CARLA → ROS | RGB 图像 | 相机原始图像 |
| 5556 | CARLA → ROS | 深度图像 | 深度估计用 |
| 5557 | CARLA → ROS | 里程计 | 车辆位置和速度 |
| 5558 | ROS → CARLA | 控制命令 | 油门/转向/刹车 |

### 消息格式

**图像数据 (multipart):**
```json
// Frame 1: metadata
{
  "timestamp": 1234567890.123,
  "frame_id": "camera_link",
  "width": 1280,
  "height": 720,
  "encoding": "rgb8",
  "dtype": "uint8"
}
// Frame 2: binary image data
```

**控制命令:**
```json
{
  "timestamp": 1234567890.123,
  "throttle": 0.5,
  "steering": 0.1,
  "brake": 0.0,
  "hand_brake": false,
  "reverse": false
}
```

## 经验教训

### ❌ 失败的尝试

1. **在虚拟环境中安装 ROS 2 的 rclpy**
   - 问题: DLL 加载失败，找不到 `_rclpy_pybind11`
   - 原因: rclpy 需要 ROS 2 特定的 PATH 和 DLL

2. **使用系统默认的 Python 3.12**
   - 问题: 即使安装了 rclpy，也无法找到 ROS 2 的其他组件
   - 原因: ROS 2 需要特定的 site-packages 路径

3. **直接导入 CARLA 到 ROS 2 节点**
   - 问题: Python 版本冲突 (3.7 vs 3.12)
   - 结果: 解释器崩溃

### ✅ 成功的方案

1. **使用 ZeroMQ 进行跨进程通信**
   - 完全隔离 Python 版本
   - 高性能，低延迟
   - 可扩展性强

2. **使用 ROS 2 自带的 pixi Python**
   - 路径: `C:\pixi_ws\.pixi\envs\default\python.exe`
   - 先调用 `setup.bat` 设置环境变量
   - 所有依赖都在这个环境中安装

3. **批处理脚本自动化环境设置**
   - `run_ros_stack.bat` 自动处理复杂的环境配置
   - 用户无需手动设置 PYTHONPATH 等变量

## 故障排除

### "ImportError: DLL load failed while importing _rclpy_pybind11"

**原因**: 没有正确 source ROS 2 环境，或者使用了错误的 Python

**解决**:
```batch
# 确保先调用 setup.bat
call C:\pixi_ws\ros2-windows\setup.bat

# 使用 pixi 的 Python，而不是系统 Python
C:\pixi_ws\.pixi\envs\default\python.exe -c "import rclpy"
```

### "ImportError: cannot import name 'Protocol' from 'typing'"

**原因**: 使用了 Python 3.7 运行 ROS 2 代码

**解决**: 
- ROS 2 Jazzy 需要 Python 3.12
- 检查 `python --version` 确认版本

### "ZMQ connection failed"

**原因**: 端口被占用或防火墙阻止

**解决**:
```batch
# 检查端口占用
netstat -ano | findstr 5555

# 关闭防火墙测试 (仅用于调试)
# 或在防火墙中添加例外规则
```

### "Cannot connect to CARLA"

**检查清单**:
1. CarlaUE4.exe 是否正在运行? (任务管理器)
2. 端口 2000 是否可访问?
3. 尝试 mock 模式: `run_carla_bridge.bat --mock`

## 下一步计划

1. ✅ 完成双环境架构设计
2. ✅ 实现 ZeroMQ 通信
3. ✅ 解决 Windows ROS 2 Python 环境问题
4. 🔄 测试完整流程: CARLA → ZMQ → ROS 2
5. ⏳ 集成真实树莓派小车
6. ⏳ 添加 rosbag2 记录功能
7. ⏳ 实现多车协同仿真

## 参考资源

- [ROS 2 Windows 安装指南](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html)
- [ZeroMQ 指南](https://zguide.zeromq.org/)
- [CARLA 文档](https://carla.readthedocs.io/)
- [ARCHITECTURE.md](ARCHITECTURE.md) - 详细架构文档
