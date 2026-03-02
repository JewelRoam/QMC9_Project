@echo off
chcp 65001 >nul
echo ==========================================
echo QMC9 ROS 2 Stack Launcher (Jazzy)
echo ==========================================
echo.

REM Set ROS 2 path - modify this if your installation is elsewhere
set "ROS2_PATH=C:\pixi_ws\ros2-windows"

if not exist "%ROS2_PATH%\setup.bat" (
    echo [ERROR] Could not find ROS 2 setup script at %ROS2_PATH%
    echo Please edit this batch file and set ROS2_PATH to your ROS 2 installation.
    pause
    exit /b 1
)

echo [INFO] Found ROS 2 at %ROS2_PATH%
echo [INFO] Activating ROS 2 environment...
echo.

REM Change to project directory first
cd /d "%~dp0.."

REM Source ROS 2 environment in current shell
call "%ROS2_PATH%\setup.bat"

REM Use the Python from ROS 2 pixi environment
set "ROS_PYTHON=C:\pixi_ws\.pixi\envs\default\python.exe"

if not exist "%ROS_PYTHON%" (
    echo [ERROR] Could not find ROS 2 Python at %ROS_PYTHON%
    pause
    exit /b 1
)

echo [INFO] Using ROS 2 Python: %ROS_PYTHON%
echo.

REM Verify rclpy is accessible
echo [INFO] Verifying ROS 2 Python bindings...
"%ROS_PYTHON%" -c "import rclpy; print('[OK] rclpy imported successfully')" 2>nul
if errorlevel 1 (
    echo [ERROR] Cannot import rclpy. ROS 2 environment may not be set correctly.
    pause
    exit /b 1
)

REM Check if required packages are installed
echo [INFO] Checking dependencies...
"%ROS_PYTHON%" -c "import zmq; import numpy; import cv2" 2>nul
if errorlevel 1 (
    echo [WARNING] Missing dependencies. Installing pyzmq, numpy, opencv-python...
    "%ROS_PYTHON%" -m pip install pyzmq numpy opencv-python pyyaml
    if errorlevel 1 (
        echo [ERROR] Failed to install dependencies.
        pause
        exit /b 1
    )
)

echo [OK] Dependencies ready
echo.

REM Parse arguments
set "RUN_MODE=--mode algorithm_only"
if "%~1"=="--full" (
    set "RUN_MODE="
    echo [INFO] Running full stack with CARLA integration
) else (
    echo [INFO] Running algorithm-only mode (requires ZMQ bridge running separately)
    echo [NOTE] Make sure carla_zmq_bridge.py is running in another terminal
)

echo.
echo Starting ROS 2 Stack...
echo Press Ctrl+C to stop
echo.

REM Run the simulation
"%ROS_PYTHON%" ros2_nodes\run_simulation.py %RUN_MODE%

pause
