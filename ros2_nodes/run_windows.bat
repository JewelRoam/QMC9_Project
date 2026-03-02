@echo off
chcp 65001 >nul
echo ==========================================
echo QMC9 ROS 2 Simulation Launcher (Windows)
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

REM Use the Python from ROS 2 installation
set "ROS_PYTHON=%ROS2_PATH%\python.exe"
if not exist "%ROS_PYTHON%" (
    set "ROS_PYTHON=%ROS2_PATH%\Scripts\python.exe"
)
if not exist "%ROS_PYTHON%" (
    for /f "delims=" %%i in ('where python') do (
        set "ROS_PYTHON=%%i"
        goto :found_python
    )
)
:found_python

echo [INFO] Using Python: %ROS_PYTHON%
echo [INFO] PYTHONPATH: %PYTHONPATH%
echo.

REM Verify rclpy is available
echo [INFO] Verifying ROS 2 Python bindings...
"%ROS_PYTHON%" -c "import rclpy; print('[OK] rclpy version:', rclpy.__version__)" 2>nul
if errorlevel 1 (
    echo [WARNING] Failed to import rclpy with detected Python.
    echo [INFO] Trying with explicit PYTHONPATH...
    
    REM Try adding all possible paths
    set "PYTHONPATH=%ROS2_PATH%\Lib\site-packages;%ROS2_PATH%\Lib;%PYTHONPATH%"
    
    "%ROS_PYTHON%" -c "import rclpy; print('[OK] rclpy version:', rclpy.__version__)" 2>nul
    if errorlevel 1 (
        echo [ERROR] Still cannot import rclpy. Please check your ROS 2 installation.
        echo.
        echo Troubleshooting steps:
        echo 1. Ensure you installed ROS 2 correctly on Windows
        echo 2. Check that Python version matches ^(ROS 2 Jazzy requires Python 3.12^)
        echo 3. Try running 'python -c "import rclpy"' manually after calling setup.bat
        pause
        exit /b 1
    )
)

echo.
echo Starting QMC9 simulation...
echo.

REM Run the simulation with all arguments passed through
"%ROS_PYTHON%" ros2_nodes\run_simulation.py %*

pause
