@echo off
chcp 65001 >nul
echo ==========================================
echo QMC9 CARLA Bridge Launcher (Python 3.7)
echo ==========================================
echo .

REM Change to project directory
cd /d "%~dp0.."

REM Find Python 3.7 - check venv_carla first, then system Python
set "PYTHON_EXE="

REM Check for venv_carla first (created by setup_carla_env.bat)
if exist "venv_carla\Scripts\python.exe" (
    set "PYTHON_EXE=venv_carla\Scripts\python.exe"
    echo [INFO] Using venv_carla Python
    goto :found_python
)

REM Try to find python3.7 in PATH
for /f "delims=" %%i in ('where python 2^>nul') do (
    set "PYTHON_EXE=%%i"
    echo [INFO] Found Python at: %%i
    goto :found_python
)

:found_python
if "%PYTHON_EXE%"=="" (
    echo [ERROR] Could not find Python. Please run setup_carla_env.bat first.
    pause
    exit /b 1
)

echo [INFO] Python: %PYTHON_EXE%
echo.

REM Check if required packages are installed
echo [INFO] Checking dependencies...
%PYTHON_EXE% -c "import zmq; import numpy; import cv2" 2>nul
if errorlevel 1 (
    echo [WARNING] Missing dependencies. Installing...
    %PYTHON_EXE% -m pip install -r ros2_nodes\carla_bridge\requirements.txt
    if errorlevel 1 (
        echo [ERROR] Failed to install dependencies.
        pause
        exit /b 1
    )
)

echo [OK] Dependencies ready
echo.

REM Parse arguments
set "BRIDGE_ARGS="
if "%~1"=="--mock" (
    set "BRIDGE_ARGS=--mock"
    echo [INFO] Running in MOCK mode (no CARLA connection)
) else (
    echo [INFO] Connecting to CARLA at localhost:2000
    echo [NOTE] Make sure CARLA is running before starting the bridge
)

echo.
echo Starting CARLA Bridge...
echo Press Ctrl+C to stop
echo.

REM Run the bridge
%PYTHON_EXE% -m ros2_nodes.carla_bridge.carla_zmq_bridge %BRIDGE_ARGS%

pause
