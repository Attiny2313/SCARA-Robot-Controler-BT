@echo off
setlocal
cd /d "%~dp0"

where py >nul 2>nul
if errorlevel 1 (
    echo Python launcher "py" not found.
    echo Install Python 3 from python.org and enable the launcher.
    pause
    exit /b 1
)

py -c "import serial" >nul 2>nul
if errorlevel 1 (
    echo Installing pyserial...
    py -m pip install -r requirements.txt
    if errorlevel 1 (
        echo Failed to install pyserial.
        pause
        exit /b 1
    )
)

py SCARA_Remote.py
