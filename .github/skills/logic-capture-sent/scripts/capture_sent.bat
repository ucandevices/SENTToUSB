@echo off
REM Capture SENT signal from Saleae Logic analyzer and export to CSV
REM Usage: capture_sent.bat [duration_ms] [sample_rate_hz]

setlocal enabledelayedexpansion

REM Get duration from argument or use default 500ms
set DURATION=%1
if "!DURATION!"=="" set DURATION=500

set SAMPLE_RATE=%2
if "!SAMPLE_RATE!"=="" set SAMPLE_RATE=

set LOGIC_EXE=C:\Program Files\Logic\Logic.exe
set PYTHON_SCRIPT=%~dp0capture_sent.py

echo.
echo ==========================================
echo Saleae Logic SENT Capture
echo ==========================================
echo.
echo Duration: !DURATION! ms
echo Sample rate: !SAMPLE_RATE! Hz
echo.

REM Check if Logic is installed
if not exist "!LOGIC_EXE!" (
    echo ERROR: Logic.exe not found at !LOGIC_EXE!
    echo Please install Saleae Logic 2 from: https://www.saleae.com/
    pause
    exit /b 1
)

REM Check if Python script exists
if not exist "!PYTHON_SCRIPT!" (
    echo ERROR: Capture script not found at !PYTHON_SCRIPT!
    pause
    exit /b 1
)

REM Launch Logic.exe in background if not already running
echo Ensuring Logic software is running...
tasklist /FI "IMAGENAME eq Logic.exe" 2>nul | findstr /i "Logic.exe" >nul
if errorlevel 1 (
    echo Launching Logic.exe...
    start "" "!LOGIC_EXE!" --loglevel info
    echo Waiting for Logic to start ^(2 seconds^)...
    timeout /t 2 /nobreak >nul
) else (
    echo Logic already running
)

echo.
echo Running Python capture script...
echo.

REM Run Python capture script with arguments
if "!SAMPLE_RATE!"=="" (
    python "!PYTHON_SCRIPT!" !DURATION!
) else (
    python "!PYTHON_SCRIPT!" !DURATION! !SAMPLE_RATE!
)

if errorlevel 1 (
    echo.
    echo ERROR: Capture failed
    echo.
    echo Troubleshooting:
    echo   1. Ensure Logic analyzer is connected to USB
    echo   2. Verify SENT signal is on channel 3
    echo   3. Check Logic software is responding
    echo.
    pause
    exit /b 1
)

echo.
echo ==========================================
echo ✓ Capture Complete
echo ==========================================
echo.
pause

