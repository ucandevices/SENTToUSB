@echo off
REM SENT Integration Test Runner
REM Usage: run_tests.bat [option]
REM Options:
REM   (none)   - Run all tests with verbose output
REM   quick    - Run basic connectivity tests only
REM   tick3    - Run tests with 3µs tick time only
REM   debug    - Run with detailed logging
REM   report   - Generate HTML report

setlocal enabledelayedexpansion

REM Check if pytest is installed
python -m pytest --version >nul 2>&1
if errorlevel 1 (
    echo Error: pytest not found. Install with:
    echo   pip install -r test_requirements.txt
    exit /b 1
)

set TEST_FILE=test_sent_integration.py

if "%1"=="" (
    echo Running all integration tests...
    python -m pytest %TEST_FILE% -v -s --tb=short
) else if "%1"=="quick" (
    echo Running quick connectivity tests...
    python -m pytest %TEST_FILE%::TestSENTIntegration::test_devices_present_and_responsive -v -s
    python -m pytest %TEST_FILE%::TestSENTIntegration::test_can_enter_rx_mode -v -s
    python -m pytest %TEST_FILE%::TestSENTIntegration::test_can_enter_tx_mode -v -s
) else if "%1"=="tick3" (
    echo Running tests with 3µs tick time...
    python -m pytest "%TEST_FILE%::TestSENTIntegration::test_tx_rx_with_tick_time[3.0]" -v -s
) else if "%1"=="debug" (
    echo Running with debug logging...
    python -m pytest %TEST_FILE% -v -s --log-cli-level=DEBUG --tb=long
) else if "%1"=="report" (
    echo Generating HTML report...
    python -m pytest %TEST_FILE% -v --tb=short --html=test_report.html --self-contained-html
    if exist test_report.html (
        echo Report generated: test_report.html
        start test_report.html
    )
) else (
    echo Unknown option: %1
    echo Usage: run_tests.bat [quick^|tick3^|debug^|report]
    exit /b 1
)

endlocal
