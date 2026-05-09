@echo off
REM Quick test runner with proper PATH setup

setlocal enabledelayedexpansion

REM Add Python user scripts to PATH
set "PYTEST_PATH=C:\Users\LJ\AppData\Local\Packages\PythonSoftwareFoundation.Python.3.13_qbz5n2kfra8p0\LocalCache\local-packages\Python313\Scripts"
set "PYTHON_PATH=C:\Users\LJ\AppData\Local\Microsoft\WindowsApps\PythonSoftwareFoundation.Python.3.13_qbz5n2kfra8p0"

set "PATH=!PYTEST_PATH!;!PYTHON_PATH!;%PATH%"

cd /d "c:\Users\LJ\STM32CubeIDE\workspace_1.19.0\SENTToUSB"

echo Running connectivity tests...
echo.
"%PYTHON_PATH%\python.exe" -m pytest test_sent_integration.py::TestSENTIntegration::test_devices_present_and_responsive test_sent_integration.py::TestSENTIntegration::test_can_enter_rx_mode test_sent_integration.py::TestSENTIntegration::test_can_enter_tx_mode -v -s --tb=short

endlocal
