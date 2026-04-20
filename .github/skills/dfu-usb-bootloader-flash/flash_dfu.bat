@echo off
setlocal enabledelayedexpansion

set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM9"
set "ELF=C:\Users\LJ\STM32CubeIDE\workspace_1.19.0\SENTToUSB\Debug\SENTToUSB.elf"
set "CUBE=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

if not exist "%ELF%" (
  echo ERROR: ELF not found: %ELF%
  pause
  exit /b 1
)

if not exist "%CUBE%" (
  echo ERROR: STM32_Programmer_CLI not found: %CUBE%
  pause
  exit /b 1
)

echo Triggering USB DFU bootloader on %PORT%...

powershell -NoProfile -Command "Start-Sleep -Seconds 1; $p = New-Object System.IO.Ports.SerialPort '%PORT%',115200,'None',8,'One'; $p.Open(); $p.Write(\"boot`r\"); Start-Sleep -Milliseconds 100; $p.Close();"
if errorlevel 1 (
  echo ERROR: Failed to send bootloader trigger on %PORT%.
  pause
  exit /b 1
)

timeout /t 3 /nobreak >nul

echo Flashing ELF via STM32_Programmer_CLI...
"%CUBE%" -c port=usb1 -w "%ELF%" -v -g
if errorlevel 1 (
  echo ERROR: STM32_Programmer_CLI flash failed.
  pause
  exit /b 1
)

echo Flash complete.
pause
exit /b 0
