@echo off
echo Waiting 1s then flashing...
timeout /t 1 /nobreak >/dev/null
"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" ^
  -c port=SWD freq=240 mode=UR ^
  -w "C:\Users\LJ\STM32CubeIDE\workspace_1.19.0\SENTToUSB\Debug\SENTToUSB.elf" ^
  -v -rst
pause
