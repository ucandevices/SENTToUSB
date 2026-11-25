@echo off
setlocal EnableDelayedExpansion

REM ---------------------------------------------------------------
REM  Flash SENT-USB-Converter over USB DFU using STM32CubeProgrammer CLI.
REM    1-channel board : STM32F042G6 (default)
REM    4-channel board : STM32C071K8 (--4ch)
REM
REM  Usage:  flash_usb.bat [--4ch] [--loop] [--port COMx] [firmware.bin] [flash_address]
REM
REM  --4ch : 4-channel board. Default firmware is then SENT4CH_<tag>.bin
REM          (asset SENTToUSB_4CH.bin of the newest release, if there is
REM          one), else the newest SENT4CH_*.bin next to this script, else
REM          the local build Debug_C071\SENTToUSB_4CH_C071.bin.
REM
REM  Before writing, the chip ID read over DFU is compared with the board
REM  type (0x445 = F042, 0x493 = C071); a mismatch is refused. This matters
REM  on the C071: its firmware disables the BOOT0 pin, so a wrong image
REM  could only be recovered over SWD.
REM
REM  Default firmware: the newest release, downloaded from
REM    https://github.com/ucandevices/SENTToUSB/releases/latest
REM    (SHA-256 checked) into this script's folder as
REM    SENTToUSB_<tag>.bin. Without internet the newest
REM    SENTToUSB_*.bin already next to this script is used, then a
REM    local build in Debug\SENTToUSB.bin.
REM  Default address : 0x08000000
REM
REM  --port COMx : a converter that is running its firmware is sent
REM           "boot<CR>" on that COM port first, which makes it
REM           restart in DFU mode. Not needed for a board that is
REM           already in DFU mode.
REM
REM  --loop : production mode. Waits for a board in DFU mode, flashes
REM           it, waits until it is unplugged / has started, and waits
REM           for the next one. Stop with Ctrl+C.
REM
REM  The board must be in DFU mode (system bootloader): a blank board
REM  starts in DFU by itself; a running converter enters it when it
REM  receives "boot<CR>" on its COM port (see --port); or (F042 only) hold BOOT0
REM  high while resetting / re-plugging the USB cable.
REM
REM  If the chip reports Read Out Protection, the script removes it
REM  (-rdu, which mass-erases the flash), asks for a USB re-plug and
REM  programs again. On STM32F042 the re-plug is required: the
REM  bootloader only notices the now-empty flash after a power-on reset.
REM ---------------------------------------------------------------

set "PROG_DIR=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin"
set "PROG=%PROG_DIR%\STM32_Programmer_CLI.exe"
set "LIST=%TEMP%\stm32_usb_list.txt"
set "LOG=%TEMP%\stm32_flash_log.txt"
set "REPO=ucandevices/SENTToUSB"
set "ASSET=SENTToUSB.bin"

set "LOOP="
set "COMPORT="
REM  board type: 1-channel F042 unless --4ch
set "BOARD_NAME=1-channel STM32F042"
set "CHIP_ID=445"
set "PREFIX=SENTToUSB_"
set "LOCAL_BUILD=%~dp0Debug\SENTToUSB.bin"
set "BOOT_HINT=hold BOOT0 high - pull to 3V3 - while resetting or re-plugging USB"
:parse_opts
if /i "%~1"=="--4ch" (
    set "BOARD_NAME=4-channel STM32C071"
    set "CHIP_ID=493"
    set "ASSET=SENTToUSB_4CH.bin"
    set "PREFIX=SENT4CH_"
    set "LOCAL_BUILD=%~dp0Debug_C071\SENTToUSB_4CH_C071.bin"
    set "BOOT_HINT=the C071 ignores BOOT0: a blank chip starts in DFU by itself, otherwise reflash over SWD"
    shift
    goto :parse_opts
)
if /i "%~1"=="--loop" (
    set "LOOP=1"
    shift
    goto :parse_opts
)
if /i "%~1"=="--port" (
    set "COMPORT=%~2"
    shift
    shift
    goto :parse_opts
)

set "FW=%~1"
set "ADDR=%~2"
if "%ADDR%"=="" set "ADDR=0x08000000"

if not exist "%PROG%" (
    echo [ERROR] STM32_Programmer_CLI.exe not found.
    echo         Install STM32CubeProgrammer or check PROG_DIR in this script.
    goto :fail
)

if "%FW%"=="" call :get_latest
if "%FW%"=="" (
    REM downloaded files are <PREFIX><tag>.bin; the last one by date is the newest
    for /f "delims=" %%F in ('dir /b /od "%~dp0%PREFIX%*.bin" 2^>nul') do set "FW=%~dp0%%F"
)
if "%FW%"=="" if exist "%LOCAL_BUILD%" set "FW=%LOCAL_BUILD%"
if "%FW%"=="" (
    echo [ERROR] No firmware: download failed, no %PREFIX%*.bin next to this
    echo         script and no local build in %LOCAL_BUILD%.
    goto :fail
)
if not exist "%FW%" (
    echo [ERROR] Firmware file not found: %FW%
    goto :fail
)

echo.
echo === STM32 USB DFU flash ===
echo Board      : %BOARD_NAME%
echo Programmer : %PROG%
echo Firmware   : %FW%
echo Address    : %ADDR%
if defined COMPORT echo COM port   : %COMPORT% - "boot" is sent to enter DFU mode
if defined LOOP echo Mode       : loop - plug boards one after another, Ctrl+C to stop
echo.

if defined COMPORT call :enter_dfu

if defined LOOP goto :loop

REM ======================= single board ==========================
set /a "FIND_SECS=1"
if defined COMPORT set /a "FIND_SECS=10"
call :find_port !FIND_SECS!
if errorlevel 1 (
    echo.
    echo [ERROR] No STM32 device found in DFU mode.
    echo         Put the board in bootloader mode: send "boot" + CR to its
    echo         COM port ^(flash_usb.bat --port COMx^), or
    echo         !BOOT_HINT!.
    goto :fail
)

call :program
if not errorlevel 1 goto :ok

REM --- Programming failed: is it Read Out Protection? ---
call :is_rdp
if errorlevel 1 goto :fail

call :unprotect
pause

call :find_port 30
if errorlevel 1 (
    echo [ERROR] Device did not come back in DFU mode.
    echo         !BOOT_HINT!, then run this script again.
    goto :fail
)

call :program
if not errorlevel 1 goto :ok

call :is_rdp
if errorlevel 1 goto :fail

REM --- Still protected: force option bytes to RDP level 0 explicitly ---
echo.
echo --- Still protected: writing option byte RDP=0xAA ---
"%PROG%" -c port=!PORT! -ob rdp=0xAA

echo.
echo ---------------------------------------------------------------
echo  UNPLUG the USB cable again, wait 2 s and plug it back in.
echo ---------------------------------------------------------------
pause

call :find_port 30
if errorlevel 1 (
    echo [ERROR] Device did not come back in DFU mode.
    goto :fail
)

call :program
if errorlevel 1 goto :fail

:ok
echo.
echo === Flash OK ===
del "%LIST%" "%LOG%" >nul 2>&1
endlocal
exit /b 0

:fail
echo.
echo === Flash FAILED ===
if exist "%LOG%" echo Last programmer log: %LOG%
del "%LIST%" >nul 2>&1
endlocal
exit /b 1


REM ========================= loop mode ===========================
:loop
set /a "OK_COUNT=0"
set /a "FAIL_COUNT=0"
:loop_next
echo.
echo ---------------------------------------------------------------
echo  Flashed OK: !OK_COUNT!   failed: !FAIL_COUNT!
echo  Waiting for the next board in DFU mode... ^(Ctrl+C to stop^)
echo ---------------------------------------------------------------
call :find_port 0
call :program
if not errorlevel 1 (
    set /a "OK_COUNT+=1"
    echo.
    echo  ##########  BOARD !OK_COUNT! FLASHED OK  ##########
    call :beep
) else (
    call :is_rdp
    if not errorlevel 1 (
        call :unprotect
    ) else (
        set /a "FAIL_COUNT+=1"
        echo.
        echo  !!!!!!!!!!  FLASH FAILED - see the log above  !!!!!!!!!!
        echo  Re-plug the board to try again.
        call :beep
        call :beep
    )
)
call :wait_gone
goto :loop_next


REM ---------------------------------------------------------------
REM  :get_latest  - download the newest release binary into this
REM  script's folder as SENTToUSB_<tag>.bin and set FW to it.
REM  A copy that is already there is kept if its SHA-256 still
REM  matches the release, otherwise it is downloaded again.
REM  Leaves FW empty when offline or GitHub is unreachable.
REM ---------------------------------------------------------------
:get_latest
echo --- Checking the newest firmware on github.com/%REPO% ---
set "PS_GET=$ErrorActionPreference='Stop'; [Net.ServicePointManager]::SecurityProtocol='Tls12';"
set "PS_GET=!PS_GET! $r=Invoke-RestMethod 'https://api.github.com/repos/%REPO%/releases/latest' -Headers @{'User-Agent'='flash_usb'};"
set "PS_GET=!PS_GET! $a=$r.assets | Where-Object { $_.name -eq '%ASSET%' } | Select-Object -First 1;"
set "PS_GET=!PS_GET! if (-not $a) { throw 'release ' + $r.tag_name + ' has no %ASSET%' };"
set "PS_GET=!PS_GET! $o=Join-Path '%~dp0' ('%PREFIX%' + $r.tag_name + '.bin');"
set "PS_GET=!PS_GET! $h={ 'sha256:' + (Get-FileHash $o -Algorithm SHA256).Hash.ToLower() };"
set "PS_GET=!PS_GET! if ((Test-Path $o) -and $a.digest -and ($a.digest -ne (& $h))) { Remove-Item $o };"
set "PS_GET=!PS_GET! if (-not (Test-Path $o)) { Invoke-WebRequest $a.browser_download_url -OutFile ($o + '.part') -UseBasicParsing; Move-Item -Force ($o + '.part') $o };"
set "PS_GET=!PS_GET! if ($a.digest -and ($a.digest -ne (& $h))) { Remove-Item $o; throw 'SHA-256 mismatch, file deleted' };"
set "PS_GET=!PS_GET! $o"
for /f "delims=" %%L in ('powershell -NoProfile -ExecutionPolicy Bypass -Command "!PS_GET!" 2^>nul') do set "FW=%%L"
if "%FW%"=="" echo     [WARN] Download failed - looking for a firmware file in this folder.
exit /b 0


REM ---------------------------------------------------------------
REM  :enter_dfu  - send "boot<CR>" to COMPORT so that a running
REM  converter restarts in the system bootloader.
REM ---------------------------------------------------------------
:enter_dfu
echo --- Sending "boot" to %COMPORT% ---
set "PS_BOOT=$ErrorActionPreference='Stop';"
set "PS_BOOT=!PS_BOOT! $p=New-Object System.IO.Ports.SerialPort '%COMPORT%',115200,'None',8,'One';"
set "PS_BOOT=!PS_BOOT! $p.Open(); $p.Write('boot' + [char]13); Start-Sleep -Milliseconds 300;"
set "PS_BOOT=!PS_BOOT! try { $p.Close() } catch { }"
powershell -NoProfile -ExecutionPolicy Bypass -Command "!PS_BOOT!" >nul 2>&1
if errorlevel 1 (
    echo     [WARN] Could not open %COMPORT% - the board may already be in DFU mode.
) else (
    REM  the converter rewrites its option bytes and re-enumerates as DFU
    ping -n 4 127.0.0.1 >nul
)
exit /b 0


REM ---------------------------------------------------------------
REM  :find_port <seconds>  - wait up to <seconds> for a DFU device
REM  (0 = wait forever), sets PORT (usb1, usb2, ...).
REM  errorlevel 1 if none found.
REM ---------------------------------------------------------------
:find_port
set "PORT="
set /a "TRIES=%~1"
if !TRIES! GTR 0 echo --- Looking for STM32 USB DFU devices ---
:find_port_loop
"%PROG%" -l usb | findstr /i "usb" > "%LIST%"
findstr /i /c:"Device Index" "%LIST%" >nul
if not errorlevel 1 goto :find_port_found
if "%~1"=="0" goto :find_port_wait
set /a "TRIES-=1"
if !TRIES! LEQ 0 exit /b 1
:find_port_wait
ping -n 2 127.0.0.1 >nul
goto :find_port_loop

:find_port_found
type "%LIST%"
REM Take the first detected USB DFU port (usb1, usb2, ...)
for /f "tokens=2 delims=:" %%A in ('findstr /i /c:"Device Index" "%LIST%"') do (
    if not defined PORT (
        set "PORT=%%A"
        set "PORT=!PORT: =!"
    )
)
if not defined PORT set "PORT=USB1"
exit /b 0


REM ---------------------------------------------------------------
REM  :wait_gone  - wait until no DFU device is present any more
REM  (the board started its application or was unplugged).
REM ---------------------------------------------------------------
:wait_gone
set /a "WAITED=0"
:wait_gone_loop
"%PROG%" -l usb | findstr /i /c:"Device Index" >nul
if errorlevel 1 exit /b 0
set /a "WAITED+=1"
if !WAITED!==5 echo  Board still in DFU mode - unplug it to continue.
ping -n 2 127.0.0.1 >nul
goto :wait_gone_loop


REM ---------------------------------------------------------------
REM  :program  - write + verify + start the firmware, output is shown
REM  and kept in %LOG% so the caller can look for RDP errors.
REM ---------------------------------------------------------------
:program
call :check_chip
if errorlevel 1 exit /b 1
echo.
echo --- Programming on port !PORT! ---
REM  -g <addr> jumps to the application; DFU has no -rst (SWD/JTAG only)
"%PROG%" -c port=!PORT! -w "%FW%" %ADDR% -v -g %ADDR% > "%LOG%" 2>&1
set "RC=!errorlevel!"
type "%LOG%"
REM  The CLI does not always return non-zero on DFU errors - check the text too
if not "!RC!"=="0" exit /b 1
findstr /i /c:"Error:" "%LOG%" >nul
if not errorlevel 1 exit /b 1
findstr /i /c:"Download verified successfully" "%LOG%" >nul
if errorlevel 1 exit /b 1
exit /b 0


REM ---------------------------------------------------------------
REM  :check_chip  - connect only and compare the chip ID with the board
REM  type (CHIP_ID). errorlevel 1 on a mismatch. If the CLI output has
REM  no "Device ID" line the check is skipped with a warning.
REM ---------------------------------------------------------------
:check_chip
"%PROG%" -c port=!PORT! > "%LOG%" 2>&1
findstr /i /c:"Device ID" "%LOG%" >nul
if errorlevel 1 (
    echo     [WARN] Could not read the chip ID - board type not checked.
    exit /b 0
)
findstr /i /r /c:"Device ID *: *0x0*%CHIP_ID%" "%LOG%" >nul
if not errorlevel 1 exit /b 0
echo.
findstr /i /c:"Device ID" /c:"Device name" "%LOG%"
echo [ERROR] This chip is not a %BOARD_NAME% ^(expected ID 0x%CHIP_ID%^).
echo         Wrong firmware for this board - nothing written.
if "%CHIP_ID%"=="445" echo         For the 4-channel board run: flash_usb.bat --4ch
if "%CHIP_ID%"=="493" echo         For the 1-channel board run flash_usb.bat without --4ch.
exit /b 1


REM ---------------------------------------------------------------
REM  :is_rdp  - errorlevel 0 if the last programming failed because
REM  of Read Out Protection.
REM ---------------------------------------------------------------
:is_rdp
findstr /i /c:"Read Out Protection" /c:"memory protection" "%LOG%" >nul
exit /b %errorlevel%


REM ---------------------------------------------------------------
REM  :unprotect  - remove Read Out Protection (mass erase).
REM ---------------------------------------------------------------
:unprotect
echo.
echo ===============================================================
echo  Device is read-out protected. Removing protection ^(-rdu^).
echo  This erases the whole flash.
echo ===============================================================
"%PROG%" -c port=!PORT! -rdu
REM  The bootloader resets after -rdu, so the CLI may report an error
REM  here even when the unprotect worked - carry on regardless.
echo.
echo ---------------------------------------------------------------
echo  Now UNPLUG the USB cable, wait 2 s and plug it back in.
echo  ^(The flash is empty now, so the board should start in DFU
echo   mode by itself; if it does not: !BOOT_HINT!.^)
echo ---------------------------------------------------------------
exit /b 0


:beep
REM  BEL character - audible signal for the operator
powershell -NoProfile -Command "[console]::beep(1000,200)" >nul 2>&1
exit /b 0
