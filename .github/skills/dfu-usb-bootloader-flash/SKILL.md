---
name: dfu-usb-bootloader-flash
description: 'Flash STM32F042 firmware via USB DFU bootloader using a serial VCOM trigger and STM32_Programmer_CLI.exe.'
argument-hint: '[COMx] (default: COM9)'
---

# DFU USB Bootloader Flash

## When to Use

- Put the STM32 board into USB DFU mode via the UART bootloader trigger
- Flash `Debug/SENTToUSB.elf` with `STM32CubeProgrammer.exe`
- Use when SWD/ST-Link is unavailable or the board must be updated via USB DFU

## Requirements

- Windows
- STM32CubeProgrammer installed at `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe`
- Board connected to a virtual COM port (for example `COM9`, `COM10`, or another available port)
- The board's USB DFU interface shows up in Device Manager after the trigger is sent

## Procedure

1. Connect the board to the PC and identify its VCOM port.
2. Run the helper script:

```bat
.github\skills\dfu-usb-bootloader-flash\flash_dfu.bat COM9
```

3. The script waits 1 second, sends the character `b` over the selected COM port, and then waits for the device to enter USB DFU mode.
4. Once the DFU device appears, the script launches `STM32CubeProgrammer.exe` and flashes `Debug\SENTToUSB.elf`.

## Default ELF and Programmer Paths

- ELF binary: `C:\Users\LJ\STM32CubeIDE\workspace_1.19.0\SENTToUSB\Debug\SENTToUSB.elf`
- STM32CubeProgrammer CLI: `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe`

## Example

```bat
.github\skills\dfu-usb-bootloader-flash\flash_dfu.bat COM10
```

## Troubleshooting

- If the COM port is not found, verify the board appears in Device Manager under Ports (COM & LPT).
- If the board does not enter DFU mode, check that the bootloader expects `b` on the UART interface and that the serial speed is 115200.
- If the DFU device does not enumerate, try unplugging and replugging the USB cable after the trigger.
- If `STM32CubeProgrammer.exe` fails, verify the installation path and use the correct DFU port for the device.

## Notes

- This skill is intended for DFU bootloader flashing only; it does not use SWD/MMB or ST-Link.
- Use the COM port that corresponds to the board's USB-to-UART adapter.
