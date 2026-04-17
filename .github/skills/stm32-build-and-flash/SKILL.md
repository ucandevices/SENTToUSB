---
name: stm32-build-and-flash
description: 'Build and flash STM32F042 SENT-to-USB firmware. Use when compiling the project and programming the microcontroller via ST-Link.'
argument-hint: '[build|flash|build-and-flash] (default: build-and-flash)'
---

# STM32 Build and Flash

## When to Use

- Compile the STM32F042G4 SENT-to-USB bridge firmware
- Flash the compiled ELF binary to the microcontroller
- Run a complete build and flash cycle before testing

## Requirements

### Windows
- **STM32CubeProgrammer**: Download from [STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeprog.html)
- **ST-Link debugger**: Connected to the STM32 board's SWD pins

### Linux
- **arm-none-eabi-gcc**: ARM embedded toolchain
- **st-flash**: STLink tools (`apt install stlink-tools`)
- **ST-Link debugger**: Connected to the STM32 board's SWD pins

## Procedure

### Option 1: Full Build and Flash

1. Run the [build script](./scripts/build-and-flash.sh) to compile and flash automatically
2. Monitor console output for compilation warnings/errors
3. Wire the ST-Link to SWD pins and verify connection
4. Press Enter to proceed with flashing after successful build

### Option 2: Build Only

Execute the build script to compile the firmware:
```bash
./build.sh
```

Output files:
- `Debug/SENTToUSB.elf` - Compiled firmware binary
- `Debug/SENTToUSB.map` - Linker map file
- `Debug/SENTToUSB.list` - Assembly listing

### Option 3: Flash Only

Flash an existing ELF file to the microcontroller (automatically uses STM32CubeProgrammer on Windows, st-flash on Linux):
```bash
./flash.sh [path/to/SENTToUSB.elf]
```

Omit the path to use the default Debug build:
```bash
./flash.sh
```

**On Windows**: Automatically uses STM32CubeProgrammer (native USB access)
**On Linux**: Uses st-flash tool

### Option 4: Clean Build

Remove intermediate build artifacts and rebuild from scratch:
```bash
./build.sh clean
./build.sh
```

## Toolchain Configuration

**Build:**
- **Compiler**: `arm-none-eabi-gcc`
- **Linker Script**: `STM32F042G4UX_FLASH.ld`
- **Target Device**: STM32F042G4 (ARM Cortex-M0)
- **Output Format**: ELF

**Flash:**
- **Primary (Windows)**: STM32CubeProgrammer CLI
- **Secondary (Linux)**: `st-flash` with `--reset` flag
- **Flash Address**: 0x08000000
- **Debug Interface**: SWD (Serial Wire Debug)

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `arm-none-eabi-gcc not found` | Install ARM embedded toolchain for your OS |
| `st-flash not found` (Linux) | Install stlink-tools: `sudo apt install stlink-tools` |
| STM32CubeProgrammer not found (Windows) | Install from [STMicroelectronics](https://www.st.com/en/development-tools/stm32cubeprog.html) |
| `ELF not found` | Run `./build.sh` first to compile the firmware |
| Flash fails with device not found | Check ST-Link connection; verify SWD pins are wired correctly |
| Windows USB passthrough issue | Use Windows terminal (`cmd.exe`) instead of WSL for flashing |
| Compilation warnings | Review compiler output; warnings may indicate configuration issues |

## Scripts

- [build.sh](./scripts/build.sh) - Compile the STM32F042 firmware
- [flash.sh](./scripts/flash.sh) - Flash ELF to microcontroller via ST-Link
