#!/bin/bash
# Build script for SENTToUSB - STM32F042G4 SENT-to-USB bridge
# Mirrors the STM32CubeIDE Debug build configuration exactly.
# Run from the project root: ./build.sh [clean]
set -eu

# ---------------------------------------------------------------------------
# Toolchain
# ---------------------------------------------------------------------------
CC="arm-none-eabi-gcc"
OBJDUMP="arm-none-eabi-objdump"
SIZE="arm-none-eabi-size"

if ! command -v "${CC}" &>/dev/null; then
    echo "ERROR: ${CC} not found. Install arm-none-eabi toolchain or add it to PATH." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
PROJ="$(cd "$(dirname "$0")" && pwd)"
OUT="${PROJ}/Debug"
LD_SCRIPT="${PROJ}/STM32F042G4UX_FLASH.ld"
ELF="${OUT}/SENTToUSB.elf"
MAP="${OUT}/SENTToUSB.map"
LST="${OUT}/SENTToUSB.list"

# ---------------------------------------------------------------------------
# Clean
# ---------------------------------------------------------------------------
if [[ "${1:-}" == "clean" ]]; then
    echo "Cleaning ${OUT}..."
    rm -f "${ELF}" "${MAP}" "${LST}" "${OUT}/default.size.stdout"
    find "${OUT}" \( -name "*.o" -o -name "*.d" -o -name "*.su" -o -name "*.cyclo" \) -delete
    echo "Done."
    exit 0
fi

# ---------------------------------------------------------------------------
# Compiler flag groups  (extracted verbatim from the CubeIDE-generated subdir.mk files)
# ---------------------------------------------------------------------------
DEFINES="-DDEBUG -DUSE_HAL_DRIVER -DSTM32F042x6"

INCLUDES=(
    "-I${PROJ}/Core/Inc"
    "-I${PROJ}/Drivers/STM32F0xx_HAL_Driver/Inc"
    "-I${PROJ}/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy"
    "-I${PROJ}/Drivers/CMSIS/Device/ST/STM32F0xx/Include"
    "-I${PROJ}/Drivers/CMSIS/Include"
    "-I${PROJ}/USB_DEVICE/App"
    "-I${PROJ}/USB_DEVICE/Target"
    "-I${PROJ}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
    "-I${PROJ}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
)

ARCH="-mcpu=cortex-m0 --specs=nano.specs -mfloat-abi=soft -mthumb"
COMMON="-ffunction-sections -fdata-sections -Wall -fstack-usage"
STD="-std=gnu11"

# Application sources: -O0 -g3  (Core/Src and Core/Src/sent)
APP_FLAGS="${ARCH} ${STD} -g3 -O0 ${COMMON}"

# Library / middleware / USB_DEVICE sources: -Os  (no -g3)
LIB_FLAGS="${ARCH} ${STD} -Os ${COMMON}"

# Startup assembly
ASM_FLAGS="${ARCH} -g3 -x assembler-with-cpp"

# Linker
LINK_FLAGS="${ARCH} --specs=nosys.specs -Wl,-Map=${MAP} -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
compile_c() {
    local src="$1" obj="$2" flags="$3"
    mkdir -p "$(dirname "${obj}")"
    echo "  CC  ${src#${PROJ}/}"
    # shellcheck disable=SC2086
    ${CC} "${src}" ${flags} ${DEFINES} "${INCLUDES[@]}" \
        -c -MMD -MP -MF "${obj%.o}.d" -MT "${obj}" -o "${obj}"
}

compile_s() {
    local src="$1" obj="$2"
    mkdir -p "$(dirname "${obj}")"
    echo "  AS  ${src#${PROJ}/}"
    # shellcheck disable=SC2086
    ${CC} ${ASM_FLAGS} -DDEBUG \
        -c -MMD -MP -MF "${obj%.o}.d" -MT "${obj}" -o "${obj}" "${src}"
}

# ---------------------------------------------------------------------------
# Create output directory tree
# ---------------------------------------------------------------------------
mkdir -p \
    "${OUT}/Core/Src/sent" \
    "${OUT}/Core/Startup" \
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src" \
    "${OUT}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src" \
    "${OUT}/Middlewares/ST/STM32_USB_Device_Library/Core/Src" \
    "${OUT}/USB_DEVICE/App" \
    "${OUT}/USB_DEVICE/Target"

# ---------------------------------------------------------------------------
# Compile — Core/Src  (O0 + g3)
# ---------------------------------------------------------------------------
echo "[Core/Src]"
for src in \
    main.c sent_app.c stm32f0xx_hal_msp.c stm32f0xx_it.c \
    syscalls.c sysmem.c system_stm32f0xx.c
do
    compile_c "${PROJ}/Core/Src/${src}" \
              "${OUT}/Core/Src/${src%.c}.o" \
              "${APP_FLAGS}"
done

# ---------------------------------------------------------------------------
# Compile — Core/Src/sent  (O0 + g3)
# ---------------------------------------------------------------------------
echo "[Core/Src/sent]"
for src in \
    bridge.c hal_stm32f042.c mode_manager.c sent_crc.c \
    sent_decoder.c sent_encoder.c sent_protocol.c slcan.c
do
    compile_c "${PROJ}/Core/Src/sent/${src}" \
              "${OUT}/Core/Src/sent/${src%.c}.o" \
              "${APP_FLAGS}"
done

# ---------------------------------------------------------------------------
# Compile — Core/Startup  (assembler)
# ---------------------------------------------------------------------------
echo "[Core/Startup]"
compile_s "${PROJ}/Core/Startup/startup_stm32f042g4ux.s" \
          "${OUT}/Core/Startup/startup_stm32f042g4ux.o"

# ---------------------------------------------------------------------------
# Compile — HAL Driver  (Os)
# ---------------------------------------------------------------------------
echo "[Drivers/STM32F0xx_HAL_Driver/Src]"
for src in \
    stm32f0xx_hal.c stm32f0xx_hal_cortex.c stm32f0xx_hal_dma.c \
    stm32f0xx_hal_exti.c stm32f0xx_hal_flash.c stm32f0xx_hal_flash_ex.c \
    stm32f0xx_hal_gpio.c stm32f0xx_hal_i2c.c stm32f0xx_hal_i2c_ex.c \
    stm32f0xx_hal_pcd.c stm32f0xx_hal_pcd_ex.c stm32f0xx_hal_pwr.c \
    stm32f0xx_hal_pwr_ex.c stm32f0xx_hal_rcc.c stm32f0xx_hal_rcc_ex.c \
    stm32f0xx_hal_tim.c stm32f0xx_hal_tim_ex.c stm32f0xx_ll_usb.c
do
    compile_c "${PROJ}/Drivers/STM32F0xx_HAL_Driver/Src/${src}" \
              "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/${src%.c}.o" \
              "${LIB_FLAGS}"
done

# ---------------------------------------------------------------------------
# Compile — USB Middleware CDC  (Os)
# ---------------------------------------------------------------------------
echo "[Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src]"
compile_c \
    "${PROJ}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c" \
    "${OUT}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o" \
    "${LIB_FLAGS}"

# ---------------------------------------------------------------------------
# Compile — USB Middleware Core  (Os)
# ---------------------------------------------------------------------------
echo "[Middlewares/ST/STM32_USB_Device_Library/Core/Src]"
for src in usbd_core.c usbd_ctlreq.c usbd_ioreq.c; do
    compile_c \
        "${PROJ}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/${src}" \
        "${OUT}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/${src%.c}.o" \
        "${LIB_FLAGS}"
done

# ---------------------------------------------------------------------------
# Compile — USB_DEVICE/App  (Os)
# ---------------------------------------------------------------------------
echo "[USB_DEVICE/App]"
for src in usb_device.c usbd_cdc_if.c usbd_desc.c; do
    compile_c "${PROJ}/USB_DEVICE/App/${src}" \
              "${OUT}/USB_DEVICE/App/${src%.c}.o" \
              "${LIB_FLAGS}"
done

# ---------------------------------------------------------------------------
# Compile — USB_DEVICE/Target  (Os)
# ---------------------------------------------------------------------------
echo "[USB_DEVICE/Target]"
compile_c "${PROJ}/USB_DEVICE/Target/usbd_conf.c" \
          "${OUT}/USB_DEVICE/Target/usbd_conf.o" \
          "${LIB_FLAGS}"

# ---------------------------------------------------------------------------
# Link  (object order matches objects.list exactly)
# ---------------------------------------------------------------------------
echo "[Link]"
OBJS=(
    "${OUT}/Core/Src/main.o"
    "${OUT}/Core/Src/sent_app.o"
    "${OUT}/Core/Src/stm32f0xx_hal_msp.o"
    "${OUT}/Core/Src/stm32f0xx_it.o"
    "${OUT}/Core/Src/syscalls.o"
    "${OUT}/Core/Src/sysmem.o"
    "${OUT}/Core/Src/system_stm32f0xx.o"
    "${OUT}/Core/Src/sent/bridge.o"
    "${OUT}/Core/Src/sent/hal_stm32f042.o"
    "${OUT}/Core/Src/sent/mode_manager.o"
    "${OUT}/Core/Src/sent/sent_crc.o"
    "${OUT}/Core/Src/sent/sent_decoder.o"
    "${OUT}/Core/Src/sent/sent_encoder.o"
    "${OUT}/Core/Src/sent/sent_protocol.o"
    "${OUT}/Core/Src/sent/slcan.o"
    "${OUT}/Core/Startup/startup_stm32f042g4ux.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_exti.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd_ex.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.o"
    "${OUT}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_usb.o"
    "${OUT}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o"
    "${OUT}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.o"
    "${OUT}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o"
    "${OUT}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o"
    "${OUT}/USB_DEVICE/App/usb_device.o"
    "${OUT}/USB_DEVICE/App/usbd_cdc_if.o"
    "${OUT}/USB_DEVICE/App/usbd_desc.o"
    "${OUT}/USB_DEVICE/Target/usbd_conf.o"
)

# shellcheck disable=SC2086
${CC} -o "${ELF}" "${OBJS[@]}" \
    -T"${LD_SCRIPT}" \
    ${LINK_FLAGS}

echo "  -> ${ELF}"

# ---------------------------------------------------------------------------
# Post-build: size + disassembly listing  (matches CubeIDE secondary-outputs)
# ---------------------------------------------------------------------------
echo "[Size]"
${SIZE} "${ELF}" | tee "${OUT}/default.size.stdout"

echo "[Disassembly]"
${OBJDUMP} -h -S "${ELF}" > "${LST}"
echo "  -> ${LST}"

echo ""
echo "Build complete."
