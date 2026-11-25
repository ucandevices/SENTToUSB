#!/bin/bash
# Build script for the 4-channel SENT-to-USB converter on the STM32C071K8T6 (LQFP32).
# Output in Debug_C071/.  The STM32F042 (1-channel) build is ./build.sh.
#
# Self-contained in C071/: application (incl. its own copies of sent_bridge,
# sent_core, slcan), the open-sent-c library copy (C071/Core/Inc/sent), the
# CDC class with configurable power fields (C071/USB_DEVICE/Class) and the
# STM32CubeC0 v1.4.1 CMSIS device + HAL (C071/Drivers).  Nothing the F042
# build compiles is used except the unchanged CMSIS core headers
# (Drivers/CMSIS/Include) and the ST USB Device core (Middlewares).
#
# Run from the project root: ./build_c071.sh [clean]
set -eu

CC="arm-none-eabi-gcc"
OBJDUMP="arm-none-eabi-objdump"
OBJCOPY="arm-none-eabi-objcopy"
SIZE="arm-none-eabi-size"

if ! command -v "${CC}" &>/dev/null; then
    echo "ERROR: ${CC} not found. Install arm-none-eabi toolchain or add it to PATH." >&2
    exit 1
fi

PROJ="$(cd "$(dirname "$0")" && pwd)"
OUT="${PROJ}/Debug_C071"
LD_SCRIPT="${PROJ}/C071/STM32C071K8TX_FLASH.ld"
ELF="${OUT}/SENTToUSB_4CH_C071.elf"
MAP="${OUT}/SENTToUSB_4CH_C071.map"
LST="${OUT}/SENTToUSB_4CH_C071.list"
BIN="${OUT}/SENTToUSB_4CH_C071.bin"
HEX="${OUT}/SENTToUSB_4CH_C071.hex"

if [[ "${1:-}" == "clean" ]]; then
    echo "Cleaning ${OUT}..."
    rm -rf "${OUT}"
    exit 0
fi

DEFINES="-DDEBUG -DUSE_HAL_DRIVER -DSTM32C071xx"

# Core/Inc of the F042 build is deliberately not on the path.
INCLUDES=(
    "-I${PROJ}/C071/Core/Inc"
    "-I${PROJ}/C071/Drivers/STM32C0xx_HAL_Driver/Inc"
    "-I${PROJ}/C071/Drivers/STM32C0xx_HAL_Driver/Inc/Legacy"
    "-I${PROJ}/C071/Drivers/CMSIS/Device/ST/STM32C0xx/Include"
    "-I${PROJ}/Drivers/CMSIS/Include"
    "-I${PROJ}/C071/USB_DEVICE/App"
    "-I${PROJ}/C071/USB_DEVICE/Target"
    "-I${PROJ}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
    "-I${PROJ}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
)

ARCH="-mcpu=cortex-m0plus --specs=nano.specs -mfloat-abi=soft -mthumb"
COMMON="-ffunction-sections -fdata-sections -Wall -fstack-usage"
STD="-std=gnu11"
APP_FLAGS="${ARCH} ${STD} -g3 -Os ${COMMON} -Wextra -Wno-unused-parameter"
LIB_FLAGS="${ARCH} ${STD} -g3 -Os ${COMMON}"
ASM_FLAGS="${ARCH} -g3 -x assembler-with-cpp"
# --no-warn-rwx-segments: the TIM2 capture handler runs from SRAM (.RamFunc,
# copied with .data), so the RAM load segment is executable on purpose.
LINK_FLAGS="${ARCH} --specs=nosys.specs -Wl,-Map=${MAP} -Wl,--gc-sections -static -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments -Wl,--start-group -lc -lm -Wl,--end-group"

OBJS=()

compile_c() {
    local src="$1" flags="$2"
    local rel="${src#${PROJ}/}"
    local obj="${OUT}/${rel%.c}.o"
    mkdir -p "$(dirname "${obj}")"
    echo "  CC  ${rel}"
    # shellcheck disable=SC2086
    ${CC} "${src}" ${flags} ${DEFINES} "${INCLUDES[@]}" \
        -c -MMD -MP -MF "${obj%.o}.d" -MT "${obj}" -o "${obj}"
    OBJS+=("${obj}")
}

compile_s() {
    local src="$1"
    local rel="${src#${PROJ}/}"
    local obj="${OUT}/${rel%.s}.o"
    mkdir -p "$(dirname "${obj}")"
    echo "  AS  ${rel}"
    # shellcheck disable=SC2086
    ${CC} ${ASM_FLAGS} -DDEBUG -c -o "${obj}" "${src}"
    OBJS+=("${obj}")
}

echo "[C071 application]"
for src in main_c071.c sent_app_c071.c stm32c0xx_it.c system_stm32c0xx.c            sent_bridge.c sent_core.c slcan.c syscalls.c sysmem.c; do
    compile_c "${PROJ}/C071/Core/Src/${src}" "${APP_FLAGS}"
done

echo "[open-sent-c]"
for src in mode_manager.c sent_crc.c sent_decoder.c sent_encoder.c \
           sent_protocol.c sent_slow_channel.c STM32/hal_stm32.c; do
    compile_c "${PROJ}/C071/Core/Inc/sent/implementation/${src}" "${LIB_FLAGS}"
done

echo "[Startup]"
compile_s "${PROJ}/C071/Startup/startup_stm32c071xx.s"

echo "[STM32C0xx HAL]"
for src in stm32c0xx_hal.c stm32c0xx_hal_cortex.c stm32c0xx_hal_flash.c \
           stm32c0xx_hal_flash_ex.c stm32c0xx_hal_gpio.c stm32c0xx_hal_pcd.c \
           stm32c0xx_hal_pcd_ex.c stm32c0xx_hal_pwr.c stm32c0xx_hal_pwr_ex.c \
           stm32c0xx_hal_rcc.c stm32c0xx_hal_rcc_ex.c stm32c0xx_ll_usb.c; do
    compile_c "${PROJ}/C071/Drivers/STM32C0xx_HAL_Driver/Src/${src}" "${LIB_FLAGS}"
done

echo "[USB Device library]"
compile_c "${PROJ}/C071/USB_DEVICE/Class/usbd_cdc.c" "${LIB_FLAGS}"
for src in usbd_core.c usbd_ctlreq.c usbd_ioreq.c; do
    compile_c "${PROJ}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/${src}" "${LIB_FLAGS}"
done
for src in App/usb_device.c App/usbd_cdc_if.c App/usbd_desc.c Target/usbd_conf.c; do
    compile_c "${PROJ}/C071/USB_DEVICE/${src}" "${LIB_FLAGS}"
done

echo "[Link]"
# shellcheck disable=SC2086
${CC} -o "${ELF}" "${OBJS[@]}" -T"${LD_SCRIPT}" ${LINK_FLAGS}
echo "  -> ${ELF}"

echo "[Size]"
${SIZE} "${ELF}" | tee "${OUT}/default.size.stdout"
${OBJDUMP} -h -S "${ELF}" > "${LST}"

# Flashable images for USB DFU (flash_usb.bat --4ch) and other programmers.
${OBJCOPY} -O binary "${ELF}" "${BIN}"
${OBJCOPY} -O ihex   "${ELF}" "${HEX}"
echo "  -> ${BIN}"
echo "  -> ${HEX}"

echo ""
echo "Build complete."
