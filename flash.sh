#!/bin/bash
# Flash SENTToUSB.elf to STM32F042 via ST-Link using st-flash.
# Requires: st-flash (apt install stlink-tools)
# Run from the project root: ./flash.sh [elf]
set -eu

PROJ="$(cd "$(dirname "$0")" && pwd)"
ELF="${1:-${PROJ}/Debug/SENTToUSB.elf}"

if ! command -v st-flash &>/dev/null; then
    echo "ERROR: st-flash not found. Install with: sudo apt install stlink-tools" >&2
    exit 1
fi

if [ ! -f "${ELF}" ]; then
    echo "ERROR: ELF not found: ${ELF}" >&2
    echo "Run ./build.sh first." >&2
    exit 1
fi

echo "Flashing ${ELF} ..."
st-flash --reset --format elf write "${ELF}"
echo "Done."
