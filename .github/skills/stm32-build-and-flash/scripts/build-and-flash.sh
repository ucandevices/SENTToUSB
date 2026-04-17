#!/bin/bash
# Combined build and flash script for STM32F042 SENT-to-USB
# Usage: ./build-and-flash.sh [clean]
set -eu

PROJ="$(cd "$(dirname "$0")" && cd ../../../.. && pwd)"

echo "=========================================="
echo "STM32F042 Build and Flash"
echo "=========================================="
echo ""

# Clean if requested
if [[ "${1:-}" == "clean" ]]; then
    echo "Step 1: Cleaning previous build..."
    bash "${PROJ}/build.sh" clean
    echo ""
fi

# Build
echo "Step 2: Building firmware..."
bash "${PROJ}/build.sh"

if [ ! -f "${PROJ}/Debug/SENTToUSB.elf" ]; then
    echo "ERROR: Build failed. Check compiler output above."
    exit 1
fi

echo ""
echo "Build successful!"
echo "Output: ${PROJ}/Debug/SENTToUSB.elf"
echo ""

# Flash
echo "Step 3: Flashing to microcontroller..."
echo "Ensure ST-Link is connected to the board's SWD pins."
read -p "Press Enter to proceed with flash, or Ctrl+C to cancel..."
echo ""

bash "${PROJ}/flash.sh"

echo ""
echo "=========================================="
echo "Build and Flash Complete!"
echo "=========================================="
