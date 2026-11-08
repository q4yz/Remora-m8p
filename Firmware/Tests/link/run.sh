#!/bin/sh
# PC test of the SPI link logic (drivers/remoraLink/RemoraLink.cpp) on a simulated SPI / DMA.
# Linux / WSL with g++:  sh Firmware/Tests/link/run.sh
cd "$(dirname "$0")"
S=../../FirmwareSource/Remora-OS6
B=$(mktemp -d)
g++ -std=c++14 -O1 -g -Wall -Wextra -include cstdint -I$S -I$S/drivers/remoraLink \
    test.cpp $S/drivers/remoraLink/RemoraLink.cpp -o "$B"/test || { rm -rf "$B"; exit 1; }
"$B"/test; r=$?
rm -rf "$B"
echo "ALL: $([ $r = 0 ] && echo PASSED || echo FAILED)"
exit $r
