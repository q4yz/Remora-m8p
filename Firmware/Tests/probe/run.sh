#!/bin/sh
# PC test of the probe latch logic (modules/probeLatch/probeLatchCore.h).
# Linux / WSL with g++:  sh Firmware/Tests/probe/run.sh
cd "$(dirname "$0")"
S=../../FirmwareSource/Remora-OS6
B=$(mktemp -d)
g++ -std=c++14 -O1 -g -Wall -Wextra -I$S/modules/probeLatch test.cpp -o "$B"/test || { rm -rf "$B"; exit 1; }
"$B"/test; r=$?
rm -rf "$B"
echo "ALL: $([ $r = 0 ] && echo PASSED || echo FAILED)"
exit $r
