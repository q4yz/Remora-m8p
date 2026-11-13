#!/bin/bash
# Build every maintained firmware target and refresh Firmware/FirmwareBin, so the committed
# binaries always match the source. Run it before committing a change to shared code.
#
#   bash Firmware/build.sh            build all, copy the binaries
#   bash Firmware/build.sh MANTA8     only the target named MANTA8 (names in TARGETS below)
#   bash Firmware/build.sh --check    build all, copy nothing (compile check)
#   bash Firmware/build.sh --out DIR  also put every binary in DIR as <name>.bin (CI artifacts)
#
# Needs the mbed CLI (mbed-cli 1) and the GNU Arm toolchain on the PATH.

copy=1
filter=""
out=""
while [ $# -gt 0 ]; do
    case "$1" in
        --check) copy=0 ;;
        --out) shift; mkdir -p "$1" && out=$(cd "$1" && pwd) ;;
        *) filter="$1" ;;
    esac
    shift
done

cd "$(dirname "$0")/FirmwareSource/Remora-OS6" || exit 1
BIN=../../FirmwareBin

# name | mbed target | extra mbed options | build folder | FirmwareBin folder ("-": compile check only)
TARGETS=(
    "MANTA8|MANTA8||BUILD/MANTA8/GCC_ARM-RELEASE|STM32G0B1/BTT_MANTA_M8P"
    "MANTA8_DEBUG|MANTA8|-D REMORA_DEBUG=1|BUILD/MANTA8_DEBUG|STM32G0B1/BTT_MANTA_M8P_DEBUG"
    "EBB42|EBB42||BUILD/EBB42|STM32G0B1/BTT_EBB42"
    "EBB_SB2209|EBB_SB2209||BUILD/EBB_SB2209|STM32G0B1/BTT_EBB_SB2209"
    # USB serial test for EBB42 / SB2209 (TARGET_EBB42/serialTestMain.cpp), runs on both boards
    "EBB_SERIAL_TEST|EBB_SB2209|-D NODE_SERIAL_TEST=1|BUILD/EBB_SERIAL_TEST|STM32G0B1/BTT_EBB_SERIAL_TEST"
    # BTT Octopus / Octopus Pro: F446 (also the Octopus Pro F446) and F429. The upstream Remora
    # binaries stay in STM32F446/BTT_446 and STM32F429/BTT_429
    "OCTOPUS_446|OCTOPUS_446||BUILD/OCTOPUS_446/GCC_ARM-RELEASE|STM32F446/BTT_OCTOPUS_446"
    "OCTOPUS_446_DEBUG|OCTOPUS_446|-D REMORA_DEBUG=1|BUILD/OCTOPUS_446_DEBUG|STM32F446/BTT_OCTOPUS_446_DEBUG"
    "OCTOPUS_429|OCTOPUS_429||BUILD/OCTOPUS_429/GCC_ARM-RELEASE|STM32F429/BTT_OCTOPUS_429"
)

failed=()
for t in "${TARGETS[@]}"; do
    IFS='|' read -r name target opts build dest <<< "$t"
    [ -n "$filter" ] && [ "$name" != "$filter" ] && continue
    built=$((built + 1))

    echo "== $name"
    log=$(mktemp)
    # --build only for folders that are not the mbed default
    buildopt=""
    [[ "$build" != "BUILD/$target/GCC_ARM-RELEASE" ]] && buildopt="--build $build"
    # shellcheck disable=SC2086
    if mbed compile -m "$target" -t GCC_ARM --profile release $opts $buildopt > "$log" 2>&1 && [ -f "$build/firmware.bin" ]; then
        grep -E "^(Total|Image)" "$log" | tail -2
        if [ "$copy" = 1 ] && [ "$dest" != "-" ]; then
            mkdir -p "$BIN/$dest"
            cp "$build/firmware.bin" "$BIN/$dest/firmware.bin"
            echo "   -> FirmwareBin/$dest/firmware.bin"
        fi
        [ -n "$out" ] && cp "$build/firmware.bin" "$out/$name.bin"
    else
        grep -E "\[Error\]|error:" "$log" | head -10
        failed+=("$name")
    fi
    rm -f "$log"
done

if [ "${built:-0}" = 0 ]; then
    echo "no target named '$filter'"
    exit 1
fi
if [ ${#failed[@]} -gt 0 ]; then
    echo "FAILED: ${failed[*]}"
    exit 1
fi
echo "ALL BUILT"
