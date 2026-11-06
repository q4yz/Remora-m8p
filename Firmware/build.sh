#!/bin/bash
# Build every maintained firmware target and refresh Firmware/FirmwareBin, so the committed
# binaries always match the source. Run it before committing a change to shared code.
#
#   bash Firmware/build.sh            build all, copy the binaries
#   bash Firmware/build.sh MANTA8     only the targets whose name contains MANTA8
#   bash Firmware/build.sh --check    build all, copy nothing (compile check)
#
# Needs the mbed CLI (mbed-cli 1) and the GNU Arm toolchain on the PATH.

cd "$(dirname "$0")/FirmwareSource/Remora-OS6" || exit 1
BIN=../../FirmwareBin

copy=1
filter=""
for a in "$@"; do
    case "$a" in
        --check) copy=0 ;;
        *) filter="$a" ;;
    esac
done

# name | mbed target | extra mbed options | build folder | FirmwareBin folder ("-": compile check only)
TARGETS=(
    "MANTA8|MANTA8||BUILD/MANTA8/GCC_ARM-RELEASE|STM32G0B1/BTT_MANTA_M8P"
    "MANTA8_DEBUG|MANTA8|-D REMORA_DEBUG=1|BUILD/MANTA8_DEBUG|STM32G0B1/BTT_MANTA_M8P_DEBUG"
    "EBB42|EBB42||BUILD/EBB42|STM32G0B1/BTT_EBB42"
    "EBB_SB2209|EBB_SB2209||BUILD/EBB_SB2209|STM32G0B1/BTT_EBB_SB2209"
    # STM32F4: the binaries in FirmwareBin come from upstream Remora, compile check only
    "OCTOPUS_446|OCTOPUS_446||BUILD/OCTOPUS_446/GCC_ARM-RELEASE|-"
    "OCTOPUS_429|OCTOPUS_429||BUILD/OCTOPUS_429/GCC_ARM-RELEASE|-"
)

failed=()
for t in "${TARGETS[@]}"; do
    IFS='|' read -r name target opts build dest <<< "$t"
    [ -n "$filter" ] && [[ "$name" != *"$filter"* ]] && continue

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
    else
        grep -E "\[Error\]|error:" "$log" | head -10
        failed+=("$name")
    fi
    rm -f "$log"
done

if [ ${#failed[@]} -gt 0 ]; then
    echo "FAILED: ${failed[*]}"
    exit 1
fi
echo "ALL BUILT"
