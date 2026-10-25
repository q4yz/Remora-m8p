#!/usr/bin/env bash
#
# reflash.sh - example script: update a Remora board's SD card from the Raspberry Pi
#
# Uploads a new config.txt and/or firmware.bin over the SPI link (see README.md),
# then reboots the board once so the SD bootloader flashes the new firmware.
# LinuxCNC must not be running.
#
# Usage:
#   ./reflash.sh firmware.bin              # new firmware only
#   ./reflash.sh config.txt                # new configuration only
#   ./reflash.sh firmware.bin config.txt   # both, in any order
#   ./reflash.sh -n config.txt my.json     # any file name, uploaded as config.txt
#
# Environment:
#   SPI_DEV        spidev device        (default /dev/spidev0.0)
#   SPI_SPEED      SPI clock in Hz      (default 4000000)
#   REMORA_UPLOAD  path to remora-upload (default: built next to this script)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPI_DEV="${SPI_DEV:-/dev/spidev0.0}"
SPI_SPEED="${SPI_SPEED:-4000000}"
REMORA_UPLOAD="${REMORA_UPLOAD:-$SCRIPT_DIR/remora-upload}"
FLASH_WAIT=15          # seconds for the bootloader to flash and the firmware to start

die()  { echo "error: $*" >&2; exit 1; }
info() { echo "==> $*"; }

usage() {
    sed -n '9,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

# ---- collect the files ------------------------------------------------------

firmware=""
config=""

while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help) usage 0 ;;
        -n)
            # explicit target name: -n firmware.bin FILE or -n config.txt FILE
            [ $# -ge 3 ] || usage 2
            case "$2" in
                firmware.bin) firmware="$3" ;;
                config.txt)   config="$3" ;;
                *) die "the board only accepts firmware.bin or config.txt" ;;
            esac
            shift 3 ;;
        *)
            case "$(basename "$1")" in
                firmware.bin) firmware="$1" ;;
                config.txt)   config="$1" ;;
                *) die "$1: name it firmware.bin or config.txt, or use -n NAME FILE" ;;
            esac
            shift ;;
    esac
done

[ -n "$firmware" ] || [ -n "$config" ] || usage 2
[ -z "$firmware" ] || [ -f "$firmware" ] || die "$firmware not found"
[ -z "$config" ]   || [ -f "$config" ]   || die "$config not found"

# ---- sanity checks ----------------------------------------------------------

if [ -n "$config" ] && command -v python3 >/dev/null; then
    # the board can't report a broken config, catch JSON errors before uploading
    python3 -m json.tool "$config" >/dev/null || die "$config is not valid JSON"
fi

if [ -n "$firmware" ]; then
    size=$(stat -c %s "$firmware")
    [ "$size" -gt 1024 ] || die "$firmware is only $size bytes, is it really a firmware image?"
fi

if pgrep -x rtapi_app >/dev/null || pgrep -x linuxcnc >/dev/null; then
    die "LinuxCNC is running, stop it first"
fi

[ -e "$SPI_DEV" ] || die "$SPI_DEV missing - add 'dtparam=spi=on' to /boot/firmware/config.txt and reboot"

SUDO=""
if [ "$(id -u)" -ne 0 ] && [ ! -w "$SPI_DEV" ]; then
    SUDO="sudo"
fi

# ---- build the upload tool if needed ----------------------------------------

if [ ! -x "$REMORA_UPLOAD" ] || [ "$SCRIPT_DIR/remora-upload.c" -nt "$REMORA_UPLOAD" ]; then
    info "Building remora-upload"
    command -v gcc >/dev/null || die "gcc not found (sudo apt install build-essential)"
    gcc -O2 -Wall -o "$REMORA_UPLOAD" "$SCRIPT_DIR/remora-upload.c"
fi

upload() {
    # upload FILE NAME [extra options]
    local file="$1" name="$2"
    shift 2
    $SUDO "$REMORA_UPLOAD" -d "$SPI_DEV" -s "$SPI_SPEED" -n "$name" "$@" "$file"
}

# ---- upload, rebooting only after the last file ------------------------------

if [ -n "$config" ] && [ -n "$firmware" ]; then
    info "Uploading configuration $config"
    upload "$config" config.txt -k
    info "Uploading firmware $firmware"
    upload "$firmware" firmware.bin
elif [ -n "$config" ]; then
    info "Uploading configuration $config"
    upload "$config" config.txt
else
    info "Uploading firmware $firmware"
    upload "$firmware" firmware.bin
fi

if [ -n "$firmware" ]; then
    info "Waiting ${FLASH_WAIT}s for the bootloader to flash the new firmware"
    for ((i = FLASH_WAIT; i > 0; i--)); do
        printf '\r    %2ds ' "$i"
        sleep 1
    done
    printf '\r'
fi

info "Done - start LinuxCNC as usual"
