#!/usr/bin/env bash
# Control script for the Teensy 4.1 hardware-in-the-loop rig (Raspberry Pi 5 host).
#
#   rig.sh flash --build-dir DIR [--board-id ID]   flash DIR/zephyr/zephyr.hex
#   rig.sh power-cycle                             cut and restore USB power
#   rig.sh serial                                  attach the DUT console (twister serial_pty)
#   rig.sh usb-device VID PID [--timeout S]        print "<bus>-<devnum> <tty>" once enumerated
#
# Twister's --flash-command inserts --build-dir/--board-id before the trailing
# arguments, so options and the subcommand may appear in any order.
#
# Environment overrides:
#   VIIKATE_HIL_HUBS            uhubctl hub locations to switch (default "1 2 3 4")
#   VIIKATE_HIL_POWER_OFF_S     seconds to keep USB power off (default 2)
#   VIIKATE_HIL_BOOTLOADER_S    seconds to wait for the Teensy bootloader (default 30)
#   VIIKATE_HIL_USB_ID          DUT console VID:PID for `serial` (default 2fe3:0004)
#   VIIKATE_HIL_SERIAL_WAIT_S   seconds `serial` waits for the console (default 30)
set -euo pipefail

TEENSY_VID=16c0
TEENSY_BOOTLOADER_PID=0478   # HalfKay bootloader
TEENSYDUINO_SERIAL_PID=0483  # Teensyduino USB Serial (supports 134-baud reboot)

log() { echo "[rig] $*" >&2; }
die() { log "error: $*"; exit 1; }

# usb_lookup VID PID -> prints "<bus>-<devnum> <tty-or-empty>", fails if absent.
usb_lookup() {
    local vid=$1 pid=$2 d t tty
    for d in /sys/bus/usb/devices/*; do
        [ -f "$d/idVendor" ] || continue
        [ "$(<"$d/idVendor")" = "$vid" ] && [ "$(<"$d/idProduct")" = "$pid" ] || continue
        tty=""
        for t in "$d"/*:*/tty/*; do
            [ -e "$t" ] && tty=/dev/$(basename "$t") && break
        done
        echo "$(<"$d/busnum")-$(<"$d/devnum") $tty"
        return 0
    done
    return 1
}

# wait_usb VID PID TIMEOUT -> like usb_lookup, polling until TIMEOUT seconds.
wait_usb() {
    local vid=$1 pid=$2 timeout=$3 end=$((SECONDS + $3)) out
    while :; do
        if out=$(usb_lookup "$vid" "$pid") && [ -n "${out#* }" ]; then
            echo "$out"; return 0
        fi
        [ "$SECONDS" -ge "$end" ] && return 1
        sleep 0.2
    done
}

cmd_flash() {
    local build_dir=$1 hex wait_s=${VIIKATE_HIL_BOOTLOADER_S:-30} dev attempt
    [ -n "$build_dir" ] || die "flash needs --build-dir"
    hex=$build_dir/zephyr/zephyr.hex
    [ -f "$hex" ] || die "missing $hex"

    if usb_lookup "$TEENSY_VID" "$TEENSY_BOOTLOADER_PID" >/dev/null; then
        log "Teensy already in bootloader"
    elif dev=$(usb_lookup "$TEENSY_VID" "$TEENSYDUINO_SERIAL_PID"); then
        log "soft-rebooting Teensyduino serial ${dev#* } into bootloader"
        stty -F "${dev#* }" 134 || true
    else
        # USB power cycling cannot enter HalfKay: the Teensy always boots its
        # flashed image on power-up. Only PROGRAM (button/pin) or firmware that
        # reboots itself into the bootloader can get us here.
        log "Teensy is not in a reflashable state; press PROGRAM (waiting ${wait_s}s)"
    fi

    # The first block write after entering HalfKay regularly fails with
    # "error writing to Teensy" on this loader version; a retry succeeds.
    for attempt in 1 2 3; do
        if timeout "$wait_s" teensy_loader_cli --mcu=TEENSY41 -w -v "$hex"; then
            return 0
        fi
        log "flash attempt $attempt failed"
        sleep 1
    done
    die "could not flash $hex"
}

cmd_power_cycle() {
    local hubs=${VIIKATE_HIL_HUBS:-1 2 3 4} off_s=${VIIKATE_HIL_POWER_OFF_S:-2} h uhubctl
    # Debian installs it in /usr/sbin, which is not on PATH for non-root shells.
    uhubctl=$(PATH=$PATH:/usr/sbin:/sbin command -v uhubctl) || die "uhubctl not installed"
    # The Pi 5 gangs VBUS for all ports: per-port switching only disconnects
    # data, the DUT stays powered. Switching every hub off really cuts power
    # (this also resets anything else on USB, e.g. the logic analyzer).
    log "USB power off (hubs: $hubs) for ${off_s}s"
    for h in $hubs; do "$uhubctl" -l "$h" -a off -r 10 >/dev/null; done
    sleep "$off_s"
    for h in $hubs; do "$uhubctl" -l "$h" -a on >/dev/null; done
    log "USB power on"
}

cmd_serial() {
    local id=${VIIKATE_HIL_USB_ID:-2fe3:0004} out tty
    out=$(wait_usb "${id%:*}" "${id#*:}" "${VIIKATE_HIL_SERIAL_WAIT_S:-30}") \
        || die "console $id did not appear"
    tty=${out#* }
    log "attached to $tty ($id)"
    stty -F "$tty" 115200 raw -echo
    exec 3<>"$tty"
    # Host -> board in the background; board -> host in the foreground, so the
    # script lives exactly as long as twister keeps reading the console.
    cat >&3 &
    exec cat <&3
}

cmd_usb_device() {
    local vid=${1:?VID} pid=${2:?PID} timeout=${3:-0}
    wait_usb "$vid" "$pid" "$timeout"
}

main() {
    local sub="" build_dir="" timeout=0 args=()
    while [ $# -gt 0 ]; do
        case $1 in
            --build-dir) build_dir=$2; shift 2 ;;
            --board-id) shift 2 ;;  # single-DUT rig; accepted for twister
            --timeout) timeout=$2; shift 2 ;;
            -*) die "unknown option $1" ;;
            *) if [ -z "$sub" ]; then sub=$1; else args+=("$1"); fi; shift ;;
        esac
    done
    case $sub in
        flash) cmd_flash "$build_dir" ;;
        power-cycle) cmd_power_cycle ;;
        serial) cmd_serial ;;
        usb-device) cmd_usb_device "${args[@]}" "$timeout" ;;
        *) die "usage: rig.sh {flash|power-cycle|serial|usb-device} ..." ;;
    esac
}

main "$@"
