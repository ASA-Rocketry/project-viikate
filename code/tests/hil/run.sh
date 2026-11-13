#!/usr/bin/env bash
# Run the hardware-in-the-loop tests on the rig. Run inside `nix develop`.
# Extra arguments are passed to twister.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(git -C "$here" rev-parse --show-toplevel)
out=${VIIKATE_HIL_OUT:-$root/twister-out-hil}
mkdir -p "$out"

# Hook paths must be absolute: twister runs them from its own working directory.
cat > "$out/hardware-map.yaml" <<EOF
- connected: true
  id: teensy41-rig
  platform: teensy41
  product: Teensy 4.1
  runner: teensy
  serial_pty: "$here/scripts/rig.sh serial"
  flash_before: true
  flash_timeout: 180
  post_flash_script: "$here/scripts/power_cycle.sh"
EOF

cd "$root"
# --disable-warnings-as-errors: test the image exactly as the firmware build
# (code/app, CI) produces it, which does not treat warnings as errors.
exec west twister \
    --testsuite-root "$here" \
    --platform teensy41 \
    --device-testing \
    --hardware-map "$out/hardware-map.yaml" \
    --flash-command "$here/scripts/rig.sh,flash" \
    --outdir "$out/twister-out" \
    --inline-logs \
    --disable-warnings-as-errors \
    "$@"
