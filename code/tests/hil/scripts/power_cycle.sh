#!/usr/bin/env bash
# Twister hardware-map hooks take a bare executable path, no arguments.
exec "$(dirname "$(readlink -f "$0")")/rig.sh" power-cycle
