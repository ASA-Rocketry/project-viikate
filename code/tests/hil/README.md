# Hardware-in-the-loop tests

Zephyr twister tests that run the flight application (`code/app`, unmodified)
on a real Teensy 4.1.

## Rig

- Raspberry Pi 5 host, Teensy 4.1 on USB.
- `teensy_loader_cli`, `uhubctl` and Nix installed. The user is in `plugdev`, with
  udev rules that let `plugdev` use the Teensy and switch USB port power.

## Running

On the rig, from the repository root:

```bash
nix develop --command code/tests/hil/run.sh
```

`run.sh` writes a twister hardware map with absolute hook paths and calls
`west twister --device-testing`. Extra arguments are passed to twister. Output
goes to `twister-out-hil/`.

## What the boot test checks

`viikate.hil.boot` flashes the image, cuts and restores USB power to force a
cold boot, then checks:

1. The board enumerates with the image's CDC ACM VID:PID (read from the build's
   `.config`) within 20 s. The USB console starts at the `APPLICATION` init
   level, after every driver has probed, so this proves boot did not stall.
2. It stays enumerated with the same device number for 10 s, so there is no
   reset loop, and the console shows no Zephyr fatal error.

## What the log test checks

`viikate.hil.log` builds the same application plus a Zephyr shell on the USB
console (`shell.conf`, test only). The test types commands into it: `fs cat`
reads the log files back off the board's flash, and `vlog fill`
(`src/log_shell.cpp`) makes the logger write many numbered lines.

1. The barometer and imu status lines `main()` logs on boot are in
   `/lfs/log.txt`. After USB power is cut and restored, every earlier line is
   still there, followed by exactly the new boot's two lines.
2. Writing one and a half files' worth of lines rotates the log: `log.1.txt`
   appears, no file exceeds the size limit, no more than the configured number
   of files is kept, and across a power cut every line arrives exactly once, in
   order and unsplit.

## Limitations

- **Power cycling cannot enter the bootloader.** On power-up the Teensy always
  boots its flashed image. `rig.sh flash` gets it into HalfKay with the
  134-baud reboot, which viikate supports (`code/app/src/bootloader.c`). If
  the running image is hung or predates that, someone has to press PROGRAM.
- **Power cycling switches every USB port.** The Pi 5 gangs VBUS: per-port
  `uhubctl` only disconnects data while the board stays powered. `rig.sh
  power-cycle` therefore switches every hub off, which also resets anything
  else on USB.
- On this loader version, the first flash write after entering HalfKay often
  fails with `error writing to Teensy`. `rig.sh flash` retries.
