# Codebase setup guide

Within this guide we will go over the steps of setting up the Project Viikate
codebase. You will need approximately **2 hours of time** to download and setup
everything necessary to build and flash the firmware for the avioncis unit.

## Prerequisites

Before we setup the toolchain you will need to install nix, the tool we use for
installing dependencies. While it is possible to avoid using nix, it can be
quite difficult.

### Windows users

People working on Windows will need to setup the [Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/install).
If you are using VSCode, it might be useful to install the [WSL Extension](https://code.visualstudio.com/docs/remote/wsl).
In order to flash the avionics board, you will have to [pass through the USB device to the VM](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).

After installing WSL, you should edit the `/etc/wsl.conf` and add the following:
```ini
[boot]
systemd=true
```

Finally, restart the WSL virtual machine.

### Installing nix

In order to install nix, you want to [visit the nix download page](https://nixos.org/download/#nix-install-windows)
and copy and run the **multi-user installation** in the command line.

## Seting up the codebase

Quick way:
`bash
nix develop
cd code
west init -l .
west build -b teensy41 app
`

Flashing:
`bash
west flash
`
