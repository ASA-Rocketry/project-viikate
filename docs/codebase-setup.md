# Codebase setup guide

Within this guide we will go over the steps of setting up the Project Viikate
codebase. You will need approximately **2 hours of time** to download and setup
everything necessary to build and flash the firmware for the avionics unit.

## Prerequisites

Dependencies come from Nix; the setup script can install it for you.

### Windows users

People working on Windows will need to setup the [Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/install).
If you are using VSCode, it might be useful to install the [WSL Extension](https://code.visualstudio.com/docs/remote/wsl).
In order to flash the avionics board, you will have to [pass through the USB device to the VM](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).

After installing WSL, you should edit the `/etc/wsl.conf` and add the following if it isn't present already (new WSL installs have this by default):
```ini
[boot]
systemd=true
```

Finally, restart the WSL virtual machine.

### Installing nix

In order to install nix, you want to [visit the nix download page](https://nixos.org/download/#nix-install-windows)
and copy and run the **multi-user installation** in the command line.

You will have to enable nix flakes by adding this to `/etc/nix/nix.conf`:
```ini
experimental-features = nix-command flakes
```
by running the command:
```ini
sudo nano /etc/nix/nix.conf
```
To save changes press 'Ctrl + O' and 'Enter', after which you can press 'Ctrl + X' to exit nano.

## Setting up the codebase

Start by cloning the Project Viikate via the command: 
```ini
git clone git@github.com:ASA-Rocketry/project-viikate.git
```
After which enter the cloned folder by typing: 
```ini
cd project-viikate
```
Now you can run the next commands.

### Automated setup

From the repository root, run:
```bash
./setup.sh --wizard --build
```

Sets up Nix, Zephyr, Git hooks, and builds the firmware. Add `--configure` to
allow system configuration edits; see `./setup.sh --help` for all options.

### Manual setup

With Nix and flakes enabled, run from the repository root:
```bash
nix develop
west init -l code
west update --narrow --fetch-opt=--depth=1
west zephyr-export
west build -b teensy41 code/app -d code/build
```

Skip `west init` on reruns. The Nix shell installs Git hooks automatically.

### Keeping West storage small

`code/west.yml` imports only the modules needed by the current Teensy 4.1
firmware: CMSIS 6 for the Arm core, the NXP HAL for the microcontroller, and
the ST HAL for the sensors. CMSIS-DSP is pinned separately for signal processing.
When adding hardware or enabling new Zephyr features, add any required modules
to the manifest's `name-allowlist` and verify with a pristine build.

Use `west update --narrow --fetch-opt=--depth=1` for subsequent dependency updates
too. `--narrow` fetches the pinned revision directly instead of all remote
branches and tags; `--depth=1` limits its history. This retains the manifest's
exact commit pins. Plain `west update` still respects the module allowlist,
but may fetch more history. Shallow fetching does not shrink existing
full-history clones.

If upgrading from the unrestricted manifest, old checkouts are not removed
automatically. From the repository root, first inspect them with
`west forall -c 'git status --short'` **before changing manifests** and preserve
any local changes or commits. Once all dependency checkouts are disposable,
remove only the downloaded directories `zephyr/`, `modules/`, `bootloader/`, and
`tools/`. Keep `.west/` and your application source. Then, inside `nix develop`,
recreate the dependencies and build from scratch:

```bash
west update --narrow --fetch-opt=--depth=1
west zephyr-export
west build -p always -b teensy41 code/app -d code/build
```

The generated `code/build/` directory is also disposable and is recreated by
the build. To inspect local storage use, run `du -sh zephyr modules code/build`
from the repository root. The Nix toolchain occupies additional space in the
shared Nix store, outside this workspace.

### Flashing

Inside the development shell, from the repository root:
```bash
west flash -d code/build
```

Flashing and opening a serial console (the `/dev/ttyACM0` part might be different based on your system, asking ChatGPT will help if you can't find it):
```bash
west flash -d code/build && sleep 1 && picocom -b 115200 /dev/ttyACM0
```

When you open a new shell (i.e. when you open VSCode for the first time), you always have to run:
```bash
nix develop
```

Without globally enabled flakes, use
`nix --extra-experimental-features "nix-command flakes" develop` instead.

### Building from VS Code

Open the repository in VS Code connected to WSL, then use **Terminal > Run Build Task** or press **Ctrl+Shift+B** ( **Cmd+Shift+B** on macOS). The default **Build firmware** task runs `west build -b teensy41 app` from `code/` inside the repository's Nix development environment. The initial West setup (`west init`, `west update`, and `west zephyr-export`) must already be complete.

This task is intended for the VS Code build shortcut. Do not use the CMake Tools build command for this workspace; CMake Tools does not provide the Nix/West environment needed by the Zephyr application.
