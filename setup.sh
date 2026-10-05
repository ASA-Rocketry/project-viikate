#!/usr/bin/env bash
# one-command environment setup for Project Viikate.
#
#   git clone https://github.com/ASA-rocketry/project-viikate \
#       && cd project-viikate && ./setup.sh
#
# The script is idempotent.
# Once inside the shell, run:
#   west build -b teensy41 code/app -d code/build

set -euo pipefail

NIX_INSTALL_URL="https://nixos.org/nix/install"

# Always request flakes explicitly so the script also works on Nix
# installations where experimental features are not enabled in nix.conf.
NIX=(nix --extra-experimental-features "nix-command flakes")

do_west_update=1
enter_shell=1
do_configure=0
wizard=0

usage() {
    cat <<'EOF'
Usage: ./setup.sh [OPTIONS]

Options:
  --wizard      Guided interactive setup for first-time users; asks about
                each choice below before doing anything.
  --zephyr      Set up the Zephyr/west workspace (default; accepted for
                compatibility, currently the only flow).
  --configure   Also modify system configuration files when needed
                (/etc/nix/nix.conf, /etc/wsl.conf; requires sudo).
                Without this flag the script never edits system files
                and only prints the required changes.
  --no-update   Skip fetching Zephyr and modules (west update).
  --no-shell    Do not enter the nix dev shell at the end.
  -h, --help    Show this help text.
EOF
}

log() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33mwarning:\033[0m %s\n' "$*" >&2; }
die() {
    printf '\033[1;31merror:\033[0m %s\n' "$*" >&2
    exit 1
}

# ask <prompt> [y|n] ; returns 0 for yes, 1 for no.
# The optional second argument sets the default (also used when not a TTY).
ask() {
    local reply default="${2:-n}" hint="[y/N]"
    if [ "$default" = "y" ]; then
        hint="[Y/n]"
    fi
    if [ ! -t 0 ]; then
        [ "$default" = "y" ]
        return
    fi
    printf '\033[1;34m==>\033[0m %s %s ' "$1" "$hint"
    read -r reply
    case "$reply" in
        [yY] | [yY][eE][sS]) return 0 ;;
        [nN] | [nN][oO]) return 1 ;;
        "") [ "$default" = "y" ] ;;
        *) return 1 ;;
    esac
}

while [ $# -gt 0 ]; do
    case "$1" in
        --wizard) wizard=1 ;;
        --zephyr) ;; # default behavior; flag kept for the documented CLI
        --configure) do_configure=1 ;;
        --no-update) do_west_update=0 ;;
        --no-shell) enter_shell=0 ;;
        -h | --help)
            usage
            exit 0
            ;;
        *) die "unknown option '$1' (see --help)" ;;
    esac
    shift
done

[ -f flake.nix ] && [ -f code/west.yml ] ||
    die "run this script from the repository root (flake.nix/code/west.yml not found)"

# --- 0. wizard: pick the options interactively ---------------------------

if [ "$wizard" -eq 1 ]; then
    cat <<'EOF'

Project Viikate setup wizard
============================

This script will set up your development environment:
  1. Check WSL configuration (systemd) if you are on Windows/WSL.
  2. Install Nix if it is missing (official installer, asks for sudo).
  3. Fetch the Zephyr RTOS sources and modules (~9 GB download).
  4. Drop you into the nix development shell when done.

You will now be asked about the optional choices.
Press Enter to accept the [default].
EOF
    if ask "Allow changes to system files (/etc/nix/nix.conf, /etc/wsl.conf, needs sudo)?" n; then
        do_configure=1
    fi
    if ! ask "Fetch Zephyr + modules now (~9 GB)?" y; then
        do_west_update=0
    fi
    if ! ask "Enter the dev shell automatically when setup finishes?" y; then
        enter_shell=0
    fi
    echo
fi

# --- 1. WSL: systemd is required for the Nix daemon ---------------------

is_wsl() { grep -qi microsoft /proc/version 2>/dev/null; }

if is_wsl && [ "$(ps -p 1 -o comm=)" != "systemd" ]; then
    warn "WSL detected, but systemd is not running as PID 1."
    if ! grep -q '^\s*systemd\s*=\s*true' /etc/wsl.conf 2>/dev/null; then
        if [ "$do_configure" -eq 1 ]; then
            printf '[boot]\nsystemd=true\n' | sudo tee -a /etc/wsl.conf >/dev/null
            log "Added [boot] systemd=true to /etc/wsl.conf"
        else
            warn "add these lines to /etc/wsl.conf (or re-run with --configure):"
            printf '    [boot]\n    systemd=true\n' >&2
        fi
    fi
    cat >&2 <<'EOF'

WSL must be restarted for this change to take effect:
  1. Exit this shell.
  2. From Windows (PowerShell/CMD):  wsl --shutdown
  3. Reopen the WSL terminal, cd back into the repo, re-run ./setup.sh
EOF
    exit 0
fi

# --- 2. Nix --------------------------------------------------------------

if ! command -v nix >/dev/null 2>&1; then
    log "Nix not found."
    command -v curl >/dev/null 2>&1 || die "curl is required to install Nix"
    if ask "Install Nix via the official installer (multi-user/daemon mode)?"; then
        curl --proto '=https' --tlsv1.2 -sSf -L "$NIX_INSTALL_URL" |
            sh -s -- --daemon
        export PATH="/nix/var/nix/profiles/default/bin:$PATH"
        command -v nix >/dev/null 2>&1 ||
            die "nix still not on PATH after install; open a new shell and re-run ./setup.sh"
    else
        die "Nix is required. Install it manually ($NIX_INSTALL_URL) and re-run ./setup.sh"
    fi
fi
log "Using $(nix --version)"

# The official installer does not enable flakes. With --configure we enable
# them globally so plain 'nix develop' works for the user later; without it
# we only print instructions. The --extra-experimental-features flags above
# keep this script working either way.
if nix flake --help >/dev/null 2>&1; then
    log "Flakes already enabled"
elif grep -Eq '^[[:space:]]*experimental-features[[:space:]]*=.*flakes' /etc/nix/nix.conf 2>/dev/null; then
    warn "/etc/nix/nix.conf already enables flakes, but nix hasn't picked it up yet"
    warn "continuing anyway: this script passes the features explicitly on every nix call"
elif [ "$do_configure" -eq 1 ]; then
    log "Enabling nix-command and flakes in /etc/nix/nix.conf (requires sudo)"
    echo 'experimental-features = nix-command flakes' | sudo tee -a /etc/nix/nix.conf >/dev/null
    nix flake --help >/dev/null 2>&1 ||
        warn "flakes still unavailable; continuing with per-command feature flags"
else
    warn "flakes are not enabled in /etc/nix/nix.conf; continuing with per-command feature flags"
    warn "to make plain 'nix develop' work, append this line to /etc/nix/nix.conf:"
    warn "    experimental-features = nix-command flakes"
    warn "(or re-run this script with --configure)"
fi

# --- 3. west workspace ---------------------------------------------------

if [ ! -f .west/config ]; then
    log "Initializing west workspace (west init -l code)"
    "${NIX[@]}" develop -c west init -l code
else
    log "west workspace already initialized, skipping init"
fi

if [ "$do_west_update" -eq 1 ]; then
    log "Fetching Zephyr and modules (west update) - this can take a while"
    "${NIX[@]}" develop -c west update
fi

# --- 4. dev shell ---------------------------------------------------------

log "Setup complete."
if [ "$enter_shell" -eq 1 ]; then
    log "Entering dev shell (exit it with 'exit' or Ctrl-D)"
    exec "${NIX[@]}" develop
else
    echo "Next steps:"
    echo "  nix develop                          # enter the dev shell"
    echo "  west build -b teensy41 code/app -d code/build"
fi

