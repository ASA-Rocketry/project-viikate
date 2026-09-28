{
  description = "Project viikate";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";

    zephyr.url = "github:zephyrproject-rtos/zephyr/v4.4.0";
    zephyr.flake = false;

    zephyr-nix = {
      url = "github:nix-community/zephyr-nix";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        zephyr.follows = "zephyr";
      };
    };

    west2nix = {
      url = "github:adisbladis/west2nix";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        zephyr-nix.follows = "zephyr-nix";
      };
    };

    git-hooks = {
      url = "github:cachix/git-hooks.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      west2nix,
      zephyr-nix,
      git-hooks,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        overlays = [ ];
        pkgs =
          import nixpkgs {
            inherit system overlays;
          }
          // {
            zephyr = zephyr-nix.packages.${system};
            west2nix = west2nix.lib.mkWest2nix { };
          };
        sdk = (
          pkgs.zephyr.sdk.override {
            targets = [
              "arm-zephyr-eabi"
            ];
          }
        );
        teensyWithSymlink = pkgs.symlinkJoin {
          name = "teensy-with-symlink";
          paths = [ pkgs.teensy-loader-cli ];
          postBuild = ''
            mkdir -p $out/bin
            ln -s ${pkgs.teensy-loader-cli}/bin/teensy-loader-cli \
                  $out/bin/teensy_loader_cli
          '';
        };
        pre-commitHooks = withTidy:
          let
            # clang-tidy shipped via pkgs.clang-tools is a nixpkgs cc-wrapper
            # script that injects the host libc headers (C_INCLUDE_PATH) into
            # every TU, which breaks cross-compilation (Zephyr ARM):
            # /.../glibc/include/gnu/stubs.h -> 'gnu/stubs-32.h' file not found.
            # Use the unwrapped binary and drop the GCC-only flags Zephyr emits
            # that clang does not understand.
            clang-tidy-zephyr = pkgs.writeShellScriptBin "clang-tidy-zephyr" ''
              set -uo pipefail
              db=$1; shift
              if [ ! -f "$db" ]; then
                echo "clang-tidy-zephyr: $db not found; skipping (run 'west build' first to generate the compile database)" >&2
                exit 0
              fi
              tmp=$(${pkgs.coreutils}/bin/mktemp -d)
              trap '${pkgs.coreutils}/bin/rm -rf "$tmp"' EXIT
              ${pkgs.python3}/bin/python3 - "$db" "$tmp/compile_commands.json" <<'PYEOF'
              import json
              import shlex
              import sys

              with open(sys.argv[1]) as f:
                  db = json.load(f)

              for entry in db:
                  argv = shlex.split(entry["command"])
                  argv = [
                      a
                      for a in argv
                      if a != "-fno-reorder-functions"
                      and not a.startswith("-mfp16-format")
                  ]
                  entry["command"] = " ".join(shlex.quote(a) for a in argv)

              with open(sys.argv[2], "w") as f:
                  json.dump(db, f)
              PYEOF
              exec ${pkgs.clang-tools}/bin/clang-tidy-unwrapped -p="$tmp" "$@"
            '';
          in
          git-hooks.lib.${system}.run {
            src = pkgs.nix-gitignore.gitignoreSource [ ".jj/" ] ./.;
            tools = {
              clang-tools = pkgs.clang-tools;
            };
            hooks = {
              clang-format.enable = true;
              clang-tidy = {
                enable = withTidy;
                entry =
                  "${clang-tidy-zephyr}/bin/clang-tidy-zephyr "
                  + "code/build/compile_commands.json";
              };
              commitizen.enable = true;
            };
          };
        devHooks = pre-commitHooks true;
        pre-commit-check = pre-commitHooks false;
        buildInputs = [
          sdk
          pkgs.zephyr.pythonEnv
          pkgs.zephyr.hosttools-nix
          pkgs.cmake
          pkgs.ninja
          pkgs.dtc
          pkgs.teensy-loader-cli
          teensyWithSymlink
          pkgs.picocom

          pkgs.meson
          pkgs.ninja

          pkgs.clang-tools
          pkgs.clang
          pkgs.valgrind
          pkgs.gdb

          pkgs.python3
          pkgs.python3Packages.matplotlib
          pkgs.python3Packages.pyserial
          devHooks.config.package
        ];
      in
      {
        checks = {
          inherit pre-commit-check;
        };

        devShells = {
          default = pkgs.mkShell {
            inherit buildInputs;
            nativeBuildInputs = devHooks.enabledPackages;

            ZEPHYR_SDK_INSTALL_DIR = "${sdk}";
            ZEPHYR_TOOLCHAIN_VARIANT = "zephyr";
            WEST_PYTHON = "${pkgs.zephyr.pythonEnv}/bin/python3";
            shellHook = ''
              ${devHooks.shellHook}
              export ZEPHYR_BASE="$(pwd)/zephyr"
            '';
          };

          pre-commit = pkgs.mkShell {
            nativeBuildInputs = devHooks.enabledPackages;
            buildInputs = [ devHooks.config.package ];
            shellHook = devHooks.shellHook;
          };
        };
      }
    );
}
