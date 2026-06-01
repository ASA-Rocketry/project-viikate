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
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      west2nix,
      zephyr-nix,
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
        ];
      in
      {
        devShells.default = pkgs.mkShell {
          inherit buildInputs;

          ZEPHYR_SDK_INSTALL_DIR = "${sdk}";
          ZEPHYR_TOOLCHAIN_VARIANT = "zephyr";
          WEST_PYTHON = "${pkgs.zephyr.pythonEnv}/bin/python3";
          shellHook = ''
            export ZEPHYR_BASE="$(pwd)/code/zephyr"
          '';
        };
      }
    );
}
