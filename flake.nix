{
  description = "eVOLVER Arduino firmware — SAMD21 Mini / miniEvolver sketches";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";

  outputs =
    { self, nixpkgs }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
      mkApp =
        program: description:
        {
          type = "app";
          inherit program;
          meta.description = description;
        };
    in
    {
      devShells = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
        in
        {
          default = pkgs.mkShell {
            name = "evolver-arduino";
            packages = [
              pkgs.arduino-cli
              pkgs.python3
              pkgs.python3Packages.pyserial
              pkgs.python3Packages.pytest
            ];
            shellHook = ''
              echo "evolver-arduino dev shell"
              echo "arduino-cli $(arduino-cli version 2>/dev/null || echo '(not configured)')"
              echo ""
              echo "First-time board setup:  nix run .#setup-arduino"
              echo "Build firmware:          nix run .#build-firmware"
              echo "Upload (set PORT):       PORT=/dev/ttyACM0 nix run .#upload-firmware"
              echo "Protocol tests:          pytest tests/"
            '';
          };
        }
      );

      apps = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};

          setup-arduino = pkgs.writeShellApplication {
            name = "setup-arduino";
            runtimeInputs = [ pkgs.arduino-cli ];
            text = ''
              set -euo pipefail
              echo "==> Configuring arduino-cli for SparkFun SAMD21..."
              arduino-cli config init --overwrite 2>/dev/null || true
              arduino-cli config set board_manager.additional_urls \
                https://raw.githubusercontent.com/sparkfun/Arduino_Boards/main/IDE_Board_Manager/package_sparkfun_index.json
              arduino-cli core update-index
              arduino-cli core install sparkfun:samd
              echo "==> Installing required libraries..."
              arduino-cli lib install "FlashStorage_SAMD" "PID" "SimpleTimer"
              echo "==> Setup complete."
            '';
          };

          build-firmware = pkgs.writeShellApplication {
            name = "build-firmware";
            runtimeInputs = [ pkgs.arduino-cli ];
            text = ''
              set -euo pipefail
              SKETCH="''${SKETCH:-SAMD21/MINEVOLVER}"
              FQBN="''${FQBN:-sparkfun:samd:sparkfun_samd21_mini}"
              OUT="''${PWD}/build/MINEVOLVER"
              echo "==> Compiling $SKETCH for $FQBN ..."
              mkdir -p "$OUT"
              arduino-cli compile --fqbn "$FQBN" --output-dir "$OUT" "$SKETCH"
              echo "==> Build artifacts: $OUT/"
            '';
          };

          # Upload is intentionally interactive and requires explicit PORT.
          # Never runs automatically — hardware safety measure.
          upload-firmware = pkgs.writeShellApplication {
            name = "upload-firmware";
            runtimeInputs = [ pkgs.arduino-cli ];
            text = ''
              set -euo pipefail
              PORT="''${PORT:-}"
              FQBN="''${FQBN:-sparkfun:samd:sparkfun_samd21_mini}"
              SKETCH="''${SKETCH:-SAMD21/MINEVOLVER}"

              if [ -z "$PORT" ]; then
                echo "ERROR: Set PORT to the serial device (e.g. PORT=/dev/ttyACM0)"
                exit 1
              fi

              echo "WARNING: This will flash firmware to $PORT."
              echo "Do NOT run against a device in an active experiment."
              printf "Continue? [y/N] "
              read -r confirm
              case "$confirm" in [yY]*) ;; *) echo "Aborted."; exit 1 ;; esac

              arduino-cli upload --port "$PORT" --fqbn "$FQBN" "$SKETCH"
              echo "==> Upload complete to $PORT."
            '';
          };

          setup-arduino-nano = pkgs.writeShellApplication {
            name = "setup-arduino-nano";
            runtimeInputs = [ pkgs.arduino-cli ];
            text = ''
              set -euo pipefail
              echo "==> Configuring arduino-cli for Arduino Nano (AVR)..."
              arduino-cli config init --overwrite 2>/dev/null || true
              arduino-cli core update-index
              arduino-cli core install arduino:avr
              echo "==> Installing required libraries..."
              arduino-cli lib install "PID"
              echo "==> Setup complete. (evolver_si is bundled in libraries/)"
            '';
          };

          build-firmware-nano = pkgs.writeShellApplication {
            name = "build-firmware-nano";
            runtimeInputs = [ pkgs.arduino-cli ];
            text = ''
              set -euo pipefail
              SKETCH="''${SKETCH:-Nano/MINEVOLVER}"
              FQBN="''${FQBN:-arduino:avr:nano:cpu=atmega328old}"
              OUT="''${PWD}/build/MINEVOLVER-Nano"
              echo "==> Compiling $SKETCH for $FQBN ..."
              mkdir -p "$OUT"
              arduino-cli compile \
                --fqbn "$FQBN" \
                --output-dir "$OUT" \
                --libraries "''${PWD}/libraries" \
                "$SKETCH"
              echo "==> Build artifacts: $OUT/"
            '';
          };

          upload-firmware-nano = pkgs.writeShellApplication {
            name = "upload-firmware-nano";
            runtimeInputs = [ pkgs.arduino-cli ];
            text = ''
              set -euo pipefail
              PORT="''${PORT:-/dev/ttyUSB0}"
              FQBN="''${FQBN:-arduino:avr:nano:cpu=atmega328old}"
              SKETCH="''${SKETCH:-Nano/MINEVOLVER}"

              echo "WARNING: This will flash firmware to $PORT (Arduino Nano)."
              echo "Do NOT run against a device in an active experiment."
              printf "Continue? [y/N] "
              read -r confirm
              case "$confirm" in [yY]*) ;; *) echo "Aborted."; exit 1 ;; esac

              OUT="''${PWD}/build/MINEVOLVER-Nano"
              arduino-cli upload \
                --port "$PORT" \
                --fqbn "$FQBN" \
                --input-dir "$OUT"
              echo "==> Upload complete to $PORT."
            '';
          };
        in
        {
          "setup-arduino" = mkApp "${setup-arduino}/bin/setup-arduino" "Set up Arduino tooling for SAMD21 firmware.";
          "build-firmware" = mkApp "${build-firmware}/bin/build-firmware" "Build SAMD21 eVOLVER firmware.";
          "upload-firmware" = mkApp "${upload-firmware}/bin/upload-firmware" "Upload SAMD21 eVOLVER firmware.";
          "setup-arduino-nano" = mkApp "${setup-arduino-nano}/bin/setup-arduino-nano" "Set up Arduino Nano tooling.";
          "build-firmware-nano" =
            mkApp "${build-firmware-nano}/bin/build-firmware-nano" "Build Arduino Nano eVOLVER firmware.";
          "upload-firmware-nano" =
            mkApp "${upload-firmware-nano}/bin/upload-firmware-nano" "Upload Arduino Nano eVOLVER firmware.";
          default = mkApp "${build-firmware}/bin/build-firmware" "Build SAMD21 eVOLVER firmware.";
        }
      );

      # Protocol tests run in sandbox (pure Python, no hardware).
      checks = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
        in
        {
          protocol-tests = pkgs.runCommand "protocol-tests" {
            nativeBuildInputs = [
              (pkgs.python3.withPackages (ps: [
                ps.pytest
              ]))
            ];
            src = ./tests;
          } ''
            cp -r "$src" tests
            cd tests
            pytest -v
            touch $out
          '';
        }
      );
    };
}
