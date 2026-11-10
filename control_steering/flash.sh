#!/usr/bin/env bash
# Build the actuator firmware and flash it to the Teensy 3.2 over USB.
# Needs: arduino-cli with the teensy:avr core, teensy-loader-cli (apt), and
# udev/99-skate-ramp.rules installed. Stop control_steering.py first.
set -euo pipefail
cd "$(dirname "$0")"
build=$(mktemp -d)
arduino-cli compile --fqbn teensy:avr:teensy31 --output-dir "$build" .
teensy_loader_cli --mcu=TEENSY31 -s -w -v "$build/control_steering.ino.hex"
