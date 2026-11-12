# Copyright (c) 2026 Centro de Inovacao EDGE
# SPDX-License-Identifier: Apache-2.0
#
# Task runner for the zbook-rf-shields firmware

set windows-shell := ["powershell.exe", "-NoLogo", "-NoProfile", "-Command"]

# Defaults, each one overridable through an environment variable
board  := env("ZBOOK_BOARD", "zbook/rp2350b/m33")
shield := env("ZBOOK_SHIELD", "zbook_lora")
app    := env("ZBOOK_APP", "rfm95_spi_check")
runner := env("ZBOOK_RUNNER", "probe-rs")
port   := env("ZBOOK_PORT", "COM19")
baud   := "115200"

# List the available recipes
default:
    @just --list

# Incremental build (extra args go to west build, e.g. -- -DCONFIG_FOO=y)
build target=app *args:
    west build -b {{board}} --shield {{shield}} -d build/{{target}} apps/{{target}} {{args}}

# Clean build from scratch (needed after editing overlays or Kconfig files)
rebuild target=app *args:
    west build -p always -b {{board}} --shield {{shield}} -d build/{{target}} apps/{{target}} {{args}}

# Flash the last build through the PROG USB port
flash target=app:
    west flash -d build/{{target}} --runner {{runner}}

# Build and flash
run target=app: (build target) (flash target)

# Open the serial console (PROG USB port); exit with Ctrl+]
monitor com=port:
    python -m serial.tools.miniterm {{com}} {{baud}}

# Build, flash and open the serial console
dev target=app com=port: (build target) (flash target) (monitor com)

# Kconfig menu for an app (the app must have been built once)
menuconfig target=app:
    west build -d build/{{target}} -t guiconfig

# Delete an app's build directory
clean target=app:
    west build -d build/{{target}} -t pristine

# List the debug probes connected (the ZBook shows up as CMSIS-DAP)
probes:
    probe-rs list

# Pull this repository and update the west workspace
update:
    git pullz
    west update