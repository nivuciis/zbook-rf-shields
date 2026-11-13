# Copyright (c) 2026 Centro de Inovacao EDGE
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mesh_node)

target_sources(app PRIVATE
  src/main.c
  src/mesh.c
  src/radio.c
  src/meshtastic.c
  src/nodedb.c
  src/pb.c
  src/aes128.c
  src/hw_models.c
)
