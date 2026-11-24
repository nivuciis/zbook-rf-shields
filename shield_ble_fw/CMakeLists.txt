# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Registra o módulo zbook (pasta pai) para o build encontrar a placa
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(shield_ble)

target_sources(app PRIVATE src/main.c src/ch592_isp.c src/ble_uart.c)

# Firmware do CH592 embutido no build: hex -> bin -> array C (ch592_fw.inc).
# Por padrão é a ponte BLE <-> UART de ch592/ble_uart, recompilada (make) a
# cada build; -DCH592_HEX=... grava outro hex no lugar.
set(ch592_ble_dir ${CMAKE_CURRENT_SOURCE_DIR}/ch592/ble_uart)
set(CH592_HEX ${ch592_ble_dir}/ble_uart.hex CACHE FILEPATH
    "Firmware (Intel HEX) a gravar no CH592")
set(ch592_bin ${CMAKE_CURRENT_BINARY_DIR}/ch592_fw.bin)

if(CH592_HEX STREQUAL ${ch592_ble_dir}/ble_uart.hex)
  add_custom_target(ch592_ble_uart
    COMMAND make -s -C ${ch592_ble_dir}
    BYPRODUCTS ${CH592_HEX}
    COMMENT "Compilando o firmware BLE do CH592"
  )
  set(ch592_deps ch592_ble_uart)
endif()

add_custom_command(
  OUTPUT ${ch592_bin}
  COMMAND ${CMAKE_OBJCOPY} -I ihex -O binary ${CH592_HEX} ${ch592_bin}
  DEPENDS ${CH592_HEX} ${ch592_deps}
)

generate_inc_file_for_target(app ${ch592_bin}
  ${ZEPHYR_BINARY_DIR}/include/generated/ch592_fw.inc)
