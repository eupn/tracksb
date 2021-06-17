#!/usr/bin/env bash

# Downloads persisted logs from logging NVRAM area of TrackSB
# and displays them on the screen

export ELF_PATH=target/thumbv7em-none-eabihf/release/ble_quaternions
export LOGS_FLASH_BASE_ADDR=08040000
export LOGS_FLASH_SIZE_BYTES=4096
export CHIP_NAME=STM32WB55CGUx

probe-rs-cli dump --chip $CHIP_NAME $LOGS_FLASH_BASE_ADDR $LOGS_FLASH_SIZE_BYTES | defmt-print $ELF_PATH
