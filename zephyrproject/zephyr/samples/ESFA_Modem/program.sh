#!/bin/sh

set -e

JLinkExe -exitonerror 1 -device STM32F407VG -if SWD -speed 200kHz -autoconnect 1 -commanderscript /home/ariyataghizadeh/zephyrproject/zephyr/samples/ESFA_Modem/program.jlink
