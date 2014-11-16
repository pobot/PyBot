#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Eric Pascual'

from pybot.abelectronics.iopi import IOPiBoard
from pybot import raspi

import time

print("""
This example blinks a LED connected between pin 8 of IC_1 header and the ground.
Hit Ctrl-C to terminate.
""")

board = IOPiBoard(raspi.i2c_bus)
led = board.get_digital_output(8)

try:
    while True:
        led.set()
        time.sleep(0.5)
        led.clear()
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nCtrl-C caught. Terminating program")

