#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Eric Pascual'

from pybot.abelectronics.iopi import IOPiBoard
from pybot import raspi

import time
import sys

print("""
This example blinks a LED connected between pin 8 of IC_1 header and the ground.
It also reads the state of a button connected between pin 1 of IC_1 header and the ground.

Hit Ctrl-C to terminate.
""")

board = IOPiBoard(raspi.i2c_bus)
led = board.get_digital_output(IOPiBoard.EXPANDER_1, 8)
button = board.get_digital_input(IOPiBoard.EXPANDER_1, 1, pullup_enabled=True)


def display_button_state():
    print("\033[30Dbutton state = %s" % button.is_set()),
    sys.stdout.flush()

try:
    on = True
    while True:
        if on:
            led.set()
        else:
            led.clear()

        display_button_state()
        on = not on
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nCtrl-C caught. Terminating program")
    led.clear()

