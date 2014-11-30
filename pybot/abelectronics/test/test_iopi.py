#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This example blinks a LED connected between pin 8 of IC_1 header and the ground.

It also reads the state of buttons connected between pin 1, 2 and 3 of IC_1 header and the ground.
"""

__author__ = 'Eric Pascual'

from pybot.abelectronics.iopi import IOPiBoard
from pybot import raspi

import time
import sys

print(sys.modules[__name__].__doc__.strip())
print("\nHit Ctrl-C to terminate.")

board = IOPiBoard(raspi.i2c_bus)
led = board.get_digital_output(IOPiBoard.EXPANDER_1, 8)
buttons = [board.get_digital_input(IOPiBoard.EXPANDER_1, i, pullup_enabled=True) for i in range(1, 4)]

last_states = [1] * 3


def display_buttons_state():
    states = [buttons[i].is_set() for i in range(0, 3)]
    if states != last_states:
        print("\033[30Dbuttons: %s" % states),
        sys.stdout.flush()
    return states

try:
    on = True
    next_change = 0
    while True:
        now = time.time()
        if now >= next_change:
            if on:
                led.set()
            else:
                led.clear()
            next_change = now + 0.5
            on = not on

        last_states = display_buttons_state()
        time.sleep(0.1)

except KeyboardInterrupt:
    print(" caught. Terminating program")
    led.clear()

