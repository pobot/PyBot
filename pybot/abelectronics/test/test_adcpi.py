#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Eric Pascual'

import sys
import time

from pybot.abelectronics.adcpi import ADCPiBoard

try:
    from pybot.raspi import i2c_bus
except ImportError:
    from pybot.i2c import SimulatedSMBus
    i2c_bus = SimulatedSMBus()


print("""
This example reads an analog input connected to pin 1 of the header.

Hit Ctrl-C to terminate.
""")

board = ADCPiBoard(i2c_bus)
ain_1 = board.get_analog_input(1, rate=ADCPiBoard.RATE_12)


def display_voltage_and_raw(v, r):
    print("\033[80Dinput voltage = %f (%d)\033[K" % (v, r)),
    sys.stdout.flush()


def display_voltage(v):
    print("\033[80Dinput voltage = %f\033[K" % v),
    sys.stdout.flush()

try:
    on = True
    while True:
        # raw = ain_1.read_raw()
        # voltage = ain_1.convert_raw(raw)
        # display_voltage_and_raw(voltage, raw)

        display_voltage(ain_1.read_voltage())
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nCtrl-C caught. Terminating program")

