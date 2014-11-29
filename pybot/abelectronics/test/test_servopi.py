#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Eric Pascual'

import time
import math

from pybot.abelectronics.servopi import ServoPiBoard, StopSpecification, Servo

try:
    from pybot.raspi import i2c_bus
except ImportError:
    from pybot.i2c import SimulatedSMBus
    i2c_bus = SimulatedSMBus()


print("""
This example makes a servo connected to port 1 sweep back and forth.

Hit Ctrl-C to terminate.
""")

board = ServoPiBoard(i2c_bus)

# set up the servo so that its position will we manipulated as an angle between
# -90 and 90
# servo = board.get_servo(1,
#                         stop_min=StopSpecification(-90, Servo.DEFAULT_MS_MIN),
#                         stop_max=StopSpecification(90, Servo.DEFAULT_MS_MAX))

servos = {
    board.get_servo(
        1, stop_min=StopSpecification(-90, Servo.DEFAULT_MS_MIN), stop_max=StopSpecification(90, Servo.DEFAULT_MS_MAX)
    ),
    board.get_servo(
        2, stop_min=StopSpecification(-90, 1), stop_max=StopSpecification(90, 2)
    ),
    board.get_servo(
        3, stop_min=StopSpecification(90, Servo.DEFAULT_MS_MIN), stop_max=StopSpecification(-90, Servo.DEFAULT_MS_MAX)
    )
}

try:
    a, d_a = 0, math.radians(10)

    while True:
        # use a sinusoidal evolution of the position to get smooth direction changes
        position = math.sin(a) * 90.0
        for servo in servos:
            servo.set_position(position)

        a += d_a
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nCtrl-C caught. Terminating program")
    for servo in servos:
        servo.set_position(0)
        time.sleep(1)
        servo.set_floating()

