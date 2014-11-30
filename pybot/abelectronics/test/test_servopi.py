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
This example makes servos connected to ports 1 to 3 sweep back and forth in various ways.

Hit Ctrl-C to terminate.
""")

board = ServoPiBoard(i2c_bus)

servos = {
    # servo 1 set to normal move extent, using the horn angle to be specified in the [-90, 90] range
    board.get_servo(
        1, stop_min=StopSpecification(-90, Servo.DEFAULT_MS_MIN), stop_max=StopSpecification(90, Servo.DEFAULT_MS_MAX)
    ),
    # servo 2 set to a reduced move extent, the horn angle being restricted to the [-45, 45] range. The servo will
    # not move when the position will be outside these bounds
    board.get_servo(
        2, stop_min=StopSpecification(-45, 1), stop_max=StopSpecification(45, 2)
    ),
    # servo 3 set as servo 1, but with direction reversed (note the logical position signs)
    board.get_servo(
        3, stop_min=StopSpecification(90, Servo.DEFAULT_MS_MIN), stop_max=StopSpecification(-90, Servo.DEFAULT_MS_MAX)
    )
}

try:
    a, d_a = 0, math.radians(10)

    while True:
        # make the position span the [-90, 90] range, using a sinusoidal control law to get smooth direction changes
        # by decelerating and acceleration at inversion points
        position = math.cos(a) * 90.0
        for servo in servos:
            servo.set_position(position)

        # don't care incrementing the motion control variable infinitely : integers are not bound in Python.
        # BTW chances are that we will have stopped the demo long before the 16 bits threshold is reached ;)
        a += d_a

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nCtrl-C caught. Terminating program")

    print('Bringing back servos to their median position')
    for servo in servos:
        servo.goto_median_position()

    # let them complete their moves before shutting down the control pulses generation
    time.sleep(1)

    print('Shutdown ServoPi board')
    board.shutdown()

