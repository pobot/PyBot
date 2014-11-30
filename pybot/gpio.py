#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This module allows code using GPIOs on the RaspberryPi to be executed on
a system which is NOT a RaspberryPi (your development station for instance).

This does not mean that we are going to play with the GPIOs of the current system
(supposing it has some) but that we will just mimic the original calls, replacing
actions by trace messages, so that you'll be able to unit test your code on the
development station.

On a real hardware, it will transparently switch to RPi.GPIO module to do the real job.

In order to use it, you just have to replace
>>> from RPi import GPIO
by
>>> from pybot.gpio import GPIO

Leave all the other statements involving GPIO untouched.

By default, reading an input will always return a low state. It is possible to define the
simulated state by using something like the following statement before reading the GPIO :
>>> GPIO.input_states[3] = GPIO.HIGH

Subsequent calls to GPIO.input(3) will return GPIO.HIGH instead of the default GPIO.LOW state.
"""

__author__ = 'Eric Pascual'

# test if we are running on a real RasPi or not and define GPIO symbol accordingly
import platform
if platform.machine().startswith('armv6'):
    from RPi import GPIO

else:
    print('[WARNING] ****** GPIO module is simulated since we are not running on a RaspberryPi ******')

    class GPIO(object):
        """ This class mimics the original RPi.GPIO module.
        (http://sourceforge.net/projects/raspberry-gpio-python)

        WARNINGS:

        1/ Constant values are NOT the real ones. Anyway, it's a very bad idea to write code depending
        on constants real value, since this can change.

        2/ the current version does not implement all the methods available in the native GPIO module
        for the moment. Maybe this will change as new fake ones will be required.
        """
        OUT, IN = 0, 1
        LOW, HIGH = 0, 1
        BOARD, BCM = 0, 1
        PUD_DOWN, PUD_UP = 0, 1

        _modes = {
            BOARD: 'BOARD',
            BCM: 'CHIP'
        }

        _directions = {
            OUT: 'OUTPUT',
            IN: 'INPUT'
        }

        _states = {
            LOW: 'LOW',
            HIGH: 'HIGH'
        }

        _pullups = {
            PUD_DOWN: 'DOWN',
            PUD_UP: 'UP'
        }

        input_states = {}

        @staticmethod
        def _trace(msg):
            print("[GPIO] %s" % msg)

        @staticmethod
        def setwarnings(enabled):
            GPIO._trace("warnings sets to %s" % enabled)

        @staticmethod
        def setmode(addressing_mode):
            GPIO._trace("addressing mode set to %s" % GPIO._modes[addressing_mode])

        @staticmethod
        def setup(io_num, direction, pull_up_down=PUD_UP):
            GPIO._trace(
                "GPIO %d configured as %s with pull up set to %s" % (
                    io_num, GPIO._directions[direction], GPIO._pullups[pull_up_down]
                ))

        @staticmethod
        def output(io_num, state):
            GPIO._trace("Output %d set to %s" % (io_num, GPIO._states[state]))

        @staticmethod
        def input(io_num):
            # get the simulated state if defined, otherwise set it to the default value
            try:
                state = GPIO.input_states[io_num]
            except KeyError:
                GPIO.input_states[io_num] = state = GPIO.LOW

            GPIO._trace("Reading input %d (simulated state=%s)" % (io_num, state))
            return state

        @staticmethod
        def cleanup():
            GPIO._trace("GPIOs returned to their default settings")

del platform