#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A set of simple classes for interacting with the ServoPi board from AB Electronics
(https://www.abelectronics.co.uk/products/3/Raspberry-Pi/44/Servo-Pi).

At the end of the chain is the individual servo, provided by the class :py:class:`Servo`.
Instances are attached to a :py:class:`Port`, modeling a real MCP23017 port. Ports are themselves attached to a
:py:class:`MCP23017`, which is itself attached to a :py:class:`IOPiBoard`.

All intricacies related to registers configuration and alike are hidden from the user,
and a lot of things are optimized by caching instances and data for avoiding paying a too high price
for a high level design. Even if the added overhead could seem penalizing in term of performances,
this is not that obvious, since all the processing done here must be done somewhere anyway. There
are thus chances that the effective penalty (if ever any) will be negligible for most of the
applications.

To allow several classes instances being used for modeling a configuration with multiple boards,
possibly of different types, the underlying bus must be created outside and provided as a dependency
(see documentation about de "Dependency Injection" design pattern). Being able to provide whatever
implementation of the basic read and write operations allows the use of a fake class simulating
real operations, which made possible to test the rest of the application on a system not having
a SMBus resource available, such as the development station PC for instance, or providing access to
an I2C bus using an USB adapter for instance.

Here is an example of the code used for configuring and reading an input.

>>> from pybot.raspi import i2c_bus
>>> from pybot.abelectronics.adcpi import ADCPiBoard
>>> ...
>>> # create an instance of the board (using default I2C addresses)
>>> board = ServoPiBoard(i2c_bus)
>>> # define a servo, connected to channel 1 on the board header
>>> servo = board.get_servo(1)
>>> ...
>>> # move it to median position
>>> servo.set_position(0.5)

Pretty simple, no ?

Beyond the basic usage illustrated above, the servo instance can be configured in a lot
of different ways :
    - pulse durations can be set to adapt to the model, or to limit the position extent
    - custom values can be assigned to these limits, so that instead of manipulating the
      angles in range 0 to 180, it is possible to use a percent of the total span, or
      shift the angles domain to range -90.0 to 90.0, or any convention representing
      something meaningful at the application level

Refer to :py:class:`Servo` and :py:class:`StopSpecification` classes for details.
"""
__author__ = 'Eric PASCUAL for POBOT'
__version__ = '1.0.0'
__email__ = 'eric@pobot.org'


import math
import time
from collections import namedtuple
from functools import total_ordering


@total_ordering
class StopSpecification(namedtuple("StopSpecification", "logical_position msecs")):
    """ A named tuple defining a stop position (i.e. extent limit) of the servo.

    The position is defined by the corresponding pulse duration and the associated
    application domain value. This value can be an angle in degrees, a 0-100 range percentage,...
    Said differently, it defines the scale, the unit and the possible direction reversal used
    by the application to provide a position (absolute or relative) for the servo.

    For instance, setting a servo with min and max stops set to (90, 0.5) and (-90, 2.5)
    respectively will allow the application to deal with degrees positions, with 90 corresponding
    to 3 o'clock and -90 to 9 o'clock (which reverses the angle sign from the default settings,
    counting them in CCW mode).

    Using pulse durations inside the physical limits of the servo provides a software extent
    limitation, since the effective orders sent to the servo will never be outside this limited
    range.

    An order relation is defined, based on the pulse duration (since the logical position can
    be in reverse direction). The equality relation is based on both instance attributes.
    """
    __slots__ = ()

    def __new__(cls, logical_position, msecs):
        """
        :param float logical_position: the application level value associated to this stop
        :param float msecs: the associated pulse duration
        :raise: ValueError if the duration is outside a reasonable range for servos
        """
        if not 0.0 <= msecs <= 3.0:
            raise ValueError('msecs must be in [0.0-3.0]')
        return super(StopSpecification, cls).__new__(cls, float(logical_position), float(msecs))

    def __lt__(self, other):
        return self.msecs < other.msecs

    def __le__(self, other):
        return self.msecs <= other.msecs

    def __eq__(self, other):
        return self.msecs == other.msecs and self.logical_position == other.logical_position

    def __sub__(self, other):
        return StopSpecification(self.logical_position - other.logical_position, self.msecs - other.msecs)

    def __add__(self, other):
        return StopSpecification(self.logical_position + other.logical_position, self.msecs + other.msecs)

    def scale(self, factor):
        """ Returns a scaled instance by multiplying both attributes by a given factor.
        :param float factor: scaling factor
        :return: a new instance, with attributes set to the original ones scaled
        """
        return StopSpecification(self.logical_position * factor, self.msecs * factor)

    def __str__(self):
        return '(p:%.1f t:%.1f)' % (self.logical_position, self.msecs)


class Servo(object):
    """ Logical model of a servo controlled by the board.
    """
    DEFAULT_POS_MIN, DEFAULT_POS_MAX = (0.0, 180.0)
    DEFAULT_MS_MIN, DEFAULT_MS_MAX = (0.6, 2.4)

    DEFAULT_STOP_MIN = StopSpecification(DEFAULT_POS_MIN, DEFAULT_MS_MIN)
    DEFAULT_STOP_MAX = StopSpecification(DEFAULT_POS_MAX, DEFAULT_MS_MAX)

    def __init__(self, board, channel, stop_min=DEFAULT_STOP_MIN, stop_max=DEFAULT_STOP_MAX):
        """ If default settings are used, the servo model is supposed to honor a standard
        0.5 to 2.5 ms pulse for a 180 degrees total horn course counted clock wise. The servo position
        will thus be given in subsequent calls as a floating point value representing the horn angle in degrees.

        This can be customized by specifying specific stop position definition, both in terms of pulse width
        and associated application level position value.

        :param ServoPiBoard board: the ServoPi board controlling the servo
        :param int channel: the channel (1 to 16) to which the servo is connected
        :param StopSpecification stop_min: specifications of the min stop position (position with the shorter pulse),
        if not the default one
        :param StopSpecification stop_max: specifications of the max stop position, if not the default one
        :raise: ValueError if a parameter value is not valid
        :raise: TypeError if the stop definitions are not of the expected type
        """
        if not board:
            raise ValueError('board parameter is mandatory')
        if not 1 <= channel <= 16:
            raise ValueError('channel must be in [1-16]')
        if not (isinstance(stop_min, StopSpecification) and isinstance(stop_max, StopSpecification)):
            raise TypeError('invalid stop definition(s) type')
        if stop_max <= stop_min:
            raise ValueError('stop definitions are reversed')

        self._board = board
        channel -= 1
        self._regs = [r + 4 * channel for r in ServoPiBoard.LED0_x]
        self._stop_min = stop_min
        self._stop_max = stop_max
        self._median = (stop_min + stop_max).scale(0.5)
        self._span = stop_max - stop_min
        self._pos_to_ms = float(self._span.msecs) / float(self._span.logical_position)
        self._position_to_msecs = self._position_to_msec_direct if self._pos_to_ms > 0 else self._position_to_msec_inverted
        self._current_position = None

    def __str__(self):
        return "{min=%s max=%s}" % (self._stop_min, self._stop_max)

    __repr__ = __str__

    def _position_to_msec_direct(self, position):
        position = min(max(position, self._stop_min.logical_position), self._stop_max.logical_position)
        return self._stop_min.msecs + self._pos_to_ms * (position - self._stop_min.logical_position), position

    def _position_to_msec_inverted(self, position):
        position = min(max(position, self._stop_max.logical_position), self._stop_min.logical_position)
        return self._stop_max.msecs + self._pos_to_ms * (position - self._stop_max.logical_position), position

    def set_position(self, position, force=False):
        """ Moves the servo to the given position.

        The position is expressed in application logical unit. No move is done if the
        requested position is the same as the current one, except if the `force` parameter
        is set to True.

        :param float position: the servo position (see :py:meth:`Servo.__init__` documentation for
        possible values
        :param bool force: if True, the move process will be engaged, whatever is the current position
        """
        if not force and position == self._current_position:
            return

        ms, real_position = self._position_to_msecs(position)
        raw = self._board.ms_to_reg(ms)
        values = (0, 0, raw & 0xff, raw >> 8)
        for r, v in zip(self._regs, values):
            self._board.write_reg(r, v)
        self._current_position = real_position

    @property
    def current_position(self):
        """ The current position set point. Be aware that it can be different from the real
        position, either because it didn't reach it already, or cannot do it.
        """
        return self._current_position

    def goto_minimum_position(self, force=False):
        """ Moves the servo to its minimum position as defined by `stop_min` constructor parameter.

        :param bool force: see :pyh:meth:`Servo.set_position`
        """
        self.set_position(self._stop_min.logical_position, force)

    def goto_maximum_position(self, force=False):
        """ Moves the servo to its maximum position as defined by `stop_max` constructor parameter.

        :param bool force: see :pyh:meth:`Servo.set_position`
        """
        self.set_position(self._stop_max.logical_position, force)

    def goto_median_position(self, force=False):
        """ Moves the servo half way between its minimum and maximum positions.

        :param bool force: see :pyh:meth:`Servo.set_position`
        """
        self.set_position(self._median.logical_position, force)

    def relative_move(self, d_position, force=False):
        """ Moves the servo by a relative amount from its current set point.

        This method requires an absolute position being previously set.

        :param float d_position: the relative move, provided in application logical unit
        :param bool force: see :pyh:meth:`Servo.set_position`
        """
        if not d_position:
            return
        if self._current_position is None:
            raise Exception("current position not yet defined")
        self.set_position(self._current_position + d_position, force)

    def set_floating(self):
        """ Puts the servo in floating mode by deactivating pulses generation.
        """
        for r in self._regs:
            self._board.write_reg(r, 0)


class ServoPiBoard(object):
    """ Models the ServoPi board.

    In addition to global configuration and chip dialogs, it provides a factory method to
    obtain Servo instances attached to it. Returned instances are cached for optimizing
    identical requests.
    """

    MODE1 = 0x00
    MODE2 = 0x01
    SUBADR1 = 0x02
    SUBADR2 = 0x03
    SUBADR3 = 0x04
    ALLCALLADR = 0x05
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    ALL_LED_ON_L = 0xFA
    ALL_LED_ON_H = 0xFB
    ALL_LED_OFF_L = 0xFC
    ALL_LED_OFF_H = 0xFD
    PRE_SCALE = 0xFE

    LED0_x = (LED0_ON_L, LED0_ON_H, LED0_OFF_L, LED0_OFF_H)

    _ENABLE_GPIO = 7

    DEFAULT_ADDRESS = 0x40
    DEFAULT_PWM_FREQ = 60

    _ms_to_reg = None

    def __init__(self, bus, i2c_addr=DEFAULT_ADDRESS, pwm_freq=DEFAULT_PWM_FREQ, use_output_enable=False):
        """
        :param bus: the I2C/SMBus the board is connected to
        :param int i2c_addr: the board I2C address
        :param int pwm_freq: the PWM frequency
        :param bool use_output_enable: set to True if we want to use the output enable signal of the
          PCA9685 chip. Remember to short the OE pads on the board in this case.
        """
        self._bus = bus
        self._i2c_addr = i2c_addr
        self._servos = {}

        self.write_reg(self.MODE1, 0)

        # optimize to GPIO stuff loading depending on the fact we really need it or not
        if use_output_enable:
            from pybot.gpio import GPIO
            self._GPIO = GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self._ENABLE_GPIO, GPIO.OUT)
            self.enable_all()
        else:
            self._GPIO = None

        self.set_pwm_freq(pwm_freq)

    def shutdown(self):
        """ Puts all servos in floating mode and disables PCA9685 outputs.
        """
        for servo in self._servos.values():
            servo.set_floating()
        self.disable_all()

    def set_pwm_freq(self, hz):
        """ Configures the PWM frequency
        :param int hz: frequency
        """
        scale_value = 25000000.0    # 25MHz
        scale_value /= 4096.0       # 12-bit
        scale_value /= float(hz)
        scale_value -= 1.0

        self._ms_to_reg = 4.096 * hz

        pre_scale = math.floor(scale_value + 0.5)
        old_mode = self.read_reg(self.MODE1)
        new_mode = (old_mode & 0x7F) | 0x10

        self.write_reg(self.MODE1, new_mode)
        self.write_reg(self.PRE_SCALE, int(math.floor(pre_scale)))
        self.write_reg(self.MODE1, old_mode)
        time.sleep(0.005)
        self.write_reg(self.MODE1, old_mode | 0x80)

    def enable_all(self):
        """ Enables all the chip outputs
        """
        if self._GPIO:
            self._GPIO.output(self._ENABLE_GPIO, 0)

    def disable_all(self):
        """ Disables all the chip outputs
        """
        if self._GPIO:
            self._GPIO.output(self._ENABLE_GPIO, 1)

    def write_reg(self, reg, value):
        """ Chip register setter
        """
        self._bus.write_byte_data(self._i2c_addr, reg, value)

    def read_reg(self, reg):
        """ Chip register getter
        """
        return self._bus.read_byte_data(self._i2c_addr, reg) & 0xff

    def ms_to_reg(self, ms):
        """ Convenience method for converting a duration to the corresponding register encoding value.
        :param float ms: duration to convert
        :return: corresponding register encoded value
        :rtype: int
        """
        return int(ms * self._ms_to_reg)

    def get_servo(self, channel, stop_min=Servo.DEFAULT_STOP_MIN, stop_max=Servo.DEFAULT_STOP_MAX):
        """ Factory method returning an instance of the class :py:meth:`Servo` with the given
        configuration.

        Instances are cached for optimization.
        :param int channel: channel number (1-16) to which the servo is connected
        :param StopSpecification stop_min: see :py:class:`Servo` constructor
        :param StopSpecification stop_max: see :py:class:`Servo` constructor
        :return: a Servo instance
        :raise: ValueError is invalid channel number
        """
        if not 1 <= channel <= 16:
            raise ValueError("invalid channel num (%d)" % channel)
        try:
            return self._servos[channel]
        except KeyError:
            servo = Servo(self, channel, stop_min=stop_min, stop_max=stop_max)
            self._servos[channel] = servo
            return servo
