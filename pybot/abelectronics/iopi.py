#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A set of simple classes for interacting with the IOPi board from AB Electronics
(https://www.abelectronics.co.uk/products/3/Raspberry-Pi/18/IO-Pi).

At the end of the chain is the individual IO, provided by the classes :py:class:`DigitalInput`
and :py:class:`DigitalOutput` (they will ease the transition for Arduino fans ;). IO instances
are attached to a :py:class:`Port`, modeling a real MCP23017 port. Ports are themselves attached to a
:py:class:`MCP23017`, which is itself attached to a :py:class:`IOPiBoard`.

All these intricacies are hidden from the user, and a lot of things are optimized by caching
instances and data for avoiding paying a too high price for a high level design.

To allow several classes instances being used for modeling a configuration with multiple boards,
possibly of different types, the underlying bus must be created outside and provided as a dependency
(see documentation about de "Dependency Injection" design pattern). Being able to provide whatever
implementation of the basic read and write operations allows the use of a mockup class simulating
real operations, which made possible to test the rest of the application on a system not having
a SMBus resource available, such as the development station PC for instance, or providing access to
an I2C bus using an USB adapter for instance. For examples of this, have a look at our i2c module
(https://github.com/Pobot/PyBot/blob/master/pybot/i2c.py).

Here is an example of the code used for controlling a LED connected to pin
3 of IC_1 expander header. We suppose that the LED
is wired in positive logic, i.e. lightened when the IO is high.

>>> # create an instance of the board, injecting into it the I2C/SMBus instance
>>> # created before, which can by the way be shared by instances of multiple boards
>>> # connected to the RasPi (be they IOpi or something else)
>>> board = IOPIBoard(bus)
>>> # define an output IO corresponding to the one connected to the LED
>>> led = board.get_digital_output(board.EXPANDER_1, 3)
>>> ...
>>> # switch the LED on by setting the IO in high state
>>> led.set()
>>> ...
>>> # switch the LED off by setting the IO in low state
>>> led.clear()

Here is another example for reading an input, f.i. a switch connected on pi 11.

>>> board = IOPIBoard(bus)
>>> switch = board._get_digital_input(board.EXPANDER_1, 11, pullup_enabled=True
>>> ...
>>> # read the state of the switch
>>> if switch.is_set():
>>> ...

Pretty simple, no ?

Note that the current version includes only basic functions, interrupts related configuration
being added when needed. Feel free to contribute on this topic if you need this now.
"""

__author__ = 'Eric PASCUAL for POBOT'
__version__ = '2.0.0'
__email__ = 'eric@pobot.org'


class IOPiBoard(object):
    """ This class represents a whole IOPi expansion board.

    Its main role is to act as a factory for individual IOs, so that the application
    will not deal with bit masking and other boring chores.


    It can be used directly to manipulate the registers of the 4 embedded I/O ports,
    but usually it's simpler to use instances of the Port class for this. This is
    fully equivalent, but user code is more readable this way.

    This costs a small overhead since the Port methods delegates to Board ones, taking
    care of passing them the additional parameters, but unless you have a critical
    performances problem, it should do the trick most of the time.
    """
    EXPANDER_1 = 0
    EXPANDER_2 = 1

    def __init__(self, bus, exp1_addr=0x20, exp2_addr=0x21):
        self._bus = bus
        self._expanders = (
            MCP23017(bus, exp1_addr),
            MCP23017(bus, exp2_addr),
        )
        self._ios = {}

    def _get_io(self, expander_num, board_io_num, direction, pullup_enabled=False):
        if expander_num not in (self.EXPANDER_1, self.EXPANDER_2):
            raise ValueError("invalid expander num (%d)" % expander_num)
        if not 1 <= board_io_num <= 16:
            raise ValueError("invalid IO num (%d)" % board_io_num)

        key = expander_num << 8 + board_io_num
        try:
            io = self._ios[key]
        except KeyError:
            port_num, io_num = self._board_io_num_to_port_io(board_io_num)
            port = self._expanders[expander_num].ports[port_num]
            if direction == IO.DIR_INPUT:
                io = DigitalInput(port, io_num, pullup_enabled=pullup_enabled)
            else:
                io = DigitalOutput(port, io_num)
            self._ios[key] = io
        return io

    def get_digital_input(self, expander_num, board_io_num, pullup_enabled=False):
        return self._get_io(expander_num, board_io_num, direction=IO.DIR_INPUT, pullup_enabled=pullup_enabled)

    def get_digital_output(self, expander_num, board_io_num):
        return self._get_io(expander_num, board_io_num, direction=IO.DIR_OUTPUT)


    @staticmethod
    def _board_io_num_to_port_io(board_io_num):
        board_io_num -= 1
        return board_io_num / 8, board_io_num % 8


class MCP23017(object):
    PORT_A = 0
    PORT_B = 1

    # the register addressing scheme used here supposes that IOCON.BANK is set to 0 (default value)
    # and thus that port registers are paired, so that address(xxxB) == address(xxxA + 1)
    IODIR = 0x00
    IPOL = 0x02
    GPINTEN = 0x04
    GPPU = 0x0C
    GPIO = 0x12
    OLAT = 0x14

    def __init__(self, bus, i2c_addr):
        self._bus = bus
        self._addr = i2c_addr

        self.ports = (
            Port(self, self.PORT_A),
            Port(self, self.PORT_B)
        )

    def read_register(self, addr):
        return self._bus.read_byte_data(self._addr, addr) & 0xff

    def write_register(self, reg, data):
        data &= 0xff
        self._bus.write_byte_data(self._addr, reg, data)
        return data


class Port(object):
    def __init__(self, expander, port_num):
        if port_num not in (MCP23017.PORT_A, MCP23017.PORT_B):
            raise ValueError("invalid port num (%d)" % port_num)
        self._expander = expander
        self._port_num = port_num
        self._IODIR_cache = self.get_io_directions()
        self._GPPU_cache = self.get_pullups_state()
        self._GPIO_cache = self.read()

    def set_io_directions(self, dirs):
        self._expander.write_register(MCP23017.IODIR + self._port_num, dirs)

    def get_io_directions(self):
        return self._expander.read_register(MCP23017.IODIR + self._port_num)

    def set_io_direction(self, io_num, direction):
        if direction == IO.DIR_INPUT:
            b = self._IODIR_cache | (1 << io_num)
        else:
            b = self._IODIR_cache & ~ (1 << io_num)
        self._expander.write_register(MCP23017.IODIR + self._port_num, b)
        self._IODIR_cache = b

    def set_pullups_state(self, states):
        self._expander.write_register(MCP23017.GPPU + self._port_num, states)

    def enable_pullup(self, io_num, enabled):
        if enabled:
            b = self._GPPU_cache | (1 << io_num)
        else:
            b = self._GPPU_cache & ~ (1 << io_num)
        self._expander.write_register(MCP23017.GPPU + self._port_num, b)
        self._GPPU_cache = b

    def get_pullups_state(self):
        return self._expander.read_register(MCP23017.GPPU + self._port_num)

    def write(self, value):
        self._GPIO_cache = self._expander.write_register(MCP23017.GPIO + self._port_num, value)
        return self._GPIO_cache

    def read(self):
        self._GPIO_cache = self._expander.read_register(MCP23017.GPIO + self._port_num)
        return self._GPIO_cache


class IO(object):
    DIR_OUTPUT = 0
    DIR_INPUT = 1

    def __init__(self, port, num, direction):
        if not 0 <= num < 8:
            raise ValueError('invalid IO num (%d)' % num)

        self._port = port
        port.set_io_direction(num, direction)
        self._mask = 1 << num


class _ReadableIOMixin(object):
    def get(self):
        return 1 if self._port.read() & self._mask else 0

    def is_set(self):
        return self.get()

    def is_clear(self):
        return not self.get()


class DigitalInput(IO, _ReadableIOMixin):
    def __init__(self, port, num, pullup_enabled=False):
        super(DigitalInput, self).__init__(port, num, IO.DIR_INPUT)
        self._port.enable_pullup(num, pullup_enabled)


class DigitalOutput(IO, _ReadableIOMixin):
    def __init__(self, port, num):
        super(DigitalOutput, self).__init__(port, num, IO.DIR_OUTPUT)

    def set(self):
        self._port.write(self._port.read() | self._mask)

    def clear(self):
        self._port.write(self._port.read() & (~ self._mask))
