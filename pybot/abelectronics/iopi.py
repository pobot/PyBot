#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A set of simple classes for interacting with the IOPi board from AB Electronics
(https://www.abelectronics.co.uk/products/3/Raspberry-Pi/18/IO-Pi).

At the end of the chain is the individual IO, provided by the classes :py:class:`DigitalInput`
and :py:class:`DigitalOutput` (they will ease the transition for Arduino fans ;). IO instances
are attached to a :py:class:`Port`, modeling a real MCP23017 port. Ports are themselves attached to a
:py:class:`MCP23017`, which is itself attached to a :py:class:`IOPiBoard`.

All these intricacies such as bit masking on register values and alike are hidden from the user,
and a lot of things are optimized by caching instances and data for avoiding paying a too high price
for a high level design. Even if the added overhead could seem penalizing in term of performances,
thinks are not that obvious, since all the processing done here must be done somewhere anyway. There
are thus chances that the effective penalty (if ever any) will be negligible for most of the
applications.

To allow several classes instances being used for modeling a configuration with multiple boards,
possibly of different types, the underlying bus must be created outside and provided as a dependency
(see documentation about de "Dependency Injection" design pattern). Being able to provide whatever
implementation of the basic read and write operations allows the use of a fake class simulating
real operations, which made possible to test the rest of the application on a system not having
a SMBus resource available, such as the development station PC for instance, or providing access to
an I2C bus using an USB adapter for instance. For examples of this, have a look at our i2c module
(https://github.com/Pobot/PyBot/blob/master/pybot/i2c.py). We have included there an enhanced smbus
class (:py:class:`pybot.i2c.SMBusI2CBus`), taking care of serializing I/O operations, in case of
multi-threaded usage.

Here is an example of the code used for controlling a LED connected to pin 3 of IC_1 expander
header. We suppose that the LED is wired in positive logic, i.e. lightened when the IO is high.

>>> from pybot.raspi import i2c_bus
>>> from pybot.abelectronics.iopi import IOPiBoard
>>> ...
>>> # create an instance of the board
>>> board = IOPiBoard(i2c_bus)
>>> # define an output IO corresponding to the one connected to the LED
>>> led = board.get_digital_output(board.EXPANDER_1, 3)
>>> ...
>>> # switch the LED on by setting the IO in high state
>>> led.set()
>>> ...
>>> # switch the LED off by setting the IO in low state
>>> led.clear()

Here is another example for reading an input, f.i. a switch connected on pin 11.

>>> board = IOPiBoard(i2c_bus)
>>> switch = board.get_digital_input(board.EXPANDER_1, 11, pullup_enabled=True
>>> ...
>>> # read the state of the switch
>>> if switch.is_set():
>>> ...

Pretty simple, no ?

Note that the current version includes only basic functions, and still lacks interrupts
configuration stuff. This will come ASAP, but feel free to contribute on this topic if you need this now.
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
        """ An instance of the I2C/SMBus class must be provided here. The calls used in the
        classes of this module are based on the smbus.SMBus interface. Any implementation providing
        the same API can thus be used, including fake ones for testing.

        :param bus: the I2C/SMBus instance
        :param int exp1_addr: I2C address of expander 1
        :param int exp2_addr: I2C address of expander 1
        """
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

        key = (expander_num << 8) + board_io_num
        try:
            # try first to get the object from the cache
            io = self._ios[key]
        except KeyError:
            # not yet created => do it
            port_num, io_num = self._board_io_num_to_port_io(board_io_num)
            port = self._expanders[expander_num].ports[port_num]

            # create the instance of the appropriate class, depending on the IO type
            if direction == IO.DIR_INPUT:
                io = DigitalInput(port, io_num, pullup_enabled=pullup_enabled)
            else:
                io = DigitalOutput(port, io_num)
            # cache the result
            self._ios[key] = io
        return io

    def get_digital_input(self, expander_num, board_io_num, pullup_enabled=False):
        """ Factory method returning a DigitalInput instance for a given IO, and configures
        it as requested.
        :param int expander_num: IOPiBoard.EXPANDER_1 or IOPiBoard.EXPANDER_2
        :param int board_io_num: the pin number of the IO on the expander header
        :param pullup_enabled: should the internal pullup be enabled or not
        :return: the IO object
        :rtype: DigitalInput
        """
        return self._get_io(expander_num, board_io_num, direction=IO.DIR_INPUT, pullup_enabled=pullup_enabled)

    def get_digital_output(self, expander_num, board_io_num):
        """ Factory method returning a DigitalOutput instance for a given IO.
        :param int expander_num: IOPiBoard.EXPANDER_1 or IOPiBoard.EXPANDER_2
        :param int board_io_num: the pin number of the IO on the expander header
        :return: the IO object
        :rtype: DigitalOutput
        """
        return self._get_io(expander_num, board_io_num, direction=IO.DIR_OUTPUT)


    @staticmethod
    def _board_io_num_to_port_io(board_io_num):
        board_io_num -= 1
        return board_io_num / 8, board_io_num % 8


class MCP23017(object):
    """ Models the MCP23017 expander chip, and handles its low level operations.

    This class aggregates the two ports included in the chip.
    """
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
        """
        :param bus: the I2C/SMBus instance
        :param int i2c_addr: expander I2C address
        """
        self._bus = bus
        self._addr = i2c_addr

        self.ports = (
            Port(self, self.PORT_A),
            Port(self, self.PORT_B)
        )

    def read_register(self, addr):
        """ Reads a chip register.
        :param int addr: register address
        :return: register content
        :rtype: int
        """
        return self._bus.read_byte_data(self._addr, addr) & 0xff

    def write_register(self, reg, data):
        """ Writes a chip register.

        Since the method takes care of clamping the passed value to a byte
        extent, it also returns the clamped value for convenience.

        :param int addr: register address
        :param int data: register value
        :return: the written data
        :rtype: int
        """
        data &= 0xff
        self._bus.write_byte_data(self._addr, reg, data)
        return data


class Port(object):
    """ Model of an IO port of an expander.

    Implements a caching mechanism for non volatile registers, so that modification
    operations are optimized by removing the need for physically reading the registers
    content.
    """
    _IODIR_cache = None
    _GPPU_cache = None

    def __init__(self, expander, port_num):
        """
        :param MCP23017 expander: MCP23017 instance this port belongs to
        :param int port_num: the port num (MCP23017.PORT_A or MCP23017.PORT_B).
        """
        if port_num not in (MCP23017.PORT_A, MCP23017.PORT_B):
            raise ValueError("invalid port num (%d)" % port_num)
        self._expander = expander
        self._port_num = port_num
        # initializes the registers cache
        self.update_cache()

    def update_cache(self):
        """ Updates the registers cache. """
        self._IODIR_cache = self._expander.read_register(MCP23017.IODIR + self._port_num)
        self._GPPU_cache = self._expander.read_register(MCP23017.GPPU + self._port_num)

    @property
    def io_directions(self):
        """ Returns the current settings of the port IO directions. """
        return self._IODIR_cache

    @io_directions.setter
    def io_directions(self, dirs):
        """ Sets the port IO directions configuration.
        :param int dirs: a byte containing the IO direction flags for all the IO of the port
        """
        self._IODIR_cache = self._expander.write_register(MCP23017.IODIR + self._port_num, dirs)

    def set_io_direction(self, io_num, direction):
        """ Sets the direction of a single IO
        :param int io_num: the IO num ([0-7])
        :param int direction: IO.DIR_INPUT or IO.DIR_INPUT
        """
        if direction == IO.DIR_INPUT:
            b = self._IODIR_cache | (1 << io_num)
        else:
            b = self._IODIR_cache & ~ (1 << io_num)
        self._expander.write_register(MCP23017.IODIR + self._port_num, b)
        self._IODIR_cache = b

    @property
    def pullups(self):
        """ Returns the current settings of the port inputs pullups. """
        return self._GPPU_cache

    @pullups.setter
    def pullups(self, states):
        """ Configures the port inputs pullups. """
        self._GPPU_cache = self._expander.write_register(MCP23017.GPPU + self._port_num, states)

    def enable_pullup(self, io_num, enabled):
        """ Configures a single input pullup.
        :param int io_num: the IO num ([0-7])
        :param bool enabled: is the pullup enabled ?
        """
        if enabled:
            b = self._GPPU_cache | (1 << io_num)
        else:
            b = self._GPPU_cache & ~ (1 << io_num)
        self._expander.write_register(MCP23017.GPPU + self._port_num, b)
        self._GPPU_cache = b

    def write(self, value):
        """ Write a value to the port
        :param int value: the value to be written
        :return: the clamped value (see :py:meth:`MCP23017.write_register`)
        :rtype: int
        """
        return self._expander.write_register(MCP23017.GPIO + self._port_num, value)

    def read(self):
        """ Reads the port.
        :return: the port content
        """
        return self._expander.read_register(MCP23017.GPIO + self._port_num)

    def dump(self):
        """ Internal method for debugging."""
        for k, v in self.__dict__.iteritems():
            print("%20s : %s" % (k, v))


class IO(object):
    """ Root class for modeling a single IO or a port.
    """
    DIR_OUTPUT = 0
    DIR_INPUT = 1

    def __init__(self, port, num, is_input):
        """
        :param Port port: the port the IO is attached to
        :param int num: IO num ([0-7])
        :param bool is_input: is an input or not ?
        """
        if not 0 <= num < 8:
            raise ValueError('invalid IO num (%d)' % num)

        self._port = port
        port.set_io_direction(num, IO.DIR_INPUT if is_input else IO.DIR_OUTPUT)
        self._mask = 1 << num


class _ReadableIOMixin(object):
    """ A mixin gathering read operations applicable to IOs.

     It can be used equally for inputs and outputs, and will read the latches
     for the later.
    """
    _port = None
    _mask = None

    def get(self):
        """ Returns the bit value (0 or 1) of the IO. """
        return 1 if self._port.read() & self._mask else 0

    def is_set(self):
        """ Returns True if the IO is high. """
        return self.get()

    def is_clear(self):
        """ Returns True if the IO is low. """
        return not self.get()


class DigitalInput(IO, _ReadableIOMixin):
    """ A specialized IO modeling an input."""
    def __init__(self, port, num, pullup_enabled=False):
        """
        :param Port port: the port this IO belongs to
        :param int num: the IO number ([0-7])
        :param bool pullup_enabled: should the pullup be enabled ?
        """
        super(DigitalInput, self).__init__(port, num, is_input=True)
        self._port.enable_pullup(num, pullup_enabled)


class DigitalOutput(IO, _ReadableIOMixin):
    """ A specialized IO modeling an output."""
    def __init__(self, port, num):
        """
        :param Port port: the port this IO belongs to
        :param int num: the IO number ([0-7])
        """
        super(DigitalOutput, self).__init__(port, num, is_input=False)

    def set(self):
        """ Turns the output high."""
        self._port.write(self._port.read() | self._mask)

    def clear(self):
        """ Turns the output low."""
        self._port.write(self._port.read() & (~ self._mask))
