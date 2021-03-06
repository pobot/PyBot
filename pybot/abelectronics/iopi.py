#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A set of simple classes for interacting with the IOPi board from AB Electronics
(https://www.abelectronics.co.uk/products/3/Raspberry-Pi/18/IO-Pi).

At the end of the chain is the individual IO, provided by the classes :py:class:`DigitalInput`
and :py:class:`DigitalOutput` (they will ease the transition for Arduino fans ;). IO instances
are attached to a :py:class:`Port`, modeling a real MCP23017 port. Ports are themselves attached to a
:py:class:`MCP23017`, which is itself attached to a :py:class:`IOPiBoard`.

All these intricacies such as bit masking on register values and alike are hidden from the user,
and a lot of things are optimized by caching instances and data for avoiding paying too high a price
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

    EXP1_DEFAULT_ADDRESS = 0x20
    EXP2_DEFAULT_ADDRESS = 0x21

    def __init__(self, bus, exp1_addr=EXP1_DEFAULT_ADDRESS, exp2_addr=EXP2_DEFAULT_ADDRESS):
        """ An instance of the I2C/SMBus class must be provided here. The calls used in the
        classes of this module are based on the smbus.SMBus interface. Any implementation providing
        the same API can thus be used, including fake ones for testing.

        :param bus: the I2C/SMBus instance
        :param int exp1_addr: I2C address of expander 1
        :param int exp2_addr: I2C address of expander 1
        """
        self._bus = bus
        self.expanders = (
            Expander(bus, exp1_addr),
            Expander(bus, exp2_addr),
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
            port = self.expanders[expander_num].ports[port_num]

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
        :param pullup_enabled: should the internal pull-up be enabled or not
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

    def read(self):
        """ Reads all ports of all expanders and returns their values as a single 32 bits integer.

        The returned integer is built as follows:
            MSB1 = expander_2.port_B
            LSB1 = expander_2.port_A
            MSB0 = expander_1.port_B
            LSB0 = expander_1.port_A

        :return: all board ports content
        """
        return ((self.expanders[self.EXPANDER_2].read() << 16) | self.expanders[self.EXPANDER_1].read()) & 0xffffffff

    def reset(self):
        """ Resets both expanders of the board
        """
        for expander in self.expanders:
            expander.reset()

    @staticmethod
    def _board_io_num_to_port_io(board_io_num):
        board_io_num -= 1
        return board_io_num / 8, board_io_num % 8


class Expander(object):
    """ Models the MCP23017 expander chip, and handles its low level operations.

    This class aggregates the two ports included in the chip.
    """
    PORT_A = 0
    PORT_B = 1

    # the register addressing scheme used here supposes that IOCON.BANK is set to 0 (default value)
    # and thus that port registers are sequential, so that address(xxxB) == address(xxxA + 1)
    IODIR = 0x00
    IPOL = 0x02
    GPINTEN = 0x04
    DEFVAL = 0x06
    INTCON = 0x08
    IOCON = 0x0A
    GPPU = 0x0C
    INTF = 0x0E
    INTCAP = 0x10
    GPIO = 0x12
    OLAT = 0x14

    # IOCON register flag masks
    IOCON_INTPOL = 0x02
    IOCON_ODR = 0x04
    IOCON_HAEN = 0x08
    IOCON_DISSLW = 0x10
    IOCON_SEQOP = 0x20
    IOCON_MIRROR = 0x40
    IOCON_BANK = 0x80

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

        :param int reg: register address
        :param int data: register value
        :return: the written data
        :rtype: int
        """
        data &= 0xff
        self._bus.write_byte_data(self._addr, reg, data)
        return data

    def read(self):
        """ Reads both expander ports and return their values as a 16 bits integer.

        :return: 2 bytes integer with PORTB and PORTA values as respectively MSB and LSB
        :rtype: int
        """
        return ((self.ports[Expander.PORT_B].read() << 8) | self.ports[Expander.PORT_A].read()) & 0xffff

    def reset(self):
        """ Resets both ports
        """
        for port in self.ports:
            port.reset()


class Port(object):
    """ Model of an IO port of an expander.

    Implements a caching mechanism for non volatile registers, so that modification
    operations are optimized by removing the need for physically reading the registers
    content.
    """
    # sets the cache in unset state
    _IODIR_cache = None
    _GPPU_cache = None
    _IPOL_cache = None
    _IOCON_cache = None
    _GPINTEN_cache = None
    _INTCON_cache = None
    _DEFVAL_cache = None

    def __init__(self, expander, port_num):
        """
        :param Expander expander: Expander instance this port belongs to
        :param int port_num: the port num (Expander.PORT_A or Expander.PORT_B).
        """
        if port_num not in (Expander.PORT_A, Expander.PORT_B):
            raise ValueError("invalid port num (%d)" % port_num)
        self._expander = expander
        self._port_num = port_num
        # initializes the registers cache
        self.update_cache()

    def update_cache(self):
        """ Updates the registers cache. """
        self._IODIR_cache = self._expander.read_register(Expander.IODIR + self._port_num)
        self._GPPU_cache = self._expander.read_register(Expander.GPPU + self._port_num)
        self._IPOL_cache = self._expander.read_register(Expander.IPOL + self._port_num)
        self._IOCON_cache = self._expander.read_register(Expander.IOCON + self._port_num)
        self._GPINTEN_cache = self._expander.read_register(Expander.GPINTEN + self._port_num)
        self._DEFVAL_cache = self._expander.read_register(Expander.DEFVAL + self._port_num)
        self._INTCON_cache = self._expander.read_register(Expander.INTCON + self._port_num)

    @staticmethod
    def _change_bit(bit_num, value, byte):
        return byte | (1 << bit_num) if value else byte & ~ (1 << bit_num)

    @staticmethod
    def _change_bit_with_mask(bit_mask, value, byte):
        return byte | bit_mask if value else byte & ~ bit_mask

    @staticmethod
    def _test_bit(bit_num, byte):
        return (byte & (1 << bit_num)) != 0

    @staticmethod
    def _test_bit_with_mask(bit_mask, byte):
        return (byte & bit_mask) != 0

    @staticmethod
    def _check_io_num(io_num):
        if not 0 <= io_num < 8:
            raise ValueError('invalid IO num (%d)' % io_num)

    @property
    def io_directions(self):
        """ Returns the current settings of the port IO directions. """
        return self._IODIR_cache

    @io_directions.setter
    def io_directions(self, dirs):
        """ Sets the port IO directions configuration.
        :param int dirs: a byte containing the IO direction flags for all the IO of the port
        """
        self._IODIR_cache = self._expander.write_register(Expander.IODIR + self._port_num, dirs)

    def set_io_direction(self, io_num, direction):
        """ Sets the direction of a single IO
        :param int io_num: the IO num ([0-7])
        :param int direction: IO.DIR_INPUT or IO.DIR_INPUT
        :raise: ValueError if out of range io_num
        """
        self._check_io_num(io_num)
        self.io_directions = self._change_bit(io_num, direction == IO.DIR_INPUT, self._GPPU_cache)

    @property
    def pullups_enabled(self):
        """ Returns the current settings of the port inputs pullups. """
        return self._GPPU_cache

    @pullups_enabled.setter
    def pullups_enabled(self, settings):
        """ Configures the port inputs pullups. """
        self._GPPU_cache = self._expander.write_register(Expander.GPPU + self._port_num, settings)

    def enable_pullup(self, io_num, enabled):
        """ Configures a single input pullup.
        :param int io_num: the IO num ([0-7])
        :param bool enabled: is the pullup enabled ?
        :raise: ValueError if out of range io_num
        """
        self._check_io_num(io_num)
        self.pullups_enabled = self._change_bit(io_num, enabled, self._GPPU_cache)

    @property
    def inputs_inverted(self):
        """ Returns the current settings of the port inputs polarity inversion. """
        return self._IPOL_cache

    @inputs_inverted.setter
    def inputs_inverted(self, settings):
        """ Configures the port inputs polarity inversion. """
        self._IPOL_cache = self._expander.write_register(Expander.IPOL + self._port_num, settings)

    def invert_input(self, io_num, inverted):
        """ Configures the inversion of a given input.
        :param int io_num: the IO num ([0-7])
        :param bool inverted: is the input inverted ?
        :raise: ValueError if out of range io_num
        """
        self._check_io_num(io_num)
        self.interrupts_enabled = self._change_bit(io_num, inverted, self._IPOL_cache)

    @property
    def interrupts_enabled(self):
        """ Returns the current settings of the port inputs interrupts enabling. """
        return self._GPINTEN_cache

    @interrupts_enabled.setter
    def interrupts_enabled(self, settings):
        """ Configures the port inputs interrupts enabling. """
        self._GPINTEN_cache = self._expander.write_register(Expander.GPINTEN + self._port_num, settings)

    def enable_interrupt(self, io_num, enabled):
        """ Enables interrupts for a given input.
        :param int io_num: the IO num ([0-7])
        :param bool enabled: is interrupt enabled ?
        :raise: ValueError if out of range io_num
        """
        self._check_io_num(io_num)
        self.interrupts_enabled = self._change_bit(io_num, enabled, self._GPINTEN_cache)

    @property
    def interrupt_sources(self):
        """ Returns the current settings of the port interrupt compare sources. """
        return self._INTCON_cache

    @interrupt_sources.setter
    def interrupt_sources(self, settings):
        """ Configures the port interrupt compare sources. """
        self._INTCON_cache = self._expander.write_register(Expander.INTCON + self._port_num, settings)

    def set_interrupt_source(self, io_num, source):
        """ Sets the compare source for input interrupts for a given input.
        :param int io_num: the IO num ([0-7])
        :param int source: IO.INT_COMPARE or IO.INT_CHANGE
        :raise: ValueError if out of range io_num
        """
        self._check_io_num(io_num)
        self.interrupt_sources = self._change_bit(io_num, source, self._INTCON_cache)

    @property
    def default_values(self):
        """ Returns the current settings of the port interrupt default values. """
        return self._DEFVAL_cache

    @default_values.setter
    def default_values(self, settings):
        """ Configures the port interrupt default values. """
        self._DEFVAL_cache = self._expander.write_register(Expander.DEFVAL + self._port_num, settings)

    def set_default_value(self, io_num, value):
        """ Sets the input change default value for a given input.
        :param int io_num: the IO num ([0-7])
        :param int value: default value (0 or 1)
        :raise: ValueError if out of range io_num
        """
        self._check_io_num(io_num)
        self.default_values = self._change_bit(io_num, value, self._DEFVAL_cache)

    @property
    def configuration(self):
        return self._IOCON_cache

    @configuration.setter
    def configuration(self, value):
        self._IOCON_cache = self._expander.write_register(self._IOCON_cache + self._port_num, value)

    def write(self, value):
        """ Write a value to the port
        :param int value: the value to be written
        :return: the clamped value (see :py:meth:`Expander.write_register`)
        :rtype: int
        """
        return self._expander.write_register(Expander.GPIO + self._port_num, value)

    def read(self):
        """ Reads the port.
        :return: the port content
        """
        return self._expander.read_register(Expander.GPIO + self._port_num) & 0xff

    def reset(self):
        """ Puts the port in default POR state
        """
        self.interrupts_enabled = 0
        self.pullups_enabled = 0
        self.io_directions = 0xff

    def dump(self):
        """ Internal method for debugging."""
        for k, v in self.__dict__.iteritems():
            print("%20s : %s" % (k, v))


class IO(object):
    """ Root class for modeling a single IO or a port.
    """
    DIR_OUTPUT = 0
    DIR_INPUT = 1
    INT_CHANGE = 0
    INT_COMPARE = 1

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

    @property
    def port(self):
        """ The port this IO belongs to.
        """
        return self._port

    @property
    def mask(self):
        """ The bit mask of this IO.

        It can be useful for manipulating IOs with port single read/write
        for optimizing access.
        """
        return self._mask


class _ReadableIOMixin(object):
    """ A mixin gathering read operations applicable to IOs.

     It can be used equally for inputs and outputs, and will read the latches
     for the later.
    """
    _port = None
    _mask = None

    def read(self):
        """ Returns the bit value (0 or 1) of the IO. """
        return 1 if self._port.read() & self._mask else 0

    def is_set(self):
        """ Returns True if the IO is high. """
        return self.read()

    def is_clear(self):
        """ Returns True if the IO is low. """
        return not self.read()


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
