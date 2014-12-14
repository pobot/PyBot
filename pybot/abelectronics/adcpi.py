#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A set of simple classes for interacting with the ADCPi board from AB Electronics
(https://www.abelectronics.co.uk/products/3/Raspberry-Pi/17/ADC-Pi-V2---Raspberry-Pi-Analogue-to-Digital-converter).

At the end of the chain is the individual input, provided by the class :py:class:`AnalogInput`
(this will ease the transition for Arduino fans ;). ADC input instances
are attached to a :py:class:`ADCPiBoard`, modeling the whole board and managing the common tasks.

Details about chip configuration are hidden from the user, and a lot of things are optimized by caching
what can be cached for avoiding paying too high a price for a high level design. Even if the added overhead
could seem penalizing in term of performances, this is not that obvious, since all the processing done here
must be done somewhere anyway. There are thus chances that the effective penalty (if ever any) will be
negligible for most of the applications.

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
>>> board = ADCPiBoard(i2c_bus)
>>> # define an input, connected to pin 3 on the board header
>>> ldr_voltage = board.get_analog_input(3, rate=..., gain=...)
>>> ...
>>> # read it
>>> v = ldr_voltage.read()

Pretty simple, no ?

"""

__author__ = 'Eric PASCUAL for POBOT'
__version__ = '2.0.0'
__email__ = 'eric@pobot.org'


class ADCPiBoard(object):
    """ This class models an ADCPi board.

    It also acts as a factory to provide instances of individual ADC inputs.
    """

    # sample rates
    RATE_12, RATE_14, RATE_16, RATE_18 = range(4)
    # gains
    GAIN_x1, GAIN_x2, GAIN_x4, GAIN_x8 = range(4)
    # resolutions (bits)
    RESOLUTIONS = (12, 14, 16, 18)

    CONV1_DEFAULT_ADDRESS = 0x68
    CONV2_DEFAULT_ADDRESS = 0x68

    def __init__(self, bus, conv1_addr=CONV1_DEFAULT_ADDRESS , conv2_addr=CONV2_DEFAULT_ADDRESS):
        """ An instance of the I2C/SMBus class must be provided here. The calls used in the
        classes of this module are based on the smbus.SMBus interface. Any implementation providing
        the same API can thus be used, including fake ones for testing.

        :param bus: the I2C/SMBus instance
        :param int conv1_addr: I2C address of converter 1
        :param int conv2_addr: I2C address of converter 1
        """
        self._bus = bus
        self._converters = (
            Converter(bus, conv1_addr),
            Converter(bus, conv2_addr),
        )
        self._adcs = {}

    def get_analog_input(self, board_input_num, rate=RATE_12, gain=GAIN_x1):
        """ Convenience factory method returning an instance of :py:class:`AnalogInput`
        representing an individual input.

        Returned instances are cached, so that requesting an already requested input
        returns the existing one and does not created a new one.

        :param int board_input_num: the input number (in [1-8])
        :param int rate: the sampling rate selector (ADCPiBoard.RATE_xx)
        :param int gain: the input amplifier gain selector (ADCPiBoard.GAIN_xn)
        :return: an instance of AnalogInput
        :rtype: AnalogInput
        """
        if not 1 <= board_input_num <= 8:
            raise ValueError("invalid input num (%d)" % board_input_num)
        try:
            # try first to get the object from the cache
            adc = self._adcs[board_input_num]

        except KeyError:
            # not yet created => do it
            conv_num, channel_num = self._board_input_num_to_conv_channel(board_input_num)

            converter = self._converters[conv_num]

            # create the instance of the ADC class
            adc = AnalogInput(converter, channel_num, rate, gain)
            # cache the result
            self._adcs[board_input_num] = adc
        return adc

    @staticmethod
    def _board_input_num_to_conv_channel(board_io_num):
        board_io_num -= 1
        return board_io_num / 4, board_io_num % 4


class Converter(object):
    """ Models the MCP3424 converter chip, and handles its low level operations.
    """
    def __init__(self, bus, i2c_addr):
        """
        :param bus: the I2C/SMBus instance
        :param int i2c_addr: expander I2C address
        """
        self._bus = bus
        self._addr = i2c_addr

    def read_raw(self, config, count=32):
        """ Performs a raw read

        We can request just the number of needed bytes, to avoid transferring
        the SMBus default 32 bytes chunk.

        :param int config: the configuration byte
        :param int count: number of requested bytes
        :return: the data bytes as returned by the chip
        :rtype: list
        """
        return self._bus.read_i2c_block_data(self._addr, config, count)


class AnalogInput(object):
    """ Models an ADC input.

    Once instantiated, signal values are obtained using methods :py:meth:`read_voltage` and
    :py:meth:`read_raw`.

    The input is configured (sampling rate and resolution, PGA gain) at instantiation time,
    and cannot be changed after.
    """

    # configuration register masks
    NOT_READY = 0x80
    CONTINUOUS_CONVERSION = 0x10

    _gain_factors = (0.5, 1.0, 2.0, 4.0)
    _lsb_factors = (0.0005, 0.000125, 0.00003125, 0.0000078125)

    def _decoder_12(self, raw):
        h, m = raw[:2]
        return 0 if h & 0x08 else ((h & 0x07) << 8) | m

    def _decoder_14(self, raw):
        h, m = raw[:2]
        return 0 if h & 0x20 else ((h & 0x1f) << 8) | m

    def _decoder_16(self, raw):
        h, m = raw[:2]
        return 0 if h & 0x80 else ((h & 0x7f) << 8) | m

    def _decoder_18(self, raw):
        h, m, l = raw[:3]
        return 0 if h & 0x02 else ((h & 0x01) << 16) | (m << 8) | l

    # for each sampling rate : (decoding_method, reply length)
    _decoding_specs = {
        ADCPiBoard.RATE_12: (_decoder_12, 3),
        ADCPiBoard.RATE_14: (_decoder_14, 3),
        ADCPiBoard.RATE_16: (_decoder_16, 3),
        ADCPiBoard.RATE_18: (_decoder_18, 4)
    }

    def __init__(self, converter, channel_num, rate=ADCPiBoard.RATE_12, gain=ADCPiBoard.GAIN_x1, single=False):
        """
        :param Converter converter: the MCP chip this input belongs to
        :param int channel_num: ADC channel num ([0-3]
        :param bool single: True for single sample mode
        :param int rate: sample rate and resolution selector (RATE_nn)
        :param int gain: PGA gain (GAIN_xn)
        """
        if not converter:
            raise ValueError('converter parameter is mandatory')
        if not 0 <= channel_num <= 3:
            raise ValueError('invalid channel number (%s)' % channel_num)
        if not ADCPiBoard.RATE_12 <= rate <= ADCPiBoard.RATE_18:
            raise ValueError('invalid resolution (%s)' % rate)
        if not ADCPiBoard.GAIN_x1 <= gain <= ADCPiBoard.GAIN_x8:
            raise ValueError('invalid gain (%s)' % gain)

        self._converter = converter
        self._channel_num = channel_num
        self._config = (
            (channel_num << 5) |
            (self.CONTINUOUS_CONVERSION if not single else 0) |
            rate | gain
        ) & 0xff
        self._decoder, self._reply_len = self._decoding_specs[rate]

        # Note: the last factor of the formula hereafter has been found by experimental measurement
        self._scale_factor = self._lsb_factors[rate] / self._gain_factors[gain] * 2.478439425

    def read_voltage(self):
        """ Samples the input and converts the raw reading to corresponding voltage.
        :return: the input voltage
        :rtype: float
        """
        return self.read_raw() * self._scale_factor

    def convert_raw(self, raw):
        return raw * self._scale_factor

    def read_raw(self):
        """ Samples the input and returns its raw value.

        Proper decoding is applied, based on configured gain and sample rate.
        :return: the input raw value
        :rtype: int
        """
        while True:
            raw = self._converter.read_raw(self._config, self._reply_len)
            cfg = raw[-1]
            if not(cfg & self.NOT_READY):
                return self._decoder(self, raw)
