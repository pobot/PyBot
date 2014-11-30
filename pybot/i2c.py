#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
A couple of classes for interacting with devices accessible on I2C buses.
"""

__author__ = "Eric Pascual (eric@pobot.org)"

import threading


class I2CBus(object):
    """
    Abstract root class for I2C interface implementations
    """

    def __init__(self, debug=False, simulate=False, msb_first=False):
        self.msb_first = msb_first
        self._debug = debug
        self._simulate = simulate

    def read_byte(self, addr):
        """ Read a single byte from a device.
        :param int addr: the device address
        :return: the byte value
        :rtype: int
        """
        raise NotImplementedError()

    def write_byte(self, addr, data):
        """ Write a single byte to a device.
        :param int addr: the device address
        :param int data: the byte value
        """
        raise NotImplementedError()

    def read_byte_data(self, addr, reg):
        """ Read the content of a byte size register of a device
        :param int addr: device address
        :param int reg: register address on the device
        :return: the register content
        :rtype: int
        """
        raise NotImplementedError()

    def write_byte_data(self, addr, reg, data):
        """ Set the content of a byte size register of a device
        :param int addr: device address
        :param int reg: register address on the device
        :param int data: the register content
        """
        raise NotImplementedError()

    def read_word_data(self, addr, reg):
        """ Read the content of a word size register of a device
        :param int addr: device address
        :param int reg: register address on the device
        :return: the register content
        :rtype: int
        """
        raise NotImplementedError()

    def write_word_data(self, addr, reg, data):
        """ Set the content of a word size register of a device
        :param int addr: device address
        :param int reg: register address on the device
        :param int data: the register content
        """
        raise NotImplementedError()

    def read_block_data(self, addr, reg):
        """ Read a block of bytes, starting at a given register of the device.

        The count of bytes of received data is defined by the device.
        :param int addr: device address
        :param int reg: register address on the device
        :return: the data read
        :rtype: array of [int]
        """
        raise NotImplementedError()

    def write_block_data(self, addr, reg, data):
        """ Write a block of bytes, starting at a given register of the device
        :param int addr: device address
        :param int reg: register address on the device
        :param data: the bytes to write
        :type data: array of [int]
        """
        raise NotImplementedError()


try:
    from smbus import SMBus

except ImportError:
    # in case of smbus support not available, we define a simulation class, usable for unit tests for instance.
    class SimulatedSMBus(object):
        """ A fake SMBus, allowing to simulate reads by providing the expected reply and tracing data
        exchanges.

        For non documented methods, just have a look at :py:class:`smbus.SMBus`, since they are identical.
        """
        @staticmethod
        def _trace(msg):
            print('[SMBus.simul] ' + msg)

        def __init__(self, bus_id=1):
            self._trace('simulated SMBus created for id=%d' % bus_id)
            self._bus_id = bus_id
            self._data = [0] * 4

        def simulate_reply(self, data):
            """ Prepare the reply which will be returned by subsequent read(s)
            :param data: data to be returned by next read operation
            """
            self._data = data[:]

        def read_byte(self, addr):
            self._trace("reading byte 0x%x from device %d:0x%x" % (self._data[0], self._bus_id, addr))
            return self._data[0]

        def write_byte(self, addr, value):
            self._trace("writing byte 0x%x to device %d:0x%x" % (value, self._bus_id, addr))

        def read_byte_data(self, addr, reg):
            self._trace("reading byte 0x%x from reg 0x%x of device %d:0x%x" % (self._data[0], reg, self._bus_id, addr))
            return self._data[0]

        def write_byte_data(self, addr, reg, value):
            self._trace("writing byte 0x%x to reg 0x%x of device %d:0x%x" % (value, reg, self._bus_id, addr))

        def read_word_data(self, addr, reg):
            self._trace("reading word 0x%x from reg 0x%x of device %d:0x%x" % (self._data[0], reg, self._bus_id, addr))
            return self._data[0]

        def write_word_data(self, addr, reg, value):
            self._trace("writing word 0x%x to reg 0x%x of device %d:0x%x" % (value, reg, self._bus_id, addr))

        def read_block_data(self, addr, reg):
            self._trace("block reading from reg 0x%x of device %d:0x%x -> %s" % (reg, self._bus_id, addr, self._data))
            return self._data

        def write_block_data(self, addr, reg, data):
            self._trace("block writing %s to reg 0x%x of device %d:0x%x" % (data, reg, self._bus_id, addr))

        def read_i2c_block_data(self, addr, reg, count):
            self._trace("I2C block reading from reg 0x%x of device %d:0x%x -> %s" %
                  (reg, self._bus_id, addr, self._data)
            )
            return self._data

        def write_i2c_block_data(self, addr, reg, data):
            self._trace("I2C block writing %s to reg 0x%x of device %d:0x%x" % (data, reg, self._bus_id, addr))

    SMBus = SimulatedSMBus


class MTSMBus(I2CBus):
    """ Multi-thread compatible SMBus bus.

    This is just a wrapper of SMBus, serializing I/O on the bus for use
    in multi-threaded context and adding _i2c_ variants of block transfers.
    """

    def __init__(self, bus_id=1, **kwargs):
        """
        :param int bus_id: the SMBus id (see Raspberry Pi documentation)
        :param kwargs: parameters transmitted to :py:class:`smbus.SMBus` initializer
        """
        I2CBus.__init__(self, **kwargs)
        self._bus = SMBus(bus_id)
        # I/O serialization lock
        self._lock = threading.Lock()

    def read_byte(self, addr):
        with self._lock:
            return self._bus.read_byte(addr)

    def write_byte(self, addr, data):
        with self._lock:
            self._bus.write_byte(addr, data)

    def read_byte_data(self, addr, reg):
        with self._lock:
            return self._bus.read_byte_data(addr, reg)

    def write_byte_data(self, addr, reg, data):
        with self._lock:
            self._bus.write_byte_data(addr, reg, data)

    def read_word_data(self, addr, reg):
        with self._lock:
            return self._bus.read_word_data(addr, reg)

    def write_word_data(self, addr, reg, data):
        with self._lock:
            self._bus.write_word_data(addr, reg, data)

    def read_block_data(self, addr, reg):
        with self._lock:
            return self._bus.read_block_data(addr, reg)

    def write_block_data(self, addr, reg, data):
        with self._lock:
            self._bus.write_block_data(addr, reg, data)

    def read_i2c_block_data(self, addr, reg, count):
        with self._lock:
            return self._bus.read_i2c_block_data(addr, reg, count)

    def write_i2c_block_data(self, addr, reg, data):
        with self._lock:
            self._bus.write_i2c_block_data(addr, reg, data)



