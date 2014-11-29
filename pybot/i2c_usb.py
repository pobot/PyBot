#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A specialized I2C interface implementation, for use with USB adapters such as
Robot Electronics one (http://www.robot-electronics.co.uk/htm/usb_i2c_tech.htm)
"""

__author__ = 'Eric Pascual'

import serial
import threading
import time

from .i2c import I2CBus


class USB2I2CBus(I2CBus):
    """ USB interface for an I2C bus.

    Gives access to an I2C bus via a virtual serial port.

    It differs a bit from standard I2C commands for block reads, since the adapter
    needs to know how many bytes are expected.
    """
    I2C_SGL = 0x53
    I2C_MUL = 0x54
    I2C_AD1 = 0x55
    I2C_AD2 = 0x56
    I2C_USB = 0x5A

    def __init__(self, dev, **kwargs):
        I2CBus.__init__(self, **kwargs)
        self._serial = serial.Serial(dev, baudrate=19200, timeout=0.5)
        self._serial.flushInput()
        self._serial.flushOutput()

        # I/O serialization lock to be as much thread safe as possible
        self._lock = threading.Lock()

    def _send_cmd(self, data):
        if isinstance(data, list):
            # stringify a byte list
            data = ''.join([chr(b) for b in data])

        if self._debug or self._simulate:
            print(':Tx> %s' % ' '.join('%02x' % ord(b) for b in data))
        if self._simulate:
            return
        with self._lock:
            self._serial.write(data)
            self._serial.flush()

    def _get_reply(self, nbytes):
        if self._simulate:
            print ('<Rx: -- no Rx data when running in simulated I/O mode --')
            return []

        with self._lock:
            data = []
            cnt = 0
            maxwait = time.time() + self._serial.timeout
            while cnt < nbytes and time.time() < maxwait:
                data = data + [ord(c) for c in self._serial.read(nbytes - cnt)]
                cnt = len(data)
            self._serial.flushInput()

        if self._debug:
            rx = ' '.join('%02x' % b for b in data)
            print('<Rx: %s' % rx)

        return data

    def read_byte(self, addr):
        self._send_cmd([self.I2C_SGL, (addr << 1) + 1])
        return self._get_reply(1)[0]

    def write_byte(self, addr, data):
        self._send_cmd([self.I2C_SGL, addr << 1, data & 0xff])
        result = self._get_reply(1)[0]
        if not result:
            raise RuntimeError('write_byte failed with result=%d' % result)

    def read_byte_data(self, addr, reg):
        return self.read_block_data(addr, reg, 1)[0]

    def write_byte_data(self, addr, reg, data):
        return self.write_block_data(addr, reg, [data])

    def read_word_data(self, addr, reg):
        data = self.read_block_data(addr, reg, 2)
        if self.msb_first:
            return (data[0] << 8) + data[1]
        else:
            return (data[1] << 8) + data[0]

    def write_word_data(self, addr, reg, data):
        if self.msb_first:
            return self.write_block_data(addr, reg,
                                         [(data >> 8) & 0xff, data & 0xff])
        else:
            return self.write_block_data(addr, reg,
                                         [data & 0xff, (data >> 8) & 0xff])

    def read_block_data(self, addr, reg, count):
        self._send_cmd([self.I2C_AD1, (addr << 1) + 1, reg, count])
        return self._get_reply(count)

    def write_block_data(self, addr, reg, data):
        self._send_cmd([self.I2C_AD1, addr << 1, reg, len(data)] + data)
        result = self._get_reply(1)[0]
        if not result:
            raise RuntimeError('write_block_data failed with result=%d' % result)

