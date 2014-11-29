#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Eric Pascual'

import platform
if not platform.machine().startswith('armv6'):
    raise ImportError('module %s can be used on Raspberry Pi only' % __name__)

else:
    import re

    # detect i2C bus id depending on the RasPi version
    for line in open('/proc/cpuinfo').readlines():
        m = re.match('(.*?)\s*:\s*(.*)', line)
        if m:
            name, value = (m.group(1), m.group(2))
            if name == "Revision":
                i2c_bus_id = 0 if value[-4:] in ('0002', '0003') else 1
                break

    # Define I2C bus and initialize it
    try:
        import smbus

    except ImportError:
        raise NotImplementedError('python-smbus is not installed on this system.')

    else:
        i2c_bus = smbus.SMBus(i2c_bus_id)