# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2022 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ags02ma`
================================================================================

AGS02MA TVOC / Gas sensor


* Author(s): ladyada

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

"""

import time
import struct
from micropython import const
from adafruit_bus_device import i2c_device

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_AGS02MA.git"

AGS02MA_I2CADDR_DEFAULT: int = const(0x1A)  # Default I2C address
_AGS02MA_TVOCSTAT_REG = const(0x00)
_AGS02MA_VERSION_REG = const(0x11)
_AGS02MA_GASRES_REG = const(0x20)
_AGS02MA_SETADDR_REG = const(0x21)
_AGS02MA_CRC8_INIT = const(0xFF)
_AGS02MA_CRC8_POLYNOMIAL = const(0x31)


def _generate_crc(data):
    """8-bit CRC algorithm for checking data"""
    crc = _AGS02MA_CRC8_INIT
    # calculates 8-Bit checksum with given polynomial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ _AGS02MA_CRC8_POLYNOMIAL
            else:
                crc <<= 1
        crc &= 0xFF
    return crc & 0xFF


class AGS02MA:
    """Driver for the AGS02MA air quality sensor

    :param ~busio.I2C i2c_bus: The I2C bus the AGS02MA is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x1A`
    """

    def __init__(self, i2c_bus, address=AGS02MA_I2CADDR_DEFAULT):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        self._buf = bytearray(5)
        self._addr = bytearray(1)
        try:
            self.firmware_version()
        except RuntimeError as exc:  # a CRC error or something!
            raise RuntimeError("Failed to find AGS02MA - check your wiring!") from exc

    def firmware_version(self):
        """Return 24-bit value which contains the firmware version"""
        return self._read_reg(_AGS02MA_VERSION_REG, 30)

    @property
    def gas_resistance(self):
        """The resistance of the MEMS gas sensor"""
        return self._read_reg(_AGS02MA_GASRES_REG, 1500) * 100  # in 0.1Kohm

    @property
    def TVOC(self):  # pylint: disable=invalid-name
        """The calculated Total Volatile Organic Compound measurement, in ppb"""
        val = self._read_reg(_AGS02MA_TVOCSTAT_REG, 1500)
        status = val >> 24
        # print(hex(status))
        if status & 0x1:
            raise RuntimeError("Sensor still preheating")
        return val & 0xFFFFFF

    def set_address(self, new_addr):
        """Set the address for the I2C interface, from 0x0 to 0x7F

        :param int new_addr: THe new address
        """

        _buf = bytearray(
            [
                _AGS02MA_SETADDR_REG,
                new_addr,
                ~new_addr & 0xFF,
                new_addr,
                ~new_addr & 0xFF,
                0,
            ]
        )
        _buf[5] = _generate_crc(_buf[1:5])
        with self.i2c_device as i2c:
            i2c.write(_buf)

    def _read_reg(self, addr, delayms):
        """Read a register

        :param int addr: The address to read
        :param int delayms: The delay between writes and reads, in milliseconds
        """

        with self.i2c_device as i2c:
            self._addr[0] = addr
            i2c.write(self._addr)
            time.sleep(delayms / 1000)
            i2c.readinto(self._buf)
        # print([hex(x) for x in self._buf])
        if _generate_crc(self._buf) != 0:
            raise RuntimeError("CRC check failed")
        val, crc = struct.unpack(">IB", self._buf)  # pylint: disable=unused-variable
        # print(hex(val), hex(crc))
        return val
