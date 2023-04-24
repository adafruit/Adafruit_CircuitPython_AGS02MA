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

**Hardware:**

* `AGS02MA Gas Sensor <http://www.adafruit.com/products/5593>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

"""

try:
    from busio import I2C
except ImportError:
    pass

import time
import struct
from micropython import const
from adafruit_bus_device import i2c_device

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_AGS02MA.git"

AGS02MA_I2CADDR_DEFAULT: int = const(0x1A)  # Default I2C address
_AGS02MA_TVOCSTAT_REG = const(0x00)
_AGS02MA_VERSION_REG = const(0x11)
_AGS02MA_GASRES_REG = const(0x20)
_AGS02MA_SETADDR_REG = const(0x21)
_AGS02MA_CRC8_INIT = const(0xFF)
_AGS02MA_CRC8_POLYNOMIAL = const(0x31)


def _generate_crc(data: int) -> int:
    """8-bit CRC algorithm for checking data

    :param int data: The data to generate a CRC for
    """

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
    """Driver for the AGS02MA air quality sensor.

    .. warning::
        I2C communication rate cannot be higher than 30KHZ. Refer to
        https://cdn-shop.adafruit.com/product-files/5593/datasheet+ags02ma.pdf
        Section 3.

    :param ~busio.I2C i2c_bus: The I2C bus the AGS02MA is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x1A`

    :raises RunTimeError: When the sensor could not be found


    **Quickstart: Importing and using the device**

    Here is an example of using the :class:`AGS02MA` class.
    First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            import busio
            from adafruit_ags02ma import AGS02MA

    Once this is done you can define your `busio.I2C` object and define your sensor object

        .. code-block:: python

            i2c = busio.I2C(board.SCL, board.SDA, frequency=20_000)
            ags = AGS02MA(i2c, address=0x1A)

    Now you have access to the :attr:`gas_resistance` and :attr:`TVOC` attributes

        .. code-block:: python

            res = ags.gas_resistance
            tvoc = ags.TVOC

    """

    def __init__(self, i2c_bus: I2C, address: int = AGS02MA_I2CADDR_DEFAULT) -> None:
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        self._buf = bytearray(5)
        self._addr = bytearray(1)
        try:
            self.firmware_version()
        except RuntimeError as exc:  # a CRC error or something!
            raise RuntimeError("Failed to find AGS02MA - check your wiring!") from exc

    def firmware_version(self) -> int:
        """Return 24-bit value which contains the firmware version"""
        return self._read_reg(_AGS02MA_VERSION_REG, 30)

    @property
    def gas_resistance(self) -> int:
        """The resistance of the MEMS gas sensor"""
        return self._read_reg(_AGS02MA_GASRES_REG, 1500) * 100  # in 0.1Kohm

    @property
    def TVOC(self) -> int:  # pylint: disable=invalid-name
        """The calculated Total Volatile Organic Compound measurement, in ppb

        :raises RunTimeError: When the sensor still preheating
        """
        val = self._read_reg(_AGS02MA_TVOCSTAT_REG, 1500)
        status = val >> 24

        if status & 0x1:
            raise RuntimeError("Sensor still preheating")
        return val & 0xFFFFFF

    def set_address(self, new_addr: int) -> None:
        """Set the address for the I2C interface, from 0x0 to 0x7F
        The sensor supports modifying the I2C address. After sending
        this command, the new address will take effect immediately,
        New address will remain after powering the sensor off.

        :param int new_addr: The new address

        :raises: ValueError: When selected address is not in the range 0x00-0x7F
        """
        if new_addr not in range(1, 128):
            raise ValueError("Selected address must be between 0x00 and 0x7F")

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

    def _read_reg(self, addr: int, delayms: int) -> int:
        """Read a register

        :param int addr: The address to read
        :param int delayms: The delay between writes and reads, in milliseconds

        :raises RunTimeError: When CRC check have failed.

        """

        with self.i2c_device as i2c:
            self._addr[0] = addr
            i2c.write(self._addr)
            time.sleep(delayms / 1000)
            i2c.readinto(self._buf)

        if _generate_crc(self._buf) != 0:
            raise RuntimeError("CRC check failed")
        val, crc = struct.unpack(">IB", self._buf)  # pylint: disable=unused-variable

        return val
