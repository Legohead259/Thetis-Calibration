"""
AMT22 driver for interfacing with the CUI devices AMT22 absolute encoder

CONOPS: The AMT22 is a SPI-driven absolute encoder with either 12- or 14-bit precision.
The device can be mounted onto a rotating shaft and report its absolute position from 0
to 2^12 or 2^14, depending on the model. The device can be digitally reset or zeroed
from the SPI bus.

CHANGELOG:
 - Version 1.0.0: Initial release
"""

__author__      = "Braidan Duffy"
__copyright__   = "Copyright 2023"
__credits__     = "Braidan Duffy"
__license__     = "MIT"
__version__     = "1.0.0"
__maintainer__  = "Braidan Duffy"
__email__       = "bduffy2018@my.fit.edu"
__status__      = "Prototype"

import spidev
from enum import Enum
from time import sleep


class AMT22Registers(Enum):
    NOP     = 0x00  # The read position byte
    RESET   = 0x60  # The secondary byte that the AMT22 uses to reset the device
    ZERO    = 0x70  # The secondary byte that the AMT22 uses to zero the device


class AMT22():
    _port: int
    _cs: int
    _resolution: int
    _speed: int
    _delay_us: int
    _spi: spidev.SpiDev
    
    def __init__(self, resolution: int, port: int=0, cs: int=0, speed: int=500000, delay: int=3) -> None:
        """The AMT22 is an absolute encoder that reports the absolute position of a rotating shaft.
        This device can be digitally read from, reset, and zeroed using the SPI bus

        Args:
            resolution (int): The resolution of the device being used. 12 for AMT22XA, 14 for AMT22XB
            port (int, optional): The SPI port used to communicate with the device. Defaults to 0.
            cs (int, optional): The chip-select pin connected to the device. Defaults to 0.
            speed (int, optional): The SPI bus speed in kHz. Defaults to 500000.
            delay (int, optional): The delay in microseconds between byte transfers over SPI. Defaults to 3.
        """
        self._resolution = resolution
        self._port = port
        self._cs = cs
        self._speed = speed
        self._delay_us = delay
        self._spi = spidev.SpiDev()
        
        self._spi.open(self._port, self._cs)
        
    def reset(self) -> bool:
        """Hex command sequence: [0x00 0x60]
        The encoder responds with the current position over the transmission then immediately resets.
        Observe the power on time when using this command.
        Encoder must be stationary to power back on.
        
        Returns:
            bool: When the power-on time has been observed and encoder reset
        """
        self._spi.xfer2([AMT22Registers.NOP.value, AMT22Registers.RESET.value], self._speed, self._delay_us)
        sleep(0.250) # Delay to allow the encoder to reset
        return True
    
    def zero(self) -> bool:
        """
        The encoder responds with the current position over this transmission then saves the current position into memory and performs a reset.  
        Encoder must be stationary for this command.  
        Observe the power on time when using this command.

        Returns:
            bool: If the position has been successfully zeroed
        """
        self._spi.xfer2([AMT22Registers.NOP.value, AMT22Registers.ZERO.value], self._speed, self._delay_us)
        sleep(0.250) # Delay to allow encoder to reset
        return self.position == 0
    
    @staticmethod
    def checksum(n: bytearray) -> bool:
        """Values K1 and K0 in the response are checkbits. The checkbits are odd parity over the odd and even bits
        in the position response shown in the equation below. The checkbits are not part of the position, but are used to verify its validity.
        The lower 14 bits are the encoder position
        
        Odd:   K1 = !(H5^H3^H1^L7^L5^L3^L1)
        Even:  K1 = !(H4^H2^H0^L6^L4^L2^L0)

        Args:
            n (bytearray): The response byte array to be checked

        Returns:
            bool: If the response passes the checksum calculation
        """
        return (not(bool(n & (1<<(8+5))) ^ bool(n & (1<<(8+3))) ^ bool(n & (1<<(8+1))) ^ bool(n & (1<<7)) ^ bool(n & (1<<5)) ^ bool(n & (1<<3)) ^ bool(n & (1<<1)))==bool(n & (1<<15))) and (not(bool(n & (1<<(8+4))) ^ bool(n & (1<<(8+2))) ^ bool(n & (1<<8)) ^ bool(n & (1<<6)) ^ bool(n & (1<<4)) ^ bool(n & (1<<2)) ^ bool(n & (1<<0)))==bool(n & (1<<14)))

        
    # ==================
    # === PROPERTIES ===
    # ==================
    
    
    @property
    def position(self) -> tuple[int, bool]:
        """Get the current position of the encoder

        Returns:
            tuple(int, bool): The first index is the current position reported by the encoder, the second is if this value passed the checksum calculation.
            Ignore this value if the checksum fails!
        """
        result = self._spi.xfer2([AMT22Registers.NOP.value, AMT22Registers.NOP.value], self._speed, self._delay_us)
        position = result[0]<<8 | result[1]    # Concatenate two bytes into 16-bit value
        data_good = AMT22.checksum(position)
        position = position & 0x3FFC if self._resolution == 12 else position & 0x3FFF # Drop checkbits and 2 LSB (AMT22A only)
        return (position, data_good)
    
    @property
    def angle(self) -> float:
        """Get the current angle of the shaft relative to the zero point

        Returns:
            float: The current angle in degrees
        """
        return self.position[0] * (360.0/2**self._resolution)
    
    @property
    def port(self, value: int):
        """Set SPI port used to communicate with the device. Automatically restarts the SPI object

        Args:
            value (int): The SPI port
        """
        self._port = value
        self._spi.close()
        self._spi.open(self._port, self._cs)
        
    @port.getter
    def port(self):
        return self._port
        
    @property
    def cs(self, value: int):
        """Set the chip-select pin used to communicate with the device. Automatically restarts the SPI object

        Args:
            value (int): the chip-select pin used.
        """
        self._cs = value
        self._spi.close()
        self._spi.open(self._port, self._cs)
        
    @cs.getter
    def cs(self):
        return self._cs