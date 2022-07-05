from __future__ import absolute_import, division, print_function, unicode_literals

import serial
from enum import Enum
from struct import pack
from sys import version_info


def int_to_bytes(integer):
    if version_info.major == 2:
        return pack("B", integer)
    else:
        return bytes([integer])


class RelayCmd(Enum):
    """
    Enum for the relay commands.
    """

    VERSION = 0x5A
    GET = 0x5B
    SET = 0x5C
    DC = 0x5D
    ALL = 0x64
    NONE = 0x6E


class RelayState(Enum):
    """
    Enum for the relay states.
    """

    ON = True
    OFF = False

    def __invert__(self):
        """
        Return the opposite state.
        """
        return RelayState(not self.value)


class RelaySet:

    ALL = 0b11111111
    NONE = 0b00000000

    def __init__(self, val=0):
        self.bitset = int(val)

    def __or__(self, other):
        if isinstance(other, RelaySet):
            self.bitset |= other.bitset
        elif isinstance(other, Relay):
            self.bitset |= other.bit()
        else:
            self.bitset |= other
        return self

    def __ror__(self, other):
        return self.__or__(other)

    def __and__(self, other):
        if isinstance(other, RelaySet):
            self.bitset &= other.bitset
        elif isinstance(other, Relay):
            self.bitset &= other.bit()
        else:
            self.bitset &= other
        return self

    def __rand__(self, other):
        return self.__and__(other)

    def __eq__(self, other):
        if isinstance(other, RelaySet):
            return self.bitset == other.bitset
        elif isinstance(other, Relay):
            return self.bitset == other.bit()
        else:
            return self.bitset == other

    def __ne__(self, other):
        return not self.__eq__(other)

    def __repr__(self):
        return "RelaySet({})".format(bin(self.bitset))

    @property
    def set(self):
        return self.bitset


class Relay(Enum):
    """
    Enum for the relay ports
    """

    R1 = 0
    R2 = 1
    R3 = 2
    R4 = 3
    R5 = 4
    R6 = 5
    R7 = 6
    R8 = 7

    def on_byte(self):
        return 0x65 + self.value

    def off_byte(self):
        return 0x6F + self.value

    def byte(self, on_off):
        return self.on_byte() if on_off is RelayState.ON else self.off_byte()

    def bit(self):
        return 1 << self.value

    def __or__(self, other):
        return RelaySet() | self | other

    def __ror__(self, other):
        return self.__or__(other)

    def __and__(self, other):
        return RelaySet() & self & other

    def __rand__(self, other):
        return self.__and__(other)

    def __invert__(self):
        return ~self.bit() & RelaySet.ALL


class UsbRelay:
    def __init__(self, port="/dev/ttyACM0", baudrate=9600, timeout=None, serial_type=serial.Serial, *args, **kwargs):
        """
        Initialize the USB relay.
        :param port (str): The serial port to use.
        :param baudrate (int): The baudrate to use.
        :param timeout (Optional[int]): The timeout to use.
        """
        self.serial = serial_type(port=port, baudrate=baudrate, timeout=timeout, *args, **kwargs)

    def read_all(self):
        self.serial.write(bytes([RelayCmd.GET.value]))
        return [RelayState(int(i)) for i in "{:08b}".format(self.serial.read(1)[0])][::-1]

    def read(self, relay):
        """
        Read the current state of the relays.
        :param relay (Relay): The relay to read.
        :return (int): The current state of the relays.
        """
        return self.read_all()[relay.value]

    def toggle(self, relay):
        """
        Toggle the relay.
        :param relay (Relay): The relay to toggle.
        """
        self.serial.write(int_to_bytes(relay.byte(~self.read(relay))))

    def close(self):
        """
        Close the relay.
        """
        self.serial.close()

    def on(self, relay):
        """
        Turn the relay on.
        :param relay (Relay): The relay to turn on.
        """
        self.serial.write(int_to_bytes(relay.byte(RelayState.ON)))

    def on_all(self):
        """
        Turn all relays on.
        """
        self.serial.write(int_to_bytes(RelayCmd.ALL.value))

    def off_all(self):
        """
        Turn all relays off.
        """
        self.serial.write(int_to_bytes(RelayCmd.NONE.value))

    def off(self, relay):
        """
        Turn the relay off.
        :param relay (Relay): The relay to turn off.
        """
        self.serial.write(int_to_bytes(relay.byte(RelayState.OFF)))
