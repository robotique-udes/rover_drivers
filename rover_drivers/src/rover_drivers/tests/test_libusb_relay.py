from unittest import TestCase
import serial

from .mocks.mock_serial import MockSerial

from ..libusb_relay import UsbRelay, Relay, RelayCmd, RelayState, RelaySet


class TestRelayState(TestCase):
    def test_from_bool_on(self):
        assert RelayState(True) is RelayState.ON

    def test_from_bool_off(self):
        assert RelayState(False) is RelayState.OFF

    def test_from_int_on(self):
        assert RelayState(1) is RelayState.ON

    def test_from_int_off(self):
        assert RelayState(0) is RelayState.OFF

    def test_invert_on_is_off(self):
        assert ~RelayState.ON is RelayState.OFF

    def test_invert_off_is_on(self):
        assert ~RelayState.OFF is RelayState.ON


class TestRelay(TestCase):

    def test_on_byte(self):
        self.assertEqual(Relay.R1.on_byte(), 0x65)
        self.assertEqual(Relay.R2.on_byte(), 0x66)
        self.assertEqual(Relay.R3.on_byte(), 0x67)
        self.assertEqual(Relay.R4.on_byte(), 0x68)
        self.assertEqual(Relay.R5.on_byte(), 0x69)
        self.assertEqual(Relay.R6.on_byte(), 0x6A)
        self.assertEqual(Relay.R7.on_byte(), 0x6B)
        self.assertEqual(Relay.R8.on_byte(), 0x6C)

    def test_off_byte(self):
        self.assertEqual(Relay.R1.off_byte(), 0x6F)
        self.assertEqual(Relay.R2.off_byte(), 0x70)
        self.assertEqual(Relay.R3.off_byte(), 0x71)
        self.assertEqual(Relay.R4.off_byte(), 0x72)
        self.assertEqual(Relay.R5.off_byte(), 0x73)
        self.assertEqual(Relay.R6.off_byte(), 0x74)
        self.assertEqual(Relay.R7.off_byte(), 0x75)
        self.assertEqual(Relay.R8.off_byte(), 0x76)

    def test_bit(self):
        self.assertEqual(Relay.R1.bit(), 0b1)
        self.assertEqual(Relay.R2.bit(), 0b10)
        self.assertEqual(Relay.R3.bit(), 0b100)
        self.assertEqual(Relay.R4.bit(), 0b1000)
        self.assertEqual(Relay.R5.bit(), 0b10000)
        self.assertEqual(Relay.R6.bit(), 0b100000)
        self.assertEqual(Relay.R7.bit(), 0b1000000)
        self.assertEqual(Relay.R8.bit(), 0b10000000)

    def test_bitfield_none_is_0_and_all_is_FF(self):
        self.assertEqual(RelaySet.ALL, 0xFF)
        self.assertEqual(RelaySet.NONE, 0x00)
        self.assertEqual(Relay.R1 & ~Relay.R1, RelaySet.NONE)

    def test_bitfield_or_combine(self):
        self.assertEqual(Relay.R1 | Relay.R2 | Relay.R3 | Relay.R4 |
                         Relay.R5 | Relay.R6 | Relay.R7 | Relay.R8, 0b11111111)
        self.assertEqual(Relay.R1 | Relay.R2 | Relay.R3, 0b111)
        self.assertEqual(Relay.R6 | Relay.R3 | Relay.R8, 0b10100100)

    def test_bitfield_and_invert_remove(self):
        self.assertEqual(Relay.R2 & ~Relay.R2, 0x00)
        self.assertEqual(~Relay.R2, 0b11111101)

    def test_relayset_or(self):
        a = Relay.R1 | Relay.R2 | Relay.R3
        b = Relay.R4 | Relay.R5 | Relay.R6
        self.assertIsInstance(a, RelaySet)
        self.assertIsInstance(b, RelaySet)
        self.assertIsInstance(a | b, RelaySet)
        self.assertEqual(a | b, 0b00111111)

    def test_relayset_or_permutations(self):
        a = Relay.R1 | Relay.R2 | Relay.R3
        b = Relay.R4 | Relay.R5 | Relay.R6
        self.assertIsInstance(a | b | Relay.R8, RelaySet)
        self.assertEqual(a | b | Relay.R8, 0b10111111)
        self.assertIsInstance(Relay.R8 | a | b, RelaySet)
        self.assertEqual(Relay.R8 | a | b, 0b10111111)
        self.assertIsInstance(a | Relay.R8 | b, RelaySet)
        self.assertEqual(a | Relay.R8 | b, 0b10111111)


class TestUsbRelay(TestCase):
    @staticmethod
    def makeRelay(*args, **kwargs):
        return UsbRelay(*args, **kwargs, serial_type=MockSerial, spec=serial.Serial)

    def test_init_default(self):
        relay = self.makeRelay()
        # relay.serial.open.assert_called_once_with(
        # port='/dev/ttyACM0', baudrate=9600, timeout=None)

    def test_init_other_values(self):
        relay = self.makeRelay(port='/dev/ttyACM1',
                               baudrate=115200, timeout=10)
        # relay.serial.open.assert_called_once_with(
        # port='/dev/ttyACM1', baudrate=115200, timeout=10)
