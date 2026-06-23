import unittest

from beuato_transport import (
    BEUATO_MODE_ASCII,
    BEUATO_MODE_BINARY,
    BeuatoHIDTransport,
    ascii_command_to_hid_payload,
    build_ascii_response,
    build_binary_response,
    normalize_hid_response,
)


class FakeHID:
    def __init__(self, reports):
        self.reports = list(reports)
        self.writes = []

    def open(self):
        return self

    def close(self):
        return None

    def write_report(self, payload):
        self.writes.append(bytes(payload))

    def read_report(self, timeout_ms=None):
        return self.reports.pop(0)


class BeuatoTransportTests(unittest.TestCase):
    def test_read_ascii_command_to_hid_payload(self):
        self.assertEqual(ascii_command_to_hid_payload(b"r 18 8 "), b"r\x18\x00\x08")

    def test_write_ascii_command_to_hid_payload(self):
        self.assertEqual(
            ascii_command_to_hid_payload(b"w 10 2 aa bb "),
            b"w\x10\x00\x02\xaa\xbb",
        )

    def test_normalize_hid_response_drops_report_id_only_for_protocol_response(self):
        self.assertEqual(normalize_hid_response(b"\x00r\x02\xaa\xbb"), b"r\x02\xaa\xbb")
        self.assertEqual(normalize_hid_response(b"\x00ERROR"), b"\x00ERROR")

    def test_binary_read_response(self):
        self.assertEqual(build_binary_response(b"r\x02\xaa\xbb\x00"), b"r\x02\xaa\xbb")

    def test_binary_write_response(self):
        self.assertEqual(build_binary_response(b"w\x10\x02\xaa\xbb\x00"), b"w\x10\x02\xaa\xbb")

    def test_ascii_response_matches_legacy_parser(self):
        self.assertEqual(build_ascii_response(b"r\x02\xaa\xbb\x00"), b"r 2 aa bb \n")

    def test_transport_keeps_zero_bytes_in_payload(self):
        fake = FakeHID([b"r\x03\x11\x00\x00".ljust(64, b"\x00")])
        dev = BeuatoHIDTransport(fake, mode=BEUATO_MODE_BINARY).open()
        dev.write(b"r 20 3 ")
        self.assertEqual(fake.writes, [b"r\x20\x00\x03"])
        self.assertEqual(dev.read(256), b"r\x03\x11\x00\x00")

    def test_transport_ascii_mode(self):
        fake = FakeHID([b"r\x02\xaa\xbb".ljust(64, b"\x00")])
        dev = BeuatoHIDTransport(fake, mode=BEUATO_MODE_ASCII).open()
        dev.write(b"r 0 2 ")
        self.assertEqual(dev.read(256), b"r 2 aa bb \n")


if __name__ == "__main__":
    unittest.main()
