import struct
import unittest

from lpc13xx_iflash import CBW_SIGNATURE, CSW_SIGNATURE


class BotStructureTests(unittest.TestCase):
    def test_cbw_layout(self):
        cdb = b"\x12\x00\x00\x00\x24\x00"
        cbw = struct.pack("<IIIBBB16s", CBW_SIGNATURE, 7, 36, 0x80, 0, len(cdb), cdb.ljust(16, b"\x00"))
        self.assertEqual(len(cbw), 31)
        self.assertEqual(cbw[:4], b"USBC")
        self.assertEqual(cbw[4:8], b"\x07\x00\x00\x00")
        self.assertEqual(cbw[8:12], b"\x24\x00\x00\x00")
        self.assertEqual(cbw[12], 0x80)
        self.assertEqual(cbw[14], len(cdb))
        self.assertEqual(cbw[15:21], cdb)

    def test_csw_layout(self):
        csw = struct.pack("<IIIB", CSW_SIGNATURE, 7, 0, 0)
        self.assertEqual(len(csw), 13)
        self.assertEqual(csw[:4], b"USBS")


if __name__ == "__main__":
    unittest.main()
