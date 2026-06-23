"""Low-level hidapi access for VStone Balancer2."""


class Balancer2HID:
    VID = 0x1962
    PID = 0x2080
    REPORT_SIZE = 64

    def __init__(self, vid=None, pid=None, timeout_ms=1000):
        self.vid = self.VID if vid is None else vid
        self.pid = self.PID if pid is None else pid
        self.timeout_ms = timeout_ms
        self.dev = None

    @staticmethod
    def enumerate(vid=None, pid=None):
        import hid

        return hid.enumerate(
            Balancer2HID.VID if vid is None else vid,
            Balancer2HID.PID if pid is None else pid,
        )

    def open(self):
        if self.dev is not None:
            return self

        import hid

        self.dev = hid.device()
        self.dev.open(self.vid, self.pid)
        self.dev.set_nonblocking(False)
        return self

    def close(self):
        if self.dev is not None:
            self.dev.close()
            self.dev = None

    def write_report(self, payload):
        if self.dev is None:
            raise RuntimeError("Balancer2 HID device is not open")

        payload = bytes(payload)
        if len(payload) > self.REPORT_SIZE:
            raise ValueError(f"HID payload too long: {len(payload)} > {self.REPORT_SIZE}")

        packet = b"\x00" + payload.ljust(self.REPORT_SIZE, b"\x00")
        written = self.dev.write(packet)
        if written != self.REPORT_SIZE + 1:
            raise IOError(f"short HID write: {written} != {self.REPORT_SIZE + 1}")
        return written

    def read_report(self, timeout_ms=None):
        if self.dev is None:
            raise RuntimeError("Balancer2 HID device is not open")

        if timeout_ms is None:
            timeout_ms = self.timeout_ms
        data = self.dev.read(self.REPORT_SIZE, timeout_ms=timeout_ms)
        return bytes(data)

    def transact(self, payload, timeout_ms=None):
        self.write_report(payload)
        return self.read_report(timeout_ms=timeout_ms)

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        self.close()
