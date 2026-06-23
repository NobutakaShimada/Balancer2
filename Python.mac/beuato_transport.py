"""Linux BeuatoCtrl driver compatible transport for Balancer2.

Existing scripts write ASCII commands such as ``b"r 18 8 "`` to
``/dev/BeuatoCtrl0``.  The Linux driver translated those commands to HID
reports.  This module keeps the same file-like API and performs that
translation in user space for macOS hidapi.
"""

import os
import platform
from dataclasses import dataclass

from balancer2_hid import Balancer2HID


BEUATO_MODE_ASCII = 0
BEUATO_MODE_BINARY = 1


class BeuatoProtocolError(ValueError):
    pass


@dataclass
class ParsedCommand:
    command: int
    address: int
    length: int
    data: bytes = b""


def parse_user_command(raw_command):
    raw_command = bytes(raw_command)
    try:
        text = raw_command.decode("ascii")
    except UnicodeDecodeError as exc:
        raise BeuatoProtocolError(f"command is not ASCII: {raw_command!r}") from exc

    parts = text.split()
    if len(parts) < 3:
        raise BeuatoProtocolError(f"too few command fields: {text!r}")

    command_name = parts[0]
    if command_name not in ("r", "w"):
        raise BeuatoProtocolError(f"unsupported command: {command_name!r}")

    try:
        address = int(parts[1], 16)
        length = int(parts[2], 16)
    except ValueError as exc:
        raise BeuatoProtocolError(f"invalid address or length: {text!r}") from exc

    if not 0 <= address <= 0xFFFF:
        raise BeuatoProtocolError(f"address out of range: 0x{address:x}")
    if not 0 <= length <= 0xFF:
        raise BeuatoProtocolError(f"length out of range: {length}")

    data = b""
    if command_name == "w":
        if len(parts) - 3 < length:
            raise BeuatoProtocolError(
                f"write command has {len(parts) - 3} data bytes, expected {length}"
            )
        try:
            data = bytes(int(tok, 16) for tok in parts[3 : 3 + length])
        except ValueError as exc:
            raise BeuatoProtocolError(f"invalid write data byte: {text!r}") from exc

    return ParsedCommand(ord(command_name), address, length, data)


def format_hid_payload(command):
    payload = bytearray()
    payload.append(command.command)
    payload.append(command.address & 0xFF)
    payload.append((command.address >> 8) & 0xFF)
    payload.append(command.length)
    if command.command == ord("w"):
        payload.extend(command.data[: command.length])
    return bytes(payload)


def ascii_command_to_hid_payload(raw_command):
    return format_hid_payload(parse_user_command(raw_command))


def normalize_hid_response(report):
    report = bytes(report)
    if not report:
        return b""

    if report[0] in (ord("r"), ord("w")):
        return report

    # macOS hidapi may include a leading report ID. The Balancer2 protocol
    # itself starts valid responses with 'r' or 'w', so drop only that case.
    if len(report) > 1 and report[0] == 0x00 and report[1] in (ord("r"), ord("w")):
        return report[1:]

    return report


def response_length(response):
    response = normalize_hid_response(response)
    if not response:
        return 0
    if response[0] == ord("r"):
        if len(response) < 2:
            return 0
        return 2 + response[1]
    if response[0] == ord("w"):
        if len(response) < 3:
            return 0
        return 3 + response[2]
    return len(response.rstrip(b"\x00"))


def build_binary_response(response):
    response = normalize_hid_response(response)
    length = response_length(response)
    if length <= 0:
        return b""
    return response[:length]


def build_ascii_response(response):
    response = build_binary_response(response)
    if not response:
        return b""
    if response[0] == ord("r"):
        size = response[1]
        data = response[2 : 2 + size]
    elif response[0] == ord("w"):
        size = response[2]
        data = response[3 : 3 + size]
    else:
        return response
    body = " ".join(f"{b:02x}" for b in data)
    if body:
        return f"r {size:x} {body} \n".encode("ascii")
    return f"r {size:x} \n".encode("ascii")


class BeuatoHIDTransport:
    def __init__(self, hid_device=None, timeout_ms=1000, mode=BEUATO_MODE_BINARY):
        self.hid = hid_device if hid_device is not None else Balancer2HID(timeout_ms=timeout_ms)
        self.timeout_ms = timeout_ms
        self.mode = mode
        self._last_response = b""
        self._closed = False

    def open(self):
        self.hid.open()
        self._closed = False
        return self

    def close(self):
        self.hid.close()
        self._closed = True

    def fileno(self):
        raise OSError("hidapi transport has no file descriptor")

    def set_mode(self, mode):
        if mode not in (BEUATO_MODE_ASCII, BEUATO_MODE_BINARY):
            raise ValueError(f"unknown Beuato read mode: {mode}")
        self.mode = mode

    def write(self, raw_command):
        payload = ascii_command_to_hid_payload(raw_command)
        self.hid.write_report(payload)
        self._last_response = self._read_logical_response()
        return len(raw_command)

    def read(self, size=-1):
        if self.mode == BEUATO_MODE_ASCII:
            data = build_ascii_response(self._last_response)
        else:
            data = build_binary_response(self._last_response)

        if size is None or size < 0:
            return data
        return data[:size]

    def flush(self):
        return None

    def _read_logical_response(self):
        chunks = bytearray()
        expected = 0

        while True:
            report = normalize_hid_response(self.hid.read_report(timeout_ms=self.timeout_ms))
            if not report:
                raise TimeoutError("timed out waiting for Balancer2 response")

            chunks.extend(report)
            expected = response_length(chunks)
            if expected and len(chunks) >= expected:
                return bytes(chunks[:expected])

            if chunks and chunks[0] not in (ord("r"), ord("w")):
                return bytes(chunks).rstrip(b"\x00")

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        self.close()


def open_beuato_device(device_path="/dev/BeuatoCtrl0", mode="r+b", buffering=0, *,
                       force_hid=None, timeout_ms=1000, read_mode=BEUATO_MODE_BINARY):
    use_hid = force_hid
    if use_hid is None:
        use_hid = platform.system() == "Darwin" or not os.path.exists(device_path)

    if use_hid:
        return BeuatoHIDTransport(timeout_ms=timeout_ms, mode=read_mode).open()
    return open(device_path, mode=mode, buffering=buffering)


def set_beuato_read_mode(dev, mode):
    if hasattr(dev, "set_mode"):
        dev.set_mode(mode)
        return

    import fcntl
    import struct

    IOCTL_READ_MODE = 0x40044201
    fcntl.ioctl(dev, IOCTL_READ_MODE, struct.pack("I", mode))


def set_beuato_debug(dev, enabled):
    if hasattr(dev, "set_mode"):
        return

    import fcntl
    import struct

    IOCTL_DEBUG = 0x40044200
    fcntl.ioctl(dev, IOCTL_DEBUG, struct.pack("I", int(enabled)))
