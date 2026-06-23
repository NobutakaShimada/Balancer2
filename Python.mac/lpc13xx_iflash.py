"""USB Mass Storage BOT access for NXP LPC13xx IFLASH bootloader."""

import struct

import usb.core
import usb.util


VID = 0x04CC
PID = 0x0003
CBW_SIGNATURE = 0x43425355
CSW_SIGNATURE = 0x53425355
DEFAULT_TIMEOUT_MS = 30000


class IFlashError(Exception):
    pass


class PhaseError(IFlashError):
    pass


class CommandFailed(IFlashError):
    def __init__(self, command_name, status, sense=None):
        super().__init__(f"{command_name} failed with CSW status {status}")
        self.command_name = command_name
        self.status = status
        self.sense = sense


def find_iflash_device(vid=VID, pid=PID):
    return usb.core.find(idVendor=vid, idProduct=pid)


def iter_bulk_endpoints(interface):
    for endpoint in interface:
        attributes = usb.util.endpoint_type(endpoint.bmAttributes)
        if attributes == usb.util.ENDPOINT_TYPE_BULK:
            yield endpoint


def endpoint_direction(endpoint):
    return usb.util.endpoint_direction(endpoint.bEndpointAddress)


class LPC13xxIFlash:
    def __init__(
        self,
        device=None,
        vid=VID,
        pid=PID,
        timeout_ms=DEFAULT_TIMEOUT_MS,
        detach_kernel=True,
        reset_on_open=True,
    ):
        self.vid = vid
        self.pid = pid
        self.timeout_ms = timeout_ms
        self.detach_kernel = detach_kernel
        self.reset_on_open = reset_on_open
        self.dev = device
        self.interface = None
        self.ep_in = None
        self.ep_out = None
        self._tag = 1

    def open(self):
        if self.dev is None:
            self.dev = find_iflash_device(self.vid, self.pid)
        if self.dev is None:
            raise IFlashError(f"NXP LPC13xx IFLASH not found: {self.vid:04x}:{self.pid:04x}")

        try:
            self.dev.set_configuration()
        except usb.core.USBError:
            # macOS usually has already selected the only configuration.
            pass

        cfg = self.dev.get_active_configuration()
        for interface in cfg:
            if (
                interface.bInterfaceClass == 0x08
                and interface.bInterfaceSubClass == 0x06
                and interface.bInterfaceProtocol == 0x50
            ):
                self.interface = interface
                break
        if self.interface is None:
            raise IFlashError("USB Mass Storage BOT interface not found")

        endpoints = list(iter_bulk_endpoints(self.interface))
        for endpoint in endpoints:
            if endpoint_direction(endpoint) == usb.util.ENDPOINT_IN:
                self.ep_in = endpoint
            elif endpoint_direction(endpoint) == usb.util.ENDPOINT_OUT:
                self.ep_out = endpoint

        if self.ep_in is None or self.ep_out is None:
            raise IFlashError("bulk IN/OUT endpoints not found")

        interface_number = self.interface.bInterfaceNumber
        if self.detach_kernel:
            try:
                if self.dev.is_kernel_driver_active(interface_number):
                    self.dev.detach_kernel_driver(interface_number)
            except (NotImplementedError, usb.core.USBError):
                pass

        usb.util.claim_interface(self.dev, self.interface.bInterfaceNumber)
        if self.reset_on_open:
            self.mass_storage_reset()
        return self

    def close(self):
        if self.dev is not None and self.interface is not None:
            try:
                usb.util.release_interface(self.dev, self.interface.bInterfaceNumber)
            except usb.core.USBError:
                pass
        if self.dev is not None:
            usb.util.dispose_resources(self.dev)

    def get_max_lun(self):
        try:
            data = self.dev.ctrl_transfer(
                0xA1,
                0xFE,
                0,
                self.interface.bInterfaceNumber,
                1,
                timeout=self.timeout_ms,
            )
            return int(data[0])
        except usb.core.USBError:
            self.clear_stall(self.ep_in)
            self.clear_stall(self.ep_out)
            return 0

    def mass_storage_reset(self):
        try:
            self.dev.ctrl_transfer(
                0x21,
                0xFF,
                0,
                self.interface.bInterfaceNumber,
                None,
                timeout=self.timeout_ms,
            )
        except usb.core.USBError:
            pass
        self.clear_stall(self.ep_in)
        self.clear_stall(self.ep_out)

    def clear_stall(self, endpoint):
        try:
            self.dev.clear_halt(endpoint.bEndpointAddress)
        except usb.core.USBError:
            pass

    def scsi_command(self, name, cdb, data_len=0, direction_in=True, payload=b""):
        tag = self._tag
        self._tag += 1

        cdb = bytes(cdb)
        if not 1 <= len(cdb) <= 16:
            raise ValueError(f"CDB length must be 1..16, got {len(cdb)}")

        flags = 0x80 if direction_in else 0x00
        cbw = struct.pack(
            "<IIIBBB16s",
            CBW_SIGNATURE,
            tag,
            data_len,
            flags,
            0,
            len(cdb),
            cdb.ljust(16, b"\x00"),
        )

        self.ep_out.write(cbw, timeout=self.timeout_ms)

        data = b""
        if data_len:
            if direction_in:
                try:
                    data = bytes(self.ep_in.read(data_len, timeout=self.timeout_ms))
                except usb.core.USBError as exc:
                    self.clear_stall(self.ep_in)
                    try:
                        csw = self._read_csw(name, tag)
                    except Exception:
                        raise IFlashError(f"{name} data-in failed: {exc}") from exc
                    if csw["status"] == 1:
                        raise CommandFailed(name, csw["status"]) from exc
                    raise IFlashError(f"{name} data-in failed after CSW {csw}: {exc}") from exc
            else:
                if len(payload) != data_len:
                    raise ValueError(f"{name} payload length {len(payload)} != {data_len}")
                try:
                    self.ep_out.write(payload, timeout=self.timeout_ms)
                except usb.core.USBError as exc:
                    self.clear_stall(self.ep_out)
                    try:
                        csw = self._read_csw(name, tag)
                    except Exception:
                        raise IFlashError(f"{name} data-out failed: {exc}") from exc
                    if csw["status"] == 1:
                        raise CommandFailed(name, csw["status"]) from exc
                    raise IFlashError(f"{name} data-out failed after CSW {csw}: {exc}") from exc

        csw = self._read_csw(name, tag)
        status = csw["status"]
        if status == 0:
            return data
        if status == 1:
            raise CommandFailed(name, status)
        raise PhaseError(f"{name} phase error: {csw}")

    def _read_csw(self, name, tag):
        raw = bytes(self.ep_in.read(13, timeout=self.timeout_ms))
        if len(raw) != 13:
            raise PhaseError(f"{name} CSW length {len(raw)} != 13")
        signature, got_tag, residue, status = struct.unpack("<IIIB", raw)
        if signature != CSW_SIGNATURE:
            raise PhaseError(f"{name} bad CSW signature: 0x{signature:08x}")
        if got_tag != tag:
            raise PhaseError(f"{name} CSW tag mismatch: {got_tag} != {tag}")
        return {"tag": got_tag, "residue": residue, "status": status}

    def inquiry(self):
        data = self.scsi_command("INQUIRY", b"\x12\x00\x00\x00\x24\x00", data_len=36)
        return {
            "peripheral": data[0],
            "removable": bool(data[1] & 0x80),
            "version": data[2],
            "vendor": data[8:16].decode("ascii", errors="replace").strip(),
            "product": data[16:32].decode("ascii", errors="replace").strip(),
            "revision": data[32:36].decode("ascii", errors="replace").strip(),
            "raw": data,
        }

    def test_unit_ready(self):
        self.scsi_command("TEST UNIT READY", b"\x00\x00\x00\x00\x00\x00")
        return True

    def request_sense(self):
        data = self.scsi_command("REQUEST SENSE", b"\x03\x00\x00\x00\x12\x00", data_len=18)
        return {
            "response_code": data[0] & 0x7F,
            "sense_key": data[2] & 0x0F,
            "asc": data[12],
            "ascq": data[13],
            "raw": data,
        }

    def read_capacity10(self):
        data = self.scsi_command("READ CAPACITY(10)", b"\x25\x00\x00\x00\x00\x00\x00\x00\x00\x00", data_len=8)
        last_lba, block_size = struct.unpack(">II", data)
        return {
            "last_lba": last_lba,
            "block_size": block_size,
            "blocks": last_lba + 1,
            "bytes": (last_lba + 1) * block_size,
            "raw": data,
        }

    def read10(self, lba, blocks, block_size=512):
        cdb = struct.pack(">BBIBHB", 0x28, 0, lba, 0, blocks, 0)
        return self.scsi_command("READ(10)", cdb, data_len=blocks * block_size)

    def write10(self, lba, data, block_size=512):
        data = bytes(data)
        if len(data) == 0 or len(data) % block_size != 0:
            raise ValueError(f"WRITE(10) data length must be a positive multiple of {block_size}")
        blocks = len(data) // block_size
        if blocks > 0xFFFF:
            raise ValueError(f"too many blocks for one WRITE(10): {blocks}")
        cdb = struct.pack(">BBIBHB", 0x2A, 0, lba, 0, blocks, 0)
        self.scsi_command("WRITE(10)", cdb, data_len=len(data), direction_in=False, payload=data)

    def synchronize_cache10(self):
        self.scsi_command("SYNCHRONIZE CACHE(10)", b"\x35\x00\x00\x00\x00\x00\x00\x00\x00\x00")

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        self.close()
