#!/usr/bin/env python3
import argparse
import importlib.util
import os
import re
import struct
import sys
from collections import defaultdict


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PROJECT = os.path.abspath(
    os.path.join(
        ROOT,
        "..",
        "MCUxpresso",
        "Default_Balancer",
        "Balancer2_firm_Simple",
    )
)
DEFAULT_MAP = os.path.join(DEFAULT_PROJECT, "Debug", "Balancer2_firm_Simple.map")
DEFAULT_BIN = os.path.join(DEFAULT_PROJECT, "Debug", "Balancer2_firm_Simple.bin")
DEFAULT_MEMMAP = os.path.join(ROOT, "BeuatoMemMap.py")


TYPE_NAMES = {
    "us": "uint16_t",
    "uc": "uint8_t",
    "c": "int8_t",
    "s": "int16_t",
    "d": "double",
    "ll": "int64_t",
    "ull": "uint64_t",
}


def load_python_memmap(path):
    spec = importlib.util.spec_from_file_location("BeuatoMemMap", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.memory_map, module.TYPE_FMT


def symbol_file(path):
    base = os.path.basename(path)
    match = re.search(r"libBalancer2_lib\.a\(([^)]+)\)", path)
    if match:
        return match.group(1)
    return base


def parse_map(path):
    text_symbols = []
    data_symbols = []
    section_headers = {}
    common_defs = {}
    current_section = None
    pending = None
    in_common_table = False

    section_re = re.compile(r"^([.\w]+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)(?:\s+load address\s+(0x[0-9a-fA-F]+))?")
    subsection_re = re.compile(r"^\s+(\.\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(.+)$")
    continuation_re = re.compile(r"^\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(.+)$")
    symbol_re = re.compile(r"^\s+(0x[0-9a-fA-F]+)\s+([A-Za-z_][A-Za-z0-9_]*)\s*$")
    common_re = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s+(0x[0-9a-fA-F]+)\s+(.+)$")

    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.rstrip("\n")
            if line.startswith("Common symbol"):
                in_common_table = True
                continue
            if in_common_table:
                if not line.strip():
                    continue
                if line.startswith("Discarded input sections"):
                    in_common_table = False
                else:
                    common = common_re.match(line)
                    if common:
                        name, size, source = common.groups()
                        common_defs[name] = {
                            "size": int(size, 16),
                            "source": source.strip(),
                        }
                    continue

            header = section_re.match(line)
            if header:
                name, addr, size, load_addr = header.groups()
                current_section = name
                section_headers[name] = {
                    "address": int(addr, 16),
                    "size": int(size, 16),
                    "load_address": int(load_addr, 16) if load_addr else None,
                }
                pending = None
                continue

            sub = subsection_re.match(line)
            cont = continuation_re.match(line)
            if sub:
                subsection, addr, size, source = sub.groups()
                pending = {
                    "section": current_section,
                    "subsection": subsection,
                    "address": int(addr, 16),
                    "size": int(size, 16),
                    "source": source.strip(),
                    "symbol": None,
                }
                continue
            if cont and pending and pending["symbol"] is None:
                addr, size, source = cont.groups()
                pending["address"] = int(addr, 16)
                pending["size"] = int(size, 16)
                pending["source"] = source.strip()
                continue

            sym = symbol_re.match(line)
            if sym and pending:
                addr, name = sym.groups()
                entry = dict(pending)
                entry["symbol"] = name
                entry["address"] = int(addr, 16)
                if name in common_defs:
                    entry["size"] = common_defs[name]["size"]
                    entry["source"] = common_defs[name]["source"]
                if entry["section"] == ".text":
                    text_symbols.append(entry)
                elif entry["section"] in (".data", ".bss", "COMMON", ".cmap") or (
                    entry["address"] >= 0x10000000
                ):
                    data_symbols.append(entry)
                elif entry["section"] == ".cmap":
                    data_symbols.append(entry)

    # COMMON symbols are listed under the .bss output section and use the prior
    # subsection as a container, so the section above is enough for RAM symbols.
    return section_headers, text_symbols, data_symbols


def flash_to_disk(flash_addr, block_size=512, data_lba=4):
    return data_lba + flash_addr // block_size, flash_addr % block_size


def decode_value(blob, offset, length, typ, fmt):
    if offset + length > len(blob):
        return "<outside image>"
    raw = blob[offset : offset + length]
    try:
        if typ == "c":
            return str(struct.unpack(fmt[typ], raw)[0])
        return repr(struct.unpack(fmt[typ], raw)[0])
    except Exception:
        return raw.hex(" ")


def print_markdown(args, sections, functions, data_symbols, memory_map, type_fmt, image):
    print("# Balancer2 Firmware Map\n")

    text = sections.get(".text", {})
    cmap = sections.get(".cmap", {})
    bss = sections.get(".bss", {})
    data = sections.get(".data", {})
    print("## Regions\n")
    print("| Region | Address | Size | Notes |")
    print("|---|---:|---:|---|")
    if text:
        print(f"| `.text` | `0x{text['address']:08x}` | `0x{text['size']:x}` | code, vector table, rodata |")
    if cmap:
        lba, off = flash_to_disk(cmap["address"])
        print(f"| `.cmap` | `0x{cmap['address']:08x}` | `0x{cmap['size']:x}` | Constmemmap defaults, disk LBA {lba} + 0x{off:x} |")
    if data:
        print(f"| `.data` | `0x{data['address']:08x}` | `0x{data['size']:x}` | RAM, load `0x{data['load_address']:08x}` |")
    if bss:
        print(f"| `.bss` | `0x{bss['address']:08x}` | `0x{bss['size']:x}` | RAM zero/common area |")
    print("")

    memmap_base = None
    const_base = None
    for sym in data_symbols:
        if sym["symbol"] == "memmap":
            memmap_base = sym["address"]
        if sym["symbol"] == "Constmemmap":
            const_base = sym["address"]
    if const_base is None and cmap:
        const_base = cmap["address"]

    print("## Communication Memory Map\n")
    print("| Field | Proto Off | Type | Bytes | RAM Addr | Flash Default | Disk LBA | Default |")
    print("|---|---:|---|---:|---:|---:|---:|---:|")
    for name, (offset, length, typ) in memory_map.items():
        ram_addr = memmap_base + offset if memmap_base is not None else None
        flash_addr = const_base + offset if const_base is not None else None
        lba = off = None
        if flash_addr is not None:
            lba, off = flash_to_disk(flash_addr)
        value = ""
        if image and const_base is not None:
            value = decode_value(image, flash_addr, length, typ, type_fmt)
        ram_s = f"`0x{ram_addr:08x}`" if ram_addr is not None else ""
        flash_s = f"`0x{flash_addr:04x}`" if flash_addr is not None else ""
        disk_s = f"`{lba}+0x{off:x}`" if lba is not None else ""
        print(
            f"| {name} | `0x{offset:03x}` | {TYPE_NAMES.get(typ, typ)} | {length} | "
            f"{ram_s} | {flash_s} | {disk_s} | `{value}` |"
        )
    print("")

    print("## Functions By Object\n")
    by_source = defaultdict(list)
    for fn in functions:
        if fn["symbol"]:
            by_source[symbol_file(fn["source"])].append(fn)
    for source in sorted(by_source):
        print(f"\n### {source}\n")
        print("| Address | Size | Function | Disk LBA |")
        print("|---:|---:|---|---:|")
        for fn in sorted(by_source[source], key=lambda x: x["address"]):
            lba, off = flash_to_disk(fn["address"])
            print(f"| `0x{fn['address']:04x}` | `0x{fn['size']:x}` | `{fn['symbol']}` | `{lba}+0x{off:x}` |")

    print("\n## RAM Symbols\n")
    print("| Address | Size | Symbol | Object |")
    print("|---:|---:|---|---|")
    for sym in sorted(data_symbols, key=lambda x: x["address"]):
        if sym["address"] < 0x10000000:
            continue
        print(
            f"| `0x{sym['address']:08x}` | `0x{sym['size']:x}` | "
            f"`{sym['symbol']}` | {symbol_file(sym['source'])} |"
        )


def main():
    parser = argparse.ArgumentParser(description="Analyze Balancer2 firmware .map/.bin memory layout.")
    parser.add_argument("--map", default=DEFAULT_MAP)
    parser.add_argument("--bin", default=DEFAULT_BIN)
    parser.add_argument("--memmap", default=DEFAULT_MEMMAP)
    args = parser.parse_args()

    if not os.path.exists(args.map):
        raise SystemExit(f"map file not found: {args.map}")
    memory_map, type_fmt = load_python_memmap(args.memmap)
    sections, functions, data_symbols = parse_map(args.map)
    image = b""
    if args.bin and os.path.exists(args.bin):
        with open(args.bin, "rb") as f:
            image = f.read()

    print_markdown(args, sections, functions, data_symbols, memory_map, type_fmt, image)


if __name__ == "__main__":
    main()
