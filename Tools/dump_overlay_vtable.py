#!/usr/bin/env python3
"""Dump the CIVDebugOverlay vtable from an engine.dll on disk (RTTI
walk), printing each entry's RVA and first bytes - to verify the
pinned AddBoxOverlay/AddLineOverlay indices (1/3) against a game
update without a live session.

Usage: python Tools/dump_overlay_vtable.py [path-to-engine.dll]
"""

import struct
import sys

DEFAULT = (r"C:\Program Files (x86)\Steam\steamapps\common"
           r"\Counter-Strike Source\bin\x64\engine.dll")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    data = open(path, "rb").read()

    pe = struct.unpack_from("<I", data, 0x3C)[0]
    nsec = struct.unpack_from("<H", data, pe + 6)[0]
    opt = pe + 24
    image_base = struct.unpack_from("<Q", data, opt + 24)[0]
    opt_size = struct.unpack_from("<H", data, pe + 20)[0]
    secs = []
    off = opt + opt_size
    for i in range(nsec):
        s = off + i * 40
        vsize, va, rsize, raw = struct.unpack_from("<IIII", data, s + 8)
        secs.append((va, max(vsize, rsize), raw, rsize))

    def rva_to_off(rva):
        for va, vsize, raw, rsize in secs:
            if va <= rva < va + vsize:
                d = rva - va
                return raw + d if d < rsize else None
        return None

    def off_to_rva(o):
        for va, vsize, raw, rsize in secs:
            if raw <= o < raw + rsize:
                return va + (o - raw)
        return None

    for name in (b".?AVCIVDebugOverlay@@", b".?AVIVDebugOverlay@@",
                 b".?AVCDebugOverlay@@"):
        so = data.find(name)
        if so < 0:
            print(f"{name.decode()}: not found")
            continue
        td_rva = off_to_rva(so - 0x10)
        print(f"{name.decode()}: TypeDescriptor RVA 0x{td_rva:X}")
        key = struct.pack("<I", td_rva)
        pos = -1
        while True:
            pos = data.find(key, pos + 1)
            if pos < 0:
                break
            col_off = pos - 0x0C
            if col_off < 0:
                continue
            if struct.unpack_from("<I", data, col_off)[0] != 1:
                continue
            col_rva = off_to_rva(col_off)
            if col_rva is None:
                continue
            if struct.unpack_from("<I", data, col_off + 0x14)[0] != col_rva:
                continue
            # vtable = the slot after the qword holding &COL
            key8 = struct.pack("<Q", image_base + col_rva)
            p = -1
            while True:
                p = data.find(key8, p + 1)
                if p < 0:
                    break
                if p % 8:
                    continue
                vt_rva = off_to_rva(p + 8)
                if vt_rva is None:
                    continue
                print(f"  COL 0x{col_rva:X} -> vtable RVA 0x{vt_rva:X}")
                # dump entries until one stops looking like code
                for i in range(20):
                    eoff = rva_to_off(vt_rva + i * 8)
                    if eoff is None:
                        break
                    va = struct.unpack_from("<Q", data, eoff)[0]
                    if va < image_base or va > image_base + 0x10000000:
                        print(f"    [{i:2d}] (end: 0x{va:X})")
                        break
                    frva = va - image_base
                    foff = rva_to_off(frva)
                    first = data[foff:foff + 24].hex() if foff else "?"
                    print(f"    [{i:2d}] rva 0x{frva:06X}  {first}")


if __name__ == "__main__":
    main()
