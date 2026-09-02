#!/usr/bin/env python3
"""Derive the CMoveHelperClient singleton + vtable RVAs from a CS:S x64
client.dll ON DISK via the MSVC RTTI chain - the same walk
Prediction.cpp now performs at runtime (this script is the offline
validator and the source of the pinned fallback constants).

Chain: ".?AVCMoveHelperClient@@" string -> TypeDescriptor (string-0x10)
-> RTTICompleteObjectLocator (u32 pTypeDescriptor RVA at +0x0C,
signature==1, pSelf==its own RVA at +0x14) -> vtable (the qword slot
holding preferred_base+col_rva; vtable begins right after) -> the
singleton (the .data qword holding preferred_base+vtable_rva).

Usage: python Tools/derive_client_rvas.py [path-to-client.dll]
"""

import struct
import sys

DEFAULT = (r"C:\Program Files (x86)\Steam\steamapps\common"
           r"\Counter-Strike Source\cstrike\bin\x64\client.dll")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    data = open(path, "rb").read()

    # ---- PE: image base + section map (file offset <-> RVA)
    pe = struct.unpack_from("<I", data, 0x3C)[0]
    assert data[pe:pe + 4] == b"PE\0\0"
    nsec = struct.unpack_from("<H", data, pe + 6)[0]
    opt = pe + 24
    magic = struct.unpack_from("<H", data, opt)[0]
    assert magic == 0x20B, "x64 PE expected"
    image_base = struct.unpack_from("<Q", data, opt + 24)[0]
    opt_size = struct.unpack_from("<H", data, pe + 20)[0]
    secs = []
    off = opt + opt_size
    for i in range(nsec):
        s = off + i * 40
        name = data[s:s + 8].rstrip(b"\0").decode(errors="replace")
        vsize, va, rsize, raw = struct.unpack_from("<IIII", data, s + 8)
        secs.append((name, va, max(vsize, rsize), raw, rsize))

    def rva_to_off(rva):
        for name, va, vsize, raw, rsize in secs:
            if va <= rva < va + vsize:
                d = rva - va
                return raw + d if d < rsize else None
        return None

    def off_to_rva(o):
        for name, va, vsize, raw, rsize in secs:
            if raw <= o < raw + rsize:
                return va + (o - raw)
        return None

    # ---- 1. TypeDescriptor
    needle = b".?AVCMoveHelperClient@@"
    so = data.find(needle)
    assert so >= 0, "RTTI name not found"
    td_off = so - 0x10                      # {vft ptr, spare, name[]}
    td_rva = off_to_rva(td_off)
    print(f"TypeDescriptor RVA: 0x{td_rva:X}")

    # ---- 2. CompleteObjectLocator: u32 pTypeDescriptor at +0x0C,
    #         signature 1 at +0x00, pSelf at +0x14 equals its own RVA
    key = struct.pack("<I", td_rva)
    cols = []
    pos = -1
    while True:
        pos = data.find(key, pos + 1)
        if pos < 0:
            break
        col_off = pos - 0x0C
        if col_off < 0:
            continue
        sig = struct.unpack_from("<I", data, col_off)[0]
        if sig != 1:
            continue
        col_rva = off_to_rva(col_off)
        if col_rva is None:
            continue
        pself = struct.unpack_from("<I", data, col_off + 0x14)[0]
        if pself == col_rva:
            cols.append(col_rva)
    print(f"COL candidates: {[hex(c) for c in cols]}")
    assert cols, "no CompleteObjectLocator"

    # ---- 3. vtable: qword == image_base + col_rva; vtable follows
    vtables = []
    for col_rva in cols:
        key8 = struct.pack("<Q", image_base + col_rva)
        p = -1
        while True:
            p = data.find(key8, p + 1)
            if p < 0:
                break
            if p % 8:
                continue
            vt_rva = off_to_rva(p + 8)
            if vt_rva is not None:
                vtables.append(vt_rva)
    print(f"vtable candidates: {[hex(v) for v in vtables]}")
    assert vtables, "no vtable"

    # ---- 4. singleton: data qword == image_base + vtable_rva
    for vt_rva in vtables:
        key8 = struct.pack("<Q", image_base + vt_rva)
        p = -1
        hits = []
        while True:
            p = data.find(key8, p + 1)
            if p < 0:
                break
            if p % 8:
                continue
            r = off_to_rva(p)
            if r is not None:
                hits.append(r)
        print(f"vtable 0x{vt_rva:X}: singleton candidates: "
              f"{[hex(h) for h in hits]}")
        for h in hits:
            print(f"  kMoveHelperRva       = 0x{h:X}")
            print(f"  kMoveHelperVtableRva = 0x{vt_rva:X}")


if __name__ == "__main__":
    main()
