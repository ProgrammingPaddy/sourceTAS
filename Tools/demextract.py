"""demextract - CS:S .dem -> spectated-player trace CSV.

The rebuild's expert-validation ingest (Docs/SolverRebuildChecklist.md
M5.3). The demos auto-spectate the runner, so every dem_packet's
democmdinfo_t view origin/angles ARE the expert's eye trajectory - no
entity parsing needed. Output: one CSV per demo (tick,x,y,z,pitch,yaw,
roll,hspeed) plus a console summary. hspeed is finite-difference
horizontal speed in u/s at 66.67tps.

Usage: python demextract.py <out_dir> <demo1.dem> [demo2.dem ...]
"""
import os
import struct
import sys

TICK = 0.015

def read_header(f):
    raw = f.read(1072)
    if len(raw) != 1072 or raw[:8] != b'HL2DEMO\x00':
        raise ValueError('not an HL2DEMO file')
    demo_proto, net_proto = struct.unpack_from('<ii', raw, 8)
    def s(ofs):
        return raw[ofs:ofs + 260].split(b'\x00', 1)[0].decode(
            'utf-8', 'replace')
    server, client, mapname, gamedir = s(16), s(276), s(536), s(796)
    time_s, ticks, frames, signon = struct.unpack_from('<fiii', raw, 1056)
    return dict(demo_proto=demo_proto, net_proto=net_proto, server=server,
                client=client, map=mapname, gamedir=gamedir, time=time_s,
                ticks=ticks, frames=frames, signon=signon)

def extract(path, out_dir):
    rows = []
    with open(path, 'rb') as f:
        h = read_header(f)
        while True:
            head = f.read(5)
            if len(head) < 5:
                break
            cmd, tick = struct.unpack('<Bi', head)
            if cmd == 7:            # dem_stop
                break
            if cmd in (1, 2):       # dem_signon / dem_packet
                info = f.read(76)   # democmdinfo_t
                if len(info) < 76:
                    break
                ox, oy, oz = struct.unpack_from('<fff', info, 4)
                pit, yaw, rol = struct.unpack_from('<fff', info, 16)
                f.read(8)           # in/out sequence
                ln = struct.unpack('<i', f.read(4))[0]
                f.seek(ln, 1)
                if cmd == 2 and tick >= 0:
                    rows.append((tick, ox, oy, oz, pit, yaw, rol))
            elif cmd == 3:          # dem_synctick
                pass
            elif cmd == 4:          # dem_consolecmd
                ln = struct.unpack('<i', f.read(4))[0]
                f.seek(ln, 1)
            elif cmd == 5:          # dem_usercmd
                f.read(4)
                ln = struct.unpack('<i', f.read(4))[0]
                f.seek(ln, 1)
            elif cmd in (6, 8):     # dem_datatables / dem_stringtables
                ln = struct.unpack('<i', f.read(4))[0]
                f.seek(ln, 1)
            else:
                raise ValueError(f'unknown dem cmd {cmd} at tick {tick}')

    # de-dup ticks (keep last packet per tick), finite-difference speed
    dedup = {}
    for r in rows:
        dedup[r[0]] = r
    rows = [dedup[t] for t in sorted(dedup)]
    name = os.path.splitext(os.path.basename(path))[0]
    out = os.path.join(out_dir, name + '.csv')
    with open(out, 'w') as o:
        o.write('tick,x,y,z,pitch,yaw,roll,hspeed\n')
        vmax = 0.0
        for i, r in enumerate(rows):
            hs = 0.0
            if i > 0:
                dt = (r[0] - rows[i - 1][0]) * TICK
                if dt > 0:
                    dx = r[1] - rows[i - 1][1]
                    dy = r[2] - rows[i - 1][2]
                    hs = (dx * dx + dy * dy) ** 0.5 / dt
                    vmax = max(vmax, hs)
            o.write(f'{r[0]},{r[1]:.3f},{r[2]:.3f},{r[3]:.3f},'
                    f'{r[4]:.2f},{r[5]:.2f},{r[6]:.2f},{hs:.1f}\n')
    span = rows[-1][0] - rows[0][0] if rows else 0
    print(f'{name}: map={h["map"]} proto {h["demo_proto"]}/{h["net_proto"]} '
          f'| {len(rows)} rows, ticks {rows[0][0]}..{rows[-1][0]} '
          f'({span * TICK:.2f}s) | max hspeed {vmax:.0f} u/s -> {out}')
    return out

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    for p in sys.argv[2:]:
        try:
            extract(p, out_dir)
        except Exception as e:
            print(f'{os.path.basename(p)}: FAILED - {e}')
    return 0

if __name__ == '__main__':
    sys.exit(main())
