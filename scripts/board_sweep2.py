#!/usr/bin/env python3
"""Run the ISA binaries on the board and judge each case by its gp trace.

Each case needs the board reprogrammed: the loader jumps into the image and
never returns, so there is no way back to the console without a reset.

Per case:
  1. program the bitstream over JTAG
  2. send the image over the console, wait for the loader's read-back report
  3. read the trace the board streams once the program comes to rest

gp follows riscv-tests: it holds the sub-test number before each sub-test and
1 on the pass path, so the recorded sequence says which sub-tests ran and where
execution stopped.

The trace is a single frame,

    marker 0xCC 0x33 | count (4 bytes LE) | count x 8 bytes | terminator 0x3C

with each entry carrying gp and the pc at which gp changed. It has to be read
to its declared length: stopping at the marker truncates the tail and looks
exactly like a program that hung.

Usage:
    python3 scripts/board_sweep2.py "rv32ui-p-*"
"""
import argparse
import glob
import os
import struct
import subprocess
import sys
import time

import serial

SENTINEL = 0xDEADBEEF          # entry written when the program comes to rest
TRACE_MARKER = b"\xcc\x33"


def program_bitstream(args):
    r = subprocess.run([args.xsdb, args.prog_tcl],
                       capture_output=True, text=True, timeout=300)
    return "PROGRAMMED" in r.stdout


def read_case(bin_path, port, wait):
    with open(bin_path, "rb") as f:
        image = f.read()

    ser = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(struct.pack("<I", len(image)))
    time.sleep(0.05)
    for i in range(0, len(image), 128):
        ser.write(image[i:i + 128])
        time.sleep(0.012)

    buf = bytearray()
    deadline = time.time() + wait
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            j = buf.find(TRACE_MARKER)
            if j >= 0 and j + 6 <= len(buf):
                n = struct.unpack("<I", buf[j + 2:j + 6])[0]
                if n < 4000 and len(buf) >= j + 6 + n * 8 + 1:
                    break
        time.sleep(0.02)
    ser.close()
    return buf


def parse(buf):
    out = {"load": None, "gps": [], "stuck_pc": None, "verdict": "NO DATA"}

    if len(buf) >= 23 and buf[0] == 0xAA and buf[1] == 0x55:
        stored, mism = struct.unpack("<II", buf[2:10])
        out["load"] = (stored, mism)

    i = buf.find(TRACE_MARKER)
    if i < 0 or i + 6 > len(buf):
        return out
    rest = buf[i + 2:]
    count = struct.unpack("<I", rest[:4])[0]

    gps, pcs = [], []
    for k in range(count):
        off = 4 + k * 8
        if off + 8 > len(rest):
            break
        gps.append(struct.unpack("<I", rest[off:off + 4])[0])
        pcs.append(struct.unpack("<I", rest[off + 4:off + 8])[0])

    if gps and gps[-1] == SENTINEL:
        out["stuck_pc"] = pcs[-1]
        gps.pop()

    out["gps"] = gps
    if gps:
        last = gps[-1]
        if last == 1:
            out["verdict"] = "PASS"
        elif last >= 1337:
            out["verdict"] = "EXC"
        elif last & 1:
            out["verdict"] = "FAIL(t%d)" % (last >> 1)
        elif last == 0:
            out["verdict"] = "gp=0"
        else:
            out["verdict"] = "gp=%d" % last
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    ap = argparse.ArgumentParser()
    ap.add_argument("pattern")
    ap.add_argument("--isa-dir", default=os.environ.get("ISA_DIR", ""),
                    help="directory holding the .bin files")
    ap.add_argument("--port", default=os.environ.get("BOARD_PORT", "COM5"))
    ap.add_argument("--xsdb", default=os.environ.get("XSDB", "xsdb"))
    ap.add_argument("--prog-tcl", default=os.path.join(root, "vivado", "prog_xsdb.tcl"))
    ap.add_argument("--wait", type=float, default=40.0,
                    help="seconds to wait for one case to come to rest")
    args = ap.parse_args()

    if not args.isa_dir:
        print("set --isa-dir or ISA_DIR to the directory with the ISA binaries")
        return 1
    cases = sorted(glob.glob(os.path.join(args.isa_dir, args.pattern + ".bin")))
    if not cases:
        print("no cases match", args.pattern)
        return 1

    passed, failed = [], []
    for path in cases:
        name = os.path.basename(path)[:-4]
        if not program_bitstream(args):
            failed.append((name, "PROG"))
            print("%-28s PROGRAM FAILED" % name)
            continue
        time.sleep(0.4)
        res = parse(read_case(path, args.port, args.wait))

        if res["verdict"] == "PASS":
            passed.append(name)
        else:
            failed.append((name, res["verdict"]))
        print("%-28s load=%-12s %-10s stopped=0x%08x"
              % (name, str(res["load"]), res["verdict"], res["stuck_pc"] or 0))

    print()
    print("total %d   pass %d   fail %d" % (len(cases), len(passed), len(failed)))
    if failed:
        print("failed:", " ".join("%s(%s)" % (n, t) for n, t in failed))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
