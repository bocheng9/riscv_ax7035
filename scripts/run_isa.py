#!/usr/bin/env python3
"""Run the official RISC-V ISA binaries against the RTL in simulation.

Each binary is converted to a hex image, loaded into the testbench's memory
model and run until it writes its verdict to tohost.

Two testbenches are compiled, because the memory path differs:

  USE_MMU=1  the real Sv32 unit with the arbiter in front of it, used for the
             virtual-memory binaries
  USE_MMU=0  a flat identity mapping, used for everything else

The build is chosen by the case name. rv32si-p-dirty is the one exception: its
name says -p- but its MPRV sub-test needs the walker, so it runs on the MMU
build.

Usage:
    python3 scripts/run_isa.py "rv32ui-p-*"
    python3 scripts/run_isa.py "rv32ui-v-*" --isa-dir /path/to/isa
"""
import argparse
import glob
import os
import re
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# Cases whose MPRV sub-test needs the page-table walker even though the name
# carries no -v- marker.
NEEDS_MMU_BY_NAME = {"rv32si-p-dirty"}

TIMEOUT_CYCLES = 2_000_000


def bin_to_hex(bin_path, hex_path):
    with open(bin_path, "rb") as f:
        data = f.read()
    if len(data) % 4:
        data += b"\x00" * (4 - len(data) % 4)
    with open(hex_path, "w") as f:
        for i in range(0, len(data), 4):
            f.write("%08x\n" % struct.unpack("<I", data[i:i + 4])[0])


def find_tohost(name, isa_dir):
    """The address the case stores its verdict to, from the disassembly."""
    dump = os.path.join(isa_dir, name + ".dump")
    if not os.path.exists(dump):
        dump = os.path.join(isa_dir, name + ".txt")
    if not os.path.exists(dump):
        return None
    with open(dump, errors="ignore") as f:
        text = f.read()
    m = re.search(r"sw\s+\w+,-?\d+\(\w+\)\s*#\s*([0-9a-f]{8})\s*<tohost>", text)
    if m:
        return int(m.group(1), 16)
    m = re.search(r"#\s*([0-9a-f]{8})\s*<tohost>", text)
    return int(m.group(1), 16) if m else None


def build(work, use_mmu):
    exe = os.path.join(work, "tb_isa_mmu%d" % use_mmu)
    sources = [os.path.join(ROOT, "sim/tb_isa.v")]
    for sub in ("rtl/core", "rtl/mmu"):
        d = os.path.join(ROOT, sub)
        sources += sorted(glob.glob(os.path.join(d, "*.v")))
    cmd = ["iverilog", "-o", exe, "-g2012",
           "-I", os.path.join(ROOT, "rtl/core"),
           "-I", os.path.join(ROOT, "rtl/mmu"),
           "-Ptb_isa.USE_MMU=%d" % use_mmu] + sources
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print("compile failed (USE_MMU=%d):" % use_mmu)
        print(r.stdout)
        print(r.stderr)
        return None
    return exe


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pattern")
    ap.add_argument("--isa-dir", default=os.environ.get("ISA_DIR", ""),
                    help="directory holding the .bin and .dump files")
    args = ap.parse_args()

    if not args.isa_dir:
        print("set --isa-dir or ISA_DIR to the directory with the ISA binaries")
        return 1
    cases = sorted(glob.glob(os.path.join(args.isa_dir, args.pattern + ".bin")))
    if not cases:
        print("no cases match", args.pattern)
        return 1

    work = tempfile.mkdtemp(prefix="isa_run_")
    exes = {}
    for use_mmu in (0, 1):
        exe = build(work, use_mmu)
        if exe is None:
            return 1
        exes[use_mmu] = exe

    passed, failed, timed_out, noaddr = [], [], [], []
    for bin_path in cases:
        name = os.path.basename(bin_path)[:-4]
        tohost = find_tohost(name, args.isa_dir)
        if tohost is None:
            noaddr.append(name)
            print("  NOADDR %-24s tohost not found in the disassembly" % name)
            continue

        hex_path = os.path.join(work, name + ".hex")
        bin_to_hex(bin_path, hex_path)

        needs_mmu = ("-v-" in name) or (name in NEEDS_MMU_BY_NAME)
        cmd = ["vvp", exes[1 if needs_mmu else 0],
               "+HEXFILE=" + hex_path, "+TOHOST=%08x" % tohost]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=180).stdout
        except subprocess.TimeoutExpired:
            timed_out.append(name)
            print("  TIMEOUT %-24s simulator did not finish" % name)
            continue

        m = re.search(r"RESULT: (\w+)\s+instret=(\d+)\s+cycles=(\d+)", out)
        if not m:
            failed.append(name)
            print("  FAIL    %-24s no result line" % name)
            continue
        verdict, instret, cycles = m.group(1), m.group(2), m.group(3)
        if verdict == "PASS":
            passed.append(name)
            print("  PASS    %-24s instret=%-8s cycles=%s" % (name, instret, cycles))
        else:
            failed.append(name)
            detail = re.search(r"tohost=([0-9a-fx]+)", out)
            print("  FAIL    %-24s tohost=%s instret=%s cycles=%s"
                  % (name, detail.group(1) if detail else "?", instret, cycles))

    print()
    print("total %d   pass %d   fail %d   timeout %d   noaddr %d"
          % (len(cases), len(passed), len(failed), len(timed_out), len(noaddr)))
    bad = failed + timed_out
    if bad:
        print("failed:", " ".join(bad))
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
