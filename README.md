# riscv_ax7035

A RISC-V RV32IMA core in Verilog, running from DDR3 on an AX7035 board. U and S
privilege modes are implemented, with Sv32 page-based translation.

The memory controller, clock generation and interconnect are Xilinx IP; what is
written here is the core, the translation unit and the path between them. The
core is checked against the official ISA test suite in simulation and on the
board.

## Supported ISA

| Extension | Description |
|---|---|
| RV32I | Base integer instruction set |
| RV32M | Integer multiplication and division |
| RV32A | Atomic operations (LR/SC, AMO) |
| Zicsr | CSR access instructions |
| Zifencei | Instruction-fetch fence |

Privilege levels: M-mode, S-mode, U-mode.

Virtual memory: Sv32, with a page-table walker and a translation cache.
Delegation through `medeleg`/`mideleg`, `mstatus.MPP`/`SPP`, direct trap mode.

`MPRV` is honoured: with it set, a machine-mode data access is translated at
the privilege in `MPP`, while instruction fetch keeps using machine mode.

## Architecture

- Five stages, IF ID EX MEM WB. Each stage carries `valid` and reads the
  `ready` of the stage after it: a stage advances on `valid && ready`, holds
  when `valid && !ready`, and an empty stage becomes a bubble. Forwarding
  covers MEM→EX and WB→EX; an EX→EX dependency costs one cycle.
- Branches resolve in EX and flush IF/ID. Exceptions are precise and taken in
  MEM, so `mepc` and `mtval` describe exactly one instruction.
- The fetch stage issues one request at a time and tags it with an epoch. A
  redirect bumps the epoch, which invalidates whatever is in flight or queued,
  so no per-request kill signal is needed.
- Instruction fetch and data accesses both need translation, so an arbiter
  serialises them onto a single Sv32 walker. Data wins by default because the
  pipeline stalls on it; fetch has a short queue ahead of it that absorbs a few
  cycles, and a fetch request that has waited too long is given priority so it
  cannot starve.
- A page fault is reported with its cause, not retried. The fetch stage
  answers a faulted request itself instead of passing the untranslated address
  to memory, so the fault is attributed to the right `pc`.

## Memory map

| Region | Base | Notes |
|---|---|---|
| BootROM | `0x0000_0000` | serial loader, read-only, one beat per access |
| UART | `0x1110_0000` | byte interface the loader polls |
| DDR3 | `0x8000_0000` | programs are linked here |

The bus is a single 32-bit AXI4 master. Programs linked at `0x8000_0000` are
aliased down to physical `0x0` inside the bridge, which is where the DDR
controller sees them.

## Getting a program onto the board

The board comes up executing the BootROM loader, not the core. The loader reads
an image over the console, writes it into DDR, reads it back and reports how
many words it stored and how many mismatched. Only then is the core released,
and it starts fetching from `0x8000_0000`.

That read-back is what makes the report trustworthy: a load that went wrong on
the wire is visible before any execution happens.

## Results

The verdict arrives the way the suite reports it, as a store to `tohost`:
1 means the case passed, `2n+1` means sub-test `n` failed.

Simulation:

```
rv32ui  42/42    rv32um   8/8    rv32ua  10/10
rv32mi  16/16    rv32si   6/6
```

Board:

```
rv32ui  42/42    rv32um   8/8    rv32ua  10/10
rv32mi  16/16    rv32si   5/6
```

`rv32si-p-dirty` fails its third sub-test on the board while passing in
simulation. That sub-test sets `MPRV` and expects a machine-mode load to be
translated at the privilege in `MPP`; on the board the load reads a different
value and the case takes the failure path. It is the one open failure and the
README does not pretend otherwise.

## Building

Requires Vivado 2019.1. From a Windows shell:

```
vivado -mode batch -source vivado/build_isa_ddr.tcl
```

The script creates the project, generates the MIG and clocking IP, and runs
synthesis, implementation and bitstream generation. The bitstream lands at
`vivado/isa_ddr/isa_ddr.runs/impl_1/isa_ddr_top.bit`.

It prints `BIT_OK <path>` only when the file really exists. A build that stops
at the DRC step still exits zero in batch mode, so the presence of that line is
what to check rather than the exit status.

## Running the tests

### Simulation

```sh
export ISA_DIR=/path/to/riscv-tests/isa
python3 scripts/run_isa.py "rv32ui-p-*"
```

Two testbenches are compiled: one with the MMU in the path, used for the
virtual-memory binaries, and one with a flat memory model for the rest. The
MMU build is selected by the case name, with one exception noted in the script:
`rv32si-p-dirty` is named `-p-` but its MPRV sub-test requires the walker, so
it has to run on the MMU build.

### On the board

```sh
export ISA_DIR=/path/to/riscv-tests/isa
export BOARD_PORT=COM5
python3 scripts/board_sweep2.py "rv32ui-p-*"
```

`XSDB` may be set to the full path of `xsdb` if it is not on `PATH`.

For each case this reprograms the FPGA over JTAG, sends the image over the
console, waits for the loader's read-back report, and reads the trace the board
streams once the program comes to rest.

Reprogramming is required because the loader jumps into the image and there is
no way back to the console without a reset.

The trace is what makes a result readable without a debug probe. `gp` is set to
the sub-test number before each sub-test and to 1 on the pass path, so the
recorded sequence says which sub-tests ran, in what order, and where execution
stopped. It is one frame — marker, count, eight bytes per entry, terminator —
and it has to be read to its declared length; stopping at the marker truncates
the tail and looks exactly like a program that hung.

## Directory layout

```
rtl/core/       decoder, ALU, register file, CSRs, fetch, mul/div, core top
rtl/mmu/        Sv32 walker, translation cache, arbiter sharing the walker
rtl/soc/        address decode, DDR bridge, BootROM, UART peripheral, top level
rtl/periph/     UART transmitter and receiver
sim/            testbench running the ISA binaries against a memory model
scripts/        simulation runner and board runner
sw/             loader and small bare-metal programs
vivado/         build script; the project, IP and bitstream are generated
constraints/    pin and clock constraints
```

Nothing binary is stored in the repository. `vivado/isa_ddr/` does not exist
until the build script runs.

## Current margin

Timing closes at roughly -2.2 ns WNS, with the violations concentrated in the
debug hub rather than in the core logic. The debug cores are temporary
instrumentation and are being removed; a build without them is expected to
close, and that is the next step.

## Licence

MIT.
