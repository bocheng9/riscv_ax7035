//=============================================================================
// File        : riscv_pkg.v
// Description : Shared constants: opcodes, ALU operations, privilege levels, widths
//=============================================================================
`ifndef RISCV_PKG_V
`define RISCV_PKG_V

//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
`define XLEN            32
`define RESET_PC        32'h8000_0000

//--------------------------------------------------------------------------
// ALU and write back
//--------------------------------------------------------------------------
`define OP_RTYPE        7'b0110011
`define OP_ITYPE        7'b0010011
`define OP_LOAD         7'b0000011
`define OP_STORE        7'b0100011
`define OP_BRANCH       7'b1100011
`define OP_JAL          7'b1101111
`define OP_JALR         7'b1100111
`define OP_LUI          7'b0110111
`define OP_AUIPC        7'b0010111
`define OP_SYSTEM       7'b1110011
`define OP_FENCE        7'b0001111
`define OP_AMO          7'b0101111   // RV32A

//--------------------------------------------------------------------------
// ALU
//--------------------------------------------------------------------------
`define ALU_ADD         5'd0
`define ALU_SUB         5'd1
`define ALU_SLL         5'd2
`define ALU_SLT         5'd3
`define ALU_SLTU        5'd4
`define ALU_XOR         5'd5
`define ALU_SRL         5'd6
`define ALU_SRA         5'd7
`define ALU_OR          5'd8
`define ALU_AND         5'd9
`define ALU_PASS_B      5'd10      // LUI
`define ALU_MUL         5'd11
`define ALU_MULH        5'd12
`define ALU_MULHSU      5'd13
`define ALU_MULHU       5'd14
`define ALU_DIV         5'd15
`define ALU_DIVU        5'd16
`define ALU_REM         5'd17
`define ALU_REMU        5'd18

//--------------------------------------------------------------------------
// write back
//--------------------------------------------------------------------------
`define WB_ALU          2'd0
`define WB_MEM          2'd1
`define WB_PC4          2'd2
`define WB_CSR          2'd3

//--------------------------------------------------------------------------
// CSR
//--------------------------------------------------------------------------
// Machine
`define CSR_MSTATUS     12'h300
`define CSR_MISA        12'h301
`define CSR_MEDELEG     12'h302
`define CSR_MIDELEG     12'h303
`define CSR_MIE         12'h304
`define CSR_MTVEC       12'h305
`define CSR_MCOUNTEREN  12'h306
`define CSR_MSCRATCH    12'h340
`define CSR_MEPC        12'h341
`define CSR_MCAUSE      12'h342
`define CSR_MTVAL       12'h343
`define CSR_MIP         12'h344
`define CSR_MCYCLE      12'hB00
`define CSR_MINSTRET    12'hB02
`define CSR_MINSTRETH   12'hB82
`define CSR_MCOUNTINHIBIT 12'h320
`define CSR_MVENDORID   12'hF11
`define CSR_MARCHID     12'hF12
`define CSR_MIMPID      12'hF13
`define CSR_MHARTID     12'hF14
// Supervisor
`define CSR_SSTATUS     12'h100
`define CSR_SEDELEG     12'h102
`define CSR_SIDELEG     12'h103
`define CSR_SIE         12'h104
`define CSR_STVEC       12'h105
`define CSR_SCOUNTEREN  12'h106
`define CSR_SSCRATCH    12'h140
`define CSR_SEPC        12'h141
`define CSR_SCAUSE      12'h142
`define CSR_STVAL       12'h143
`define CSR_SIP         12'h144
`define CSR_SATP        12'h180
// PMP
`define CSR_PMPCFG0     12'h3A0
`define CSR_PMPCFG1     12'h3A1
`define CSR_PMPCFG2     12'h3A2
`define CSR_PMPCFG3     12'h3A3
`define CSR_PMPADDR0    12'h3B0
// User counters
`define CSR_CYCLE       12'hC00
`define CSR_TIME        12'hC01
`define CSR_INSTRET     12'hC02

//--------------------------------------------------------------------------
// privilege level
//--------------------------------------------------------------------------
`define PRV_U           2'b00
`define PRV_S           2'b01
`define PRV_M           2'b11

//--------------------------------------------------------------------------
// mcause/scause
//--------------------------------------------------------------------------
`define EXC_INST_MISALIGN    4'd0
`define EXC_INST_ACCESS      4'd1
`define EXC_ILLEGAL_INST     4'd2
`define EXC_BREAKPOINT       4'd3
`define EXC_LOAD_MISALIGN    4'd4
`define EXC_LOAD_ACCESS      4'd5
`define EXC_STORE_MISALIGN   4'd6
`define EXC_STORE_ACCESS     4'd7
`define EXC_ECALL_U          4'd8
`define EXC_ECALL_S          4'd9
`define EXC_ECALL_M          4'd11
`define EXC_INST_PAGE_FAULT  4'd12
`define EXC_LOAD_PAGE_FAULT  4'd13
`define EXC_STORE_PAGE_FAULT 4'd15

//--------------------------------------------------------------------------
// interrupt
//--------------------------------------------------------------------------
`define IRQ_S_SOFT      1
`define IRQ_M_SOFT      3
`define IRQ_S_TIMER     5
`define IRQ_M_TIMER     7
`define IRQ_S_EXT       9
`define IRQ_M_EXT       11

//--------------------------------------------------------------------------
// AXI-like
//--------------------------------------------------------------------------
`define BUS_RD          2'd0
`define BUS_WR          2'd1

`endif // RISCV_PKG_V
