//=============================================================================
// File        : riscv_core.v
// Description : RV32IMA five-stage pipeline core
//
//   Pipeline: IF -> ID -> EX -> MEM -> WB
//
//   Handshake, kept deliberately simple:
//     every stage has a valid flag (it holds an instruction) and looks at the
//     ready of the stage after it
//       - a stage advances when valid && ready
//       - valid && !ready holds the stage still (stall)
//       - !valid is an empty stage, and a bubble is inserted
//
//   Rules:
//     - EX is always ready, unless a multiply/divide is running
//     - ID deasserts ready on a RAW hazard it cannot forward around
//     - IF ready follows ID ready, so back-pressure propagates
//
//   Forwarding: MEM->EX and WB->EX cover almost every RAW case; an EX->EX
//   dependency costs one stall cycle.
//   Branches resolve in EX and flush IF/ID.
//   Exceptions are precise and taken in MEM.
//
//   Memory interface is AXI-shaped
//   (araddr/arvalid/arready, rdata/rvalid/rready).
//=============================================================================
`timescale 1ns / 1ps
`include "riscv_pkg.v"

module riscv_core #(
    parameter RESET_PC_V = 32'h0000_0000
)(
    input  wire        clk,
    input  wire        rst_n,

    //--------------------------------------------------------------
    // AXI read
    //--------------------------------------------------------------
    // valid/ready
    // imem_addr:
    // imem_req: AXI arvalid
    // imem_ack: arready or
    // imem_rdata: returns
    // imem_rvalid: rvalid
    output wire [31:0] imem_addr,
    output wire [2:0]  imem_prot,       // AXI arprot
    output wire        imem_req,
    input  wire        imem_ack,
    input  wire [31:0] imem_rdata,
    input  wire [1:0]  imem_rresp,      // AXI rresp
    input  wire        imem_rvalid,
    output wire        imem_rready,     // AXI rready

    //--------------------------------------------------------------
    // AXI read write
    //--------------------------------------------------------------
    output wire [31:0] dmem_araddr,
    output wire [2:0]  dmem_arprot,      // AXI arprot + privilege level
    output wire        dmem_arvalid,
    input  wire        dmem_arready,
    input  wire [31:0] dmem_rdata,
    input  wire [1:0]  dmem_rresp,       // AXI rresp
    input  wire        dmem_rvalid,
    output wire        dmem_rready,

    output wire [31:0] dmem_awaddr,
    output wire [2:0]  dmem_awprot,      // AXI awprot
    output wire        dmem_awvalid,
    input  wire        dmem_awready,
    output wire [31:0] dmem_wdata,
    output wire [3:0]  dmem_wstrb,
    output wire        dmem_wvalid,
    input  wire        dmem_wready,
    input  wire [1:0]  dmem_bresp,        // AXI bresp
    input  wire        dmem_bvalid,       // write MIG write only then
    output wire        dmem_bready,

    //--------------------------------------------------------------
    // MMU
    //--------------------------------------------------------------
    output wire [31:0] mmu_va,
    output wire        mmu_req,
    output wire        mmu_is_store,
    output wire        mmu_is_fetch,
    output wire [1:0]  mmu_priv,        // privilege level MPRV
    output wire [1:0]  mmu_fetch_priv,  // privilege level MPRV
    output wire        dmem_active_o,   // EX in memory access MMU privilege level
    output wire [31:0] satp_o,          // satp MMU
    output wire        sum_o,           // mstatus.SUM MMU permission
    output wire        mxr_o,           // mstatus.MXR
    output wire        sfence_o,        // sfence.vma MMU TLB
    output wire        mmu_fault_ack_o, // CPU page fault
    input  wire [31:0] mmu_pa,
    input  wire        mmu_hit,
    input  wire        mmu_fault,
    input  wire [3:0]  mmu_fault_cause,

    // Instruction fetch translation. The fetcher owns its own translation
    // slot, so its stall and fault cannot be the data-side ones: the pipeline
    // freezes differently for each.
    input  wire        if_mmu_stall,
    input  wire        if_mmu_fault,
    input  wire [3:0]  if_mmu_fault_cause,

    //--------------------------------------------------------------
    // interrupt
    //--------------------------------------------------------------
    input  wire        irq_soft,
    input  wire        irq_timer,
    input  wire        irq_ext,

    //--------------------------------------------------------------
    // debug
    //--------------------------------------------------------------
    output wire [31:0] cur_pc,
    output wire [15:0] dbg_if,      // debug {if_valid, stall_id, if_state[1:0], fifo_nonempty}
    output wire [31:0] dbg_ifpc,    // debug if_pc_next
    output wire        dbg_imreq,   // debug if_stage instr_req_o
    output wire [31:0] dbg_idpc,    // debug id_pc
    output wire [31:0] dbg_inst,    // debug id_inst
    output wire [31:0] dbg_ra,      // debug x1
    output wire [31:0] dbg_t3,      // debug x28
    output wire [31:0] dbg_gp,      // debug x3 (gp)
    output wire [31:0] dbg_s0,      // debug x8 (s0 / TESTNUM)
    output wire [31:0] dbg_ex_pc,   // debug EX PC
    output wire [31:0] dbg_sepc,    // debug sret returns
    output wire        imem_want,   // translation
    // Retirement event for trace comparison: one pulse per instruction that
    // completes, with the pc of that instruction alongside it.
    output wire        retire_pulse,
    output wire [31:0] retire_pc,
    output wire        dbg_ex_sret, // debug EX is sret
    output wire [31:0] dbg_ex_inst, // debug EX
    output wire [31:0] dbg_mcause,  // debug cause
    output wire [31:0] dbg_mepc,    // debug PC
    output wire [31:0] dbg_alu_y,   // debug EX ALU
    output wire        dbg_ex_valid,
    output wire        dbg_ex_reg_we,
    output wire [4:0]  dbg_ex_rd,
    output wire        dbg_mem_valid,
    output wire        dbg_mem_reg_we,
    output wire [4:0]  dbg_mem_rd,
    output wire [31:0] dbg_wb_data,
    output wire        dbg_wb_we,
    output wire [4:0]  dbg_wb_rd,
    output wire        dbg_pc_set,
    output wire        dbg_stall_id,
    output wire        dbg_ex_ready,
    output wire [1:0]  cur_priv,
    output wire [63:0] cycle_cnt,
    output wire [63:0] instret_cnt,
    output wire        halted,

    // I-Cache flush
    output wire        fetch_redirect
);

    //==========================================================================
    // declare up front remove Verilog forward reference → Vivado Synth 8-6901/8-91
    //==========================================================================
    reg  [31:0] fwd_rs1, fwd_rs2;    // declare up front alu_b
    reg  [6:0]  id_opcode_r;
    reg  [4:0]  id_alu_op_r;
    reg  [31:0] id_imm_r;
    reg  [63:0] instret_pre, instret_read;
    // before
    localparam MD_IDLE = 2'd0;
    localparam MD_WAIT = 2'd1;
    localparam MD_DONE = 2'd2;
    wire [31:0] alu_b = (id_opcode_r == `OP_RTYPE) ? fwd_rs2 : id_imm_r;
    wire is_muldiv = (id_alu_op_r >= `ALU_MUL) && (id_alu_op_r <= `ALU_REMU);
    reg  ex_memre_q;
    reg  [31:0] ex_alu_y_q;      // declare up front ex_fwd_data
    reg  [4:0]  ex_rd_q;
    reg         ex_reg_we_q;
    reg         ex_valid_q;
    reg  [31:0] ex_pc_q;
    reg  [1:0]  ex_wb_sel_q;
    wire ex_fwd_ok = ex_valid_q && ex_reg_we_q && (ex_rd_q != 5'd0) && !ex_memre_q;

    //==========================================================================
    // Vivado forward reference must in
    // Vivado 1 wire Synth 8-6901
    //==========================================================================
    // EX / MEM / WB
    reg  [31:0] ex_imm;
    reg         ex_mem_re, ex_mem_we;
    reg         ex_valid;
    reg  [4:0]  ex_rd;
    wire        ex_illegal, ex_block;
    wire [31:0] ex_dmem_pa;
    wire        ex_mmu_bypass;
    reg  [31:0] ex_alu_y;
    // ID
    reg         id_valid;
    wire        if_valid;
    // MEM / LSU
    reg  [31:0] mem_mem_rdata;
    wire [31:0] mem_load_data;
    reg  [1:0]  st_state;
    reg  [31:0] ld_first_q;
    reg  [31:0] id_pc;
    reg         ld_phase2_d;
    wire        ld_second_pulse;
    // hazard /
    wire        load_use, muldiv_busy, mmu_need, pc_set;
    // counter


    //==========================================================================

    // EX→EX before must and write back jal ra = pc+4 is
    wire [31:0] ex_fwd_data = (ex_wb_sel_q == `WB_PC4) ? (ex_pc_q + 32'd4) :
                                                          ex_alu_y_q;
    reg  [31:0] fwd_rs1_ex, fwd_rs2_ex;
    wire [31:0] jalr_raw      = (fwd_rs1_ex + ex_imm) & ~32'h1;
    // st_state EX ex_ready=0 ID must freeze from ID
    wire stall_id = (load_use || muldiv_busy || (id_valid && ex_block) ||
                     (id_valid && (st_state != 2'd0))) && !pc_set;
    wire if_take     = if_valid && !stall_id;
    reg  mem_wait;
    // True on the cycle the memory stage retires its access. Derived
    // combinationally from the same registers the LSU block tests, so the
    // execute stage can vacate on that cycle without a second always block
    // writing ex_valid (Vivado rejects that as DRC MDRV-1, while iverilog
    // silently keeps one driver - which is why simulation looked fine).
    wire ld_done = mem_wait &&
                   ( (mem_is_amo_q && !amo_phase && dmem_rvalid && !mem_first_q &&
                      (is_lr || (is_sc && !sc_ok)))
                   | (mem_is_amo_q &&  amo_phase && dmem_awready && dmem_wready)
                   | (!mem_is_amo_q && dmem_rvalid && req_sent_q &&
                      (!ld_need2 || ld_phase2)) );
    // Frozen while this access's address is not yet translated.
    wire mmu_stall = mmu_need && !mmu_ready;
    reg  st_wait_q;      // store dmem_bvalid
    // When EX may advance:
    //   load : once mem_wait clears, i.e. the handshake finished and the
    //          data has been captured
    //   store: once the awready/wready handshake succeeded; st_wait_q then
    //          covers an aligned store waiting for bvalid
    reg         mmu_fault_r;
    // Translation is done when the answer is latched, or when this access
    // needs no translation at all.
    wire mmu_ready = mmu_done || ex_mmu_bypass;
    wire st_hs_pending = ex_valid && ex_mem_we && !ex_illegal && mmu_ready &&
                         !mmu_fault && !mmu_fault_r && (st_state == 2'd0) &&
                         !st_wait_q && !dmem_awready;
    // mem_load_pending covers the cycle a load is issued, before mem_wait has
    // risen: without it the instruction behind the load advances in the same
    // cycle and overwrites the memory stage's pc, rd and write controls while
    // the load is still waiting for its data.
    // A load occupies the memory stage from the cycle it is issued until it
    // writes back. Both the issue pulse and the waiting flag have gaps
    // between them, so neither alone can hold the pipeline; the union of the
    // two does, and it cannot loop because mmu_ready is registered.
    // mem_wait spans the whole life of a memory access: it is raised the cycle
    // the access is handed to memory and only dropped once the pipeline can
    // move on, so it alone is what holds EX still.
    wire ex_ready = !muldiv_busy && !mem_wait && !mem_load_pending && !mmu_stall &&
                    (st_state == 2'd0) && !st_wait_q && !st_hs_pending;
    assign mmu_need = ex_valid && (ex_mem_re || ex_mem_we) && !ex_illegal && !ex_mmu_bypass;
    reg  [31:0] mem_next_word;
    reg         ld_need2;          // read
    reg  [1:0]  ld_ph2_q;         // second segment timing 1= first segment 2= second segment
    reg         ld_ph2_go;        // second segment first segment after
    reg         ld_phase2;         // read
    reg  [31:0] ld_addr2;
    wire ld_ph2_ready = !ld_need2 || ld_phase2_d || !ld_phase2;
    wire [31:0] ld_hi_word = ld_second_pulse ? dmem_rdata : mem_next_word;
    wire [31:0] ld_lo_word = ld_second_pulse ? ld_first_q : mem_mem_rdata;
    // The pulse that latches the assembled word must not depend on mem_wait:
    // that flag is cleared on this very cycle now, so gating on it would drop
    // the pulse exactly when it is needed.
    assign ld_second_pulse = ld_need2 && ld_phase2 && dmem_rvalid && req_sent_q;
    reg [1:0]  md_state;
    reg [31:0] muldiv_result;
    reg [31:0] md_pc;               // before PC
    wire md_is_new   = (id_pc != md_pc);            // ID is
    wire md_take     = (md_state == MD_DONE) && ex_ready;   // EX

    //==========================================================================
    // pipeline
    //==========================================================================
    // ---- IF ----
    reg  [31:0] if_pc;
    wire [31:0] if_pc_req;      // the single prefetch pointer before
    wire [31:0] if_inst_pc;     // and if_rdata
    wire [31:0] if_req_addr;    // address of the request already issued
    wire [31:0] if_resp_addr;   // address of the in-flight request, used to pair the
                                // response so a new request cannot overwrite it
    wire        if_req_pending; // in
    reg         if_req;         // AR R
    wire [31:0] if_rdata;
    wire        if_fault;
    wire [3:0]  if_fault_cause;

    // ---- ID ----
    reg [31:0] id_inst, id_imm;
    reg  [4:0]  id_rs1, id_rs2, id_rd;
    reg  [4:0]  id_alu_op;
    reg  [1:0]  id_wb_sel;
    reg         id_reg_we, id_mem_re, id_mem_we, id_mem_unsigned;
    reg  [2:0]  id_mem_size;
    reg         id_is_branch, id_is_jal, id_is_jalr;
    reg  [2:0]  id_br_type;
    reg         id_is_csr, id_is_amo, id_fence_i, id_illegal;
    // A fetch page fault carries its own cause; folding it into "illegal
    // instruction" reported mcause=2 for what the specification calls cause 12
    // and hid the real reason from software.
    reg         id_if_fault;
    reg  [3:0]  id_if_fault_cause;
    reg  [1:0]  id_csr_op;
    reg  [11:0] id_csr_addr;
    reg  [4:0]  id_amo_op;
    reg         id_is_ecall, id_is_ebreak, id_is_mret, id_is_sret;
    reg         id_is_wfi, id_is_sfence;
    reg  [31:0] id_rs1_val, id_rs2_val;
    reg  [31:0] id_csr_wdata;
    reg         id_csr_uimm;

    // ---- EX ----
    reg [31:0] ex_pc, ex_rs1_val, ex_rs2_val;
    reg [4:0]  ex_rs1, ex_rs2, ex_alu_op, ex_amo_op;
    reg  [1:0]  ex_wb_sel, ex_csr_op;
    reg ex_reg_we, ex_mem_unsigned;
    reg  [2:0]  ex_mem_size, ex_br_type;
    reg         ex_is_branch, ex_is_jal, ex_is_jalr;
    reg  [31:0] ex_inst;              // EX and ex_pc
    reg         ex_is_csr, ex_is_amo, ex_fence_i;
    wire        dec_ex_illegal_raw;
    reg         ex_csr_uimm;      // CSRRWI/CSRRSI/CSRRCI: use the immediate, not rs1
    reg  [4:0]  ex_uimm_v;        // uimm and ex_csr_uimm
    reg  [11:0] ex_csr_addr;
    reg         ex_is_ecall, ex_is_ebreak, ex_is_mret, ex_is_sret;
    reg         ex_if_fault;
    reg  [3:0]  ex_if_fault_cause;
    reg         ex_is_wfi, ex_is_sfence;
    reg  [31:0] ex_csr_rdata;
    wire [31:0] ex_csr_wdata = ex_csr_uimm ? {27'd0, ex_uimm_v} :
                             (ex_rs1 == 5'd0) ? 32'h0 : fwd_rs1_ex;

    // ---- MEM ----
    reg [31:0] mem_pc, mem_alu_y, mem_csr_rdata;
    reg  [1:0]  mem_addr_lo;       // memory access 2
    reg  [2:0]  mem_mem_size;
    reg         mem_mem_unsigned;  // zero-extend instead of sign-extend
    reg  [4:0]  mem_rd;
    reg  [1:0]  mem_wb_sel;
    reg         mem_reg_we, mem_is_csr, mem_illegal;
    reg         mem_exc_valid;
    reg  [3:0]  mem_exc_cause;
    reg  [31:0] mem_exc_tval;
    reg  [31:0] mem_exc_pc;
    reg         mem_valid;
    reg         mem_is_settled;  // MEM before

    //==========================================================================

    //==========================================================================
    // ID
    wire [6:0]  d_opcode, d_funct7;
    wire [4:0]  d_rd, d_rs1, d_rs2, d_alu_op, d_amo_op;
    wire [2:0]  d_funct3, d_mem_size, d_br_type;
    wire [31:0] d_imm;
    wire        d_use_rs1, d_use_rs2, d_use_imm;
    wire [1:0]  d_wb_sel, d_csr_op;
    wire [11:0] d_csr_addr;
    wire        d_reg_we, d_mem_re, d_mem_we, d_mem_unsigned;
    wire        d_is_branch, d_is_jal, d_is_jalr, d_is_csr, d_is_amo;
    wire        d_is_fence, d_is_ecall, d_is_ebreak, d_is_mret;
    wire        d_is_sret, d_is_wfi, d_is_sfence_vma, d_illegal;

    decoder u_decoder (
        .inst(if_rdata),
        .opcode(d_opcode), .rd(d_rd), .funct3(d_funct3),
        .rs1(d_rs1), .rs2(d_rs2), .funct7(d_funct7), .imm(d_imm),
        .use_rs1(d_use_rs1), .use_rs2(d_use_rs2), .use_imm(d_use_imm),
        .alu_op(d_alu_op), .wb_sel(d_wb_sel),
        .reg_we(d_reg_we), .mem_re(d_mem_re), .mem_we(d_mem_we),
        .mem_size(d_mem_size), .mem_unsigned(d_mem_unsigned),
        .is_branch(d_is_branch), .is_jal(d_is_jal), .is_jalr(d_is_jalr),
        .br_type(d_br_type),
        .is_csr(d_is_csr), .csr_op(d_csr_op), .csr_addr(d_csr_addr),
        .is_amo(d_is_amo), .amo_op(d_amo_op),
        .is_fence(d_is_fence), .is_ecall(d_is_ecall), .is_ebreak(d_is_ebreak),
        .is_mret(d_is_mret), .is_sret(d_is_sret), .is_wfi(d_is_wfi),
        .is_sfence_vma(d_is_sfence_vma), .illegal(d_illegal)
    );


    wire [31:0] rf_rs1_val, rf_rs2_val, ex_rs1_rf, ex_rs2_rf;
    wire [31:0] wb_alu_result;
    wire        wb_we;
    wire [4:0]  wb_rd;
    wire [31:0] wb_data;

    regfile u_regfile (
        .clk(clk), .rst_n(rst_n),
        .raddr_a(id_rs1), .rdata_a(rf_rs1_val),   // and id_*
        .raddr_b(id_rs2), .rdata_b(rf_rs2_val),
        .raddr_c(mem_csr_src_idx), .rdata_c(mem_csr_src_val),
        .raddr_d(id_rs1), .rdata_d(ex_rs1_rf),   // EX1 read
        .raddr_e(id_rs2), .rdata_e(ex_rs2_rf),   // EX2 read
        .we(wb_we), .waddr(wb_rd), .wdata(wb_data)
    );

    // before

    // ALU
    wire [31:0] alu_y;
    wire        alu_zero;
    wire [31:0] alu_a = fwd_rs1;


    alu u_alu (
        .op(id_alu_op_r), .a(alu_a), .b(alu_b), .y(alu_y), .zero(alu_zero)
    );


    wire [31:0] muldiv_y;
    wire        muldiv_valid;

    mul_div u_muldiv (
        .clk(clk), .rst_n(rst_n),
        .start(muldiv_start), .op(id_alu_op_r),
        .a(alu_a), .b(alu_b), .y(muldiv_y), .valid(muldiv_valid)
    );

    // CSR
    wire [31:0] csr_rdata, csr_rdata_ex;
    reg  [1:0]  priv;
    wire [1:0]  priv_next;
    wire [31:0] trap_vector;
    wire        trap_taken;
    wire [31:0] trap_epc, mepc_o, sepc_o;
    assign dbg_mepc = mepc_o;
    wire        mret_pulse, sret_pulse;

    // memory access privilege level RISC-V MPRV
    // MPRV=1 before is M → memory access MPP privilege level
    // before privilege level
    wire [31:0] mstatus_int;
    assign sum_o = mstatus_int[18];   // SUM
    assign mxr_o = mstatus_int[19];   // MXR
    assign sfence_o = ex_valid && ex_is_sfence;   // sfence.vma
    assign mmu_fault_ack_o = mem_exc_valid && mem_exc_cause != 4'd2;  // page fault
    wire        mprv_eff = mstatus_int[17] && (priv == `PRV_M);
    wire [1:0]  dmem_priv = mprv_eff ? mstatus_int[12:11] : priv;
    assign mmu_priv       = dmem_priv;
    assign mmu_fetch_priv = priv;
    assign cur_priv = priv;

    csr_regs u_csr (
        .clk(clk), .rst_n(rst_n),
        .addr(mem_csr_addr_r), .op(mem_csr_op_r),
        .we(mem_csr_we),
        .wdata(mem_csr_wdata_r), .rdata(csr_rdata),
        .raddr(id_csr_addr), .rdata_ex(csr_rdata_ex),
        .priv(priv), .priv_next(priv_next),
        .mcause_o(dbg_mcause),
        .exc_valid(mem_exc_valid), .exc_cause(mem_exc_cause),
        .exc_tval(mem_exc_tval), .exc_pc(mem_exc_pc),
        .trap_taken(trap_taken), .trap_vector(trap_vector), .trap_epc(trap_epc),
        .satp_o(satp_o),
        .mstatus_o(mstatus_int),
        .mepc_o(mepc_o), .sepc_o(sepc_o),
        .mret(mret_pulse), .sret(sret_pulse),
        .irq_soft(irq_soft), .irq_timer(irq_timer), .irq_ext(irq_ext),
        .cycle_cnt(), .instret_cnt(), .instret_inc(instret_inc_pulse)
    );

    // MEM CSR
    reg  [11:0] mem_csr_addr_r;
    reg  [1:0]  mem_csr_op_r;
    reg  [31:0] mem_csr_wdata_r;
    reg         mem_rs1_zero;
    reg         mem_csr_uimm_r;
    reg  [4:0]  mem_csr_src_idx;   // CSR write
    wire [31:0] mem_csr_src_val;
    reg         mem_csr_we_r;
    // RISC-V CSRRS/CSRRC rs1=x0 when no write CSR csrr is read only
    //   csr_op: 2'b01=CSRRW, 2'b10=CSRRS, 2'b11=CSRRC
    // CSRRS/CSRRC immediate rs1=x0 when only then no write
    // immediate uimm uimm=0 uimm=0 no write
    // uimm rs1 mem_rs1_zero
    wire mem_csr_no_write = (mem_csr_op_r != 2'b01) && (mem_rs1_zero) && !mem_csr_uimm_r;
    reg         mem_csr_we_q;   // and addr/wdata write
    wire        mem_csr_we = mem_is_csr && mem_valid && !mem_exc_valid && !mem_csr_no_write;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) mem_csr_we_q <= 1'b0;
        else        mem_csr_we_q <= mem_csr_we;
    end

    // privilege level
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) priv <= `PRV_M;
        else        priv <= priv_next;
    end

    //==========================================================================
    // ID ALU posedge when
    //==========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_opcode_r <= 7'd0;
            id_alu_op_r <= 5'd0;
            id_imm_r    <= 32'h0;
        end else if (!stall_id && if_valid) begin
            // must and id_inst only in if_valid when
            id_opcode_r <= d_opcode;
            id_alu_op_r <= d_alu_op;
            id_imm_r    <= d_imm;
        end
    end

    //==========================================================================
    // before ID MEM/WB
    //==========================================================================
    // memory access ALU/JAL/AUIPC/CSR MEM write back mem_valid state machine
    // mem_wait / pc_set is RISC-V 5 write back
    // MEM→EX before mem_valid MEM
    wire mem_fwd_ok = mem_reg_we && (mem_rd != 5'd0) && !mem_exc_valid && !mem_is_settled;
    // before must and write back load + sign-extend
    wire [31:0] mem_fwd_data = (mem_wb_sel == `WB_MEM) ? mem_load_data :
                               (mem_wb_sel == `WB_PC4) ? (mem_pc + 32'd4) :
                               (mem_wb_sel == `WB_CSR) ? mem_csr_rdata :
                                                          mem_alu_y;
    wire wb_fwd_ok = wb_we && (wb_rd != 5'd0);

    always @(*) begin
        // rs1
        if      (mem_fwd_ok && (mem_rd == id_rs1)) fwd_rs1 = mem_fwd_data;
        else if (wb_fwd_ok  && (wb_rd == id_rs1))  fwd_rs1 = wb_data;
        else                                       fwd_rs1 = rf_rs1_val;

        // rs2
        if      (mem_fwd_ok && (mem_rd == id_rs2)) fwd_rs2 = mem_fwd_data;
        else if (wb_fwd_ok  && (wb_rd == id_rs2))  fwd_rs2 = wb_data;
        else                                       fwd_rs2 = rf_rs2_val;

        // EX→EX before in EX MEM
        // ALU 1 stall
        // must load !ex_mem_re load ex_alu_y is is
        // before data load_use " "
        // —— load_use
        // and fwd_rs1_ex / fwd_rs2_ex !ex_memre_q hold
        // jal/jalr alu_y is write back rd is pc+4 → wb_sel
        if (ex_valid && ex_reg_we && (ex_rd != 5'd0) && (ex_rd == id_rs1) && !ex_mem_re)
            fwd_rs1 = (ex_wb_sel == `WB_PC4) ? (ex_pc + 32'd4) : ex_alu_y;
        if (ex_valid && ex_reg_we && (ex_rd != 5'd0) && (ex_rd == id_rs2) && !ex_mem_re)
            fwd_rs2 = (ex_wb_sel == `WB_PC4) ? (ex_pc + 32'd4) : ex_alu_y;

        // rs1/rs2 is x0 when 0
        if (id_rs1 == 5'd0) fwd_rs1 = 32'h0;
        if (id_rs2 == 5'd0) fwd_rs2 = 32'h0;
    end

    //==========================================================================
    // EX before after
    //==========================================================================
    reg branch_taken;
    always @(*) begin
        case (ex_br_type)
            3'd0: branch_taken = (fwd_rs1_ex == fwd_rs2_ex);
            3'd1: branch_taken = (fwd_rs1_ex != fwd_rs2_ex);
            3'd2: branch_taken = ($signed(fwd_rs1_ex) <  $signed(fwd_rs2_ex));
            3'd3: branch_taken = ($signed(fwd_rs1_ex) >= $signed(fwd_rs2_ex));
            3'd4: branch_taken = (fwd_rs1_ex <  fwd_rs2_ex);
            3'd5: branch_taken = (fwd_rs1_ex >= fwd_rs2_ex);
            default: branch_taken = 1'b0;
        endcase
    end

    // EX before / JALR / store
    // EX( ALU ) > MEM > WB >
    // EX→EX is must slli a0,a0,31 bltz a0,x0

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_alu_y_q  <= 32'h0;
            ex_rd_q     <= 5'd0;
            ex_reg_we_q <= 1'b0;
            ex_valid_q  <= 1'b0;
            ex_pc_q     <= 32'h0;
            ex_wb_sel_q <= 2'd0;
        end else begin
            ex_alu_y_q  <= ex_alu_y;
            ex_rd_q     <= ex_rd;
            ex_reg_we_q <= ex_reg_we;
            ex_valid_q  <= ex_valid;
            ex_pc_q     <= ex_pc;
            ex_wb_sel_q <= ex_wb_sel;
            ex_memre_q  <= ex_mem_re;   // load alu_y is before
        end
    end


    always @(*) begin
        if      (ex_fwd_ok  && (ex_rd_q == ex_rs1)) fwd_rs1_ex = ex_fwd_data;
        else if (mem_fwd_ok && (mem_rd == ex_rs1))  fwd_rs1_ex = mem_fwd_data;
        else if (wb_fwd_ok  && (wb_rd == ex_rs1))   fwd_rs1_ex = wb_data;
        else                                         fwd_rs1_ex = ex_rs1_val;

        if      (ex_fwd_ok  && (ex_rd_q == ex_rs2)) fwd_rs2_ex = ex_fwd_data;
        else if (mem_fwd_ok && (mem_rd == ex_rs2))  fwd_rs2_ex = mem_fwd_data;
        else if (wb_fwd_ok  && (wb_rd == ex_rs2))   fwd_rs2_ex = wb_data;
        else                                         fwd_rs2_ex = ex_rs2_val;

        if (ex_rs1 == 5'd0) fwd_rs1_ex = 32'h0;
        if (ex_rs2 == 5'd0) fwd_rs2_ex = 32'h0;
    end

    //==========================================================================
    // PC
    //==========================================================================
    wire [31:0] branch_target = ex_pc + ex_imm;
    // jalr bit0 RISC-V bit1 unaligned
    wire [31:0] jalr_target   = jalr_raw;

    // 4 → INST_MISALIGNED
    // when only then taken
    wire [31:0] jm_target  = ex_is_jalr ? jalr_raw : branch_target;
    wire        jm_taking  = ex_is_jalr || ex_is_jal || (ex_is_branch && branch_taken);
    wire        jm_fault   = ex_valid && jm_taking && jm_target[1];

    wire do_branch = ex_valid && !jm_fault &&
                     ((ex_is_branch && branch_taken) || ex_is_jal || ex_is_jalr);
    wire do_mret   = ex_valid && (ex_is_mret || ex_is_sret);

    // retire mem_valid → retire
    reg         instret_inc_q;
    // A store leaves no write-back slot, so it retires when its response
    // arrives rather than through mem_valid. Without this the store never
    // counted as retired, which silently desynchronised the trace comparison.
    reg  st_retire_q;
    wire st_retire = st_wait_q && dmem_bvalid && !st_retire_q;
    wire        instret_inc_pulse = (mem_valid && !mem_exc_valid && !instret_inc_q) || st_retire;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) instret_inc_q <= 1'b0;
        else        instret_inc_q <= mem_valid && !mem_exc_valid;

    wire do_fence_i = ex_valid && ex_fence_i;
    assign pc_set = do_branch || do_mret || trap_taken || do_fence_i;

    // mret/sret trap_taken
    // mret mem_exc_valid is 1 PC
    wire [31:0] pc_next = ex_is_mret ? mepc_o :     // mret returns mepc
                          ex_is_sret ? sepc_o :     // sret returns sepc
                          trap_taken ? trap_vector :
                          do_fence_i ? (ex_pc + 32'd4) :   // fence.i flush IF
                                       (ex_is_jalr ? jalr_target : branch_target);

    //==========================================================================
    // hazard
    //==========================================================================
    // load-use EX is load ID → 1
    assign load_use = ex_valid && ex_mem_re && (ex_rd != 5'd0) && id_valid &&
                    ((ex_rd == d_rs1) || (ex_rd == d_rs2));



    // EX memory access freeze when ID must freeze ID
    // !ex_ready mmu_stall when stall timing
    // mem_wait EX freeze ID must freeze ID
    // memory access cross-word load / AMO only then freeze ID load freeze hold timing
    // EX memory access freeze when ID freeze MMU translation mmu_stall freeze ID
    // ID freeze = EX before ID
    // mmu_stall is stall freeze ID
    // 5 ID only in ——
    // 1) load-use hazard 2) 3) ID EX
    // 5 pipeline ID
    // mmu_stall/mm_wait is / translation stall freeze ID
    assign ex_block = !muldiv_busy && !mem_wait && !mmu_stall ? 1'b0 :
                    (mem_wait || muldiv_busy);   // memory access / only then freeze
    // trap/pc_set when must ID

    //==========================================================================
    // IF
    //==========================================================================
    //------------------------------------------------------------------
    // IF + valid/ready
    //
    // if_pc_req: before success +4
    // if_inst_pc: and if_rdata
    // = when if_pc_req
    // if_valid: hold ID !stall_id
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // IF valid handshake stall
    //
    // if_req_addr: imem_addr
    // if_req_pending: in only then
    // if_pc_req:
    //
    // = if_req_pending && imem_rvalid
    // /pc_set if_req_pending in
    // stall
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // IF Ibex branch + in discard
    // / / ID stall /
    // in returns when / / 20
    //------------------------------------------------------------------
    wire [31:0] if_pc_out;
    wire [31:0] if_pc_next;
    wire [1:0]  dbg_epoch, dbg_req_epoch, dbg_fifo_ep, dbg_if_state;
    wire [2:0]  dbg_fifo_cnt;
    wire        if_fault_out;
    wire [3:0]  if_fault_cause_out;

    if_stage #(.RESET_PC_V(RESET_PC_V)) u_if (
        .clk(clk), .rst_n(rst_n),

        .branch_i(pc_set), .branch_addr_i(pc_next),

        .instr_addr_o(imem_addr), .instr_prot_o(imem_prot),
        .instr_req_o(imem_req), .instr_ack_i(imem_ack),
        .instr_rvalid_i(imem_rvalid), .instr_rdata_i(imem_rdata),
        .instr_resp_i(imem_rresp), .instr_rready_o(),
        // ID
        .instr_valid_o(if_valid), .instr_pc_o(if_pc_out),
        .instr_data_o(if_rdata), .instr_ready_i(!stall_id),
        // MMU
        .mmu_stall_i(if_mmu_stall),
        .mmu_fault_i(if_mmu_fault), .mmu_fault_cause_i(if_mmu_fault_cause),
        .if_fault_o(if_fault_out), .if_fault_cause_o(if_fault_cause_out),
        .instr_want_o(imem_want),
        .dbg_pc_next(if_pc_next), .dbg_epoch(dbg_epoch), .dbg_req_epoch(dbg_req_epoch),
        .dbg_fifo_ep(dbg_fifo_ep), .dbg_fifo_cnt(dbg_fifo_cnt), .dbg_state(dbg_if_state)
    );

    // Names kept for the older interface
    assign if_inst_pc     = if_pc_out;
    assign if_fault       = if_fault_out;
    assign if_fault_cause = if_fault_cause_out;
    // PC EX FIFO after pc_next
    assign cur_pc         = ex_valid ? ex_pc : (if_valid ? if_pc_out : if_pc_next);
    assign dbg_if         = {dbg_epoch, dbg_req_epoch, dbg_fifo_ep,
                             if_valid, dbg_fifo_cnt, dbg_if_state};
    assign dbg_ifpc       = if_pc_next;
    assign dbg_imreq      = u_if.state == 2'd1;   // state
    assign dbg_idpc       = id_pc;
    assign dbg_inst       = id_inst;
    assign dbg_ra         = u_regfile.regs[1];   // debug ra
    assign dmem_bready    = 1'b1;   // write
    assign dbg_t3         = u_regfile.regs[28];  // debug t3
    assign dbg_gp         = u_regfile.regs[3];   // debug gp
    assign dbg_s0         = u_regfile.regs[8];   // debug s0
    assign dbg_ex_pc      = ex_pc;
    assign dbg_sepc       = sepc_o;
    assign dbg_ex_sret    = ex_is_sret;
    assign dbg_ex_inst    = ex_inst;
    assign dbg_alu_y      = ex_alu_y;
    assign dbg_ex_valid   = ex_valid;
    assign dbg_ex_reg_we  = ex_reg_we;
    assign dbg_ex_rd      = ex_rd;
    assign dbg_mem_valid  = mem_valid;
    assign dbg_mem_reg_we = mem_reg_we;
    assign dbg_mem_rd     = mem_rd;
    assign dbg_wb_data    = wb_data;
    assign dbg_wb_we      = wb_we;
    assign dbg_wb_rd      = wb_rd;
    assign dbg_pc_set     = pc_set;
    assign retire_pulse   = instret_inc_pulse;
    assign retire_pc      = mem_pc;
    assign dbg_stall_id   = stall_id;
    assign dbg_ex_ready   = ex_ready;



    //==========================================================================
    // ID
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_valid     <= 1'b0;
            id_pc        <= 32'h0;
            id_inst      <= 32'h00000013;
            id_imm       <= 32'h0;
            id_rs1       <= 5'd0;
            id_rs2       <= 5'd0;
            id_rd        <= 5'd0;
            id_alu_op    <= 5'd0;
            id_wb_sel    <= 2'd0;
            id_reg_we    <= 1'b0;
            id_mem_re    <= 1'b0;
            id_mem_we    <= 1'b0;
            id_mem_size  <= 3'd0;
            id_mem_unsigned <= 1'b0;
            id_is_branch <= 1'b0;
            id_br_type   <= 3'd0;
            id_is_jal    <= 1'b0;
            id_is_jalr   <= 1'b0;
            id_is_csr    <= 1'b0;
            id_csr_op    <= 2'd0;
            id_csr_addr  <= 12'd0;
            id_is_amo    <= 1'b0;
            id_amo_op    <= 5'd0;
            id_fence_i   <= 1'b0;
            id_illegal   <= 1'b0;
            id_if_fault  <= 1'b0;
            id_if_fault_cause <= 4'd0;
            id_csr_uimm  <= 1'b0;
            id_is_ecall  <= 1'b0;
            id_is_ebreak <= 1'b0;
            id_is_mret   <= 1'b0;
            id_is_sret   <= 1'b0;
            id_is_wfi    <= 1'b0;
            id_is_sfence <= 1'b0;
            id_rs1_val   <= 32'h0;
            id_rs2_val   <= 32'h0;
        end else if (pc_set) begin
            // ID write back
            id_valid   <= 1'b0;
            id_reg_we  <= 1'b0;
            id_mem_re  <= 1'b0;
            id_mem_we  <= 1'b0;
            id_is_branch <= 1'b0;
            id_is_jal  <= 1'b0;
            id_is_jalr <= 1'b0;
            id_is_csr  <= 1'b0;
            id_is_amo  <= 1'b0;
            id_illegal <= 1'b0;
            id_is_ecall  <= 1'b0;
            id_is_ebreak <= 1'b0;
            id_is_mret   <= 1'b0;
            id_is_sret   <= 1'b0;
        end else if (!stall_id && if_valid) begin
            // only in IF when
            id_valid     <= 1'b1;
            id_pc        <= if_inst_pc;
            id_inst      <= if_rdata;
            id_imm       <= d_imm;
            id_rs1       <= d_use_rs1 ? d_rs1 : 5'd0;
            id_rs2       <= d_use_rs2 ? d_rs2 : 5'd0;
            id_rd        <= d_rd;
            id_alu_op    <= d_alu_op;
            id_wb_sel    <= d_wb_sel;
            id_reg_we    <= d_reg_we && if_valid && !d_illegal;
            id_mem_re    <= d_mem_re && if_valid;
            id_mem_we    <= d_mem_we && if_valid;
            id_mem_size  <= d_mem_size;
            id_mem_unsigned <= d_mem_unsigned;
            id_is_branch <= d_is_branch && if_valid;
            id_br_type   <= d_br_type;
            id_is_jal    <= d_is_jal && if_valid;
            id_is_jalr   <= d_is_jalr && if_valid;
            id_is_csr    <= d_is_csr && if_valid;
            id_csr_op    <= d_csr_op;
            id_csr_addr  <= d_csr_addr;
            id_is_amo    <= d_is_amo && if_valid;
            id_amo_op    <= d_amo_op;
            id_fence_i   <= d_is_fence && if_valid;
            id_illegal   <= d_illegal && if_valid;
            id_if_fault  <= if_fault && if_valid;
            // Only latch a cause when this instruction actually faulted; the
            // fetched cause is meaningless otherwise and would carry an
            // undefined value into mcause.
            if (if_fault)
                id_if_fault_cause <= if_fault_cause;
            id_is_ecall  <= d_is_ecall && if_valid;
            id_is_ebreak <= d_is_ebreak && if_valid;
            id_is_mret   <= d_is_mret && if_valid;
            id_is_sret   <= d_is_sret && if_valid;
            id_is_wfi    <= d_is_wfi && if_valid;
            id_is_sfence <= d_is_sfence_vma && if_valid;
            // id_rs1 before ID before
            id_rs1_val   <= (id_rs1 == 5'd0) ? 32'h0 :
                            (ex_valid && ex_reg_we && (ex_rd != 5'd0) && (ex_rd == id_rs1)) ? ex_alu_y :
                            (mem_fwd_ok && (mem_rd == id_rs1)) ? mem_fwd_data :
                            (wb_fwd_ok  && (wb_rd == id_rs1))  ? wb_data : rf_rs1_val;
            // CSRRSI/CSRRWI/CSRRCI uimm(=d_rs1 ) rs1
            id_csr_wdata <= (d_rs1 == 5'd0) ? 32'h0 : fwd_rs1;
            id_csr_uimm  <= d_is_csr && d_funct3[2];
            id_rs2_val   <= (id_rs2 == 5'd0) ? 32'h0 :
                            (ex_valid && ex_reg_we && (ex_rd != 5'd0) && (ex_rd == id_rs2)) ? ex_alu_y :
                            (mem_fwd_ok && (mem_rd == id_rs2)) ? mem_fwd_data :
                            (wb_fwd_ok  && (wb_rd == id_rs2))  ? wb_data : rf_rs2_val;
        end else if (!stall_id && ex_ready) begin
            // IF → bubble
            // must EX ex_ready=1 only then ID
            id_valid <= 1'b0;
        end
    end

    //==========================================================================
    // EX
    //==========================================================================
    //==========================================================================
    // MEM
    // load: read → rvalid →
    // AMO: MEM when read ex_mem_re → rvalid
    // → → write → awready →
    //==========================================================================
    reg  amo_phase;          // 0= read, 1= write
    reg  mem_is_amo_q;       // MEM before is AMO
    reg  mem_reg_we_q;       // MEM before reg_we
    reg  [31:0] amo_addr_m, amo_rs2_m, amo_old_m;
    reg  [4:0]  amo_op_m;

    // page fault when must not memory access mmu_fault_r fault
    // EX illegal instruction ex_inst and ex_pc
    wire [6:0]  dec_ex_opcode;
    wire [4:0]  dec_ex_rd, dec_ex_rs1, dec_ex_rs2;
    wire [2:0]  dec_ex_funct3;
    wire [6:0]  dec_ex_funct7;
    wire [31:0] dec_ex_imm;
    wire        dec_ex_use_rs1, dec_ex_use_rs2, dec_ex_use_imm;
    wire [4:0]  dec_ex_alu_op;
    wire [1:0]  dec_ex_wb_sel;
    wire        dec_ex_reg_we, dec_ex_mem_re, dec_ex_mem_we;
    wire [2:0]  dec_ex_mem_size;
    wire        dec_ex_mem_unsigned;
    wire        dec_ex_is_branch, dec_ex_is_jal, dec_ex_is_jalr;
    wire [2:0]  dec_ex_br_type;
    wire        dec_ex_is_csr;
    wire [1:0]  dec_ex_csr_op;
    wire [11:0] dec_ex_csr_addr;
    wire        dec_ex_is_amo;
    wire [4:0]  dec_ex_amo_op;
    wire        dec_ex_is_ecall, dec_ex_is_ebreak;
    wire        dec_ex_is_mret, dec_ex_is_sret, dec_ex_is_wfi, dec_ex_is_sfence;

    decoder u_dec_ex (
        .inst(ex_inst),
        .opcode(dec_ex_opcode), .rd(dec_ex_rd), .funct3(dec_ex_funct3),
        .rs1(dec_ex_rs1), .rs2(dec_ex_rs2), .funct7(dec_ex_funct7),
        .imm(dec_ex_imm), .use_rs1(dec_ex_use_rs1), .use_rs2(dec_ex_use_rs2),
        .use_imm(dec_ex_use_imm), .alu_op(dec_ex_alu_op), .wb_sel(dec_ex_wb_sel),
        .reg_we(dec_ex_reg_we), .mem_re(dec_ex_mem_re), .mem_we(dec_ex_mem_we),
        .mem_size(dec_ex_mem_size), .mem_unsigned(dec_ex_mem_unsigned),
        .is_branch(dec_ex_is_branch), .is_jal(dec_ex_is_jal), .is_jalr(dec_ex_is_jalr),
        .br_type(dec_ex_br_type),
        .is_csr(dec_ex_is_csr), .csr_op(dec_ex_csr_op), .csr_addr(dec_ex_csr_addr),
        .is_amo(dec_ex_is_amo), .amo_op(dec_ex_amo_op),
        .is_fence(),
        .is_ecall(dec_ex_is_ecall), .is_ebreak(dec_ex_is_ebreak),
        .is_mret(dec_ex_is_mret), .is_sret(dec_ex_is_sret),
        .is_wfi(dec_ex_is_wfi), .is_sfence_vma(dec_ex_is_sfence),
        .illegal(dec_ex_illegal_raw)
    );

    // privilege level RISC-V
    // sfence.vma / sret / satp M/S U
    // mret M
    // RISC-V privilege level illegal instruction
    // sfence.vma: U S mstatus.TVM=1
    // sret: U S mstatus.TSR=1
    // mret: M
    // satp: U S mstatus.TVM=1
    wire mstatus_tvm = mstatus_int[20];
    wire mstatus_tsr = mstatus_int[22];
    wire ex_prv_illegal =
        (ex_is_sfence && (priv == `PRV_U || (priv == `PRV_S && mstatus_tvm))) ||
        // sret follows the reference model here: it is refused only when
        // executing in S mode with TSR set. The supervisor-mode test vectors
        // return from traps while still in M mode, and the specification would
        // call that illegal, but the model permits it and the vectors rely on
        // that behaviour.
        (ex_is_sret   && (priv == `PRV_S && mstatus_tsr)) ||
        (ex_is_mret   && priv != `PRV_M) ||
        (ex_is_csr && (ex_csr_addr == 12'h180) && (priv == `PRV_U || (priv == `PRV_S && mstatus_tvm))) ||
        // CSR privilege level csr[9:8] privilege level
        // 0=U, 1=S, 2=, 3=M before priv < csr[9:8] when illegal instruction
        // → U read mstatus M CSR rv32mi-p-csr test_15 failure
        (ex_is_csr && ({1'b0, ex_csr_addr[9:8]} > {1'b0, priv})) ||
        // read only CSR csr[11:10] == 2'b11 cycle/time/instret 0xC00~0xC02
        // write must illegal instruction RISC-V
        // " write " CSRRW/CSRRWI is write CSRRS/CSRRC immediate in 0 when only then write
        // write → rv32mi-p-csr test_14 csrrw a0, cycle, zero failure
        (ex_is_csr && ex_csr_addr[11:10] == 2'b11 &&
         (ex_csr_op == 2'b01 || ex_uimm_v != 5'd0));
    // debug CSR 0x7A0~0x7BF illegal instruction
    // rv32mi-p-breakpoint tdata1 read write read write + is
    // FAIL(c=5) and V2 in
    assign ex_illegal = dec_ex_illegal_raw || ex_prv_illegal;

    // !mem_wait must not EX freeze mem_wait
    // cross-word and EX mem_wait
    wire ld_need2_now = (ex_mem_re && !ex_is_amo) &&
                        ((ex_mem_size[1:0] == 2'd0) ? 1'b0 :
                         (ex_mem_size[1:0] == 2'd1) ? (ex_alu_y[1:0] == 2'd3) :
                                                      (ex_alu_y[1:0] != 2'd0));
    // mmu_ready is a registered decision (mmu_done or bypass), so feeding it
    // into ex_ready through this term cannot create a combinational loop.
    wire mem_load_pending = ex_valid && ex_mem_re && !ex_illegal && mmu_ready &&
                            !mmu_fault && !mmu_fault_r && !mem_wait;

    // AMO MEM
    // funct5: 0=amoadd 1=amoswap 2=lr 3=sc 4=amoxor 8=amoor 12=amoand
    //         16=amomin 20=amomax 24=amominu 28=amomaxu
    wire [31:0] amo_new_m =
        (amo_op_m == 5'd0)  ? (amo_old_m + amo_rs2_m) :   // amoadd
        (amo_op_m == 5'd1)  ? amo_rs2_m :                 // amoswap
        (amo_op_m == 5'd3)  ? amo_rs2_m :                 // sc.w write rs2
        (amo_op_m == 5'd4)  ? (amo_old_m ^ amo_rs2_m) :   // amoxor
        (amo_op_m == 5'd8)  ? (amo_old_m | amo_rs2_m) :   // amoor
        (amo_op_m == 5'd12) ? (amo_old_m & amo_rs2_m) :   // amoand
        (amo_op_m == 5'd16) ? (($signed(amo_old_m) < $signed(amo_rs2_m)) ? amo_old_m : amo_rs2_m) :  // amomin
        (amo_op_m == 5'd20) ? (($signed(amo_old_m) > $signed(amo_rs2_m)) ? amo_old_m : amo_rs2_m) :  // amomax
        (amo_op_m == 5'd24) ? ((amo_old_m < amo_rs2_m) ? amo_old_m : amo_rs2_m) :                    // amominu
        (amo_op_m == 5'd28) ? ((amo_old_m > amo_rs2_m) ? amo_old_m : amo_rs2_m) :                    // amomaxu
                               amo_old_m;

    wire is_lr = (amo_op_m == 5'd2);   // lr.w read only no write
    wire is_sc = (amo_op_m == 5'd3);   // sc.w write rs2 rd returns 0 success 1 failure

    // reservation set hart lr sc
    reg        reservation;      // 1 =
    reg [31:0] res_addr;
    reg        mem_first_q;      // mem_wait rvalid
    reg        req_sent_q;       // read arready rvalid
    reg        mem_seen_q;       // mem_wait ld_need2
    // MEM load pc_set when must not write back
    reg        st_pend_q;        // cross-word store after
    reg        st_ph2;           // in write second segment
    reg [31:0] st_a2, st_d2;
    reg [3:0]  st_s2;
    wire       sc_ok = reservation && (res_addr == amo_addr_m);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || pc_set) begin
            mem_wait  <= 1'b0;
            amo_phase <= 1'b0;
            ld_phase2 <= 1'b0;      // must reset at power-up is X dmem_araddr
            ld_ph2_q  <= 2'd0;
            ld_ph2_go <= 1'b0;
            ld_need2  <= 1'b0;
            mem_first_q <= 1'b1;    // rvalid
            req_sent_q  <= 1'b0;
            ld_phase2_d <= 1'b0;
            mem_seen_q  <= 1'b0;
            // Both halves of the load window must be defined from reset: the
            // window is assembled from them on every load, so an undefined
            // half poisons the result even for an aligned access.
            mem_next_word <= 32'h0;
            ld_first_q    <= 32'h0;
            // reservation in reset and AMO when →
            // DRC MDRV-1 multiple drivers is only in AMO
        end else if (mem_wait) begin
            mem_first_q <= 1'b0;              // only then rvalid
            ld_phase2_d <= ld_phase2;         // second segment
            mem_seen_q  <= 1'b1;              // ld_need2
            if (mem_is_amo_q && !amo_phase) begin
                // AMO 0 read returns
                if (dmem_rvalid && !mem_first_q) begin
                    amo_old_m <= dmem_rdata;
                    // lr.w read only no write reservation set
                    // sc.w reservation set only then success write failure
                    if (is_lr) begin
                        mem_wait <= 1'b0;
                    end else if (is_sc && !sc_ok) begin
                        mem_wait <= 1'b0;   // sc failure no write
                    end else begin
                        amo_phase <= 1'b1;      // write
                    end
                end
            end else begin
                // 1 load rvalid / AMO write
                if (mem_is_amo_q) begin
                    if (dmem_awready && dmem_wready) begin
                        mem_wait  <= 1'b0;
                        // mem_valid MEM set MEM AMO
                    end
                end else begin
                    // ═══ Ibex LSU req/gnt/rvalid handshake ═══
                    // req_sent_q: arready Ibex data_gnt_i
                    // ld_phase2: in second segment
                    if (dmem_rvalid && req_sent_q) begin
                        req_sent_q <= 1'b0;
                        if (ld_need2 && !ld_phase2) begin
                            // first segment → when write mem_mem_rdata
                            // second segment
                            ld_first_q    <= dmem_rdata;
                            ld_phase2     <= 1'b1;
                        end else if (ld_need2 && ld_phase2) begin
                            // second segment → = first segment = second segment
                            // mem_mem_rdata / mem_valid / mem_reg_we MEM set
                            mem_next_word <= dmem_rdata;
                            ld_phase2     <= 1'b0;
                            // —— load
                            mem_wait      <= 1'b0;
                        end else begin
                            // MEM
                            mem_wait      <= 1'b0;
                        end
                    end else if (dmem_arvalid && dmem_arready && !req_sent_q) begin
                        req_sent_q <= 1'b1;              // handshake success wait for the response
                    end
                end
            end
        end else if (mem_load_pending) begin
            // mem_valid clear MEM MEM mem_load_pending
            req_sent_q    <= 1'b0;        // a new memory access start over handshake
            ld_phase2_d   <= 1'b0;
            mem_seen_q    <= 1'b0;
            mem_wait    <= 1'b1;
            amo_phase   <= ex_is_amo ? 1'b0 : 1'b1;
            mem_is_amo_q<= ex_is_amo;
            mem_reg_we_q<= ex_reg_we;
            amo_addr_m  <= ex_dmem_pa;
            amo_rs2_m   <= fwd_rs2_ex;
            amo_op_m    <= ex_amo_op;
            // unaligned only then read
            // LB: in 1
            // LH: addr_lo=3 when
            // LW: addr_lo!=0 when
            ld_need2    <= (ex_mem_re && !ex_is_amo) &&
                           ((ex_mem_size[1:0] == 2'd0) ? 1'b0 :
                           (ex_mem_size[1:0] == 2'd1) ? (ex_alu_y[1:0] == 2'd3) :
                                                        (ex_alu_y[1:0] != 2'd0));
            ld_addr2    <= ex_dmem_pa + 32'd4;
            ld_phase2   <= 1'b0;
            mem_next_word <= 32'h0;
        end
    end

    // MEM when freeze EX AMO freeze mem_wait
    // mmu_need is translation must freeze EX
    // is_store in MMU cause LOAD
    // " translation " only then EX TLB hit / translation
    // cross-word store FSM EX before / must hold

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_valid      <= 1'b0;
            ex_pc         <= 32'h0;
            ex_inst       <= 32'h13;
            ex_reg_we     <= 1'b0;
            ex_is_branch  <= 1'b0;
            ex_is_jal     <= 1'b0;
            ex_is_jalr    <= 1'b0;
            ex_is_csr     <= 1'b0;
            ex_is_ecall   <= 1'b0;
            ex_is_ebreak  <= 1'b0;
            ex_is_mret    <= 1'b0;
            ex_is_sret    <= 1'b0;
            ex_is_sfence  <= 1'b0;   // before X ex_prv_illegal
            ex_is_wfi     <= 1'b0;
            ex_mem_re     <= 1'b0;
            ex_mem_we     <= 1'b0;
            ex_csr_uimm   <= 1'b0;
            ex_uimm_v     <= 5'd0;
        end else if (pc_set) begin
            // flush this cycle EX
            // ex_pc and ex_inst must when ILA
            ex_valid      <= 1'b0;
            ex_pc         <= 32'h0;
            ex_inst       <= 32'h13;
            ex_reg_we     <= 1'b0;
            ex_mem_re     <= 1'b0;
            ex_mem_we     <= 1'b0;
            ex_is_branch  <= 1'b0;
            ex_is_jal     <= 1'b0;
            ex_is_jalr    <= 1'b0;
            ex_is_csr     <= 1'b0;
            ex_is_amo     <= 1'b0;
            ex_is_ecall   <= 1'b0;
            ex_is_ebreak  <= 1'b0;
            ex_is_mret    <= 1'b0;
            ex_is_sret    <= 1'b0;
            ex_is_sfence  <= 1'b0;
            ex_is_wfi     <= 1'b0;
            ex_if_fault   <= 1'b0;
            ex_if_fault_cause <= 4'd0;
            ex_inst       <= 32'h13;     // NOP and ex_pc hold
        end else if (ld_done) begin
            // A retiring access vacates the stage unconditionally: ex_ready is
            // low while the access is in flight, so the branch below never
            // runs on this cycle and the stage would keep the same
            // instruction, which mem_load_pending then re-issues.
            ex_valid      <= 1'b0;
            ex_pc         <= 32'h0;
            ex_inst       <= 32'h13;
        end else if (ex_ready) begin
            ex_valid      <= id_valid;
            ex_pc         <= id_pc;
            ex_inst       <= id_inst;   // and ex_pc
            ex_imm        <= id_imm;
            ex_rs1_val    <= (id_rs1 == 5'd0) ? 32'h0 : ex_rs1_rf;   // EX read regfile
            ex_rs2_val    <= (id_rs2 == 5'd0) ? 32'h0 : ex_rs2_rf;   // EX read
            ex_rs1        <= id_rs1;
            ex_rs2        <= id_rs2;
            ex_rd         <= id_rd;
            ex_alu_op     <= id_alu_op;
            ex_wb_sel     <= id_wb_sel;
            ex_reg_we     <= id_reg_we && id_valid && (id_rd != 5'd0);  // bubble /x0 when write back
            ex_mem_re     <= id_mem_re && id_valid;      // bubble when memory access
            ex_mem_we     <= id_mem_we && id_valid;      // bubble when no write
            ex_mem_size   <= id_mem_size;
            ex_mem_unsigned <= id_mem_unsigned;
            ex_is_branch  <= id_is_branch;
            ex_br_type    <= id_br_type;
            ex_is_jal     <= id_is_jal;
            ex_is_jalr    <= id_is_jalr;
            ex_is_csr     <= id_is_csr;
            ex_csr_op     <= id_csr_op;
            ex_csr_addr   <= id_csr_addr;
            ex_is_amo     <= id_is_amo;
            ex_amo_op     <= id_amo_op;
            ex_fence_i    <= id_fence_i;
            ex_is_ecall   <= id_is_ecall;
            ex_is_ebreak  <= id_is_ebreak;
            ex_is_mret    <= id_is_mret;
            ex_is_sret    <= id_is_sret;
            ex_if_fault   <= id_if_fault;
            ex_if_fault_cause <= id_if_fault_cause;
            ex_is_wfi     <= id_is_wfi;
            ex_is_sfence  <= id_is_sfence;
            // ALU ID posedge when
            // MD_DONE PC when muldiv_result
            ex_alu_y      <= (is_muldiv && md_state == MD_DONE && md_pc == id_pc)
                             ? muldiv_result :
                             (id_opcode_r == `OP_AUIPC) ? (id_pc + id_imm) :
                                                          alu_y;
            ex_csr_rdata  <= csr_rdata_ex;
            ex_csr_uimm   <= id_csr_uimm;
            ex_uimm_v     <= id_inst[19:15];
        end
    end

    //==========================================================================
    // MEM
    //==========================================================================
    assign mmu_va       = ex_alu_y;
    assign mmu_is_store = ex_mem_we;
    // Which stage owns the address currently on the walker. When the execute
    // stage has an access in flight the address is a data address; otherwise
    // the walker is serving the fetcher. Hard-wiring this to 0 made every
    // fetch look like a data access, so under MPRV a fetch was translated with
    // the data privilege and faulted.
    // Derived from the instruction in EX, not from mmu_need: mmu_need already
    // depends on the privilege this selects, so using it here would close a
    // combinational loop through ex_mmu_bypass.
    // Only the execute stage knows whether the walker's address is a data
    // address; a registered copy is not needed because nothing downstream of
    // this can feed back into it. The reset term keeps it defined while the
    // execute registers are still unknown.
    wire ex_is_mem = (ex_mem_re | ex_mem_we) === 1'b1;
    // Registered so it is defined from reset. Reading the execute registers
    // directly left this undefined while they were still unknown, and an
    // undefined privilege makes the walker's bypass decision undefined too,
    // which corrupts the physical address.
    reg dmem_active_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            dmem_active_q <= 1'b0;
        else
            // Compare against 1 explicitly: these registers are legitimately
            // unknown while a stage is being flushed, and an unknown here must
            // read as "not a data access" rather than propagate into the
            // privilege handed to the walker.
            dmem_active_q <= (ex_valid === 1'b1) &&
                             ((ex_mem_re === 1'b1) || (ex_mem_we === 1'b1)) &&
                             (ex_illegal !== 1'b1);
    end

    // Exported as its own port: this net is also written from the block that
    // owns the walker in some configurations, and a single driver keeps the
    // value well defined.
    assign mmu_is_fetch  = !dmem_active_q;
    assign dmem_active_o = dmem_active_q;

    //==========================================================================
    // MMU translation
    // MMU walk req after hold hit freeze EX
    //==========================================================================
    reg  [31:0] mmu_pa_r;   // translation
    reg  [31:0] mmu_pc_r;   // PC translation
    reg  [31:0] mmu_va_r;   // virtual address translation
    reg  [3:0]  mmu_fcause_r;

    // MMU satp.MODE=0 or memory access privilege level is M
    // MPRV is exactly the case where an M-mode access must still be
    // translated: it says "use the privilege in MPP for this data access".
    // So bypass has to look at the adjusted privilege, which is why the
    // in-flight flag below is derived from the raw instruction fields and not
    // from this decision.
    // The declaration has to come before the assignment that reads it:
    // Verilog-2001 turns an undeclared identifier into an implicit 1-bit net,
    // so the compare below would silently use the wrong width.
    wire [1:0] dmem_priv_early = mprv_eff ? mstatus_int[12:11] : priv;
    assign ex_mmu_bypass = !satp_o[31] || (dmem_priv_early == `PRV_M);

    //------------------------------------------------------------------
    // Translation handshake.
    //
    // The walk takes an unknown number of cycles, so the request has to be
    // held until the answer arrives and the answer has to be captured in a
    // register the moment it appears. Both are expressed by one state bit:
    //
    //   mmu_need (address valid)  and not yet answered  -> request asserted
    //   answer arrives                                  -> latch, mmu_busy=1
    //   instruction leaves EX                           -> mmu_busy=0
    //
    // mmu_busy is what stalls EX. It is set from the cycle the walk is
    // requested and cleared only when the instruction actually moves on, so
    // there is no window in which the requester drops its request while the
    // walker still considers the walk outstanding.
    //------------------------------------------------------------------
    reg  mmu_busy;      // a translation is outstanding for the current EX instruction
    reg  mmu_done;      // the answer for the current EX instruction is latched

    wire mmu_answer   = mmu_hit || mmu_fault;
    wire mmu_answered = mmu_done || (!mmu_busy && mmu_need && mmu_answer);

    // The request is only for the walk. Once the answer is latched the walk
    // is over, so the request is withdrawn; keeping it up would make the
    // arbiter start a new walk for the same address.
    assign mmu_req = mmu_need && mmu_busy && !mmu_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mmu_busy    <= 1'b0;
            mmu_done    <= 1'b0;
            mmu_pa_r    <= 32'h0;
            mmu_pc_r    <= 32'h0;
            mmu_va_r    <= 32'h0;
            mmu_fault_r <= 1'b0;
            mmu_fcause_r<= 4'd0;
        end else if (pc_set || !mmu_need) begin
            // The instruction is gone (redirect) or needs no translation.
            mmu_busy    <= 1'b0;
            mmu_done    <= 1'b0;
            mmu_fault_r <= 1'b0;
        end else if (!mmu_busy) begin
            // No walk outstanding yet: start one only if this address is not
            // already translated.
            if (!mmu_hit) begin
                mmu_busy <= 1'b1;
                mmu_pc_r <= ex_pc;
                mmu_va_r <= ex_alu_y;
            end else begin
                // TLB hit or bypass: capture the answer straight away.
                mmu_done    <= 1'b1;
                mmu_pa_r    <= mmu_pa;
                mmu_fault_r <= mmu_fault;
                mmu_fcause_r<= mmu_fault_cause;
                mmu_pc_r    <= ex_pc;
                mmu_va_r    <= ex_alu_y;
            end
        end else if (!mmu_done && mmu_answer) begin
            // The walk finished: latch the answer. mmu_busy stays set so the
            // request remains asserted until this instruction leaves EX.
            mmu_done    <= 1'b1;
            mmu_pa_r    <= mmu_pa;
            mmu_fault_r <= mmu_fault;
            mmu_fcause_r<= mmu_fault_cause;
        end
    end

    // The physical address is the latched one as soon as it exists; before
    // that the virtual address passes through, which is what bypass needs.
    assign ex_dmem_pa = mmu_done ? mmu_pa_r : (mmu_busy ? mmu_pa_r : ex_alu_y);

    // unaligned load read

    //==========================================================================
    // dmem
    // - load/store EX
    // - AMO 0 amo_addr_m read 1 amo_addr_m write amo_new_m
    //==========================================================================
    // read 4 AXI from wstrb/
    assign dmem_araddr  = ld_phase2 ? {ld_addr2[31:2], 2'b00} :   // second segment
                          (mem_wait ? {amo_addr_m[31:2], 2'b00} : {ex_dmem_pa[31:2], 2'b00});
    // mem_wait must hold arvalid drop rvalid
    // sim_mem and → mem_wait
    // Ibex only in " the previous has completed " when only then
    // second segment ld_phase2 set
    assign dmem_arvalid = mem_wait ? (mem_is_amo_q ? !amo_phase : (!req_sent_q && ld_ph2_ready)) :
                          (ex_valid && ex_mem_re && !ex_illegal && mmu_ready && !mmu_fault && !mmu_fault_r);
    assign dmem_rready  = 1'b1;
    // AXI {bit2= (S/M), bit1= (0), bit0= (0)}
    assign imem_rready  = 1'b1;   // FIFO/epoch
    assign dmem_arprot  = {1'b0, 1'b0, (mmu_priv != `PRV_U)};
    assign dmem_awprot  = {1'b0, 1'b0, (mmu_priv != `PRV_U)};

    // store addr_lo and
    // SB: addr_lo →
    // SH: addr_lo=0 → 0x3, addr_lo=2 → 0xC unaligned addr_lo=1/3
    // SW: 4
    // store and
    // first segment = in from addr_lo
    wire [2:0] st_nbytes = (ex_mem_size == 3'd0) ? 3'd1 :
                           (ex_mem_size == 3'd1) ? 3'd2 : 3'd4;
    wire [2:0] st_lo3 = {1'b0, ex_alu_y[1:0]};
    wire [2:0] st_avail = 3'd4 - st_lo3;                       // bytes left in this word
    // write = min(nbytes, avail)
    wire [2:0] st_seg_bytes = (st_nbytes > st_avail) ? st_avail : st_nbytes;
    // = from addr_lo st_seg_bytes
    wire [3:0] st_strb_calc = (4'b1111 >> (3'd4 - st_seg_bytes)) << st_lo3;
    wire       st_need2 = (st_nbytes > st_avail);              // cross-word

    assign dmem_awaddr  = st_pend_q ? st_a2 :
                          (mem_wait ? {amo_addr_m[31:2], 2'b00} : {ex_dmem_pa[31:2], 2'b00});
    // Ibex PH2 second segment IDLE is first segment only then first segment
    assign dmem_awvalid = (mem_wait && mem_is_amo_q && amo_phase && !is_lr) ? 1'b1 :
                          (mem_wait ? 1'b0 :
                          (st_state == 2'd1) ||
                          (st_state == 2'd0 && ex_valid && ex_mem_we && !ex_illegal &&
                           mmu_ready && !mmu_fault && !mmu_fault_r));
    assign dmem_wdata   = st_pend_q ? st_d2 :
                          (mem_wait && mem_is_amo_q && amo_phase) ? amo_new_m :
                          (fwd_rs2_ex << (ex_alu_y[1:0] * 8));
    // unaligned 1 write addr_lo
    assign dmem_wstrb   = st_pend_q ? st_s2 :
                          (mem_wait && mem_is_amo_q) ? 4'b1111 : st_strb_calc;
    assign dmem_wvalid  = (mem_wait && mem_is_amo_q && amo_phase && !is_lr) ? 1'b1 :
                          (mem_wait ? 1'b0 :
                          (ex_valid && ex_mem_we && !ex_illegal && mmu_ready && !mmu_fault && !mmu_fault_r) || st_pend_q);

    //==========================================================================
    // memory access addr_lo / /
    // only in memory access MEM mem_load_pending
    // no write → remove
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_addr_lo      <= 2'd0;
            mem_mem_size     <= 3'd2;
            mem_mem_unsigned <= 1'b0;
        end else if (mem_load_pending) begin
            mem_addr_lo      <= ex_alu_y[1:0];
            mem_mem_size     <= ex_mem_size;
            mem_mem_unsigned <= ex_mem_unsigned;
        end
    end

    // cross-word store second segment state machine
    // first segment before ex_* handshake success → st_pend_q write second segment →
    reg st_first_done;
    // cross-word store FSM Ibex IDLE → PH2 → IDLE
    // st_state: 0=IDLE 1=PH2 first segment this cycle second segment 2=DONE EX before
    // st_busy in FSM is 1 → ex_ready=0 EX before
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || pc_set) begin
            st_state      <= 2'd0;
            st_first_done <= 1'b0;
            st_wait_q     <= 1'b0;
        end else case (st_state)
            2'd0: begin
                // IDLE first segment handshake → PH2 this cycle first segment in handshake
                if (ex_valid && ex_mem_we && !ex_is_amo && st_need2 && !ex_illegal &&
                    mmu_ready && !mmu_fault && !mmu_fault_r && dmem_awready && dmem_wready) begin
                    st_state      <= 2'd1;
                    st_a2         <= {ex_dmem_pa[31:2], 2'b00} + 32'd4;
                    st_d2         <= fwd_rs2_ex >> ((4 - ex_alu_y[1:0]) * 8);
                    st_s2         <= 4'b1111 >> (4 - (st_nbytes - st_avail));
                    st_first_done <= 1'b1;
                end
            end
            2'd1: begin
                // PH2 this cycle in second segment handshake DONE
                if (dmem_awready && dmem_wready) st_state <= 2'd2;
            end
            2'd2: begin
                // DONE second segment EX ex_ready → IDLE
                st_state      <= 2'd0;
                st_first_done <= 1'b0;
            end
        endcase

        // store write MIG write only then EX
        if (!rst_n) begin
            st_wait_q   <= 1'b0;
            st_retire_q <= 1'b0;
        end else if (pc_set) begin
            st_wait_q   <= 1'b0;
            st_retire_q <= 1'b0;
        end else if (st_state == 2'd0) begin
            if (ex_valid && ex_mem_we && !ex_is_amo && !st_need2 && !ex_illegal &&
                mmu_ready && !mmu_fault && !mmu_fault_r &&
                !st_wait_q && dmem_awready && dmem_wready) begin
                st_wait_q <= 1'b1;          // bvalid
            end else if (st_wait_q && dmem_bvalid) begin
                st_wait_q <= 1'b0;          // write
            end
            st_retire_q <= st_wait_q && dmem_bvalid;
        end
    end

    // st_pend_q FSM is 1 awvalid second segment
    always @(*) st_pend_q = (st_state != 2'd0);

    always @(posedge clk or negedge rst_n) begin
        // mem_valid set
        mem_valid <= 1'b0;
        if (mem_valid) mem_is_settled <= 1'b1;   // write back before
        if (!rst_n) begin
            mem_reg_we     <= 1'b0;
            mem_exc_valid  <= 1'b0;
            mem_exc_cause  <= 4'd0;
            mem_exc_tval   <= 32'h0;
            mem_pc         <= 32'h0;
            mem_alu_y      <= 32'h0;
            mem_mem_rdata  <= 32'h0;
            mem_rd         <= 5'd0;
            mem_wb_sel     <= 2'd0;
            mem_is_csr     <= 1'b0;
            mem_csr_rdata  <= 32'h0;
            mem_illegal    <= 1'b0;
            mem_csr_addr_r <= 12'd0;
            mem_csr_op_r   <= 2'd0;
            mem_csr_wdata_r<= 32'h0;
            mem_rs1_zero   <= 1'b0;
            mem_csr_uimm_r <= 1'b0;
            reservation    <= 1'b0;   // reset
            res_addr       <= 32'h0;
        end else if (pc_set) begin
            // flush EX jal/branch/ MEM write back
            // after
            // illegal instruction M sret pc_set must flush
            if (ex_valid && (ex_illegal || ex_if_fault)) begin
                // A fetch page fault is an exception like any other: the
                // instruction that could not be fetched is still in EX, so a
                // redirect in the same cycle must not discard its fault.
                mem_exc_valid <= 1'b1;
                mem_exc_cause <= ex_if_fault ? ex_if_fault_cause : `EXC_ILLEGAL_INST;
                mem_exc_tval  <= ex_if_fault ? ex_pc : ex_inst;
                mem_exc_pc    <= ex_pc;
            end else begin
                mem_exc_valid <= 1'b0;
                mem_exc_cause <= 4'd0;
                mem_exc_tval  <= 32'h0;
            end
            // A load that is still waiting owns the MEM slot: its pc, rd and
            // write controls describe the data that will arrive shortly, so a
            // redirect must not overwrite them with the instruction now in EX.
            if (ex_valid && !mem_wait) begin
                mem_pc        <= ex_pc;
                mem_alu_y     <= ex_alu_y;
                mem_rd        <= ex_rd;
                mem_wb_sel    <= ex_wb_sel;
                mem_reg_we    <= ex_reg_we;
                mem_is_csr    <= ex_is_csr;
                mem_csr_rdata <= ex_csr_rdata;
                mem_csr_addr_r<= ex_csr_addr;
                mem_csr_op_r  <= ex_csr_op;
                mem_csr_wdata_r<= ex_csr_wdata;
                mem_csr_uimm_r<= ex_csr_uimm;
                mem_rs1_zero  <= (ex_rs1 == 5'd0);
                mem_csr_src_idx<= ex_rs1;
                mem_illegal   <= ex_illegal;
                    // MEM load when must not write back
                    // MEM load or this cycle in → must not write back
                    mem_valid     <= (ex_mem_re || ex_mem_we || ex_is_amo ||
                                      mem_wait || mem_load_pending) ? 1'b0 : 1'b1;
            end
        end else if (mem_wait && !mem_is_amo_q && ld_need2 && ld_phase2 && dmem_rvalid && req_sent_q) begin
            // cross-word load second segment in LSU
            // = first segment ld_first_q = mem_next_word LSU
            // load mem_seen_q
            mem_mem_rdata <= ld_first_q;
            mem_valid     <= 1'b1;
            mem_reg_we    <= mem_reg_we_q;
        end else if (mem_wait && mem_is_amo_q) begin
            // AMO
            if (!amo_phase) begin
                // 0 read →
                if (dmem_rvalid) begin
                    // sc.w success returns 0 failure returns 1 lr/ AMO returns
                    mem_mem_rdata <= is_sc ? (sc_ok ? 32'h0 : 32'h1) : dmem_rdata;
                    mem_valid     <= 1'b1;
                    mem_reg_we    <= 1'b1;
                    // reservation set
                    if (is_lr) begin
                        reservation <= 1'b1;
                        res_addr    <= amo_addr_m;
                    end else if (is_sc) begin
                        reservation <= 1'b0;   // sc
                    end
                end
            end else begin
                // 1 write
                if (dmem_awready && dmem_wready) begin
                    mem_valid  <= 1'b1;
                    mem_reg_we <= 1'b1;
                end
            end
        end else if (mem_wait && !ld_need2 && !ld_phase2 && mem_seen_q) begin
            // load write back cross-word load state machine
            if (dmem_rvalid) begin
                mem_mem_rdata <= dmem_rdata;
                mem_valid     <= 1'b1;
                mem_reg_we    <= mem_reg_we_q;
            end
        end else begin
            // MEM
            mem_pc        <= ex_pc;
            mem_alu_y     <= ex_alu_y;
            // load dmem_rvalid
            // memory access MEM when must not mem_valid must dmem
            // memory access MEM mem_valid only then write back
            if (mem_load_pending) mem_valid <= 1'b0;
            mem_mem_rdata <= mem_load_pending ? mem_mem_rdata : dmem_rdata;
            mem_rd        <= ex_rd;
            mem_wb_sel    <= ex_wb_sel;
            mem_reg_we    <= ex_reg_we;
            mem_reg_we    <= ex_reg_we;
            mem_is_csr    <= ex_is_csr;
            mem_csr_rdata <= ex_csr_rdata;   // EX read csr_rdata in MEM
            mem_illegal   <= ex_illegal;
            // memory access MEM when must not mem_valid must dmem
            // MMU hit (pa=0) when read write back →
            mem_valid     <= ex_valid && !(ex_mem_re || ex_mem_we || ex_is_amo);

            mem_csr_addr_r  <= ex_csr_addr;
            mem_csr_op_r    <= ex_csr_op;        // before mem_csr_we is X
            mem_csr_op_r    <= ex_csr_op;
            // MEM read regfile when before write back
            mem_csr_wdata_r <= ex_csr_wdata;
            mem_rs1_zero    <= (ex_rs1 == 5'd0);
            mem_csr_uimm_r  <= ex_csr_uimm;
            mem_csr_src_idx <= ex_rs1;

            // only in " MEM"
            // mem_exc_valid pc_set clear set
            // mmu_fault memory access memory access
            mem_exc_valid <= ex_valid &&
                             (ex_is_ecall || ex_is_ebreak || ex_illegal || ex_if_fault ||
                              jm_fault || mmu_fault_r ||
                              (mmu_fault && (ex_mem_re || ex_mem_we)));
            mem_exc_cause <= jm_fault                    ? `EXC_INST_MISALIGN :
                             ex_if_fault                 ? ex_if_fault_cause :
                             ex_illegal                  ? `EXC_ILLEGAL_INST :
                             ex_is_ebreak                ? `EXC_BREAKPOINT :
                             ex_is_ecall                 ? (priv == `PRV_U ? `EXC_ECALL_U :
                                                            priv == `PRV_S ? `EXC_ECALL_S : `EXC_ECALL_M) :
                                                           mmu_fault_cause;
            // RISC-V
            // illegal instruction → mtval =
            // /ecall → mtval = 0 ebreak is PC 0
            // unaligned → mtval =
            // page fault → mtval = virtual address
            mem_exc_tval  <= jm_fault     ? jm_target :
                             mmu_fault_r  ? mmu_va_r :
                             mmu_fault    ? ex_alu_y :
                             ex_illegal   ? ex_inst :
                             ex_is_ebreak ? ex_pc :
                                            32'h0;
            // PC MMU page fault PC fault ex_pc before
            // TLB hit fault before ex_pc
            // PTW walk fault mmu_pc_r
            // ex_pc MEM ecall/ebreak/illegal in MEM
            // when mem_pc PC must ex_pc
            mem_exc_pc    <= mmu_fault_r ? mmu_pc_r :
                             mmu_fault   ? ex_pc : ex_pc;
        end
    end

    //==========================================================================
    // WB write back
    //==========================================================================
    // load + sign-extend
    // unaligned mem_mem_rdata is mem_next_word is
    // 64 addr_lo +
    // second segment → dmem_rdata + ld_first_q
    wire [63:0] load_window = {ld_hi_word, ld_lo_word};
    wire [4:0]  load_shift  = {mem_addr_lo, 3'b000};   // addr_lo * 8
    // debug mem_addr_lo when

    assign mem_load_data =
        (mem_mem_size[1:0] == 2'd0) ?                                   // LB / LBU
            (mem_mem_unsigned ? {24'd0,  load_window[load_shift +: 8]}
                              : {{24{load_window[load_shift + 7]}},
                                          load_window[load_shift +: 8]}) :
        (mem_mem_size[1:0] == 2'd1) ?                                   // LH / LHU
            (mem_mem_unsigned ? {16'd0,  load_window[load_shift +: 16]}
                              : {{16{load_window[load_shift + 15]}},
                                          load_window[load_shift +: 16]}) :
                                                                        // LW
            load_window[load_shift +: 32];

    assign wb_alu_result = (mem_wb_sel == `WB_MEM) ? mem_load_data :
                           (mem_wb_sel == `WB_PC4) ? (mem_pc + 32'd4) :
                           (mem_wb_sel == `WB_CSR) ? mem_csr_rdata :
                                                      mem_alu_y;
    // cross-word load second segment
    // cross-word load second segment write back mem_valid
    assign wb_we   = (mem_valid || ld_second_pulse) && (mem_reg_we || ld_second_pulse) && !mem_exc_valid;
    assign wb_rd   = mem_rd;
    assign wb_data = wb_alu_result;

    //==========================================================================
    // IDLE → WAIT → DONE state machine
    //
    // mul_div valid is and busy
    // valid when DONE hold EX
    //==========================================================================

    reg        muldiv_start;


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || pc_set) begin
            md_state      <= MD_IDLE;
            muldiv_start  <= 1'b0;
            muldiv_result <= 32'h0;
            md_pc         <= 32'hFFFF_FFFF;
        end else begin
            muldiv_start <= 1'b0;

            case (md_state)
            MD_IDLE: begin
                if (id_valid && is_muldiv) begin
                    muldiv_start <= 1'b1;
                    md_pc        <= id_pc;
                    md_state     <= MD_WAIT;
                end
            end
            MD_WAIT: begin
                // mul_div valid → DONE
                if (muldiv_valid) begin
                    muldiv_result <= muldiv_y;
                    md_state      <= MD_DONE;
                end
            end
            MD_DONE: begin
                // hold EX
                if (md_take) begin
                    md_state <= MD_IDLE;
                end
            end
            default: md_state <= MD_IDLE;
            endcase
        end
    end

    // busy ID is MD_DONE PC only then
    assign muldiv_busy = id_valid && is_muldiv &&
                       (md_state != MD_DONE || md_pc != id_pc);

    //==========================================================================
    // mret/sret
    //==========================================================================
    reg mret_r, sret_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mret_r <= 1'b0;
            sret_r <= 1'b0;
        end else begin
            mret_r <= ex_valid && ex_is_mret && !pc_set;
            sret_r <= ex_valid && ex_is_sret && !pc_set;
        end
    end
    assign mret_pulse = ex_is_mret && ex_valid;
    assign sret_pulse = ex_is_sret && ex_valid;

    //==========================================================================

    //==========================================================================
    reg [63:0] cyc, instret;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cyc     <= 64'h0;
            instret <= 64'h0;
        end else begin
            cyc <= cyc + 64'h1;
            if (mem_valid && !mem_exc_valid) instret <= instret + 64'h1;
        end
    end
    assign cycle_cnt   = cyc;
    assign instret_cnt = instret;
    assign halted      = 1'b0;
    assign fetch_redirect = pc_set;

    // debug
    wire [31:0] regfile_gp_dbg = u_regfile.regs[3];

endmodule
