`timescale 1ns / 1ps
//=============================================================================
// tb_isa.v
//
// Runs one case from the official riscv-tests suite.
//
// The suite is written against a memory map where the program starts at
// 0x8000_0000 and reports through a word at 0x8000_1000: a test passes when it
// writes 1 there, and reports (test_number << 1) | 1 on a failure.
//
// No cache, no MMU, no boot ROM: instruction and data memory are two arrays.
//=============================================================================
module tb_isa;

    parameter HEXFILE    = "rv32ui-p-add.hex";
    // With USE_MMU the bench instantiates the real Sv32 unit and routes fetch
    // through it; without it the virtual address is used directly, which is
    // what the -p tests need.
    parameter USE_MMU    = 1'b0;
    parameter PTDUMP     = 1'b0;
    parameter MAX_CYCLE  = 2000000;
    parameter MEM_WORDS  = 262144;      // 1 MiB, enough for the test images

    //------------------------------------------------------------------
    // Clock and reset
    //------------------------------------------------------------------
    reg clk = 1'b0;
    reg rst_n = 1'b0;

    always #5 clk = ~clk;               // 100 MHz

    //------------------------------------------------------------------
    // Memory: one flat array addressed from 0x8000_0000
    //------------------------------------------------------------------
    reg [31:0] mem [0:MEM_WORDS-1];

    parameter MEM_BASE = 32'h8000_0000;

    // The tohost address is fixed by each case's linker script, so the runner
    // passes it in rather than relying on a default.
    reg [31:0] tohost_addr;
    initial begin
        if (!$value$plusargs("TOHOST=%h", tohost_addr))
            tohost_addr = 32'h8000_1000;
    end

    integer i;
    reg [1023:0] hexfile;
    initial begin
        for (i = 0; i < MEM_WORDS; i = i + 1)
            // Zero-filled, matching the reference model: memory that was never
            // written reads as 0. Filling with NOP instead made unwritten page
            // table entries look like valid non-leaf entries and sent the test
            // down a path the model never takes.
            mem[i] = 32'h00000000;
        if (!$value$plusargs("HEXFILE=%s", hexfile))
            hexfile = HEXFILE;
        $readmemh(hexfile, mem);
    end

    // Index inside the array for a byte address.
    function [31:0] idx;
        input [31:0] addr;
        begin
            idx = (addr - MEM_BASE) >> 2;
        end
    endfunction

    //------------------------------------------------------------------
    // Core
    //------------------------------------------------------------------
    wire [31:0] imem_addr;
    wire [2:0]  imem_prot;
    wire        imem_req, imem_ack;
    wire [31:0] imem_rdata;
    wire [1:0]  imem_rresp;
    wire        imem_rvalid, imem_rready;

    wire [31:0] dmem_araddr, dmem_awaddr, dmem_wdata;
    wire [2:0]  dmem_arprot, dmem_awprot;
    wire        dmem_arvalid, dmem_arready;
    wire        dmem_awvalid, dmem_awready;
    wire [31:0] dmem_rdata;
    wire [1:0]  dmem_rresp;
    wire        dmem_rvalid, dmem_rready;
    wire [3:0]  dmem_wstrb;
    wire        dmem_wvalid, dmem_wready;
    wire [1:0]  dmem_bresp;
    wire        dmem_bvalid, dmem_bready;

    wire [31:0] mmu_va, mmu_pa;
    wire        mmu_req, mmu_is_store, mmu_is_fetch;
    wire [1:0]  mmu_priv, mmu_fetch_priv;
    wire        dmem_active_w;
    wire [31:0] satp_o;
    wire        sum_o, mxr_o, sfence_o, mmu_fault_ack_o;
    wire        mmu_hit, mmu_fault;
    wire [3:0]  mmu_fault_cause;

    wire [63:0] instret_dbg, cycle_dbg;
    wire        dbg_wb_we;
    wire [4:0]  dbg_wb_rd;
    wire [31:0] dbg_wb_data;

    riscv_core #(.RESET_PC_V(32'h8000_0000)) u_core (
        .clk            (clk),
        .rst_n          (rst_n),

        .imem_addr      (imem_addr),
        .imem_prot      (imem_prot),
        .imem_req       (imem_req),
        .imem_ack       (imem_ack),
        .imem_rdata     (imem_rdata),
        .imem_rresp     (imem_rresp),
        .imem_rvalid    (imem_rvalid),
        .imem_rready    (imem_rready),

        .dmem_araddr    (dmem_araddr),
        .dmem_arprot    (dmem_arprot),
        .dmem_arvalid   (dmem_arvalid),
        .dmem_arready   (dmem_arready),
        .dmem_rdata     (dmem_rdata),
        .dmem_rresp     (dmem_rresp),
        .dmem_rvalid    (dmem_rvalid),
        .dmem_rready    (dmem_rready),

        .dmem_awaddr    (dmem_awaddr),
        .dmem_awprot    (dmem_awprot),
        .dmem_awvalid   (dmem_awvalid),
        .dmem_awready   (dmem_awready),
        .dmem_wdata     (dmem_wdata),
        .dmem_wstrb     (dmem_wstrb),
        .dmem_wvalid    (dmem_wvalid),
        .dmem_wready    (dmem_wready),
        .dmem_bresp     (dmem_bresp),
        .dmem_bvalid    (dmem_bvalid),
        .dmem_bready    (dmem_bready),

        .mmu_va         (mmu_va),
        .mmu_req        (mmu_req),
        .mmu_is_store   (mmu_is_store),
        .mmu_is_fetch   (mmu_is_fetch),
        .mmu_priv       (mmu_priv),
        .mmu_fetch_priv (mmu_fetch_priv),
        .dmem_active_o  (dmem_active_w),
        .satp_o         (satp_o),
        .sum_o          (sum_o),
        .mxr_o          (mxr_o),
        .sfence_o       (sfence_o),
        .mmu_fault_ack_o(mmu_fault_ack_o),
        .mmu_pa         (mmu_pa),
        .mmu_hit        (mmu_hit),
        .mmu_fault      (mmu_fault),
        .mmu_fault_cause(mmu_fault_cause),
        .if_mmu_stall   (if_mmu_stall),
        .if_mmu_fault   (if_mmu_fault),
        .if_mmu_fault_cause(if_mmu_fault_cause),

        .instret_cnt    (instret_dbg),
        .cycle_cnt      (cycle_dbg),
        .dbg_wb_we      (dbg_wb_we),
        .dbg_wb_rd      (dbg_wb_rd),
        .dbg_wb_data    (dbg_wb_data),
        .dbg_ifpc       (if_pc_next_w),
        .imem_want      (imem_want),
        .retire_pulse   (retire_pulse),
        .retire_pc      (retire_pc),
        .dbg_idpc       (dbg_idpc),
        .dbg_inst       (dbg_inst),
        .cur_priv       (cur_priv),
        .dbg_pc_set     (dbg_pc_set),
        .dbg_ex_pc      (dbg_ex_pc),
        .dbg_ex_valid   (dbg_ex_valid),
        .dbg_sepc       (dbg_sepc),
        .dbg_ex_sret    (dbg_ex_sret),
        .dbg_ex_inst    (dbg_ex_inst),
        .dbg_mcause     (mcause_w),
        .dbg_mepc       (mepc_w)
    );

    //------------------------------------------------------------------
    // Translation
    //------------------------------------------------------------------
    wire [31:0] imem_pa, if_pc_next_w, mcause_w, mepc_w;
    wire        imem_want;
    wire        retire_pulse;
    wire [31:0] retire_pc;
    wire        imem_rvalid_bus, imem_ack_fault, imem_rv_fault;
    wire [31:0] dbg_idpc, dbg_inst;
    wire [1:0]  cur_priv;
    wire        dbg_pc_set;
    wire [31:0] dbg_ex_pc, dbg_ex_inst;
    wire        dbg_ex_valid;
    wire [31:0] dbg_sepc;
    wire        dbg_ex_sret;
    wire        imem_hit, imem_fault, if_mmu_stall, if_mmu_fault;
    wire [3:0]  if_mmu_fault_cause;
    wire [31:0] arb_va, arb_pa;
    wire        arb_req, arb_is_store, arb_is_fetch, arb_hit, arb_fault;
    wire [3:0]  arb_fcause, mmu_fcause;
    wire        arb_if_busy;
    wire [31:0] pt_addr, pt_waddr, pt_wdata, pt_rdata;
    wire        pt_req, pt_we, pt_ready;

    generate if (USE_MMU) begin : g_mmu
        mmu_arb u_arb (
            .clk(clk), .rst_n(rst_n),
            .if_va(imem_addr), .if_req(imem_want),
            .if_pa(imem_pa), .if_hit(imem_hit),
            .if_fault(imem_fault), .if_fault_cause(arb_fcause),
            .dm_va(mmu_va), .dm_req(mmu_req), .dm_is_store(mmu_is_store),
            .dm_pa(mmu_pa), .dm_hit(mmu_hit),
            .dm_fault(mmu_fault), .dm_fault_cause(mmu_fault_cause),
            .va(arb_va), .req(arb_req), .is_store(arb_is_store),
            .is_fetch(arb_is_fetch), .pa(arb_pa), .hit(arb_hit),
            .fault(arb_fault), .fault_cause(mmu_fcause),
            .if_busy(arb_if_busy)
        );
        mmu_sv32 u_mmu (
            .clk(clk), .rst_n(rst_n),
            .va(arb_va), .req(arb_req), .is_store(arb_is_store),
            .is_fetch(mmu_is_fetch), .priv(arb_is_fetch ? mmu_fetch_priv : mmu_priv), 
            .pa(arb_pa), .hit(arb_hit), .fault(arb_fault),
            .fault_cause(mmu_fcause),
            .satp(satp_o), .satp_mode(), .sum(sum_o), .mxr(mxr_o),
            .pt_addr(pt_addr), .pt_req(pt_req), .pt_rdata(pt_rdata),
            .pt_ready(pt_ready), .pt_waddr(pt_waddr), .pt_wdata(pt_wdata),
            .pt_we(pt_we), .fault_ack(mmu_fault_ack_o),
            .sfence_vma(sfence_o), .sfence_va_en(1'b0), .sfence_va(32'h0)
        );
        assign if_mmu_stall      = satp_o[31] && (arb_if_busy || (imem_req && !imem_hit));
        assign if_mmu_fault      = imem_fault & imem_hit;
        assign if_mmu_fault_cause= arb_fcause;

        // A faulted fetch is answered locally so the fetcher records the fault
        // against the right pc instead of consuming an untranslated address.
        assign imem_ack_fault    = (satp_o[31] && imem_hit && imem_fault) ? 1'b1 : imem_ack_bus;
        assign imem_rv_fault     = (satp_o[31] && imem_hit && imem_fault) ? 1'b1 : imem_rvalid_bus;

        // Page table reads: the walker asks for a physical address and expects
        // one answer. Modelling it here is what lets the translation complete
        // in simulation at all.
        reg        pt_ready_q;
        reg [31:0] pt_rdata_q;
        reg        pt_pending;

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                pt_ready_q <= 1'b0;
                pt_rdata_q <= 32'h0;
                pt_pending <= 1'b0;
            end else begin
                pt_ready_q <= 1'b0;
                if (pt_req && !pt_pending) begin
                    pt_rdata_q <= mem[idx(pt_addr)];
                    if (PTDUMP)
                        $display("PTE addr=0x%08x idx=0x%0x data=0x%08x",
                                 pt_addr, idx(pt_addr), mem[idx(pt_addr)]);
                    pt_pending <= 1'b1;
                end else if (pt_pending) begin
                    pt_ready_q <= 1'b1;
                    pt_pending <= 1'b0;
                end
                if (pt_we)
                    mem[idx(pt_waddr)] <= pt_wdata;
            end
        end

        assign pt_rdata = pt_rdata_q;
        assign pt_ready = pt_ready_q;
    end else begin : g_bare
        assign mmu_pa    = mmu_va;
        assign mmu_hit   = 1'b1;
        assign mmu_fault = 1'b0;
        assign mmu_fault_cause = 4'd0;
        assign imem_pa   = imem_addr;
        assign imem_hit  = 1'b1;
        assign imem_fault= 1'b0;
        assign if_mmu_stall = 1'b0;
        assign if_mmu_fault = 1'b0;
        assign if_mmu_fault_cause = 4'd0;
        // The bare configuration has no page tables to read.
        assign pt_rdata = 32'h0;
        assign pt_ready = 1'b0;
    end endgenerate

    //------------------------------------------------------------------
    // Instruction memory
    //------------------------------------------------------------------
    reg        imem_req_d;
    reg [31:0] imem_rdata_q;

    // The address is captured into a register on the request cycle, so the
    // data that comes back is guaranteed to belong to the request that was
    // issued, not to whatever the address bus shows a cycle later.
    reg [31:0] imem_addr_q;

    // The model's memory is the DDR image, addressed from its base. A
    // translated fetch comes back as a physical offset, so it is rebased here
    // the same way the top level rebases it for the decoder.
    wire mmu_active_b = satp_o[31];
    wire [31:0] imem_pa_fetch = (mmu_active_b && imem_pa[31:28] != 4'h8)
                                ? (imem_pa | 32'h8000_0000) : imem_pa;

    // Memory answers one cycle after it is asked. With translation on, the
    // answer must also wait for the address to be translated, so the request
    // that asked for it is remembered until the translation lands.
    reg imem_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imem_req_d   <= 1'b0;
            imem_rdata_q <= 32'd0;
            imem_addr_q  <= 32'd0;
            imem_pending <= 1'b0;
        end else begin
            imem_req_d <= 1'b0;
            if (imem_req && (!USE_MMU || !satp_o[31] || imem_hit)) begin
                imem_addr_q  <= imem_pa;
                imem_rdata_q <= mem[idx(imem_pa_fetch)];
                imem_req_d   <= 1'b1;
                imem_pending <= 1'b0;
            end else if (imem_req && USE_MMU && satp_o[31]) begin
                imem_pending <= 1'b1;
            end
            if (imem_pending && imem_hit) begin
                imem_addr_q  <= imem_pa;
                imem_rdata_q <= mem[idx(imem_pa_fetch)];
                imem_req_d   <= 1'b1;
                imem_pending <= 1'b0;
            end
        end
    end

    // In the translated configuration a fetch is only acknowledged once its
    // address has been translated; acknowledging on the request cycle would
    // make the fetcher consume whatever the memory returned for the untranslated
    // address.
    // An untranslatable fetch is acknowledged with a fault rather than data:
    // the fetcher records the fault alongside the instruction so the pipeline
    // can take a page-fault trap for the right pc.
    wire imem_ack_bus = (USE_MMU && satp_o[31]) ? (imem_req && imem_hit) : imem_req;
    assign imem_ack    = USE_MMU ? imem_ack_fault : imem_ack_bus;
    assign imem_rvalid_bus = imem_req_d;
    assign imem_rvalid = USE_MMU ? imem_rv_fault : imem_rvalid_bus;
    assign imem_rdata  = imem_rdata_q;
    assign imem_rresp  = 2'b00;

    //------------------------------------------------------------------
    // Data memory
    //------------------------------------------------------------------
    reg        dmem_ar_d;
    reg [31:0] dmem_rdata_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dmem_ar_d    <= 1'b0;
            dmem_rdata_q <= 32'd0;
        end else begin
            dmem_ar_d <= dmem_arvalid;
            if (dmem_arvalid)
                dmem_rdata_q <= mem[idx(dmem_araddr)];
        end
    end

    assign dmem_arready = 1'b1;
    assign dmem_rvalid  = dmem_ar_d;
    assign dmem_rdata   = dmem_rdata_q;
    assign dmem_rresp   = 2'b00;

    // The write side follows the AXI handshake: a write is taken once, when
    // both channels are valid, and the response is held until it is accepted.
    // Driving bvalid straight off the request lines let a request that stayed
    // asserted produce a fresh response every cycle.
    reg dmem_bvalid_q;
    reg dmem_aw_taken;
    reg [31:0] dmem_awaddr_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dmem_bvalid_q <= 1'b0;
            dmem_aw_taken <= 1'b0;
        end else begin
            // A write is accepted on the handshake. The "taken" flag only
            // suppresses a second acceptance of the same request, so it is
            // cleared when either side drops, not only when the request does:
            // a back-to-back pair of writes keeps awvalid high across both.
            if ((!dmem_aw_taken || dmem_awaddr != dmem_awaddr_q) && dmem_awvalid && dmem_wvalid) begin
                if (dmem_wstrb[0]) mem[idx(dmem_awaddr)][7:0]   <= dmem_wdata[7:0];
                if (dmem_wstrb[1]) mem[idx(dmem_awaddr)][15:8]  <= dmem_wdata[15:8];
                if (dmem_wstrb[2]) mem[idx(dmem_awaddr)][23:16] <= dmem_wdata[23:16];
                if (dmem_wstrb[3]) mem[idx(dmem_awaddr)][31:24] <= dmem_wdata[31:24];
                dmem_bvalid_q <= 1'b1;
                dmem_aw_taken <= 1'b1;
            end else if (dmem_bvalid_q && dmem_bready) begin
                dmem_bvalid_q <= 1'b0;
                dmem_aw_taken <= 1'b0;
            end
            dmem_awaddr_q <= dmem_awaddr;
            if (!dmem_awvalid)
                dmem_aw_taken <= 1'b0;
        end
    end

    assign dmem_awready = 1'b1;
    assign dmem_wready  = 1'b1;
    assign dmem_bvalid  = dmem_bvalid_q;
    assign dmem_bresp   = 2'b00;

    //------------------------------------------------------------------
    // Run
    //------------------------------------------------------------------
    integer cyc;
    initial begin
        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        for (cyc = 0; cyc < MAX_CYCLE; cyc = cyc + 1) begin
            @(posedge clk);
            if (mem[idx(tohost_addr)] != 32'h0000_0000) begin
                if (mem[idx(tohost_addr)] == 32'h0000_0001)
                    $display("RESULT: PASS   instret=%0d cycles=%0d", instret_dbg, cyc);
                else
                    $display("RESULT: FAIL   tohost=0x%08x (test %0d)  instret=%0d cycles=%0d",
                             mem[idx(tohost_addr)], mem[idx(tohost_addr)] >> 1, instret_dbg, cyc);
                $finish;
            end
        end

        $display("RESULT: TIMEOUT after %0d cycles  instret=%0d  pc=0x%08x",
                 MAX_CYCLE, instret_dbg, imem_addr);
        $finish;
    end

// Temporary: log translations in the region the vm_boot code sets up, so a
// wrong mapping shows up as a mismatch rather than a mysterious exit.
always @(posedge clk) begin
    if (rst_n && USE_MMU && mmu_req && mmu_hit &&
        mmu_va[31:24] == 8'h7f && xl_count < 12) begin
        xl_count <= xl_count + 1;
        $display("XLATE va=0x%08x pa=0x%08x store=%0d",
                 mmu_va, mmu_pa, mmu_is_store);
    end
end
integer xl_count = 0;

// Temporary: report the first exception with the state that produced it.
reg trap_seen = 0;
reg satp_seen = 0;
always @(posedge clk) begin
    if (rst_n && cyc < 40 && dbg_pc_set)
        $display("PRIVT cyc=%0d priv=%0d mst=0x%08x satp=0x%08x idpc=0x%08x inst=0x%08x",
                 cyc, cur_priv, 32'h0, satp_o, dbg_ex_pc, dbg_ex_inst);  // pc_set high
end
integer ptw = 0;
always @(posedge clk) begin
    if (rst_n && pt_req && ptw < 12) begin
        ptw <= ptw + 1;
        $display("PTW%0d addr=0x%08x rdata=0x%08x ready=%0d",
                 ptw, pt_addr, pt_rdata, pt_ready);
    end
end
always @(posedge clk) begin
    if (rst_n && dbg_inst == 32'h18071073 && !satp_seen)
        $display("SATPWR cycle=%0d csrw satp executed", cyc);
end
integer idtr = 0;
always @(posedge clk) begin
    if (rst_n && satp_o[31] && idtr < 10 && dbg_wb_we) begin
        idtr <= idtr + 1;
        $display("IDTR pc=0x%08x inst=0x%08x", dbg_idpc, dbg_inst);
    end
end
integer xf = 0;
always @(posedge clk) begin
    if (rst_n && satp_o[31] && imem_req && xf < 10) begin
        xf <= xf + 1;
        $display("XFER va=0x%08x pa=0x%08x hit=%0d rdata=0x%08x",
                 imem_addr, imem_pa, imem_hit, imem_rdata);
    end
end
reg vm_seen = 0;
reg [31:0] last_pc = 32'hFFFFFFFF;
integer pc_n = 0;
always @(posedge clk) begin
    if (rst_n && imem_req && pc_n < 400) begin
        pc_n <= pc_n + 1;
        $display("C%0d pc_next=0x%08x imem_pa=0x%08x req=%0d ack=%0d rv=%0d ifst=%0d",
                 cyc, if_pc_next_w, imem_pa, imem_req, imem_ack, imem_rvalid, imem_rdata);
    end
end
always @(posedge clk) begin
    if (rst_n && if_pc_next_w[31:8] == 24'h800028 && !vm_seen) begin
        vm_seen <= 1;
        $display("VMBOOT cycle=%0d pc=0x%08x", cyc, if_pc_next_w);
    end
end
always @(posedge clk) begin
    if (rst_n && satp_o[31] && !satp_seen) begin
        satp_seen <= 1;
        $display("SATP_SET cycle=%0d satp=0x%08x pc_next=0x%08x", cyc, satp_o, if_pc_next_w);
    end
end
always @(posedge clk) begin
    if (rst_n && mcause_w != 32'd0 && !trap_seen) begin
        trap_seen <= 1;
        $display("EXC cycle=%0d mcause=0x%08x mepc=0x%08x satp=0x%08x",
                 cyc, mcause_w, mepc_w, satp_o);
    end
end

// Trace gating for scripts/trace_rtl.py: collect from reset until the program
// writes its tohost word. Both edges are derived from signals the bench already
// has, so nothing has to change in the design.
reg trace_on = 0;
always @(posedge clk) begin
    if (!rst_n) trace_on <= 1'b0;
    else        trace_on <= 1'b1;
end
endmodule
