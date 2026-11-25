//=============================================================================
// File        : isa_ddr_top.v
// Description : Board top level for running the official RISC-V ISA tests
//
//   riscv_core (RV32IMA)  ->  Sv32 MMU  ->  cpu_ddr_bridge  ->  MIG (32-bit AXI)
//                                ^
//                          mmu_arb shares one walker between fetch and data
//
// Programs are linked at 0x8000_0000. The bridge aliases that to physical
// 0x0, which is where the controller sees it, so only the core and the memory
// path are in scope here: a failure can then only come from the DDR path.
//
// Loading
//   The board comes up in the BootROM loader, which reads an image over the
//   console, writes it into DDR, reads it back and reports how many words it
//   stored and how many mismatched. Once the report is out the core is
//   released and starts fetching from 0x8000_0000.
//
// Result
//   The suite writes its verdict to the tohost word. A store to that location
//   is captured here and reported over the console: 1 means the case passed,
//   a value 2n+1 means sub-test n failed.
//
//=============================================================================
module isa_ddr_top #(
    parameter [31:0] TOHOST_ADDR = 32'h8000_1000,
    parameter        CLK_FREQ    = 100_000_000
) (
    input  wire        sys_clk,        // Y18, 50 MHz
    input  wire        rst_n,          // F20, active low
    output wire [3:0]  led,            // F19 E21 D20 C20
    output wire        uart_tx,        // G16
    input  wire        uart_rx,        // G15

    output wire [13:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_ras_n,
    output wire        ddr3_cas_n,
    output wire        ddr3_we_n,
    output wire        ddr3_reset_n,
    output wire [0:0]  ddr3_ck_p,
    output wire [0:0]  ddr3_ck_n,
    output wire [0:0]  ddr3_cke,
    output wire [0:0]  ddr3_cs_n,
    output wire [1:0]  ddr3_dm,
    output wire [0:0]  ddr3_odt,
    inout  wire [15:0] ddr3_dq,
    inout  wire [1:0]  ddr3_dqs_p,
    inout  wire [1:0]  ddr3_dqs_n
);

    //-------------------------------------------------------------------------
    // Clocks
    //
    // The controller's MMCM expects a 200 MHz reference (its PRJ records
    // InputClkFreq 200), and its AXI side runs at a quarter of that.
    //-------------------------------------------------------------------------
    wire clk_ref_200;
    wire locked;

    clk_wiz_0 u_clk (
        .clk_in1  (sys_clk),
        .reset    (1'b0),
        .clk_out1 (clk_ref_200),
        .locked   (locked)
    );

    wire ui_clk;
    wire ui_clk_sync_rst;
    wire calib_done;
    wire mmcm_locked;

    // The AXI-side reset is tied off. Feeding the controller's own
    // ui_clk_sync_rst back into aresetn deadlocks it, which is what kept the
    // earlier builds from ever finishing calibration.
    // Controller AXI reset: fixed high. Driving it from ui_clk_sync_rst closes
    // a loop, because that signal stays high until the controller has started.
    wire mig_aresetn = 1'b1;

    // Everything else needs a real power-on reset. ui_clk_sync_rst is asserted
    // from configuration until ui_clk is stable, which is exactly that.
    wire sys_rst_n = ~ui_clk_sync_rst & rst_n;

    //-------------------------------------------------------------------------
    // Core
    //-------------------------------------------------------------------------
    wire [31:0] imem_addr, imem_rdata;
    wire        imem_req, imem_ack, imem_rvalid;
    wire [2:0]  imem_prot;
    wire [1:0]  imem_rresp;
    wire        imem_rready;

    wire [31:0] dmem_araddr, dmem_awaddr, dmem_wdata, dmem_rdata;
    wire [2:0]  dmem_arprot, dmem_awprot;
    wire [1:0]  dmem_rresp, dmem_bresp;
    wire        dmem_arvalid, dmem_arready, dmem_rvalid, dmem_rready;
    wire        dmem_awvalid, dmem_awready, dmem_wvalid, dmem_wready;
    wire [3:0]  dmem_wstrb;
    wire        dmem_bvalid, dmem_bready;

    wire [63:0] instret_cnt;
    wire [63:0] cycle_cnt;
    wire [31:0] mmu_va;
    wire [31:0] satp_sig, mmu_pa_sig;
    wire        sum_sig, mxr_sig, sfence_sig, mmu_fault_ack;
    wire        mmu_hit_sig, mmu_fault_sig;
    wire [3:0]  mmu_fcause_sig;
    wire [31:0] pt_addr, pt_waddr, pt_wdata, pt_rdata;
    wire        pt_req, pt_we, pt_ready;
    wire [31:0] arb_va, arb_pa;
    wire        arb_req, arb_is_store, arb_is_fetch, arb_hit, arb_fault;
    wire [3:0]  arb_fcause;
    wire        arb_if_busy;
    wire [31:0] imem_pa;
    wire [31:0] if_pc_next;
    wire        imem_want;
    wire        imem_ack_core, imem_rvalid_core;
    wire [31:0] imem_rdata_core;
    wire [1:0]  imem_rresp_core;
    wire        mmu_active = satp_sig[31];
    wire        imem_hit, imem_fault;
    wire        if_mmu_stall_sig, if_mmu_fault_sig;
    wire [3:0]  if_mmu_fcause_sig;
    wire        mmu_req, mmu_is_store, mmu_is_fetch;
    wire [1:0]  mmu_priv, mmu_fetch_priv;
    wire        dmem_active_w;

    riscv_core #(.RESET_PC_V(32'h0000_0000)) u_core (
        .clk          (ui_clk),
        .rst_n        (sys_rst_n),

        .imem_addr    (imem_addr),
        .imem_prot    (imem_prot),
        .imem_req     (imem_req),
        .imem_ack     (imem_ack_core),
        .imem_rdata   (imem_rdata_core),
        .imem_rresp   (imem_rresp_core),
        .imem_rvalid  (imem_rvalid_core),
        .imem_rready  (imem_rready),

        .dmem_araddr  (dmem_araddr),
        .dmem_arprot  (dmem_arprot),
        .dmem_arvalid (dmem_arvalid),
        .dmem_arready (dmem_arready),
        .dmem_rdata   (dmem_rdata),
        .dmem_rresp   (dmem_rresp),
        .dmem_rvalid  (dmem_rvalid),
        .dmem_rready  (dmem_rready),

        .dmem_awaddr  (dmem_awaddr),
        .dmem_awprot  (dmem_awprot),
        .dmem_awvalid (dmem_awvalid),
        .dmem_awready (dmem_awready),
        .dmem_wdata   (dmem_wdata),
        .dmem_wstrb   (dmem_wstrb),
        .dmem_wvalid  (dmem_wvalid),
        .dmem_wready  (dmem_wready),
        .dmem_bresp   (dmem_bresp),
        .dmem_bvalid  (dmem_bvalid),
        .dmem_bready  (dmem_bready),

        .mmu_va           (mmu_va),
        .mmu_req          (mmu_req),
        .mmu_is_store     (mmu_is_store),
        .mmu_is_fetch     (mmu_is_fetch),
        .mmu_priv         (mmu_priv),
        .mmu_fetch_priv   (mmu_fetch_priv),
        .dmem_active_o    (dmem_active_w),
        .satp_o           (satp_sig),
        .sum_o            (sum_sig),
        .mxr_o            (mxr_sig),
        .sfence_o         (sfence_sig),
        .mmu_fault_ack_o  (mmu_fault_ack),
        .mmu_pa           (mmu_pa_sig),
        .mmu_hit          (mmu_hit_sig),
        .mmu_fault        (mmu_fault_sig),
        .mmu_fault_cause  (mmu_fcause_sig),

        .if_mmu_stall     (if_mmu_stall_sig),
        .if_mmu_fault     (if_mmu_fault_sig),
        .if_mmu_fault_cause (if_mmu_fcause_sig),

        .irq_soft         (1'b0),
        .irq_timer        (1'b0),
        .irq_ext          (1'b0),

        .cycle_cnt        (cycle_cnt),
        .instret_cnt      (instret_cnt),
        .dbg_mcause       (mcause_dbg),
        .dbg_mepc         (mepc_dbg),
        .dbg_idpc         (id_pc_dbg),
        .dbg_ifpc         (if_pc_next),
        .imem_want        (imem_want),
        .dbg_inst         (id_inst_dbg),
        .dbg_ex_pc        (ex_pc_dbg),
        .dbg_alu_y        (ex_alu_y_dbg),
        .dbg_wb_data      (wb_data_dbg),
        .dbg_gp           (gp_dbg),
        .dbg_s0           (s0_dbg)
    );

    //-------------------------------------------------------------------------
    // Bridge: instruction fetch and data accesses share one AXI master
    //-------------------------------------------------------------------------
    wire [3:0]   bx_awid, bx_arid, bx_bid, bx_rid;
    wire [27:0]  bx_awaddr, bx_araddr;
    wire [31:0]  bx_cpu_addr;
    wire [31:0]  mcause_dbg;
    wire [31:0]  mepc_dbg;
    wire [31:0]  id_pc_dbg;
    wire [31:0]  id_inst_dbg;
    wire [2:0]   bridge_state_dbg;
    wire [31:0]  ex_pc_dbg;
    wire [31:0]  ex_alu_y_dbg;
    wire [31:0]  wb_data_dbg;
    wire [31:0]  gp_dbg;
    wire         loader_txd;
    wire         trace_txd;
    wire         prog_start;
    wire [7:0]   trace_state;
    wire [9:0]   trace_count;
    wire [31:0]  s0_dbg;
    wire [31:0]  mem_pc_dbg;
    wire [31:0]  dbg_instret_lo;

    assign mem_pc_dbg     = id_pc_dbg;
    assign dbg_instret_lo = instret_cnt[31:0];
    wire [7:0]   bx_awlen, bx_arlen;
    wire [2:0]   bx_awsize, bx_arsize;
    wire [1:0]   bx_awburst, bx_arburst;
    wire         bx_awvalid, bx_awready;
    // 32-bit AXI side: one beat per access, so no word selection is needed.
    wire [31:0]  bx_wdata, bx_rdata;
    wire [3:0]   bx_wstrb;
    wire         bx_wlast, bx_wvalid, bx_wready;
    wire [1:0]   bx_bresp, bx_rresp;
    wire         bx_bvalid, bx_bready;
    wire         bx_arvalid, bx_arready;
    wire         bx_rlast, bx_rvalid, bx_rready;

    cpu_ddr_bridge #(.DW(32)) u_bridge (
        .clk          (ui_clk),
        .rst          (~sys_rst_n),
        .ddr_ready    (1'b1),

        // The MMU returns a physical address; the decoder classifies by CPU
        // address and would otherwise treat a page below 0x2000_0000 as
        // BootROM. Re-adding the DDR base makes the fetch look like what it is.
        .imem_addr    (mmu_active && imem_pa[31:28] != 4'h8
                                   ? (imem_pa | 32'h8000_0000) : imem_pa),
        // The fetcher's request only reaches the bus once its address has been
        // translated; until then imem_pa is not yet the right address.
        .imem_req     (imem_req && (imem_hit || !satp_sig[31])),
        .imem_ack     (imem_ack),
        .imem_rdata   (imem_rdata),
        .imem_rvalid  (imem_rvalid),

        .dmem_araddr  (dmem_araddr),
        .dmem_arvalid (dmem_arvalid),
        .dmem_arready (dmem_arready),
        .dmem_rdata   (dmem_rdata),
        .dmem_rvalid  (dmem_rvalid),
        .dmem_rready  (dmem_rready),

        .dmem_awaddr  (dmem_awaddr),
        .dmem_awvalid (dmem_awvalid),
        .dmem_awready (dmem_awready),
        .dmem_wdata   (dmem_wdata),
        .dmem_wstrb   (dmem_wstrb),
        .dmem_wvalid  (dmem_wvalid),
        .dmem_wready  (dmem_wready),
        .dmem_bvalid  (dmem_bvalid),
        .dmem_bready  (dmem_bready),

        // Page tables live in DDR, but the decoder classifies by CPU address
        // and treats anything below 0x2000_0000 as BootROM or UART. Adding the
        // DDR base makes a page-table access look like the DDR access it is.
        .pt_addr      (pt_addr + 32'h8000_0000),
        .pt_req       (pt_req),
        .pt_waddr     (pt_waddr + 32'h8000_0000),
        .pt_wdata     (pt_wdata),
        .pt_we        (pt_we),
        .pt_rdata     (pt_rdata),
        .pt_ready     (pt_ready),

        .m_awid       (bx_awid),
        .m_awaddr     (bx_awaddr),
        .m_awlen      (bx_awlen),
        .m_awsize     (bx_awsize),
        .m_awburst    (bx_awburst),
        .m_awvalid    (bx_awvalid),
        .m_awready    (bx_awready),

        .m_wdata      (bx_wdata),
        .m_wstrb      (bx_wstrb),
        .m_wlast      (bx_wlast),
        .m_wvalid     (bx_wvalid),
        .m_wready     (bx_wready),

        .m_bid        (bx_bid),
        .m_bresp      (bx_bresp),
        .m_bvalid     (bx_bvalid),
        .m_bready     (bx_bready),

        .m_arid       (bx_arid),
        .m_araddr     (bx_araddr),
        .m_arlen      (bx_arlen),
        .m_arsize     (bx_arsize),
        .m_arburst    (bx_arburst),
        .m_arvalid    (bx_arvalid),
        .m_arready    (bx_arready),

        .m_rid        (bx_rid),
        .m_rdata      (bx_rdata),
        .m_rresp      (bx_rresp),
        .m_rlast      (bx_rlast),
        .m_rvalid     (bx_rvalid),
        .m_rready     (bx_rready),

        .cpu_addr_o   (bx_cpu_addr),
        .axi_addr_o   (),
        .cpu_is_rd_o  (),

        .dbg_busy     (bridge_busy),
        .dbg_state    (bridge_state_dbg)
    );

    //-------------------------------------------------------------------------
    // Address decode: BootROM, UART, DDR
    //-------------------------------------------------------------------------
    // ROM
    wire [27:0] rom_araddr, rom_awaddr;
    wire        rom_arvalid, rom_arready;
    wire [31:0] rom_rdata;
    wire        rom_rvalid;
    wire        rom_rready;
    wire        rom_awvalid, rom_awready;
    wire [31:0] rom_wdata;
    wire [3:0]  rom_wstrb;
    wire        rom_wvalid, rom_wready;
    wire [1:0]  rom_bresp;
    wire        rom_bvalid, rom_bready;

    // UART registers
    wire [27:0] uart_araddr, uart_awaddr;
    wire        uart_arvalid, uart_arready;
    wire [31:0] uart_rdata;
    wire        uart_rvalid;
    wire        uart_rready;
    wire        uart_awvalid, uart_awready;
    wire [31:0] uart_wdata;
    wire [3:0]  uart_wstrb;
    wire        uart_wvalid, uart_wready;
    wire [1:0]  uart_bresp;
    wire        uart_bvalid, uart_bready;

    // DDR side, before the converter
    wire [27:0] ddr_awaddr, ddr_araddr;
    wire [7:0]  ddr_awlen,  ddr_arlen;
    wire [2:0]  ddr_awsize, ddr_arsize;
    wire [1:0]  ddr_awburst,ddr_arburst;
    wire        ddr_awvalid,ddr_awready;
    wire [31:0] ddr_wdata;
    wire [3:0]  ddr_wstrb;
    wire        ddr_wlast,  ddr_wvalid, ddr_wready;
    wire [1:0]  ddr_bresp;
    wire        ddr_bvalid, ddr_bready;
    wire        ddr_arvalid,ddr_arready;
    wire [31:0] ddr_rdata;
    wire [1:0]  ddr_rresp;
    wire        ddr_rlast,  ddr_rvalid, ddr_rready;

    axi_decode32 u_decode (
        .clk        (ui_clk),
        .aresetn    (sys_rst_n),

        // The unaliased CPU address, used for region selection.
        .m_cpu_addr (bx_cpu_addr),

        .m_awaddr   (bx_awaddr),
        .m_awlen    (bx_awlen),
        .m_awsize   (bx_awsize),
        .m_awburst  (bx_awburst),
        .m_awvalid  (bx_awvalid),
        .m_awready  (bx_awready),
        .m_wdata    (bx_wdata),
        .m_wstrb    (bx_wstrb),
        .m_wlast    (bx_wlast),
        .m_wvalid   (bx_wvalid),
        .m_wready   (bx_wready),
        .m_bresp    (bx_bresp),
        .m_bvalid   (bx_bvalid),
        .m_bready   (bx_bready),

        .m_araddr   (bx_araddr),
        .m_arlen    (bx_arlen),
        .m_arsize   (bx_arsize),
        .m_arburst  (bx_arburst),
        .m_arvalid  (bx_arvalid),
        .m_arready  (bx_arready),
        .m_rdata    (bx_rdata),
        .m_rresp    (bx_rresp),
        .m_rlast    (bx_rlast),
        .m_rvalid   (bx_rvalid),
        .m_rready   (bx_rready),

        .rom_araddr (rom_araddr),
        .rom_arvalid(rom_arvalid),
        .rom_arready(rom_arready),
        .rom_rdata  (rom_rdata),
        .rom_rvalid (rom_rvalid),
        .rom_rready (rom_rready),
        .rom_awaddr (rom_awaddr),
        .rom_awvalid(rom_awvalid),
        .rom_awready(rom_awready),
        .rom_wdata  (rom_wdata),
        .rom_wstrb  (rom_wstrb),
        .rom_wvalid (rom_wvalid),
        .rom_wready (rom_wready),
        .rom_bresp  (rom_bresp),
        .rom_bvalid (rom_bvalid),
        .rom_bready (rom_bready),

        .uart_araddr (uart_araddr),
        .uart_arvalid(uart_arvalid),
        .uart_arready(uart_arready),
        .uart_rdata  (uart_rdata),
        .uart_rvalid (uart_rvalid),
        .uart_rready (uart_rready),
        .uart_awaddr (uart_awaddr),
        .uart_awvalid(uart_awvalid),
        .uart_awready(uart_awready),
        .uart_wdata  (uart_wdata),
        .uart_wstrb  (uart_wstrb),
        .uart_wvalid (uart_wvalid),
        .uart_wready (uart_wready),
        .uart_bresp  (uart_bresp),
        .uart_bvalid (uart_bvalid),
        .uart_bready (uart_bready),

        .ddr_awaddr (ddr_awaddr),
        .ddr_awlen  (ddr_awlen),
        .ddr_awsize (ddr_awsize),
        .ddr_awburst(ddr_awburst),
        .ddr_awvalid(ddr_awvalid),
        .ddr_awready(ddr_awready),
        .ddr_wdata  (ddr_wdata),
        .ddr_wstrb  (ddr_wstrb),
        .ddr_wlast  (ddr_wlast),
        .ddr_wvalid (ddr_wvalid),
        .ddr_wready (ddr_wready),
        .ddr_bresp  (ddr_bresp),
        .ddr_bvalid (ddr_bvalid),
        .ddr_bready (ddr_bready),

        .ddr_araddr (ddr_araddr),
        .ddr_arlen  (ddr_arlen),
        .ddr_arsize (ddr_arsize),
        .ddr_arburst(ddr_arburst),
        .ddr_arvalid(ddr_arvalid),
        .ddr_arready(ddr_arready),
        .ddr_rdata  (ddr_rdata),
        .ddr_rresp  (ddr_rresp),
        .ddr_rlast  (ddr_rlast),
        .ddr_rvalid (ddr_rvalid),
        .ddr_rready (ddr_rready)
    );

    //-------------------------------------------------------------------------
    // BootROM: the loader that pulls a program in over UART
    //-------------------------------------------------------------------------
    bootrom32 #(
        .ADDR_W   (12),
        .DEPTH    (1024),
        .INIT_FILE("D:/04.Code/Verilog/RISCV/rom/bootrom_uart.hex")
    ) u_rom (
        .clk        (ui_clk),
        .aresetn    (sys_rst_n),

        .s_arid     (4'd0),
        .s_araddr   (rom_araddr),
        .s_arlen    (8'd0),
        .s_arsize   (3'b010),
        .s_arburst  (2'b01),
        .s_arvalid  (rom_arvalid),
        .s_arready  (rom_arready),
        .s_rid      (),
        .s_rdata    (rom_rdata),
        .s_rresp    (),
        .s_rlast    (),
        .s_rvalid   (rom_rvalid),
        .s_rready   (rom_rready),

        .s_awid     (4'd0),
        .s_awaddr   (rom_awaddr),
        .s_awlen    (8'd0),
        .s_awsize   (3'b010),
        .s_awburst  (2'b01),
        .s_awvalid  (rom_awvalid),
        .s_awready  (rom_awready),
        .s_wdata    (rom_wdata),
        .s_wstrb    (rom_wstrb),
        .s_wlast    (1'b1),
        .s_wvalid   (rom_wvalid),
        .s_wready   (rom_wready),
        .s_bid      (),
        .s_bresp    (rom_bresp),
        .s_bvalid   (rom_bvalid),
        .s_bready   (rom_bready)
    );

    //-------------------------------------------------------------------------
    // UART: byte interface the loader polls
    //-------------------------------------------------------------------------
    uart_mmio32 #(
        .CLK_FREQ  (100_000_000),
        .BAUD_RATE (115_200)
    ) u_uart (
        .clk        (ui_clk),
        .aresetn    (sys_rst_n),

        .s_arid     (4'd0),
        .s_araddr   (uart_araddr),
        .s_arlen    (8'd0),
        .s_arsize   (3'b010),
        .s_arburst  (2'b01),
        .s_arvalid  (uart_arvalid),
        .s_arready  (uart_arready),
        .s_rid      (),
        .s_rdata    (uart_rdata),
        .s_rresp    (),
        .s_rlast    (),
        .s_rvalid   (uart_rvalid),
        .s_rready   (uart_rready),

        .s_awid     (4'd0),
        .s_awaddr   (uart_awaddr),
        .s_awlen    (8'd0),
        .s_awsize   (3'b010),
        .s_awburst  (2'b01),
        .s_awvalid  (uart_awvalid),
        .s_awready  (uart_awready),
        .s_wdata    (uart_wdata),
        .s_wstrb    (uart_wstrb),
        .s_wlast    (1'b1),
        .s_wvalid   (uart_wvalid),
        .s_wready   (uart_wready),
        .s_bid      (),
        .s_bresp    (uart_bresp),
        .s_bvalid   (uart_bvalid),
        .s_bready   (uart_bready),

        .rxd        (uart_rx),
        .txd        (loader_txd),

        .dbg_rx_hold  (uart_rx_hold),
        .dbg_rx_ready (uart_rx_ready),
        .dbg_rx_valid (uart_rx_valid),
        .dbg_rx_byte  (uart_rx_byte),
        .dbg_rx_state (uart_rx_state),
        .dbg_rx_rst   (uart_rx_rst),
        .dbg_rxd_raw  (uart_rxd_raw),
        .dbg_rxd_sync (uart_rxd_sync),
        .dbg_rx_cnt   (uart_rx_cnt),
        .prog_start   (prog_start)
    );

    //-------------------------------------------------------------------------
    // The memory controller takes 32-bit AXI, the same width the CPU bus uses,
    // so the two connect directly. A converter here would only insert another
    // state machine between two identical ports.
    //-------------------------------------------------------------------------
    wire [27:0] s_awaddr = ddr_awaddr;
    wire [7:0]  s_awlen  = ddr_awlen;
    wire [2:0]  s_awsize = ddr_awsize;
    wire [1:0]  s_awburst= ddr_awburst;
    wire [3:0]  s_awcache= 4'b0011;
    wire [2:0]  s_awprot = 3'b000;
    wire [3:0]  s_awqos  = 4'b0;
    wire [3:0]  s_awregion = 4'b0;
    wire        s_awvalid= ddr_awvalid;
    wire        s_awready;

    wire [31:0] s_wdata  = ddr_wdata;
    wire [3:0]  s_wstrb  = ddr_wstrb;
    wire        s_wlast  = ddr_wlast;
    wire        s_wvalid = ddr_wvalid;
    wire        s_wready;

    wire [1:0]  s_bresp;
    wire        s_bvalid;
    wire        s_bready = ddr_bready;

    wire [27:0] s_araddr = ddr_araddr;
    wire [7:0]  s_arlen  = ddr_arlen;
    wire [2:0]  s_arsize = ddr_arsize;
    wire [1:0]  s_arburst= ddr_arburst;
    wire [3:0]  s_arcache= 4'b0011;
    wire [2:0]  s_arprot = 3'b000;
    wire [3:0]  s_arqos  = 4'b0;
    wire [3:0]  s_arregion = 4'b0;
    wire        s_arvalid= ddr_arvalid;
    wire        s_arready;

    wire [31:0] s_rdata;
    wire [1:0]  s_rresp;
    wire        s_rlast;
    wire        s_rvalid;
    wire        s_rready = ddr_rready;

    assign ddr_awready = s_awready;
    assign ddr_wready  = s_wready;
    assign ddr_bvalid  = s_bvalid;
    assign ddr_bresp   = s_bresp;
    assign ddr_arready = s_arready;
    assign ddr_rdata   = s_rdata;
    assign ddr_rresp   = s_rresp;
    assign ddr_rlast   = s_rlast;
    assign ddr_rvalid  = s_rvalid;

    //-------------------------------------------------------------------------
    // Memory controller
    //-------------------------------------------------------------------------
    ddr3 u_ddr3 (
        .ddr3_addr    (ddr3_addr),
        .ddr3_ba      (ddr3_ba),
        .ddr3_cas_n   (ddr3_cas_n),
        .ddr3_ck_n    (ddr3_ck_n),
        .ddr3_ck_p    (ddr3_ck_p),
        .ddr3_cke     (ddr3_cke),
        .ddr3_ras_n   (ddr3_ras_n),
        .ddr3_we_n    (ddr3_we_n),
        .ddr3_dq      (ddr3_dq),
        .ddr3_dqs_n   (ddr3_dqs_n),
        .ddr3_dqs_p   (ddr3_dqs_p),
        .ddr3_reset_n (ddr3_reset_n),
        .ddr3_cs_n    (ddr3_cs_n),
        .ddr3_dm      (ddr3_dm),
        .ddr3_odt     (ddr3_odt),

        .ui_clk           (ui_clk),
        .ui_clk_sync_rst  (ui_clk_sync_rst),
        .mmcm_locked      (mmcm_locked),
        .init_calib_complete (calib_done),

        // AXI-side reset. Tied high exactly as in the configuration that has
        // been shown to work; driving it from ui_clk_sync_rst closes a loop.
        .aresetn          (mig_aresetn),

        // Refresh and power-down requests are unused.
        .app_sr_req       (1'b0),
        .app_ref_req      (1'b0),
        .app_zq_req       (1'b0),
        .app_sr_active    (),
        .app_ref_ack      (),
        .app_zq_ack       (),
        .device_temp      (),

        .sys_clk_i    (clk_ref_200),
        .sys_rst      (rst_n),

        .s_axi_awid    (4'b0),
        .s_axi_awaddr  (s_awaddr),
        .s_axi_awlen   (s_awlen),
        .s_axi_awsize  (s_awsize),
        .s_axi_awburst (s_awburst),
        .s_axi_awlock  (1'b0),
        .s_axi_awcache (s_awcache),
        .s_axi_awprot  (s_awprot),
        .s_axi_awqos   (s_awqos),
        .s_axi_awvalid (s_awvalid),
        .s_axi_awready (s_awready),

        .s_axi_wdata   (s_wdata),
        .s_axi_wstrb   (s_wstrb),
        .s_axi_wlast   (s_wlast),
        .s_axi_wvalid  (s_wvalid),
        .s_axi_wready  (s_wready),

        .s_axi_bid     (),
        .s_axi_bresp   (s_bresp),
        .s_axi_bvalid  (s_bvalid),
        .s_axi_bready  (s_bready),

        .s_axi_arid    (4'b0),
        .s_axi_araddr  (s_araddr),
        .s_axi_arlen   (s_arlen),
        .s_axi_arsize  (s_arsize),
        .s_axi_arburst (s_arburst),
        .s_axi_arlock  (1'b0),
        .s_axi_arcache (s_arcache),
        .s_axi_arprot  (s_arprot),
        .s_axi_arqos   (s_arqos),
        .s_axi_arvalid (s_arvalid),
        .s_axi_arready (s_arready),

        .s_axi_rid     (),
        .s_axi_rdata   (s_rdata),
        .s_axi_rresp   (s_rresp),
        .s_axi_rlast   (s_rlast),
        .s_axi_rvalid  (s_rvalid),
        .s_axi_rready  (s_rready)
    );

    //-------------------------------------------------------------------------
    // Status
    //-------------------------------------------------------------------------
    reg [23:0] hb_cnt;
    reg        hb;

    always @(posedge sys_clk) begin
        if (!rst_n) begin
            hb_cnt <= 24'd0;
            hb     <= 1'b0;
        end else begin
            hb_cnt <= hb_cnt + 24'd1;
            if (hb_cnt == 24'hFFFFFF) hb <= ~hb;
        end
    end

    assign led = {instret_cnt[5], calib_done, locked, hb};

    // Free-running counter on the reference clock: shows the clock tree is up
    // even when the controller is still training.
    reg [31:0] alive;
    always @(posedge ui_clk) begin
        if (!sys_rst_n) alive <= 32'd0;
        else            alive <= alive + 32'd1;
    end

    // Last store the bridge issued: address and data. Reading these over the
    // VIO shows exactly what reached the bus for a given store.
    reg [31:0] last_waddr;
    reg [31:0] last_wdata;
    reg [31:0] wcount;
    wire       w_fire = bx_awvalid && bx_awready && bx_wvalid && bx_wready;
    always @(posedge ui_clk) begin
        if (!sys_rst_n) begin
            last_waddr <= 32'h0;
            last_wdata <= 32'h0;
            wcount     <= 32'h0;
        end else if (w_fire) begin
            // Only record stores that target memory, so the UART traffic the
            // loader generates afterwards does not overwrite the evidence.
            if (bx_cpu_addr[31:16] == 16'h8000) begin
                last_waddr <= bx_cpu_addr;
                last_wdata <= bx_wdata;
            end
            wcount <= wcount + 32'd1;
        end
    end

    // Sticky indicators. Each one latches the first time its condition is seen,
    // so a single VIO read is enough to answer the question even though the
    // event itself lasts one cycle.
    //
    //   saw44      a store to bus address 0x44 was accepted
    //   bad_strb   a store to an address with bit 2 set drove a strobe that
    //              was not all ones
    //   b_44       a write response came back for a store to 0x44
    reg saw44, bad_strb, b_44, w_44, aw_wait_w, wv_held;

    wire st44 = bx_awvalid && bx_awready && (bx_awaddr == 28'h44);

    always @(posedge ui_clk) begin
        if (!sys_rst_n) begin
            saw44    <= 1'b0;
            bad_strb <= 1'b0;
            b_44     <= 1'b0;
            w_44     <= 1'b0;
            aw_wait_w<= 1'b0;
            wv_held  <= 1'b0;
        end else begin
            if (st44)
                saw44 <= 1'b1;
            if (st44 && bx_wvalid && bx_wready && (bx_wstrb != 4'hF))
                bad_strb <= 1'b1;
            if (st44 && bx_bvalid)
                b_44 <= 1'b1;
            // The data channel side of the same store: did the beat complete?
            if (st44 && bx_wvalid && bx_wready)
                w_44 <= 1'b1;
            // The address was accepted but its data never was.
            if (st44 && !w_44 && !bx_wready)
                aw_wait_w <= 1'b1;
            // The data channel was still asserted after the address went.
            if (aw_wait_w && bx_wvalid)
                wv_held <= 1'b1;
        end
    end

    // Latch what the router saw at the moment a UART store was accepted: the
    // CPU address it classified and whether that classification was UART.
    // This settles whether a stalled console store is caused by the address
    // register holding a stale value.
    reg [31:0] uart_wr_addr;
    reg        uart_wr_seen;
    reg        uart_wr_misrouted;

    wire uart_store = bx_awvalid && bx_awready && (bx_cpu_addr[31:16] == 16'h1110);

    always @(posedge ui_clk) begin
        if (!sys_rst_n) begin
            uart_wr_addr      <= 32'h0;
            uart_wr_seen      <= 1'b0;
            uart_wr_misrouted <= 1'b0;
        end else if (uart_store) begin
            uart_wr_addr <= bx_cpu_addr;
            uart_wr_seen <= 1'b1;
            // A store accepted while the CPU address said UART should reach the
            // console; if the console never saw a handshake, it was misrouted.
            if (!(uart_awvalid && uart_awready && uart_wvalid && uart_wready))
                uart_wr_misrouted <= 1'b1;
        end
    end

    //------------------------------------------------------------------
    // DDR write channel: sticky events plus the live handshake.
    //
    // A stalled store is either never presented to the bus, or presented and
    // never completed. These flags separate those cases without needing a
    // capture, so one read of the VIO decides where the problem is.
    //------------------------------------------------------------------
    reg ddr_aw_seen;      // an address was accepted for the DDR region
    reg ddr_w_seen;       // data was accepted for the DDR region
    reg ddr_b_seen;       // the DDR region answered a write at least once
    reg ddr_tohost_seen;  // a store to the tohost word was accepted

    always @(posedge ui_clk) begin
        if (!sys_rst_n) begin
            ddr_aw_seen     <= 1'b0;
            ddr_w_seen      <= 1'b0;
            ddr_b_seen      <= 1'b0;
            ddr_tohost_seen <= 1'b0;
        end else begin
            if (bx_awvalid && bx_awready) begin
                ddr_aw_seen <= 1'b1;
                if (bx_awaddr[19:0] == 20'h0_1000)
                    ddr_tohost_seen <= 1'b1;
            end
            if (bx_wvalid && bx_wready)
                ddr_w_seen <= 1'b1;
            if (bx_bvalid && bx_bready)
                ddr_b_seen <= 1'b1;
        end
    end

    // Total bytes the receiver has ever completed. A running count is the only
    // reading that cannot be confused with a byte that was consumed already.
    reg [31:0] rx_total;
    always @(posedge ui_clk) begin
        if (!sys_rst_n)      rx_total <= 32'd0;
        else if (uart_rx_valid) rx_total <= rx_total + 32'd1;
    end

    // The probes observe signals in the ui_clk domain, so the VIO has to be
    // clocked by ui_clk as well. Sampling them from the 200 MHz reference
    // crosses clock domains with no synchroniser and the readings are garbage.
    //-------------------------------------------------------------------------
    // Watch the word the test suite writes to report its verdict.
    //-------------------------------------------------------------------------
    wire [31:0] tohost_value;
    wire [31:0] tohost_count;

    tohost_watch #(.TOHOST_ADDR(TOHOST_ADDR)) u_tohost (
        .clk          (ui_clk),
        .aresetn      (sys_rst_n),
        .awaddr       (bus_awaddr),
        .awvalid      (bus_awvalid),
        .awready      (bx_awready_w),
        .wdata        (bus_wdata),
        .wvalid       (bus_wvalid),
        .wready       (bx_wready_w),
        .cpu_addr     (bx_cpu_addr),
        .tohost_value (tohost_value),
        .tohost_count (tohost_count)
    );

    //------------------------------------------------------------------
    // gp trace: records the sub-test number sequence and streams it out once
    // the program has settled. Both transmitters idle high, so the shared
    // console line is just an AND of the two.
    //------------------------------------------------------------------
    gp_trace #(
        .DEPTH     (512),
        .ADDR_W    (9),
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (115_200)
    ) u_gp_trace (
        .clk       (ui_clk),
        .rst_n     (sys_rst_n),
        .gp_i      (gp_dbg),
        .id_pc_i   (id_pc_dbg),
        .enable    (prog_start),
        .txd       (trace_txd),
        .busy      (),
        .dbg_state (trace_state),
        .dbg_count (trace_count)
    );

    assign uart_tx = loader_txd & trace_txd;

    //------------------------------------------------------------------
    // A fetch that faults is answered locally: the bus is not touched, and the
    // fetcher is told the cycle failed so it can record the fault against the
    // right pc. Without this the request would go to the bus with an
    // untranslated address and come back as an ordinary instruction.
    //------------------------------------------------------------------
    assign imem_ack_core    = (satp_sig[31] && imem_hit && imem_fault) ? 1'b1 : imem_ack;
    assign imem_rvalid_core = (satp_sig[31] && imem_hit && imem_fault) ? 1'b1 : imem_rvalid;
    assign imem_rdata_core  = (satp_sig[31] && imem_hit && imem_fault) ? 32'h0000_0013 : imem_rdata;
    assign imem_rresp_core  = (satp_sig[31] && imem_hit && imem_fault) ? 2'b00 : imem_rresp;

    //------------------------------------------------------------------
    // Sv32 MMU shared by fetch and data.
    //
    // Only one walker exists, so the two requesters arbitrate for it. A
    // translation is owned from acceptance until hit or fault, which keeps a
    // two-level walk from being spliced into another translation.
    //------------------------------------------------------------------
    mmu_arb #(.AGE_LIMIT(6)) u_mmu_arb (
        .clk             (ui_clk),
        .rst_n           (sys_rst_n),

        // instr_addr_o is pc_next while idle and the latched address while a
        // request is outstanding, so it is the address the request actually
        // refers to at every cycle.
        .if_va           (imem_addr),
        // Translation is requested as soon as the fetcher wants an address,
        // not when the bus request is finally raised: stall is derived from
        // the translation, so gating on the request would deadlock the two.
        .if_req          (imem_want),
        .if_pa           (imem_pa),
        .if_hit          (imem_hit),
        .if_fault        (imem_fault),
        .if_fault_cause  (if_mmu_fcause_sig),

        .dm_va           (mmu_va),
        .dm_req          (mmu_req),
        .dm_is_store     (mmu_is_store),
        .dm_pa           (mmu_pa_sig),
        .dm_hit          (mmu_hit_sig),
        .dm_fault        (mmu_fault_sig),
        .dm_fault_cause  (mmu_fcause_sig),

        .va              (arb_va),
        .req             (arb_req),
        .is_store        (arb_is_store),
        // is_fetch is an output here: it reports who owns the address being
        // presented. Driving it from the core created a second driver on that
        // net, which Vivado rejects (DRC MDRV-1).

        .pa              (arb_pa),
        .hit             (arb_hit),
        .fault           (arb_fault),
        .fault_cause     (arb_fcause),
        .if_busy         (arb_if_busy)
    );

    // The fetcher raises imem_req as soon as it has an address, and that same
    // request drives the arbiter. The stall only tells the fetcher to hold its
    // ground while the translation is still outstanding, so it must be derived
    // from whether a translation is in progress rather than from the request
    // itself, which would deadlock against can_start.
    // The fetcher stalls only once it has actually asked for an address; tying
    // this to hit alone made can_start and the walker depend on each other.
    assign if_mmu_stall_sig  = satp_sig[31] && (arb_if_busy || (imem_req && !imem_hit));
    assign if_mmu_fault_sig  = imem_fault;

    mmu_sv32 u_mmu (
        .clk           (ui_clk),
        .rst_n         (sys_rst_n),

        .va            (arb_va),
        .req           (arb_req),
        .is_store      (arb_is_store),
        .is_fetch      (mmu_is_fetch),
        // MPRV applies to data accesses only, so the privilege handed to the
        // walker depends on who owns the address currently presented.
        .priv          (dmem_active_w ? mmu_priv : mmu_fetch_priv),

        .pa            (arb_pa),
        .hit           (arb_hit),
        .fault         (arb_fault),
        .fault_cause   (arb_fcause),

        .satp          (satp_sig),
        .satp_mode     (),
        .sum           (sum_sig),
        .mxr           (mxr_sig),

        .pt_addr       (pt_addr),
        .pt_req        (pt_req),
        .pt_rdata      (pt_rdata),
        .pt_ready      (pt_ready),
        .pt_waddr      (pt_waddr),
        .pt_wdata      (pt_wdata),
        .pt_we         (pt_we),

        .fault_ack     (mmu_fault_ack),
        .sfence_vma    (sfence_sig),
        .sfence_va_en  (1'b0),
        .sfence_va     (32'h0)
    );

    vio_state u_vio (
        .clk        (ui_clk),
        .probe_in0  (rx_total),
        .probe_in1  (gp_dbg),
        .probe_in2  (mcause_dbg),
        // 8 + 8 + 2 + 1 + 1 + 1 + 11 = 32 bits
        // The address the CPU is currently accessing: 0x0000_xxxx means the
        // BootROM, 0x8000_xxxx means the program in DDR.
        .probe_in3  (id_pc_dbg),
        .probe_in4  (s0_dbg),
        .probe_in5  ({14'd0, trace_count, trace_state}),
        .probe_out0 (),
        .probe_out1 ()
    );

    //-------------------------------------------------------------------------
    // Probes on the receive path. The question this answers is whether the
    // serial line shows any activity at all: a trigger on the state machine
    // leaving idle means a start bit was seen, silence means nothing arrived.
    //-------------------------------------------------------------------------
    // Watch one UART read transaction end to end: what the CPU asked for,
    // what the register block returned, and what the CPU actually received.
    // Watch stores: the address the CPU targeted, whether the write channels
    // were accepted, and the value being written. The test suite reports its
    // verdict with a single store, so this shows the moment it happens.
    // Capture the fetch of the instruction the trap handler reports as the
    // one that faulted: what address was requested and what came back.
    // Capture a store as it appears on the bus. The trigger is the bus address
    // itself, not the CPU address: the CPU address register is shared with
    // instruction fetches and is overwritten while a store is still in flight,
    // so it cannot be trusted as a trigger.
    ila_uart u_ila (
        .clk     (ui_clk),
        // Instruction trace for comparison against the reference model:
        // the program counter at decode, the instruction word, and the
        // registers that decide the branch, so a divergence is visible.
        .probe0  (id_pc_dbg),
        .probe1  (id_inst_dbg),
        .probe2  (ex_pc_dbg),
        .probe3  (ex_alu_y_dbg),
        .probe4  (mem_pc_dbg),
        .probe5  (gp_dbg),
        .probe6  (dbg_instret_lo)
    );

endmodule
