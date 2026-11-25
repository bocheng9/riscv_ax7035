//============================================================================
// File        : cpu_ddr_bridge.v
// Description : Converts the CPU's 32-bit interface into an AXI4 master,
//               driving the AXI4 flavour of MIG
//
// Architecture
//   CPU (32-bit) -> cpu_ddr_bridge -> axi4_master -> MIG(s_axi_*) -> DDR3
//
// Why AXI4 rather than the native interface
//   MIG's native app_* interface makes the user responsible for the address
//   unit (APP_DATA_WIDTH/8), the BL8 burst length, the polarity of
//   app_wdf_mask and the timing of app_wdf_end. With AXI4 all of that is
//   handled inside the MIG IP, and the user only has to obey the AXI4
//   protocol.
//
// Address mapping
//   The CPU byte address is passed straight through and MIG handles its own
//   alignment. Addresses alias from 0x8xxx_xxxx down to 0x0xxx_xxxx, because
//   programs are linked at 0x8000_0000.
//
//============================================================================
module cpu_ddr_bridge #(
    // Width of the AXI master side. 128 keeps the original behaviour; 32 makes
    // the bridge a plain 32-bit master that needs no word selection.
    parameter integer DW = 128
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        ddr_ready,       // MIG init_calib_complete

    // ==================== CPU 32 ====================
    input  wire [31:0] imem_addr,
    input  wire        imem_req,
    output wire        imem_ack,
    output reg  [31:0] imem_rdata,
    output reg         imem_rvalid,

    // ==================== CPU 32 ====================
    input  wire [31:0] dmem_araddr,
    input  wire        dmem_arvalid,
    output wire        dmem_arready,
    output reg  [31:0] dmem_rdata,
    output reg         dmem_rvalid,
    input  wire        dmem_rready,

    input  wire [31:0] dmem_awaddr,
    input  wire        dmem_awvalid,
    output wire        dmem_awready,
    input  wire [31:0] dmem_wdata,
    input  wire [3:0]  dmem_wstrb,
    input  wire        dmem_wvalid,
    output wire        dmem_wready,
    output reg         dmem_bvalid,
    input  wire        dmem_bready,

    // PTW page table MMU walk
    input  wire [31:0] pt_addr,
    input  wire        pt_req,
    input  wire [31:0] pt_waddr,
    input  wire [31:0] pt_wdata,
    input  wire        pt_we,
    output reg  [31:0] pt_rdata,
    output reg         pt_ready,

    //==================== AXI4 Master ====================
    // write
    output wire [3:0]     m_awid,
    output wire [27:0]    m_awaddr,
    output wire [7:0]     m_awlen,
    output wire [2:0]     m_awsize,
    output wire [1:0]     m_awburst,
    output wire           m_awvalid,
    input  wire           m_awready,
    // write
    output wire [DW-1:0]  m_wdata,
    output wire [DW/8-1:0] m_wstrb,
    output wire           m_wlast,
    output wire           m_wvalid,
    input  wire           m_wready,
    // write
    output wire           m_bready,
    input  wire [3:0]     m_bid,
    input  wire [1:0]     m_bresp,
    input  wire           m_bvalid,
    // read
    output wire [3:0]     m_arid,
    output wire [27:0]    m_araddr,
    output wire [7:0]     m_arlen,
    output wire [2:0]     m_arsize,
    output wire [1:0]     m_arburst,
    output wire           m_arvalid,
    input  wire           m_arready,
    // read
    output wire           m_rready,
    input  wire [3:0]     m_rid,
    input  wire [DW-1:0]  m_rdata,
    input  wire [1:0]     m_rresp,
    input  wire           m_rlast,
    input  wire           m_rvalid,

    // ==================== debug ====================
    output wire [2:0]   dbg_state,
    output wire [3:0]   dbg_axi_state,   // axi4_master
    output wire [7:0]   dbg_axi_beat,
    output wire [7:0]   dbg_axi_lenq,
    output wire         dbg_busy,
    output wire         dbg_src,
    output wire [31:0]  cpu_addr_o,      // 32 CPU hit
    output wire [27:0]  axi_addr_o,      // axi4_master AXI and m_araddr
    output wire         cpu_is_rd_o,     // 1= read 0= write
    output wire [255:0] dbg_rd_cap,
    output wire [1:0]   dbg_word_sel
);

    //------------------------------------------------------------------------
    // 0x8xxx_xxxx → 0x0xxx_xxxx
    //------------------------------------------------------------------------
    // RISC-V
    // CPU 0x8000_0000 ~ 0x8FFF_FFFF → DDR 0x0000_0000 ~ 0x0FFF_FFFF
    // MIG s_axi_araddr is 28 256MB
    // 0x0000_xxxx BootROM
    function [27:0] phys_addr;
        input [31:0] a;
        reg   [31:0] m;
        begin
            if (a[31:28] == 4'h8)
                m = a - 32'h8000_0000;      // DDR
            else
                m = a;                      // DDR
            phys_addr = m[27:0];
        end
    endfunction

    //------------------------------------------------------------------------
    // Burst geometry
    //------------------------------------------------------------------------
    // Beats per access and the width of one beat, both derived from DW.
    localparam integer BPE    = DW / 8;

    // Address of the beat that contains the requested word, and the index of
    // the word inside that beat.
    function [27:0] beat_addr(input [27:0] a);
        begin
            beat_addr = (BPE == 16) ? (a & ~28'h1F) :
                        (BPE ==  8) ? (a & ~28'h0F) :
                                      (a & ~28'h03);
        end
    endfunction

    function [2:0] word_of(input [27:0] a);
        begin
            // With a 32-bit beat the whole beat is the word that was asked
            // for, so there is nothing to select. Picking a[2] here would make
            // every access to an address with bit 2 set read the upper half of
            // the returned beats, which is always zero.
            word_of = (BPE == 16) ? a[4:2] :
                      (BPE ==  8) ? {1'b0, a[3:2]} :
                                    3'b000;
        end
    endfunction
    localparam integer WORDS  = BPE / 4;          // 32-bit words per beat
    localparam [7:0]   LEN    = (BPE == 16) ? 8'd1 : 8'd0;   // 128 -> 2 beats, 32 -> 1 beat

    reg [1:0]  src_q;       // 0=IMEM 1=DMEM 2=PTW
    reg [31:0] addr_q;
    reg [2:0]  word_sel_q;      // beat 32 DW=32 when is 0
    reg [3:0]  wstrb_q;
    reg        is_write_q = 1'b0;

    //------------------------------------------------------------------------
    // dmem imem
    //------------------------------------------------------------------------
    wire dmem_wr = dmem_awvalid && dmem_wvalid;
    wire dmem_rd = dmem_arvalid;
    // PTW read write dmem/imem translation before
    wire ptw_any = pt_req | pt_we;

    reg  busy;      // in

    // state machine before is idle state
    localparam S_IDLE = 3'd0;
    localparam S_WAIT = 3'd1;
    localparam S_RESP = 3'd2;
    localparam S_HOLD = 3'd3;   // after CPU rvalid
    reg [2:0] state;

    // ready must in S_IDLE busy
    // is busy in S_WAIT→S_RESP S_RESP ready
    // CPU is handshake success req_sent_q=1 bridge
    wire idle = !busy && ddr_ready && (state == S_IDLE);

    // " " ready S_IDLE handshake
    // PTW > DMEM > IMEM
    wire g_ptw  = idle && ptw_any;
    wire g_dmem = idle && !ptw_any && (dmem_wr || dmem_rd);
    wire g_imem = idle && !ptw_any && !dmem_wr && !dmem_rd && imem_req;

    assign imem_ack     = g_imem;
    assign dmem_arready = g_dmem && dmem_rd && !dmem_wr;
    assign dmem_awready = g_dmem && dmem_wr;
    assign dmem_wready  = g_dmem && dmem_wr;

    // PTW must in S_HOLD ready
    // is pt_rdata in S_RESP <= only then
    // in S_RESP ready MMU X
    always @(*) begin
        pt_ready = 1'b0;
        if (state == S_HOLD && src_q == 2'd2) pt_ready = 1'b1;
    end

    //------------------------------------------------------------------------
    // AXI4 Master
    //------------------------------------------------------------------------
    reg  [31:0] read_word;      // word selected out of the returned beats
    reg         m_start;
    reg  [2:0]  m_op;
    reg  [27:0] m_addr;
    reg  [DW-1:0]    m_wdata_i;
    reg  [DW-1:0]    m_wdata2_i;
    reg  [DW/8-1:0]  m_wstrb_i;
    reg  [DW/8-1:0]  m_wstrb2_i;
    reg  [7:0]       m_len     = 8'd0;
    wire        m_done;
    wire [2*DW-1:0]  m_rdata_out;   // 2 beats
    wire        m_err;

    axi4_master #(.AW(28), .DW(DW)) u_axi (
        .aclk      (clk),
        .aresetn   (!rst),
        .start     (m_start),
        .op        (m_op),
        .addr      (m_addr),
        .wdata     (m_wdata_i),
        .wdata2    (m_wdata2_i),
        .wstrb     (m_wstrb_i),
        .wstrb2    (m_wstrb2_i),
        .len       (m_len),
        .done      (m_done),
        .rdata     (m_rdata_out),
        .err       (m_err),
        .m_awid    (m_awid),
        .m_awaddr  (m_awaddr),
        .m_awlen   (m_awlen),
        .m_awsize  (m_awsize),
        .m_awburst (m_awburst),
        .m_awvalid (m_awvalid),
        .m_awready (m_awready),
        .m_wdata   (m_wdata),
        .m_wstrb   (m_wstrb),
        .m_wlast   (m_wlast),
        .m_wvalid  (m_wvalid),
        .m_wready  (m_wready),
        .m_bready  (m_bready),
        .m_bid     (m_bid),
        .m_bresp   (m_bresp),
        .m_bvalid  (m_bvalid),
        .m_arid    (m_arid),
        .m_araddr  (m_araddr),
        .m_arlen   (m_arlen),
        .m_arsize  (m_arsize),
        .m_arburst (m_arburst),
        .m_arvalid (m_arvalid),
        .m_arready (m_arready),
        .m_rready  (m_rready),
        .m_rid     (m_rid),
        .m_rdata   (m_rdata),
        .m_rresp   (m_rresp),
        .m_rlast   (m_rlast),
        .m_rvalid  (m_rvalid),
        .dbg_state (dbg_axi_state),
        .dbg_beat  (dbg_axi_beat),
        .dbg_lenq  (dbg_axi_lenq)
    );

    //------------------------------------------------------------------------
    // state machine before idle before
    //------------------------------------------------------------------------

    assign dbg_state    = state;
    assign dbg_busy     = busy;
    // dbg_axi_beat / dbg_axi_lenq are driven by the port connections above.
    assign dbg_src      = src_q;
    assign cpu_addr_o   = addr_q;
    assign axi_addr_o   = m_addr;        // the address axi4_master sees
    assign cpu_is_rd_o  = !is_write_q;
    assign dbg_rd_cap   = m_rdata_out;
    assign dbg_word_sel = word_sel_q;

    //------------------------------------------------------------------------
    // write 128 32 word_sel
    //------------------------------------------------------------------------
    // Place the 32-bit word at its lane inside the beat. With DW=32 the shift
    // is always zero and this collapses to a straight assignment.
    wire [2*DW-1:0] wdata_wide = {{DW{1'b0}}, dmem_wdata} << (word_of(dmem_awaddr) * 32);
    wire [DW/4-1:0] wstrb_wide = {{(DW/4-4){1'b0}}, dmem_wstrb} << (word_of(dmem_awaddr) * 4);

    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            busy       <= 1'b0;
            src_q      <= 1'b0;
            addr_q     <= 32'h0;
            word_sel_q <= 3'd0;
            wstrb_q    <= 4'h0;
            is_write_q <= 1'b0;
            m_start    <= 1'b0;
            m_op       <= 3'd0;
            m_addr     <= 28'h0;
            m_wdata_i  <= {DW{1'b0}};
            m_wstrb_i  <= {(DW/8){1'b0}};
            m_wdata2_i <= {DW{1'b0}};
            m_wstrb2_i <= {(DW/8){1'b0}};
            m_len      <= 8'd0;
            imem_rdata <= 32'h0;
            imem_rvalid<= 1'b0;
            dmem_rdata <= 32'h0;
            dmem_rvalid<= 1'b0;
            dmem_bvalid<= 1'b0;
        end else begin
            // clear
            imem_rvalid <= 1'b0;
            dmem_rvalid <= 1'b0;
            dmem_bvalid <= 1'b0;

            case (state)
                //==========================================================
                S_IDLE: begin
                    m_start <= 1'b0;

                    if (ddr_ready && pt_we) begin
                        // ---- PTW page table write A/D ----
                        src_q      <= 2'd2;
                        addr_q     <= pt_waddr;
                        word_sel_q <= word_of(pt_waddr);
                        wstrb_q    <= 4'b1111;
                        is_write_q <= 1'b1;
                        m_op       <= 3'd0;
                        m_addr     <= beat_addr(phys_addr(pt_waddr));
                        m_wdata_i  <= {96'd0, pt_wdata} << (pt_waddr[4:2] * 32);
                        m_wstrb_i  <= 4'hF << (pt_waddr[4:2] * 4);
                        m_wdata2_i <= 128'h0;
                        m_wstrb2_i <= 16'h0;
                        m_len      <= 8'd1;
                        m_start    <= 1'b1;
                        busy       <= 1'b1;
                        state      <= S_WAIT;
                    end else if (ddr_ready && pt_req) begin
                        // ---- PTW page table read ----
                        src_q      <= 2'd2;
                        addr_q     <= pt_addr;
                        word_sel_q <= word_of(pt_addr);
                        is_write_q <= 1'b0;
                        m_op       <= 3'd1;
                        m_addr     <= beat_addr(phys_addr(pt_addr));
                        m_len      <= 8'd1;
                        m_start    <= 1'b1;
                        busy       <= 1'b1;
                        state      <= S_WAIT;
                    end else if (ddr_ready && dmem_wr) begin
                        // ---- write ----
                        src_q      <= 1'b1;
                        addr_q     <= dmem_awaddr;
                        word_sel_q <= word_of(dmem_awaddr);
                        wstrb_q    <= dmem_wstrb;
                        is_write_q <= 1'b1;
                        m_op       <= 3'd0;             // write
                        m_addr     <= beat_addr(phys_addr(dmem_awaddr));
                        m_wdata_i  <= wdata_wide[DW-1:0];
                        m_wstrb_i  <= wstrb_wide[DW/8-1:0];
                        m_wdata2_i <= wdata_wide[2*DW-1:DW];
                        m_wstrb2_i <= wstrb_wide[DW/4-1:DW/8];
                        // A store carries exactly one beat. Asking for two
                        // makes the beat cover the neighbouring word as well,
                        // and that half lands in memory with every strobe low.
                        m_len      <= 8'd0;
                        m_start    <= 1'b1;
                        busy       <= 1'b1;
                        state      <= S_WAIT;
                    end else if (ddr_ready && dmem_rd) begin
                        // ---- read ----
                        src_q      <= 1'b1;
                        addr_q     <= dmem_araddr;
                        word_sel_q <= word_of(dmem_araddr);
                        is_write_q <= 1'b0;
                        m_op       <= 3'd1;             // read
                        m_addr     <= beat_addr(phys_addr(dmem_araddr));
                        m_len      <= LEN;
                        m_start    <= 1'b1;
                        busy       <= 1'b1;
                        state      <= S_WAIT;
                    end else if (ddr_ready && imem_req) begin
                        // ---- read ----
                        src_q      <= 1'b0;
                        addr_q     <= imem_addr;
                        word_sel_q <= word_of(imem_addr);
                        is_write_q <= 1'b0;
                        m_op       <= 3'd1;             // read
                        m_addr     <= beat_addr(phys_addr(imem_addr));
                        m_len      <= LEN;
                        m_start    <= 1'b1;
                        busy       <= 1'b1;
                        state      <= S_WAIT;
                    end
                end

                //==========================================================
                S_WAIT: begin
                    m_start <= 1'b0;

                    if (m_done) begin
                        busy  <= 1'b0;
                        state <= S_RESP;
                    end
                end

                //==========================================================
                S_RESP: begin
                    if (is_write_q) begin
                        // write
                        dmem_bvalid <= 1'b1;
                    end else begin
                        // Read completion: take the 32-bit word the request
                        // asked for out of the returned beat(s).
                        read_word = m_rdata_out[word_sel_q * 32 +: 32];

                        if (src_q == 2'd0)      imem_rdata <= read_word;
                        else if (src_q == 2'd1) dmem_rdata <= read_word;
                        else                    pt_rdata   <= read_word;

                        if (src_q == 2'd0)      imem_rvalid <= 1'b1;
                        else if (src_q == 2'd1) dmem_rvalid <= 1'b1;
                        // PTW pt_ready in rvalid
                    end

                    // S_HOLD is S_IDLE rvalid ack
                    state <= S_HOLD;
                end

                //==========================================================
                // S_HOLD rvalid
                //==========================================================
                S_HOLD: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
