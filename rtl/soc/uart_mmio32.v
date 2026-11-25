`timescale 1ns / 1ps
//=============================================================================
// uart_mmio32.v
//
// 32-bit AXI4 slave wrapping the byte receiver and a small transmitter, at the
// register layout the loader expects:
//
//   +0x00  RBR / THR   read: received byte, write: byte to send
//   +0x14  LSR         bit 0 = data ready, bit 5 = transmit holding empty
//
// Only single-beat accesses are supported, which is what the CPU issues.
//=============================================================================
module uart_mmio32 #(
    parameter integer CLK_FREQ  = 100_000_000,
    parameter integer BAUD_RATE = 115_200
) (
    input  wire        clk,
    input  wire        aresetn,

    input  wire [3:0]  s_arid,
    input  wire [27:0] s_araddr,
    input  wire [7:0]  s_arlen,
    input  wire [2:0]  s_arsize,
    input  wire [1:0]  s_arburst,
    input  wire        s_arvalid,
    output reg         s_arready,

    output reg  [3:0]  s_rid,
    output reg  [31:0] s_rdata,
    output wire [1:0]  s_rresp,
    output wire        s_rlast,
    output reg         s_rvalid,
    input  wire        s_rready,

    input  wire [3:0]  s_awid,
    input  wire [27:0] s_awaddr,
    input  wire [7:0]  s_awlen,
    input  wire [2:0]  s_awsize,
    input  wire [1:0]  s_awburst,
    input  wire        s_awvalid,
    output wire        s_awready,
    input  wire [31:0] s_wdata,
    input  wire [3:0]  s_wstrb,
    input  wire        s_wlast,
    input  wire        s_wvalid,
    output wire        s_wready,
    output reg  [3:0]  s_bid,
    output reg  [1:0]  s_bresp,
    output reg         s_bvalid,
    input  wire        s_bready,

    input  wire        rxd,
    output wire        txd,

    // Status brought out for the debug probes.
    output wire [7:0]  dbg_rx_hold,
    output wire        dbg_rx_ready,
    output wire        dbg_rx_valid,
    output wire [7:0]  dbg_rx_byte,
    output wire [1:0]  dbg_rx_state,
    output wire        dbg_rx_rst,
    output wire        dbg_rxd_raw,
    output wire        dbg_rxd_sync,
    output wire [15:0] dbg_rx_cnt,

    // Set when the loader hands over to the program.
    output reg         prog_start
);


    //-------------------------------------------------------------------------
    // Receiver
    //-------------------------------------------------------------------------
    wire [7:0] rx_byte;
    wire       rx_valid;

    uart_rx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_rx (
        .clk        (clk),
        .rst        (!aresetn),
        .rxd        (rxd),
        .data       (rx_byte),
        .data_valid (rx_valid),
        .dbg_rxd_raw  (dbg_rxd_raw),
        .dbg_rxd_sync (dbg_rxd_sync),
        .dbg_state    (),
        .dbg_cnt      (dbg_rx_cnt)
    );

    // One byte of buffering: the loader polls LSR and then reads RBR, so the
    // byte has to survive between those two accesses.
    reg [7:0] rx_hold;
    reg       rx_ready;

    assign dbg_rx_hold  = rx_hold;
    assign dbg_rx_ready = rx_ready;
    assign dbg_rx_valid = rx_valid;
    assign dbg_rx_byte  = rx_byte;
    assign dbg_rx_state = u_rx.state;
    assign dbg_rx_rst   = !aresetn;

    //-------------------------------------------------------------------------
    // Transmitter
    //-------------------------------------------------------------------------
    reg  [7:0] tx_byte;
    reg        tx_start;
    wire       tx_busy;

    uart_tx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_tx (
        .clk      (clk),
        .rst_n    (aresetn),
        .tx_data  (tx_byte),
        .tx_start (tx_start),
        .tx_busy  (tx_busy),
        .txd      (txd)
    );

    //-------------------------------------------------------------------------
    // Register decode
    //-------------------------------------------------------------------------
    wire [4:0] raddr = s_araddr[4:0];
    wire [4:0] waddr = s_awaddr[4:0];

    reg [31:0] rdata_mux;
    always @(*) begin
        case (raddr)
            // Received byte.
            5'h00:   rdata_mux = {24'd0, rx_hold};
            // Line status: bit 0 data ready, bit 5 transmitter empty.
            5'h14:   rdata_mux = {26'd0, !tx_busy, 4'd0, rx_ready};
            default: rdata_mux = 32'd0;
        endcase
    end

    assign s_rresp = 2'b00;
    assign s_rlast = 1'b1;
    assign s_awready = 1'b1;
    assign s_wready  = 1'b1;

    // AXI handshakes reduced to strobes, so the register block below is the
    // only place that drives rx_ready / tx_start.
    wire rd_fire = s_arvalid && s_arready;
    wire wr_fire = s_awvalid && s_awready && s_wvalid && s_wready;
    wire rd_rbr  = rd_fire && (raddr == 5'h00);
    // 0x1C is unused by the 16550 register set and, unlike 0x20, its low five
    // bits do not collide with the data register.
    wire wr_start_flag = wr_fire && (waddr == 5'h1C);
    wire wr_thr  = wr_fire && (waddr == 5'h00);

    //-------------------------------------------------------------------------
    // All registers in one process
    //-------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!aresetn) begin
            rx_hold   <= 8'd0;
            rx_ready  <= 1'b0;
            tx_byte   <= 8'd0;
            tx_start  <= 1'b0;

            s_arready <= 1'b1;
            s_rvalid  <= 1'b0;
            s_rid     <= 4'd0;
            s_rdata   <= 32'd0;

            s_bvalid  <= 1'b0;
            s_bid     <= 4'd0;
            s_bresp   <= 2'b00;
            prog_start <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            // A write to +0x20 marks the moment the loader hands over to the
            // program. Debug blocks that must ignore the loader use this.
            if (wr_start_flag)
                prog_start <= 1'b1;

            //-----------------------------------------------------------------
            // Receive buffer
            //-----------------------------------------------------------------
            if (rx_valid) begin
                rx_hold  <= rx_byte;
                rx_ready <= 1'b1;
            end else if (rd_rbr) begin
                rx_ready <= 1'b0;       // reading the data register consumes it
            end

            //-----------------------------------------------------------------
            // Read channel
            //-----------------------------------------------------------------
            if (rd_fire) begin
                s_rdata   <= rdata_mux;
                s_rid     <= s_arid;
                s_arready <= 1'b0;
                s_rvalid  <= 1'b1;
            end else if (s_rvalid && s_rready) begin
                s_rvalid  <= 1'b0;
                s_arready <= 1'b1;
            end

            //-----------------------------------------------------------------
            // Write channel
            //-----------------------------------------------------------------
            if (wr_fire) begin
                s_bvalid <= 1'b1;
                s_bid    <= s_awid;
                s_bresp  <= 2'b00;
                if (wr_thr && !tx_busy) begin
                    tx_byte  <= s_wdata[7:0];
                    tx_start <= 1'b1;
                end
            end else if (s_bvalid && s_bready) begin
                s_bvalid <= 1'b0;
            end
        end
    end

endmodule
