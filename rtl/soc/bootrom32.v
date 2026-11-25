`timescale 1ns / 1ps
//=============================================================================
// bootrom32.v
//
// Read-only 32-bit AXI4 slave holding the loader.
//
// The CPU fetches one word at a time, so every read is a single beat: arlen is
// ignored beyond accepting it, and rlast is asserted on the only beat. This is
// much simpler than the wide multi-beat version the old 128-bit bus needed, and
// it matches the 32-bit bridge exactly.
//
// Words are addressed by araddr[ADDR_W-1:2]; one word per address.
//=============================================================================
module bootrom32 #(
    parameter ADDR_W    = 12,               // 4 KB
    parameter DEPTH     = 1024,             // 4 KB / 4 = 1024 words
    parameter INIT_FILE = ""
) (
    input  wire        clk,
    input  wire        aresetn,

    // read address
    input  wire [3:0]  s_arid,
    input  wire [27:0] s_araddr,
    input  wire [7:0]  s_arlen,
    input  wire [2:0]  s_arsize,
    input  wire [1:0]  s_arburst,
    input  wire        s_arvalid,
    output reg         s_arready,

    // read data
    output reg  [3:0]  s_rid,
    output wire [31:0] s_rdata,
    output wire [1:0]  s_rresp,
    output wire        s_rlast,
    output reg         s_rvalid,
    input  wire        s_rready,

    // write channel, accepted and discarded: the ROM is read-only
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
    input  wire        s_bready
);

    reg [31:0] mem [0:DEPTH-1];
    integer i;

    initial begin
        for (i = 0; i < DEPTH; i = i + 1) mem[i] = 32'h00000013;   // nop
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end

    //-------------------------------------------------------------------------
    // Read: the address is presented combinationally, so a single cycle after
    // the handshake the data is valid.
    //-------------------------------------------------------------------------
    reg [9:0] r_addr_q;
    reg       r_active;

    assign s_rdata = mem[r_addr_q];
    assign s_rresp = 2'b00;
    assign s_rlast = 1'b1;             // one beat is always the last

    always @(posedge clk) begin
        if (!aresetn) begin
            s_arready <= 1'b1;
            s_rvalid  <= 1'b0;
            s_rid     <= 4'd0;
            r_addr_q  <= 10'd0;
            r_active  <= 1'b0;
        end else begin
            if (s_arvalid && s_arready) begin
                r_addr_q  <= s_araddr[ADDR_W-1:2];
                s_rid     <= s_arid;
                s_arready <= 1'b0;
                s_rvalid  <= 1'b1;
                r_active  <= 1'b1;
            end else if (s_rvalid && s_rready) begin
                s_rvalid  <= 1'b0;
                s_arready <= 1'b1;
                r_active  <= 1'b0;
            end
        end
    end

    //-------------------------------------------------------------------------
    // Write: accepted and dropped so a stray store cannot hang the bus.
    //-------------------------------------------------------------------------
    assign s_awready = 1'b1;
    assign s_wready  = 1'b1;

    always @(posedge clk) begin
        if (!aresetn) begin
            s_bvalid <= 1'b0;
            s_bid    <= 4'd0;
            s_bresp  <= 2'b00;
        end else if (s_awvalid && s_awready && s_wvalid && s_wready) begin
            s_bvalid <= 1'b1;
            s_bid    <= s_awid;
            s_bresp  <= 2'b00;
        end else if (s_bvalid && s_bready) begin
            s_bvalid <= 1'b0;
        end
    end

endmodule
