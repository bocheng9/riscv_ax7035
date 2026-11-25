//=============================================================================
// File        : regfile.v
// Description : 32 x 32-bit register file, two read ports and one write port
// - x0 is 0
// - write read write after read
// - read write
//=============================================================================
module regfile (
    input  wire        clk,
    input  wire        rst_n,

    // read A
    input  wire [4:0]  raddr_a,
    output wire [31:0] rdata_a,
    // read B
    input  wire [4:0]  raddr_b,
    input  wire [4:0]  raddr_c,
    input  wire [4:0]  raddr_d,
    input  wire [4:0]  raddr_e,
    output wire [31:0] rdata_b,
    output wire [31:0] rdata_c,
    output wire [31:0] rdata_d,
    output wire [31:0] rdata_e,

    // write
    input  wire        we,
    input  wire [4:0]  waddr,
    input  wire [31:0] wdata
);

    reg [31:0] regs [1:31];   // 0

    //--------------------------------------------------------------------------
    // read write
    //--------------------------------------------------------------------------
    assign rdata_a = (raddr_a == 5'd0)                    ? 32'h0 :
                     (we && (waddr == raddr_a) && waddr!=0) ? wdata : regs[raddr_a];

    assign rdata_b = (raddr_b == 5'd0)                    ? 32'h0 :
                     (we && (waddr == raddr_b) && waddr!=0) ? wdata : regs[raddr_b];

    assign rdata_c = (raddr_c == 5'd0)                    ? 32'h0 :
                     (we && (waddr == raddr_c) && waddr!=0) ? wdata : regs[raddr_c];

    assign rdata_d = (raddr_d == 5'd0)                    ? 32'h0 :
                     (we && (waddr == raddr_d) && waddr!=0) ? wdata : regs[raddr_d];

    assign rdata_e = (raddr_e == 5'd0)                    ? 32'h0 :
                     (we && (waddr == raddr_e) && waddr!=0) ? wdata : regs[raddr_e];

    //--------------------------------------------------------------------------
    // write x0 write
    //--------------------------------------------------------------------------
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 1; i < 32; i = i + 1)
                regs[i] <= 32'h0;
        end else if (we && (waddr != 5'd0)) begin
            regs[waddr] <= wdata;
        end
    end

endmodule
