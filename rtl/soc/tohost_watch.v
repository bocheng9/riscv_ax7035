`timescale 1ns / 1ps
//=============================================================================
// tohost_watch.v
//
// Records what the program writes to the tohost word.
//
// The test suite reports its verdict by storing to a fixed address; the value
// 1 means the case passed. The CPU owns the bus while it runs, so the word
// cannot be read back over JTAG; watching the write is the only way to see it
// without stopping the core.
//
// A write is captured when the address matches and both the address and data
// channels are accepted on the same cycle. Both conditions are checked because
// the bridge issues one beat per access, so the two always coincide.
//=============================================================================
module tohost_watch #(
    parameter [31:0] TOHOST_ADDR = 32'h8000_1000
) (
    input  wire        clk,
    input  wire        aresetn,

    // AXI write address channel
    input  wire [27:0] awaddr,
    input  wire        awvalid,
    input  wire        awready,
    // AXI write data channel
    input  wire [31:0] wdata,
    input  wire        wvalid,
    input  wire        wready,

    // The unaliased CPU address, so the comparison is against what the program
    // actually targeted rather than the address after the DDR offset is applied.
    input  wire [31:0] cpu_addr,

    output reg  [31:0] tohost_value,
    output reg  [31:0] tohost_count   // number of stores seen
);

    // The address and data channels are not guaranteed to handshake on the
    // same cycle, so the two are tracked separately: the address match arms a
    // pending flag, and the data handshake that follows records the value.
    reg pending;

    wire aw_hit = (cpu_addr == TOHOST_ADDR) && awvalid && awready;
    wire w_hit  = wvalid && wready;

    always @(posedge clk) begin
        if (!aresetn) begin
            tohost_value <= 32'h0;
            tohost_count <= 32'h0;
            pending      <= 1'b0;
        end else begin
            if (aw_hit) begin
                pending <= 1'b1;
            end else if (pending && w_hit) begin
                tohost_value <= wdata;
                tohost_count <= tohost_count + 32'd1;
                pending      <= 1'b0;
            end
        end
    end

endmodule
