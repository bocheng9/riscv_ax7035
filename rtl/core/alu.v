//=============================================================================
// File        : alu.v
// Description : RV32I arithmetic/logic unit, including the RV32M multiply logic; division lives in its own module
//=============================================================================
`include "riscv_pkg.v"

module alu (
    input  wire [4:0]  op,
    input  wire [31:0] a,
    input  wire [31:0] b,
    output reg  [31:0] y,
    output wire        zero            // a == b
);

    assign zero = (a == b);

    // 5
    wire [4:0] shamt = b[4:0];

    always @(*) begin
        case (op)
            `ALU_ADD   : y = a + b;
            `ALU_SUB   : y = a - b;
            `ALU_SLL   : y = a << shamt;
            `ALU_SLT   : y = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
            `ALU_SLTU  : y = (a < b) ? 32'd1 : 32'd0;
            `ALU_XOR   : y = a ^ b;
            `ALU_SRL   : y = a >> shamt;
            `ALU_SRA   : y = $signed(a) >>> shamt;
            `ALU_OR    : y = a | b;
            `ALU_AND   : y = a & b;
            `ALU_PASS_B: y = b;                        // LUI
            // ---- RV32M ----
            // The result comes from mul_div, which registers its operands and
            // its product so the path closes at 100 MHz. Computing it here as
            // well would instantiate a second pair of multipliers and make
            // this the slowest path in the design.
            `ALU_MUL, `ALU_MULH, `ALU_MULHSU, `ALU_MULHU: y = 32'h0;
            default    : y = 32'h0;
        endcase
    end

endmodule
