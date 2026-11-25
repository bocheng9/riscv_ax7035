//=============================================================================
// File        : mul_div.v
// Description : Iterative multiply/divide unit
// - synthesis DSP48
// - 32 LUT /
//=============================================================================
`include "riscv_pkg.v"

module mul_div (
    input  wire        clk,
    input  wire        rst_n,


    input  wire        start,          // 1
    input  wire [4:0]  op,             // ALU_MUL ~ ALU_REMU
    input  wire [31:0] a,
    input  wire [31:0] b,


    output reg  [31:0] y,
    output reg         valid           // 1
);

    //--------------------------------------------------------------------------
    // DSP48
    //--------------------------------------------------------------------------
    // The multiply operands are registered before the DSP and the products are
    // registered after it. A purely combinational 32x32 multiply needs about
    // 15 ns, which does not close at 100 MHz; splitting it costs one extra
    // cycle and closes timing.
    reg  [31:0] mul_a_q, mul_b_q;
    reg  [4:0]  mul_op_q;
    reg         mul_wait;      // one cycle between latching operands and result

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_a_q  <= 32'h0;
            mul_b_q  <= 32'h0;
            mul_op_q <= 5'd0;
            mul_wait <= 1'b0;
        end else if (start && !is_div_op) begin
            mul_a_q  <= a;
            mul_b_q  <= b;
            mul_op_q <= op;
            mul_wait <= 1'b1;
        end else begin
            mul_wait <= 1'b0;
        end
    end

    wire [63:0] mul_u  = mul_a_q * mul_b_q;
    wire signed [63:0] mul_s  = $signed(mul_a_q) * $signed(mul_b_q);
    wire signed [63:0] mul_su = $signed(mul_a_q) * $signed({1'b0, mul_b_q});

    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    wire is_div_op = (op == `ALU_DIV) || (op == `ALU_DIVU) ||
                     (op == `ALU_REM) || (op == `ALU_REMU);

    reg  [5:0]  cnt;
    reg  [31:0] quo, rem;
    reg  [31:0] dvd_r, dvs_r;
    reg  [32:0] rem_r;              // 33
    reg         busy;
    reg         is_signed;
    reg         is_rem;
    reg         dvd_neg, dvs_neg;

    wire [31:0] dvs_abs = dvs_neg ? (~dvs_r + 1'b1) : dvs_r;


    // // rem_r {, } shift left 1 after and
    wire [32:0] rem_sh = {rem_r[30:0], quo[31]};   // shift left from quo
    wire [32:0] trial  = rem_sh - {1'b0, dvs_abs};
    wire        trial_neg = trial[32];

    // from dvd_r quo
    wire [31:0] quo_sh = {quo[30:0], ~trial_neg};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy  <= 1'b0;
            valid <= 1'b0;
            y     <= 32'h0;
            cnt   <= 5'd0;
        end else begin
            valid <= 1'b0;

            // mul_wait was set when the operands were latched, so the product
            // of those operands is available on this edge.
            if (mul_wait) begin
                case (mul_op_q)
                    `ALU_MUL:    y <= mul_u[31:0];
                    `ALU_MULH:   y <= mul_s[63:32];
                    `ALU_MULHSU: y <= mul_su[63:32];
                    `ALU_MULHU:  y <= mul_u[63:32];
                    default:     y <= mul_u[31:0];
                endcase
                valid <= 1'b1;
            end

            if (start) begin
                case (op)
                    // ---- ----
                    `ALU_MUL, `ALU_MULH, `ALU_MULHSU, `ALU_MULHU: begin
                        // handled by the registered path below
                    end

                    // ---- ----
                    `ALU_DIV, `ALU_DIVU, `ALU_REM, `ALU_REMU: begin
                        if (b == 32'd0) begin

                            y     <= (op == `ALU_DIV || op == `ALU_REM) ? 32'hFFFF_FFFF : 32'hFFFF_FFFF;
                            if (op == `ALU_REM || op == `ALU_REMU) y <= a;
                            valid <= 1'b1;
                        end else if (is_signed_div(op) && a == 32'h8000_0000 && b == 32'hFFFF_FFFF) begin
                            // -2^31 / -1
                            y     <= (op == `ALU_REM) ? 32'h0 : 32'h8000_0000;
                            valid <= 1'b1;
                        end else begin
                            busy     <= 1'b1;
                            cnt      <= 6'd32;      // 32
                            // // quo = shift left rem_r
                            //   rem_r = 0
                            quo      <= (((op == `ALU_DIV) || (op == `ALU_REM)) && a[31]) ? (~a + 1'b1) : a;
                            rem_r    <= 33'h0;
                            rem      <= 32'h0;
                            dvs_r    <= b;
                            is_signed<= (op == `ALU_DIV) || (op == `ALU_REM);
                            is_rem   <= (op == `ALU_REM) || (op == `ALU_REMU);
                            dvd_neg  <= ((op == `ALU_DIV) || (op == `ALU_REM)) && a[31];
                            dvs_neg  <= ((op == `ALU_DIV) || (op == `ALU_REM)) && b[31];
                        end
                    end
                    default: ;
                endcase
            end
            else if (busy) begin
                if (cnt == 6'd0) begin
                    busy  <= 1'b0;
                    valid <= 1'b1;

                    if (is_rem) begin
                        y <= dvd_neg ? (~rem_r[31:0] + 1'b1) : rem_r[31:0];
                    end else begin
                        y <= (dvd_neg ^ dvs_neg) ? (~quo + 1'b1) : quo;
                    end
                end else begin
                    // rem_r shift left + quo and
                    if (!trial_neg) begin
                        rem_r <= trial;                 // rem = rem - dvs
                        quo   <= {quo[30:0], 1'b1};     // = 1
                    end else begin
                        rem_r <= rem_sh;                // hold
                        quo   <= {quo[30:0], 1'b0};     // = 0
                    end
                    cnt   <= cnt - 6'd1;
                end
            end
        end
    end

    function is_signed_div;
        input [4:0] o;
        begin
            is_signed_div = (o == `ALU_DIV) || (o == `ALU_REM);
        end
    endfunction

endmodule
