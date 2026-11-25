//=============================================================================
// File        : pipe_reg.v
// Description : Pipeline register helper
//

// · valid
// · ready
// · valid && ready → this cycle success
// · !valid → bubble valid
// · valid && !ready → must hold stall
//=============================================================================
module pipe_reg #(
    parameter WIDTH = 32
)(
    input  wire             clk,
    input  wire             rst_n,

    // flush valid
    input  wire             flush,


    input  wire             in_valid,
    output wire             in_ready,
    input  wire [WIDTH-1:0] in_data,


    output wire             out_valid,
    input  wire             out_ready,
    output reg  [WIDTH-1:0] out_data
);

    reg valid_r;

    // is or
    assign in_ready  = !valid_r || out_ready;
    assign out_valid = valid_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_r  <= 1'b0;
            out_data <= {WIDTH{1'b0}};
        end else if (flush) begin
            valid_r  <= 1'b0;
        end else if (in_ready) begin
            // is bubble in_valid=0
            valid_r  <= in_valid;
            if (in_valid)
                out_data <= in_data;
        end
    end

endmodule
