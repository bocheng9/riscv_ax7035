//=============================================================================
// gp_trace.v
//
// Records the sequence of gp (x3) values a test program passes through, and
// sends it out over the console once the program has settled.
//
// riscv-tests sets gp to the sub-test number before each sub-test and to 1 on
// the pass path, so the sequence of changes is a compact record of which
// sub-tests ran and in what order. Comparing it against the reference model
// answers "did the same path execute" without capturing every instruction.
//
// The CPU cannot help with the export: the tests end in a self-branch at
// write_tohost and never return to the loader. So this block watches gp, and
// once gp has been unchanged for STABLE_CYCLES it streams the buffer out.
//
// Output, little endian:
//   0xCC 0x33
//   4 bytes   entry count
//   per entry 4 bytes gp, 4 bytes instret low word
//   0x3C
//
// The console transmitter is a separate instance, so this module only has to
// present bytes one at a time and wait for the busy flag to drop.
//=============================================================================
module gp_trace #(
    parameter DEPTH         = 512,
    parameter ADDR_W        = 9,
    parameter STABLE_CYCLES = 24'd2_000_000,

    parameter CLK_FREQ      = 100_000_000,
    parameter BAUD_RATE     = 115_200
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] gp_i,
    input  wire [31:0] id_pc_i,     // gp pc

    // Recording is only meaningful once the program is running; before that gp
    // holds an unknown value and every cycle would look like a change.
    input  wire        enable,

    output wire        txd,
    output wire        busy,

    // Debug: what the exporter is doing right now.
    output wire [7:0]  dbg_state,
    output wire [ADDR_W:0] dbg_count
);

    //------------------------------------------------------------------
    // Recording
    //------------------------------------------------------------------
    reg [63:0] mem [0:DEPTH-1];

    reg [31:0]     gp_q;
    reg [ADDR_W:0] wr_idx;
    reg [23:0]     idle_cnt;
    reg            settled;

    always @(posedge clk) begin
        if (!rst_n) begin
            gp_q     <= 32'hFFFF_FFFF;   // sentinel: the first real value records
            wr_idx   <= 0;
            idle_cnt <= 0;
            settled  <= 1'b0;
        end else if (!enable) begin
            // waiting for the program to start
        end else if (gp_i !== gp_q) begin
            gp_q <= gp_i;
            if (wr_idx < DEPTH)
                mem[wr_idx[ADDR_W-1:0]] <= {id_pc_i, gp_i};
            wr_idx   <= wr_idx + 1'b1;
            idle_cnt <= 0;
        end else begin
            if (idle_cnt != STABLE_CYCLES)
                idle_cnt <= idle_cnt + 1'b1;
            else if (!settled) begin
                // Record where the program came to rest: gp stopped changing,
                // so this entry is the only clue to which instruction it is
                // stuck on.
                if (wr_idx < DEPTH)
                    mem[wr_idx[ADDR_W-1:0]] <= {id_pc_i, 32'hDEAD_BEEF};
                wr_idx   <= wr_idx + 1'b1;
                settled  <= 1'b1;
            end
        end
    end

    //------------------------------------------------------------------
    // Byte source
    //
    // A small counter walks the export: marker, count, then four bytes of gp
    // and four of instret per entry, then the end marker.
    //------------------------------------------------------------------
    reg [2:0]  phase;        // 0 idle, 1 marker, 2 count, 3 entries, 4 end
    reg [3:0]  sub;          // byte within the current field
    reg [ADDR_W-1:0] rd_idx;
    reg [31:0] cur_gp, cur_inst;
    reg [7:0]  out_byte;
    reg        out_valid;
    reg        started;

    // Zero-extend so slicing the four bytes never runs off the end.
    wire [31:0] count = {{(32-ADDR_W-1){1'b0}}, wr_idx};

    always @(posedge clk) begin
        if (!rst_n) begin
            phase     <= 3'd0;
            sub       <= 4'd0;
            rd_idx    <= 0;
            cur_gp    <= 32'h0;
            cur_inst  <= 32'h0;
            out_byte  <= 8'h0;
            out_valid <= 1'b0;
            started   <= 1'b0;
        end else begin
            // Hold the request until the transmitter reports busy: tx_start is
            // sampled a cycle after it is asserted, so dropping the request on
            // the same cycle would let the next byte overwrite the data.
            if (busy)
                out_valid <= 1'b0;
            else if (out_valid)
                out_valid <= 1'b1;

            if (!settled) begin
                phase <= 3'd0;
            end else begin
                case (phase)
                    3'd0: begin
                        if (!started) begin
                            started <= 1'b1;
                            rd_idx  <= 0;
                            sub     <= 4'd0;
                            phase   <= 3'd1;
                        end
                    end

                    // 0xCC then 0x33
                    3'd1: begin
                        if (!busy && !out_valid) begin
                            out_byte  <= (sub == 4'd0) ? 8'hCC : 8'h33;
                            out_valid <= 1'b1;
                            if (sub == 4'd0) begin
                                sub <= 4'd1;
                            end else begin
                                sub   <= 4'd0;
                                phase <= 3'd2;
                            end
                        end
                    end

                    // Entry count, four bytes little endian.
                    3'd2: begin
                        if (!busy && !out_valid) begin
                            out_byte  <= count[sub*8 +: 8];
                            out_valid <= 1'b1;
                            if (sub == 4'd3) begin
                                sub   <= 4'd0;
                                phase <= 3'd3;
                            end else begin
                                sub <= sub + 4'd1;
                            end
                        end
                    end

                    // gp then instret, four bytes each, for every entry.
                    3'd3: begin
                        // sub 0 fetches the entry and sends nothing: a
                        // non-blocking read is not visible until the next
                        // cycle, so sending on the same cycle would put the
                        // previous entry's byte on the wire. Bytes go out on
                        // sub 1..8 instead.
                        if (!busy && !out_valid) begin
                            if (sub == 4'd0) begin
                                {cur_inst, cur_gp} <= mem[rd_idx];
                                sub <= 4'd1;
                            end else begin
                                out_byte  <= (sub <= 4'd4) ? cur_gp[(sub-1)*8 +: 8]
                                                           : cur_inst[(sub-5)*8 +: 8];
                                out_valid <= 1'b1;
                                if (sub == 4'd8) begin
                                    sub <= 4'd0;
                                    if (rd_idx + 1 >= count)
                                        phase <= 3'd4;
                                    else
                                        rd_idx <= rd_idx + 1'b1;
                                end else begin
                                    sub <= sub + 4'd1;
                                end
                            end
                        end
                    end

                    3'd4: begin
                        if (!busy && !out_valid) begin
                            out_byte  <= 8'h3C;
                            out_valid <= 1'b1;
                            phase     <= 3'd5;
                        end
                    end

                    default: ;   // done; the console holds its last byte
                endcase
            end
        end
    end

    //------------------------------------------------------------------
    // Console transmitter
    //------------------------------------------------------------------
    assign dbg_state = {settled, enable, phase};
    assign dbg_count = wr_idx;

    uart_tx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_tx (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (out_byte),
        .tx_start (out_valid),
        .txd      (txd),
        .tx_busy  (busy)
    );

endmodule
