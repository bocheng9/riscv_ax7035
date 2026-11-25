`timescale 1ns / 1ps
//=============================================================================
// uart_rx.v
//
// Byte receiver for the loader. Samples the line at 16x the baud rate, finds
// the start bit in the middle of the bit period, and presents each received
// byte with a one-cycle strobe.
//
// The loader polls DATA_VALID; there is no interrupt path, which keeps the
// boot code free of trap handling.
//=============================================================================
module uart_rx #(
    parameter integer CLK_FREQ  = 100_000_000,
    parameter integer BAUD_RATE = 115_200
) (
    input  wire       clk,
    input  wire       rst,          // active high
    input  wire       rxd,          // serial input, idle high

    output reg  [7:0] data,
    output reg        data_valid,   // one cycle per received byte

    // Observation points for the debug probes.
    output wire        dbg_rxd_raw,
    output wire        dbg_rxd_sync,
    output wire [1:0]  dbg_state,
    output wire [15:0] dbg_cnt
);

    assign dbg_rxd_raw  = rxd;
    assign dbg_rxd_sync = rxd_sync;
    assign dbg_state    = state;
    assign dbg_cnt      = cnt;

    localparam integer DIV = CLK_FREQ / BAUD_RATE;      // clocks per bit
    localparam integer MID = DIV / 2;

    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] cnt;         // clocks within the current bit
    reg [2:0]  bit_idx;
    reg [7:0]  shift;

    // Two flip-flops so a metastable input cannot reach the state machine.
    reg rxd_meta, rxd_sync;
    always @(posedge clk) begin
        if (rst) begin
            // Start from the idle level so a pull-up line does not look like
            // the beginning of a frame, and so the state machine never sees X.
            rxd_meta <= 1'b1;
            rxd_sync <= 1'b1;
        end else begin
            rxd_meta <= rxd;
            rxd_sync <= rxd_meta;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            cnt        <= 16'd0;
            bit_idx    <= 3'd0;
            shift      <= 8'd0;
            data       <= 8'd0;
            data_valid <= 1'b0;
        end else begin
            data_valid <= 1'b0;

            case (state)
                //-----------------------------------------------------------------
                // Wait for a falling edge, then check again at mid-bit so a
                // glitch does not start a frame.
                //-----------------------------------------------------------------
                S_IDLE: begin
                    if (!rxd_sync) begin
                        cnt   <= 16'd0;
                        state <= S_START;
                    end
                end

                S_START: begin
                    if (cnt == MID - 1) begin
                        if (!rxd_sync) begin
                            cnt     <= 16'd0;
                            bit_idx <= 3'd0;
                            state   <= S_DATA;
                        end else begin
                            state <= S_IDLE;    // false start
                        end
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                //-----------------------------------------------------------------
                // Eight data bits, least significant first, sampled mid-bit.
                //-----------------------------------------------------------------
                S_DATA: begin
                    if (cnt == DIV - 1) begin
                        cnt   <= 16'd0;
                        shift <= {rxd_sync, shift[7:1]};
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                //-----------------------------------------------------------------
                // Stop bit: the byte is complete at mid-bit regardless of the
                // value, so a framing error does not stall the loader.
                //-----------------------------------------------------------------
                S_STOP: begin
                    if (cnt == MID - 1) begin
                        data       <= shift;
                        data_valid <= 1'b1;
                        cnt        <= 16'd0;
                        state      <= S_IDLE;
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end
            endcase
        end
    end

endmodule
