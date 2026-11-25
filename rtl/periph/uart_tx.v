//============================================================================
// File        : uart_tx.v
// Description : UART transmitter
//
// :
// = when /
//           50MHz / 115200 = 434
//============================================================================
module uart_tx #(
    parameter integer CLK_FREQ  = 50_000_000,   // 50MHz when
    parameter integer BAUD_RATE = 115_200        // 115200 8N1
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] tx_data,
    input  wire       tx_start,
    output reg        tx_busy,      // is 1
    output reg        txd           // UART TX
);

    localparam integer DIV = (CLK_FREQ / BAUD_RATE < 4) ? 4 : (CLK_FREQ / BAUD_RATE);

    // state machine
    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] baud_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  data_r;

    wire baud_tick = (baud_cnt == DIV - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            baud_cnt <= 16'd0;
            bit_idx  <= 3'd0;
            data_r   <= 8'd0;
            tx_busy  <= 1'b0;
            txd      <= 1'b1;       // when TX is
        end else begin
            case (state)
                S_IDLE: begin
                    txd      <= 1'b1;
                    baud_cnt <= 16'd0;
                    bit_idx  <= 3'd0;
                    if (tx_start) begin
                        data_r  <= tx_data;
                        tx_busy <= 1'b1;
                        state   <= S_START;
                    end else begin
                        tx_busy <= 1'b0;
                    end
                end

                S_START: begin
                    txd <= 1'b0;
                    if (baud_tick) begin
                        baud_cnt <= 16'd0;
                        state    <= S_DATA;
                    end else begin
                        baud_cnt <= baud_cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    txd <= data_r[bit_idx];         // LSB first
                    if (baud_tick) begin
                        baud_cnt <= 16'd0;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 16'd1;
                    end
                end

                S_STOP: begin
                    txd <= 1'b1;
                    if (baud_tick) begin
                        baud_cnt <= 16'd0;
                        tx_busy  <= 1'b0;
                        state    <= S_IDLE;
                    end else begin
                        baud_cnt <= baud_cnt + 16'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
