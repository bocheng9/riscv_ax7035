//============================================================================
// File        : if_stage.v
// Description : Instruction fetch stage (instruction FIFO plus an epoch tag)
//
// Design points
//   1. Instruction FIFO, depth 4: holds instructions that have come back but
//      have not been taken by ID yet.
//   2. Epoch tag: a redirect bumps the epoch, and any instruction in the FIFO
//      carrying a stale epoch is discarded on the fly. That removes the need
//      for per-request tracking such as req_kill or resp_is_target.
//   3. One outstanding request at a time: the bridge is serial, so only one
//      fetch can be in flight.
//   4. Standard AXI handshake: instr_req_o is held until instr_ack_i.
//
// State machine, three states
//   F_IDLE : idle; may issue a request, or present an instruction from the FIFO
//   F_REQ  : request issued, waiting for ack
//   F_RESP : acked, waiting for rvalid
//
// Redirect handling
//   On the branch_i cycle the epoch is bumped and pc_next becomes the target,
//   so any in-flight request is now tagged with a stale epoch and its response
//   is dropped when it arrives. Instructions already in the FIFO are cleared
//   for the same reason.
//
//============================================================================
module if_stage #(
    parameter FIFO_DEPTH = 4,
    parameter RESET_PC_V = 32'h0000_0000   // reset PC BootROM=0x0 ISA =0x8000_0000
)(
    input  wire        clk,
    input  wire        rst_n,               // reset

    // ---- ----
    input  wire        branch_i,
    input  wire [31:0] branch_addr_i,

    // ---- AXI4-Lite read ----
    output wire [31:0] instr_addr_o,        // = araddr
    output wire [2:0]  instr_prot_o,        // = arprot 3'b100 = + +
    output wire        instr_req_o,         // = arvalid hold ack
    input  wire        instr_ack_i,         // = arready
    input  wire [31:0] instr_rdata_i,       // = rdata
    input  wire [1:0]  instr_resp_i,        // = rresp OKAY when access fault
    input  wire        instr_rvalid_i,      // = rvalid
    output wire        instr_rready_o,      // = rready 1 FIFO

    // ---- ID FIFO ----
    output wire        instr_valid_o,
    output wire [31:0] instr_pc_o,
    output wire [31:0] instr_data_o,
    input  wire        instr_ready_i,

    // ---- MMU ----
    input  wire        mmu_stall_i,
    input  wire        mmu_fault_i,
    input  wire [3:0]  mmu_fault_cause_i,
    output wire        if_fault_o,
    output wire [3:0]  if_fault_cause_o,

    // debug
    output wire [31:0] dbg_pc_next,
    output wire [1:0]  dbg_epoch,
    output wire [1:0]  dbg_req_epoch,
    output wire [1:0]  dbg_fifo_ep,
    output wire [2:0]  dbg_fifo_cnt,
    // The fetch address is wanted. Declared before the translation can stall
    // the fetcher, so an address translation can start one cycle earlier than
    // the request itself.
    output wire        instr_want_o,
    output wire [1:0]  dbg_state
);

    localparam RESET_PC = RESET_PC_V;

    //========================================================================

    //========================================================================
    localparam F_IDLE = 2'd0;
    localparam F_REQ  = 2'd1;
    localparam F_RESP = 2'd2;

    reg [1:0]  state;
    reg [31:0] pc_next;
    reg [31:0] req_addr;         // in bridge before
    reg [1:0]  epoch;            // before
    reg [1:0]  req_epoch;        // in
    // and req_addr after
    reg [31:0] inflight_addr;
    reg [1:0]  inflight_epoch;
    reg        inflight_valid;     // bridge

    //------------------------------------------------------------------------
    // FIFO 4
    // {pc, data, epoch}
    //------------------------------------------------------------------------
    reg [31:0] fifo_pc   [0:FIFO_DEPTH-1];
    reg [31:0] fifo_data [0:FIFO_DEPTH-1];
    reg [1:0]  fifo_ep   [0:FIFO_DEPTH-1];
    reg        fifo_flt  [0:FIFO_DEPTH-1];      // fetch fault for this entry
    reg [3:0]  fifo_fc   [0:FIFO_DEPTH-1];      // fault cause for this entry
    reg [2:0]  fifo_cnt;
    reg [1:0]  wr_ptr;           // write
    reg [1:0]  rd_ptr;           // read

    wire fifo_empty = (fifo_cnt == 3'd0);
    wire fifo_full  = (fifo_cnt == FIFO_DEPTH[2:0]);

    //========================================================================
    // AXI
    //========================================================================
    // The address must be valid in the cycle the request is issued, and the
    // request is issued from F_IDLE where req_addr has not been latched yet.
    // Driving pc_next until the request is registered keeps the address and
    // the request in step, which matters as soon as translation sits between
    // the fetcher and memory.
    assign instr_addr_o = (state == F_REQ) ? req_addr : pc_next;
    assign instr_prot_o = 3'b100;      // + + IF
    assign instr_rready_o = 1'b1;      // IF epoch
    assign instr_req_o  = (state == F_REQ);

    //========================================================================
    // FIFO epoch must before epoch only then
    // after epoch " " after
    //========================================================================
    wire        head_ep_ok = !fifo_empty && (fifo_ep[rd_ptr] == epoch);
    assign instr_valid_o = head_ep_ok;
    // must and instr_data_o FIFO
    // before is cur_pc pc/data → PC and
    assign instr_pc_o    = fifo_pc[rd_ptr];
    assign instr_data_o  = fifo_data[rd_ptr];

    // debug
    assign dbg_pc_next  = pc_next;
    assign dbg_epoch    = epoch;
    assign dbg_req_epoch= req_epoch;
    assign dbg_fifo_ep  = fifo_ep[rd_ptr];
    assign dbg_fifo_cnt = fifo_cnt;
    assign dbg_state    = state;
    // fault from and pc/data
    assign if_fault_o       = head_ep_ok && fifo_flt[rd_ptr];
    // The cause is only meaningful when this entry actually faulted; reporting
    // the raw fifo contents otherwise propagates an uninitialised value into
    // mcause.
    assign if_fault_cause_o = (head_ep_ok && fifo_flt[rd_ptr]) ? fifo_fc[rd_ptr] : 4'd0;

    // FIFO
    wire fifo_pop = head_ep_ok && instr_ready_i;

    //========================================================================

    // -
    // - in avoid using a stale pc_next
    // - FIFO
    // - MMU
    //========================================================================
    wire fetch_wanted = (state == F_IDLE) && !branch_i && !fifo_full;
    assign instr_want_o = fetch_wanted;
    wire can_start = fetch_wanted && !mmu_stall_i;

    //========================================================================

    //========================================================================
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= F_IDLE;
            pc_next          <= RESET_PC;
            req_addr         <= RESET_PC;
            inflight_addr    <= RESET_PC;
            inflight_epoch   <= 2'd0;
            inflight_valid   <= 1'b0;
            epoch            <= 2'd0;
            req_epoch        <= 2'd0;
            fifo_cnt         <= 3'd0;
            wr_ptr           <= 2'd0;
            rd_ptr           <= 2'd0;
            for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
                fifo_pc[i]   <= RESET_PC;
                fifo_data[i] <= 32'h00000013;   // NOP
                fifo_ep[i]   <= 2'd0;
                fifo_flt[i]  <= 1'b0;
                fifo_fc[i]   <= 4'd0;
            end
        end else begin

            //================================================================
            // ① epoch++ pc_next = target
            // in req_epoch FIFO
            //================================================================
            if (branch_i) begin
                epoch          <= epoch + 2'd1;
                inflight_valid <= 1'b0;    // in
                pc_next  <= branch_addr_i;
                state    <= F_IDLE;
                fifo_cnt <= 3'd0;
                rd_ptr   <= 2'd0;
                wr_ptr   <= 2'd0;
            end

            //================================================================
            // ② issue the request not issued on a redirect cycle avoid using a stale pc_next
            //================================================================
            // ② issue the request not issued on a redirect cycle avoid using a stale pc_next
            // bridge ack is ack busy
            // must in ack inflight —— ack only then bridge
            if (!branch_i && can_start) begin
                if (instr_ack_i) begin
                    // ack F_RESP
                    state          <= F_RESP;
                    req_addr       <= pc_next;
                    req_epoch      <= epoch;
                    inflight_addr  <= pc_next;
                    inflight_epoch <= epoch;
                    inflight_valid <= 1'b1;
                    pc_next        <= pc_next + 32'd4;
                end else begin
                    // ack F_REQ
                    state     <= F_REQ;
                    req_addr  <= pc_next;
                    req_epoch <= epoch;
                    pc_next   <= pc_next + 32'd4;
                end
            end

            //================================================================
            // ③ ack
            //================================================================
            if (!branch_i && (state == F_REQ) && instr_ack_i) begin
                state          <= F_RESP;
                inflight_addr  <= req_addr;    // bridge is
                inflight_epoch <= req_epoch;
                inflight_valid <= 1'b1;
            end

            //================================================================
            // ④ rvalid —— // bridge/MIG rvalid is
            // epoch state machine
            //================================================================
            if (instr_rvalid_i) begin
                // bridge S_HOLD rvalid when ack
                // F_RESP rvalid is
                if (state == F_RESP) begin
                    state          <= F_IDLE;
                    inflight_valid <= 1'b0;
                end
                // fifo_cnt clear +1
                // epoch is after head_ep_ok →
                if ((state == F_RESP) && (inflight_epoch == epoch) && !branch_i) begin
                    fifo_pc[wr_ptr]   <= inflight_addr;   // is req_addr
                    fifo_data[wr_ptr] <= instr_rdata_i;
                    fifo_ep[wr_ptr]   <= inflight_epoch;
                    fifo_flt[wr_ptr]  <= mmu_fault_i;
                    fifo_fc[wr_ptr]   <= mmu_fault_cause_i;
                    wr_ptr            <= wr_ptr + 2'd1;
                    fifo_cnt          <= fifo_cnt + 3'd1;
                end
            end

            //============================================================
            // ⑤ FIFO FIFO
            // and ④ when cnt +1 -1
            //============================================================
            if (fifo_pop && !branch_i) begin
                rd_ptr <= rd_ptr + 2'd1;
                // must and ④ inflight_epoch is req_epoch
                if (instr_rvalid_i && (state == F_RESP) &&
                    (inflight_epoch == epoch)) begin
                    fifo_cnt <= fifo_cnt;           // when +
                end else begin
                    fifo_cnt <= fifo_cnt - 3'd1;
                end
            end

        end
    end

endmodule
