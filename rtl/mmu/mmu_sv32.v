//=============================================================================
// File        : mmu_sv32.v
// Description : Sv32 page-table walker
// - 2 page table 4KB + 4MB
// - page table walk state machine PTW
// - TLB
// - A/D
//
// Sv32 virtual address 32
// [31:22] VPN[1] 10
// [21:12] VPN[0] 10
// [11:0] offset 12
//
// page table PTE 4
//     [31:10] PPN
//     [9:8]   RSW
//     [7]     D
//     [6]     A
//     [5]     G
//     [4]     U
//     [3]     X
//     [2]     W
//     [1]     R
//     [0]     V
//=============================================================================
`include "riscv_pkg.v"

module mmu_sv32 (
    input  wire        clk,
    input  wire        rst_n,

    // ---- CPU ----
    input  wire [31:0] va,
    input  wire        req,
    input  wire        is_store,
    input  wire        is_fetch,
    input  wire [1:0]  priv,

    // ---- ----
    output wire [31:0] pa,
    output wire        hit,             // TLB hit, pa is valid
    output wire        fault,           // page fault
    output wire [3:0]  fault_cause,     // 12/13/15

    // ---- satp ----
    input  wire [31:0] satp,
    output wire        satp_mode,       // bit31: 1=Sv32, 0=Bare

    // ---- mstatus ----
    input  wire        sum,
    input  wire        mxr,

    // ---- page table read ----
    output reg  [31:0] pt_addr,
    output reg         pt_req,
    input  wire [31:0] pt_rdata,
    input  wire        pt_ready,

    // ---- page table write A/D ----
    output reg  [31:0] pt_waddr,
    output reg  [31:0] pt_wdata,
    output reg         pt_we,

    // ---- flush ----
    input  wire        fault_ack,      // CPU fault
    input  wire        sfence_vma,
    input  wire        sfence_va_en,
    input  wire [31:0] sfence_va
);

    //==========================================================================
    // satp
    //==========================================================================
    assign satp_mode = satp[31];
    wire [8:0]  satp_asid = satp[30:22];
    wire [21:0] satp_ppn  = satp[21:0];

    //==========================================================================
    // TLB
    //==========================================================================
    wire [31:0] tlb_pa;
    wire        tlb_hit;
    wire [5:0]  tlb_perm;
    wire [5:0]  tlb_perm_dbg;

    wire        tlb_fill_en;
    wire [31:0] tlb_fill_va, tlb_fill_pa;

    // Eight entries keeps the match and replacement trees shallow enough to
    // close at 100 MHz; locality in these tests and in Linux is high enough that
    // a small TLB still mostly hits.
    tlb #(.ENTRIES(8)) u_tlb (
        .clk(clk), .rst_n(rst_n),
        .va(va), .asid(satp_asid),
        .is_store(is_store), .is_fetch(is_fetch),
        .pa(tlb_pa), .hit(tlb_hit), .perm(tlb_perm),
        .fill_en(tlb_fill_en), .fill_va(tlb_fill_va), .fill_pa(tlb_fill_pa),
        .fill_asid(satp_asid), .fill_size(ptw_size),
        .fill_r(ptw_perm_r), .fill_w(ptw_perm_w),
        .fill_x(ptw_perm_x), .fill_u(ptw_perm_u), .fill_a(pte_a), .fill_d(pte_d),
        .flush_all(sfence_vma && !sfence_va_en),
        .flush_va_en(sfence_va_en), .flush_va(sfence_va),
        .dbg_perm_idx(tlb_perm_dbg)
    );

    //==========================================================================
    // page table walk state machine
    //==========================================================================
    localparam S_IDLE  = 3'd0;
    localparam S_L1    = 3'd1;   // read page table
    localparam S_L1_W  = 3'd2;
    localparam S_L2    = 3'd3;   // read page table
    localparam S_L2_W  = 3'd4;
    localparam S_UPD   = 3'd5;   // A/D
    localparam S_DONE  = 3'd6;

    reg [2:0]  state;
    reg [31:0] va_lat;
    reg        is_store_lat, is_fetch_lat;
    reg [1:0]  priv_lat;

    reg [31:0] pte;
    reg [21:0] ppn_base;
    reg [9:0]  vpn1, vpn0;

    reg [31:0] result_pa;
    reg        result_fault;
    reg [3:0]  result_cause;

    // PTW TLB
    reg        ptw_valid;
    reg [31:0] ptw_va, ptw_pa;
    reg [1:0]  ptw_size;
    reg        ptw_perm_r, ptw_perm_w, ptw_perm_x, ptw_perm_u;

    //==========================================================================

    //==========================================================================
    // Bare or M
    wire bypass = (!satp_mode) || (priv == `PRV_M);

    // Python RVSIMU_V3 TLB walk
    assign pa = bypass ? va : (tlb_hit ? tlb_pa : result_pa);
    // hit = " translation has completed " success or fault
    // hit fault is
    assign hit = bypass || tlb_hit || ptw_valid;

    // permission SweRV / CVA6 permission in MMU in TLB
    //   perm = {d, a, u, x, w, r}
    function automatic perm_check;
        input [5:0] p;
        input       f_fetch;
        input       f_store;
        input [1:0] f_priv;
        input       f_sum;
        input       f_mxr;
        reg p_r, p_w, p_x, p_u, p_a, p_d;
        reg bad;
        begin
            p_r = p[0]; p_w = p[1]; p_x = p[2];
            p_u = p[3]; p_a = p[4]; p_d = p[5];
            bad = 1'b0;
            if (f_fetch) begin
                // must X U in U
                if (!p_x) bad = 1'b1;
                if (p_u && f_priv != `PRV_U) bad = 1'b1;
            end else begin
                if (f_store) begin
                    if (!p_w) bad = 1'b1;
                    if (p_u && f_priv != `PRV_U && !(f_priv == `PRV_S && f_sum)) bad = 1'b1;
                end else begin
                    if (!(p_r || (f_mxr && p_x))) bad = 1'b1;
                    if (p_u && f_priv != `PRV_U && !(f_priv == `PRV_S && f_sum)) bad = 1'b1;
                end
                if (!p_u && f_priv == `PRV_U) bad = 1'b1;
                // A/D A=0 fault D=0 write fault
                if (!p_a) bad = 1'b1;
                if (f_store && !p_d) bad = 1'b1;
            end
            perm_check = bad;
        end
    endfunction

    wire tlb_perm_fault = perm_check(tlb_perm, is_fetch, is_store, priv, sum, mxr);

    assign fault = bypass ? 1'b0 :
                   (tlb_hit ? tlb_perm_fault :
                    (ptw_valid ? result_fault : 1'b0));

    // page table walk state machine result_cause in when
    // TLB hit permission before TLB is
    wire [3:0] cur_type_cause = is_fetch ? `EXC_INST_PAGE_FAULT :
                                is_store ? `EXC_STORE_PAGE_FAULT :
                                           `EXC_LOAD_PAGE_FAULT;
    assign fault_cause = (tlb_hit && tlb_perm_fault) ? cur_type_cause : result_cause;

    // fill only in ptw_valid sfence after
    reg ptw_valid_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ptw_valid_d <= 1'b0;
        else        ptw_valid_d <= ptw_valid;
    end
    assign tlb_fill_en = ptw_valid && !ptw_valid_d && !result_fault;
    assign tlb_fill_va = va_lat;
    assign tlb_fill_pa = ptw_pa;

    //==========================================================================
    // PTE
    //==========================================================================
    wire pte_v = pte[0];
    wire pte_r = pte[1];
    wire pte_w = pte[2];
    wire pte_x = pte[3];
    wire pte_u = pte[4];
    wire pte_g = pte[5];
    wire pte_a = pte[6];
    wire pte_d = pte[7];

    wire pte_leaf  = pte_r | pte_x;                       // leaf page
    wire pte_invalid = ~pte_v | (~pte_r & pte_w);         // V=0 or (R=0,W=1)

    //==========================================================================
    // permission RISC-V / Python cpu.cpp
    // privilege level U U=0 → S U=1 SUM=0 →
    // permission FETCH X LOAD R or MXR=1 STORE/AMO W
    //==========================================================================
    wire priv_ok = !( (priv_lat == `PRV_U && !pte_u) ||
                      (priv_lat == `PRV_S && sum == 1'b0 && pte_u) );

    wire perm_ok = is_fetch_lat ? pte_x :
                   is_store_lat ? pte_w :
                                  (pte_r | mxr);

    wire acl_fault = !priv_ok || !perm_ok;

    //==========================================================================
    // state machine
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            pt_req       <= 1'b0;
            pt_addr      <= 32'h0;
            pt_we        <= 1'b0;
            ptw_valid    <= 1'b0;
            result_fault <= 1'b0;
            result_cause <= 4'd0;
            result_pa    <= 32'h0;
            vpn1         <= 10'd0;
            vpn0         <= 10'd0;
            ppn_base     <= 22'd0;
            is_store_lat <= 1'b0;
            is_fetch_lat <= 1'b1;
            va_lat       <= 32'h0;
            pte          <= 32'h0;
        end else begin
            // ptw_valid in
            // 1
            // in S_IDLE when
            // pt_req/pt_we bridge when
            // is handshake success (pt_ready) after only then

            // sfence walk when translation
            if (sfence_vma) begin
                state        <= S_IDLE;
                ptw_valid    <= 1'b0;
                result_fault <= 1'b0;
            end else if (fault_ack) begin
                // CPU fault
                result_fault <= 1'b0;
                ptw_valid    <= 1'b0;
                state        <= S_IDLE;
            end else begin
                case (state)
                //---------------------------------------------
                S_IDLE: begin
                    // when only then
                    // the previous ptw_valid=1 when fault
                    // mmu_hit fault=0 is success
                    // CPU fault →
                    if (fault_ack) begin
                        result_fault <= 1'b0;
                        ptw_valid    <= 1'b0;
                    end else if (req && !bypass && !tlb_hit && !ptw_valid) begin
                        // !ptw_valid when walk translation
                        ptw_valid    <= 1'b0;
                        result_pa    <= 32'h0;
                        result_fault <= 1'b0;
                        va_lat       <= va;
                        is_store_lat <= is_store;
                        is_fetch_lat <= is_fetch;
                        priv_lat     <= priv;
                        vpn1         <= va[31:22];
                        vpn0         <= va[21:12];
                        ppn_base     <= satp_ppn;

                        // page table = (satp.ppn << 12) + vpn1*4
                        pt_addr <= {satp_ppn, 12'd0} + {20'd0, va[31:22], 2'b00};
                        pt_req  <= 1'b1;
                        state   <= S_L1;
                    end
                end

                //---------------------------------------------
                S_L1: begin
                    if (pt_ready) begin
                        pte      <= pt_rdata;
                        pt_req   <= 1'b0;      // handshake success only then
                        state    <= S_L1_W;
                    end
                end

                S_L1_W: begin
                    if (pte_invalid) begin
                        result_fault <= 1'b1;
                        // is_fetch/is_store
                        result_cause <= is_fetch ? `EXC_INST_PAGE_FAULT :
                                        is_store ? `EXC_STORE_PAGE_FAULT :
                                                   `EXC_LOAD_PAGE_FAULT;
                        ptw_valid    <= 1'b1;
                        state        <= S_IDLE;
                    end else if (pte_leaf) begin
                        // → 4MB
                        if (pte[19:10] != 10'd0) begin

                            result_fault <= 1'b1;
                            result_cause <= is_fetch_lat ? `EXC_INST_PAGE_FAULT :
                                            is_store_lat ? `EXC_STORE_PAGE_FAULT :
                                                           `EXC_LOAD_PAGE_FAULT;
                            ptw_valid    <= 1'b1;
                            state        <= S_IDLE;
                        end else begin
                            result_pa <= {pte[31:20], va_lat[21:0]};
                            ptw_pa    <= {pte[31:20], va_lat[21:0]};
                            ptw_va    <= va_lat;
                            ptw_size  <= 2'd1;
                            ptw_perm_r<= pte_r; ptw_perm_w<= pte_w;
                            ptw_perm_x<= pte_x; ptw_perm_u<= pte_u;
                            result_fault <= 1'b0;

                            // A/D 4MB —— set page fault
                            // A=0 → page fault
                            // D=0 and the access is a write → page fault
                            // permission page fault
                            if (acl_fault || !pte_a || (is_store_lat && !pte_d)) begin
                                result_fault <= 1'b1;
                                result_cause <= is_fetch_lat ? `EXC_INST_PAGE_FAULT :
                                                is_store_lat ? `EXC_STORE_PAGE_FAULT :
                                                               `EXC_LOAD_PAGE_FAULT;
                                ptw_valid    <= 1'b1;
                                state        <= S_IDLE;
                            end else begin
                                ptw_valid <= 1'b1;
                                state     <= S_IDLE;
                            end
                        end
                    end else begin
                        // non-leaf → read page table
                        ppn_base <= pte[31:10];
                        pt_addr  <= {pte[31:10], 12'd0} + {20'd0, vpn0, 2'b00};
                        pt_req   <= 1'b1;
                        state    <= S_L2;
                    end
                end

                //---------------------------------------------
                S_L2: begin
                    if (pt_ready) begin
                        pte      <= pt_rdata;
                        pt_req   <= 1'b0;      // handshake success only then
                        state    <= S_L2_W;
                    end
                end

                S_L2_W: begin
                    if (pte_invalid) begin
                        result_fault <= 1'b1;
                        // The cause has to be recorded with the fault, or the
                        // consumer sees a fault with cause 0 and reports the
                        // wrong exception.
                        result_cause <= is_fetch_lat ? `EXC_INST_PAGE_FAULT :
                                        is_store_lat ? `EXC_STORE_PAGE_FAULT :
                                                       `EXC_LOAD_PAGE_FAULT;
                        ptw_valid    <= 1'b1;
                        state        <= S_IDLE;
                    end else if (pte_leaf) begin
                        // PA and TLB
                        result_pa <= {pte[31:10], va_lat[11:0]};
                        ptw_pa    <= {pte[31:10], va_lat[11:0]};
                        ptw_va    <= va_lat;
                        ptw_size  <= 2'd0;
                        ptw_perm_r<= pte_r; ptw_perm_w<= pte_w;
                        ptw_perm_x<= pte_x; ptw_perm_u<= pte_u;
                        result_fault <= 1'b0;

                        // A/D 4KB —— set page fault
                        // A=0 → page fault
                        // D=0 and the access is a write → page fault
                        // permission page fault
                        if (acl_fault || !pte_a || (is_store_lat && !pte_d)) begin
                            result_fault <= 1'b1;
                            result_cause <= is_fetch_lat ? `EXC_INST_PAGE_FAULT :
                                            is_store_lat ? `EXC_STORE_PAGE_FAULT :
                                                           `EXC_LOAD_PAGE_FAULT;
                            ptw_valid    <= 1'b1;
                            state        <= S_IDLE;
                        end else begin
                            ptw_valid <= 1'b1;
                            state     <= S_IDLE;
                        end
                    end else begin
                        // non-leaf → Sv32 2
                        result_fault <= 1'b1;
                        result_cause <= is_fetch_lat ? `EXC_INST_PAGE_FAULT :
                                        is_store_lat ? `EXC_STORE_PAGE_FAULT :
                                                       `EXC_LOAD_PAGE_FAULT;
                        ptw_valid    <= 1'b1;
                        state        <= S_IDLE;
                    end
                end

                //----------------------------------------------------------
                // S_UPD A/D write back → TLB returns
                //----------------------------------------------------------
                S_UPD: begin
                    pt_we     <= 1'b0;
                    ptw_valid <= 1'b1;
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
