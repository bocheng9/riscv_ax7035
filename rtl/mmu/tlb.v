//=============================================================================
// File        : tlb.v
// Description : Fully-associative translation cache with LRU replacement
// - hit physical address and permission
// - hit when PTW
//=============================================================================
`include "riscv_pkg.v"

module tlb #(
    parameter ENTRIES = 32
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- ----
    input  wire [31:0] va,
    input  wire [8:0]  asid,
    input  wire        is_store,
    input  wire        is_fetch,
    output reg  [31:0] pa,
    output reg         hit,
    output wire [5:0]  perm,           // {d,a,u,x,w,r}
    output wire [5:0]  dbg_perm_idx,

    // ---- ----
    input  wire        fill_en,
    input  wire [31:0] fill_va,
    input  wire [31:0] fill_pa,
    input  wire [8:0]  fill_asid,
    input  wire [1:0]  fill_size,      // 0=4KB, 1=4MB
    input  wire        fill_r,
    input  wire        fill_w,
    input  wire        fill_x,
    input  wire        fill_u,
    input  wire        fill_a,        // A
    input  wire        fill_d,        // D

    // ---- flush ----
    input  wire        flush_all,      // sfence.vma x0,x0
    input  wire        flush_va_en,    // sfence.vma with rs1
    input  wire [31:0] flush_va,

    // ---- permission ----
    input  wire [1:0]  priv,
    input  wire        sum,            // mstatus.SUM
    input  wire        mxr             // mstatus.MXR
);

    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    reg [31:0] tag    [0:ENTRIES-1];   // VPN size
    reg [31:0] data   [0:ENTRIES-1];   // PPN
    reg [8:0]  tlb_asid[0:ENTRIES-1];
    reg [1:0]  tlb_size[0:ENTRIES-1];
    reg [5:0]  tlb_perm[0:ENTRIES-1];  // {d, a, u, x, w, r}
    reg        tlb_valid[0:ENTRIES-1];
    reg [4:0]  lru    [0:ENTRIES-1];

    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    integer i;
    reg found;
    reg [4:0] hit_idx;      // hit permission
    reg [4:0] victim_idx;   // LRU fill
    reg [4:0] min_lru;

    always @(*) begin
        pa         = 32'h0;
        hit        = 1'b0;
        found      = 1'b0;
        hit_idx    = 5'd0;
        victim_idx = 5'd0;
        min_lru    = 5'd31;

        for (i = 0; i < ENTRIES; i = i + 1) begin
            if (tlb_valid[i] && (tlb_asid[i] == asid)) begin
                if (tlb_size[i] == 2'd1) begin
                    // 4MB VPN[1] va[31:22]
                    if (tlb_tag_match_4m(i, va)) begin
                        pa  = {data[i][31:22], va[21:0]};
                        hit = 1'b1;
                        hit_idx = i;
                        found = 1'b1;
                    end
                end else begin
                    // 4KB VPN va[31:12]
                    if (tag[i][31:12] == va[31:12]) begin
                        pa  = {data[i][31:12], va[11:0]};
                        hit = 1'b1;
                        hit_idx = i;
                        found = 1'b1;
                    end
                end
            end
            if (lru[i] < min_lru) begin
                min_lru    = lru[i];
                victim_idx = i;
            end
        end
    end

    function tlb_tag_match_4m;
        input integer idx;
        input [31:0] v;
        begin
            tlb_tag_match_4m = (tag[idx][31:22] == v[31:22]);
        end
    endfunction

    assign perm = hit ? tlb_perm[hit_idx] : 6'h0;
    assign dbg_perm_idx = tlb_perm[hit_idx];

    //--------------------------------------------------------------------------
    // / flush
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < ENTRIES; i = i + 1) begin
                tlb_valid[i] <= 1'b0;
                lru[i]       <= 5'd0;
            end
        end else begin
            // LRU
            for (i = 0; i < ENTRIES; i = i + 1)
                if (lru[i] < 5'd31) lru[i] <= lru[i] + 5'd1;

            // hit → flush LRU
            if (hit) lru[hit_idx] <= 5'd0;


            if (fill_en) begin
                tag[victim_idx]     <= fill_va;
                data[victim_idx]    <= fill_pa;
                tlb_asid[victim_idx]<= fill_asid;
                tlb_size[victim_idx]<= fill_size;
                tlb_perm[victim_idx]<= {fill_d, fill_a, fill_u, fill_x, fill_w, fill_r};
                tlb_valid[victim_idx] <= 1'b1;
                lru[victim_idx]     <= 5'd0;
            end

            // flush
            if (flush_all)
                for (i = 0; i < ENTRIES; i = i + 1)
                    tlb_valid[i] <= 1'b0;

            // flush
            if (flush_va_en)
                for (i = 0; i < ENTRIES; i = i + 1)
                    if (tag[i][31:12] == flush_va[31:12])
                        tlb_valid[i] <= 1'b0;
        end
    end

endmodule
