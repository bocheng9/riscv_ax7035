//=============================================================================
// mmu_arb.v
//
// Lets instruction fetch and data access share one SV32 translation unit.
//
// Two requesters want the same walker, so one waits. Data access wins by
// default: the pipeline stalls on it, while fetch has a small FIFO ahead of it
// that absorbs a few cycles. A fetch request that has waited too long is
// promoted anyway, so a run of loads cannot starve it.
//
// A translation is owned from the cycle its request is accepted until the unit
// answers with hit or fault, so a two-level walk is never handed to the other
// requester part way through: that would splice one walk's addresses into
// another's.
//
// The page table walk port is not this module's business; the walker is
// instantiated directly and its port goes to the bridge.
//=============================================================================
module mmu_arb #(
    parameter AGE_LIMIT = 6        // cycles before fetch is promoted
) (
    input  wire        clk,
    input  wire        rst_n,

    // Instruction fetch side
    input  wire [31:0] if_va,
    input  wire        if_req,
    output wire [31:0] if_pa,
    output wire        if_hit,
    output wire        if_fault,
    output wire [3:0]  if_fault_cause,

    // Data side
    input  wire [31:0] dm_va,
    input  wire        dm_req,
    input  wire        dm_is_store,
    output wire [31:0] dm_pa,
    output wire        dm_hit,
    output wire        dm_fault,
    output wire [3:0]  dm_fault_cause,

    // Shared walker
    output wire [31:0] va,
    output wire        req,
    output wire        is_store,
    output wire        is_fetch,
    input  wire [31:0] pa,
    input  wire        hit,
    input  wire        fault,
    input  wire [3:0]  fault_cause,

    // A fetch translation is in flight; the fetch address is not usable yet.
    output wire        if_busy
);

    //------------------------------------------------------------------
    // Ownership. 0 = idle, 1 = fetch, 2 = data. Held until the walker
    // answers, which also covers the case where the answer is a fault.
    //------------------------------------------------------------------
    reg [1:0] owner;
    reg [5:0] if_age;

    wire busy     = (owner != 2'd0);
    // Ownership is released once the answer has been given AND the requester
    // has stopped asking. Releasing on the answer alone made owner fall back
    // to idle every other cycle, so hit flickered and the consumer waiting on
    // it never saw a stable result.
    wire answered = (owner == 2'd1) ? (hit || fault) : 1'b0;
    wire dm_answered = (owner == 2'd2) && (hit || fault);
    wire settle   = answered && !if_req;
    wire settle_dm = dm_answered && !dm_req;

    // Data has priority unless the fetcher has been waiting too long.
    wire grant_if = !busy && if_req && (!dm_req || if_age >= AGE_LIMIT);
    wire grant_dm = !busy && dm_req && !grant_if;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner  <= 2'd0;
            if_age <= 6'd0;
        end else begin
            if (!busy) begin
                if (grant_if)      owner <= 2'd1;
                else if (grant_dm) owner <= 2'd2;
            end else if (settle || settle_dm) begin
                owner <= 2'd0;
            end

            if (if_req && owner != 2'd1 && if_age != 6'h3F)
                if_age <= if_age + 6'd1;
            else if (owner == 2'd1)
                if_age <= 6'd0;
        end
    end

    //------------------------------------------------------------------
    // Request presented to the walker. while idle the fetch address is
    // presented, which lets a bare-mode translation settle in one cycle.
    //------------------------------------------------------------------
    assign va       = (owner == 2'd2) ? dm_va : if_va;
    // The walker has to run whenever a fetch address is waiting, even before
    // the fetcher has committed to a request: stall is derived from hit, and
    // the fetcher only raises the request once it is not stalled, so gating
    // req on the request would deadlock the pair.
    assign req      = busy;
    assign is_store = (owner == 2'd2) && dm_is_store;
    // is_fetch describes the address currently presented to the walker, not
    // who owns it. While idle the fetch address is presented, so it must read
    // as a fetch or the privilege selected for the walk would be the data
    // one and a supervisor access would bypass translation entirely.
    assign is_fetch = (owner != 2'd2);

    // The fetcher must not use the fetch address until the translation for that
    // address has completed: while a walk is in progress the address on the
    // walker is valid but the answer is not.
    assign if_busy = (owner == 2'd1) && !hit;

    //------------------------------------------------------------------
    // Results routed back to the owner
    //------------------------------------------------------------------
    assign if_pa          = pa;
    // While idle the walker is presented with the fetch address, so its TLB
    // answer is already the one the fetcher needs; reporting it keeps the
    // fetcher from stalling on a translation that has already been resolved.
    // hit and fault both describe the walker's answer for the address being
    // presented, so neither is qualified by ownership: gating them on owner
    // would drop the answer in the cycle the walk completes and the owner is
    // reset, leaving the fetcher to continue with an untranslated address.
    assign if_hit         = (owner == 2'd2) ? 1'b0 : hit;
    assign if_fault       = (owner == 2'd2) ? 1'b0 : fault;
    assign if_fault_cause = fault_cause;

    // The core latches the answer itself, so the arbiter only has to present
    // it while the data path is asking. Owner gating is still needed: while
    // the walker serves a fetch, the address on it is the fetch address and
    // the answer does not belong to the data path.
    assign dm_pa          = pa;
    assign dm_hit         = (owner == 2'd2) && hit;
    assign dm_fault       = (owner == 2'd2) && fault;
    assign dm_fault_cause = fault_cause;

endmodule
