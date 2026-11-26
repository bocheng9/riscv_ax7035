//=============================================================================
// File        : csr_regs.v
// Description : Control and status registers, trap entry and privilege transitions
// - M / S / U privilege level
// - / returns mret / sret
// - interrupt / when /
// - counter mcycle / minstret
//=============================================================================
`include "riscv_pkg.v"

module csr_regs (
    input  wire        clk,
    input  wire        rst_n,

    // CSR EX
    input  wire [11:0] addr,        // write MEM
    input  wire        we,
    input  wire [1:0]  op,          // write MEM
    input  wire [31:0] wdata,       // write MEM
    output reg  [31:0] rdata,       // write read

    // read EX csrrs read
    input  wire [11:0] raddr,
    output reg  [31:0] rdata_ex,

    // privilege level
    input  wire [1:0]  priv,
    output wire [1:0]  priv_next,

    // MEM
    input  wire        exc_valid,
    input  wire [3:0]  exc_cause,
    input  wire [31:0] exc_tval,
    input  wire [31:0] exc_pc,


    output wire        trap_taken,
    output wire [31:0] trap_vector,
    output reg  [31:0] trap_epc,
    output wire [31:0] mepc_o,        // mret returns
    output wire [31:0] sepc_o,        // sret returns
    output wire [31:0] mcause_o,      // cause
    output wire [31:0] mtval_o,       // tval
    output wire [31:0] satp_o,        // satp MMU
    output wire [31:0] mstatus_o,     // mstatus MPRV/MPP memory access privilege level
    output wire [31:0] scause_o,      // for auditing
    output wire [31:0] stval_o,       // for auditing
    input  wire        mret,
    input  wire        sret,

    // interrupt
    input  wire        irq_soft,
    input  wire        irq_timer,
    input  wire        irq_ext,

    // counter
    output reg  [63:0] cycle_cnt,
    output wire [63:0] instret_cnt,
    input  wire        instret_inc      // this cycle retire write back core
);

    // declare up front Vivado Synth 8-6901 1
    wire [63:0] instret_pre;
    wire [63:0] instret_read;
    //==========================================================================
    // declare up front remove Verilog forward reference → Vivado Synth 8-6901/8-91
    //==========================================================================
    reg [31:0] mcountinhibit;
    reg [63:0] instret_cnt_r;

    // → write +
    reg         minstret_wr_req,  minstreth_wr_req,  mcinhibit_wr_req;
    reg  [31:0] wr_minstret_q,    wr_minstreth_q,    wr_mcinhibit_q;

    //==========================================================================

    //==========================================================================
    // ---- Machine ----
    reg [31:0] mstatus;
    reg [31:0] misa;
    reg [31:0] medeleg;
    reg [31:0] mideleg;
    reg [31:0] mie;
    reg [31:0] mtvec;
    reg [31:0] mcounteren;
    reg [31:0] mscratch;
    reg [31:0] mepc;
    reg [31:0] mcause;
    reg [31:0] mtval;
    reg [31:0] mip;
    reg        ssip_sw;    // write SSIP
    // PMP 16 + 4 pmpaddr in RV32 is 32 addr[33:2]
    reg [31:0] pmpaddr [0:15];
    reg [31:0] pmpcfg  [0:3];
    integer    i_pmp;      // PMP reset

    // ---- Supervisor ----
    reg [31:0] stvec;
    reg [31:0] sscratch;
    reg [31:0] sepc;
    reg [31:0] scause;
    reg [31:0] stval;
    reg [31:0] satp;
    reg [31:0] scounteren;

    // ---- read only ----
    localparam [31:0] MVENDORID = 32'h0;
    localparam [31:0] MARCHID   = 32'h1;
    localparam [31:0] MIMPID    = 32'h1;
    localparam [31:0] MHARTID   = 32'h0;

    //==========================================================================
    // mstatus
    //==========================================================================
    // [0] UIE/SIE (fs=0 when )
    // [1]      SIE
    // [3]      MIE
    // [4]      SPIE
    // [5] UB E / (MPIE )
    // [7]      MPIE
    // [8]      SPP
    // [12:11]  MPP
    // [14:13]  FS
    // [16:15]  XS
    // [17]     MPRV
    // [18]     SUM
    // [19]     MXR
    // [20]     TVM
    // [21]     TW
    // [22]     TSR

    wire mstatus_mie  = mstatus[3];
    wire mstatus_sie  = mstatus[1];
    wire mstatus_mpie = mstatus[7];
    wire mstatus_spie = mstatus[5];
    wire mstatus_spp  = mstatus[8];
    wire [1:0] mstatus_mpp = mstatus[12:11];

    //==========================================================================
    // interrupt M / S
    //==========================================================================
    // mip SSIP=1, STIP=5, SEIP=9 | MSIP=3, MTIP=7, MEIP=11
    wire s_soft_pend  = mip[1];
    wire s_timer_pend = mip[5];
    wire s_ext_pend   = mip[9];
    wire m_soft_pend  = mip[3];
    wire m_timer_pend = mip[7];
    wire m_ext_pend   = mip[11];

    wire s_int_en = mstatus_sie;
    // interrupt MIE csrsi mstatus,8 interrupt
    // epc in csrsi interrupt in
    reg mstatus_mie_q;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) mstatus_mie_q <= 1'b0;
        else        mstatus_mie_q <= mstatus_mie;
    wire m_int_en = mstatus_mie_q;

    // S interrupt mideleg
    wire s_int_pending = s_int_en && (
        (s_soft_pend  && mie[1] && mideleg[1]) ||
        (s_timer_pend && mie[5] && mideleg[5]) ||
        (s_ext_pend   && mie[9] && mideleg[9]));

    // M interrupt
    wire m_int_pending = m_int_en && (
        (m_soft_pend  && mie[3]) ||
        (m_timer_pend && mie[7]) ||
        (m_ext_pend   && mie[11]));

    // M interrupt > S interrupt MEI > MSI > MTI
    reg        int_valid;
    reg [3:0]  int_cause;
    reg        int_to_m;         // 1 = M 0 = S

    always @(*) begin
        int_valid = 1'b0; int_cause = 4'd0; int_to_m = 1'b1;
        // M
        if (m_int_pending && m_ext_pend)          begin int_valid=1; int_cause=`IRQ_M_EXT;  int_to_m=1; end
        else if (m_int_pending && m_soft_pend)    begin int_valid=1; int_cause=`IRQ_M_SOFT; int_to_m=1; end
        else if (m_int_pending && m_timer_pend)   begin int_valid=1; int_cause=`IRQ_M_TIMER;int_to_m=1; end
        // S interrupt mideleg=0 → M
        else if (priv == `PRV_M && !mideleg[1] && m_int_en && s_soft_pend && mie[1])
                                                  begin int_valid=1; int_cause=`IRQ_S_SOFT; int_to_m=1; end
        else if (priv == `PRV_M && !mideleg[5] && m_int_en && s_timer_pend && mie[5])
                                                  begin int_valid=1; int_cause=`IRQ_S_TIMER;int_to_m=1; end
        else if (priv == `PRV_M && !mideleg[9] && m_int_en && s_ext_pend && mie[9])
                                                  begin int_valid=1; int_cause=`IRQ_S_EXT;  int_to_m=1; end
        // S + S
        else if (priv != `PRV_M && s_int_pending && s_ext_pend)  begin int_valid=1; int_cause=`IRQ_S_EXT;  int_to_m=0; end
        else if (priv != `PRV_M && s_int_pending && s_soft_pend) begin int_valid=1; int_cause=`IRQ_S_SOFT; int_to_m=0; end
        else if (priv != `PRV_M && s_int_pending && s_timer_pend)begin int_valid=1; int_cause=`IRQ_S_TIMER;int_to_m=0; end
    end

    //==========================================================================
    // > interrupt
    //==========================================================================
    reg do_trap, trap_to_m;
    reg trap_is_int;      // trap is interrupt SIE after int_valid 0
    reg [3:0] cause4;
    reg [31:0] tval_r, epc_r;

    always @(*) begin
        do_trap  = 1'b0;
        // do_trap is trap_taken
        trap_to_m= 1'b1;
        cause4   = 4'd0;
        tval_r   = 32'h0;
        epc_r    = 32'h0;

        trap_is_int = 1'b0;
        if (exc_valid) begin
            do_trap = 1'b1;
            cause4  = exc_cause;
            tval_r  = exc_tval;
            epc_r   = exc_pc;
            // whether the trap is delegated to S mode
            if (priv != `PRV_M && medeleg[exc_cause])
                trap_to_m = 1'b0;
            else
                trap_to_m = 1'b1;
        end else if (int_valid) begin
            do_trap     = 1'b1;
            trap_is_int = 1'b1;
            cause4    = int_cause;
            tval_r    = 32'h0;
            // interrupt in returns is
            epc_r     = exc_pc + 32'd4;
            trap_to_m = int_to_m;
        end
    end

    //==========================================================================
    // +
    assign trap_taken = do_trap;
    assign mepc_o = mepc;
    assign sepc_o = sepc;
    // vectored mtvec[1:0]==1 interrupt base + 4*cause base
    wire [31:0] mtvec_base = {mtvec[31:2], 2'b00};
    wire [31:0] stvec_base = {stvec[31:2], 2'b00};
    wire        mtvec_vec  = mtvec[1:0] == 2'b01;
    wire        stvec_vec  = stvec[1:0] == 2'b01;
    wire        this_int   = trap_is_int;   // trap SIE after exc_valid/int_valid
    assign trap_vector = trap_to_m
        ? (mtvec_vec && this_int ? (mtvec_base + {24'd0, cause4, 2'b00}) : mtvec_base)
        : (stvec_vec && this_int ? (stvec_base + {24'd0, cause4, 2'b00}) : stvec_base);

    //==========================================================================
    // priv_next is is mret and do_trap
    // + mret after priv and PC → scall/ma_fetch/sbreak
    // " U ecall" cause
    // mret/sret returns > do_trap > hold before
    //==========================================================================
    // sret/mret tsr/tvm or privilege level " returns "
    // It has to take effect on that cycle: otherwise the return logic
    // would swallow the illegal-instruction trap raised by sret.
    // mstatus.MPP mret returns privilege level rv32mi-p-illegal @270
    wire mstatus_tsr = mstatus[22];                        // TSR S sret
    wire mret_ok = mret && (priv == `PRV_M);              // M mret
    // The reference model allows sret from any privilege level as long as TSR
    // is clear, and the supervisor-mode vectors rely on that: they return from
    // a trap while still in M mode. Keeping the architectural "S mode only"
    // rule here left priv stuck at M and the subsequent fetches untranslated.
    wire sret_ok = sret && !mstatus_tsr;

    assign priv_next = mret_ok ? mstatus_mpp :
                       sret_ok ? (mstatus_spp ? `PRV_S : `PRV_U) :
                       do_trap ? (trap_to_m ? `PRV_M : `PRV_S) :
                       priv;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trap_epc    <= 32'h0;
        end else begin
            if (do_trap) trap_epc <= epc_r;
        end
    end

    //==========================================================================
    // interrupt
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mip <= 32'h0;
        end else begin
            // / when / interrupt
            mip[3]  <= irq_soft;    // MSIP
            mip[7]  <= irq_timer;   // MTIP
            mip[11] <= irq_ext;     // MEIP
            // S
            // SSIP irq_soft or write csr_write set
            // interrupt SSIP
            mip[1]  <= (ssip_sw | irq_soft) && !(do_trap && trap_is_int && int_cause == 4'd1);
            mip[5]  <= irq_timer;   // STIP
            mip[9]  <= irq_ext;     // SEIP
        end
    end

    //==========================================================================
    // CSR read
    //==========================================================================
    always @(*) begin
        case (addr)
            // Machine
            `CSR_MSTATUS:    rdata = mstatus;
            `CSR_MISA:       rdata = misa;
            `CSR_MEDELEG:    rdata = medeleg;
            `CSR_MIDELEG:    rdata = mideleg;
            `CSR_MIE:        rdata = mie;
            `CSR_MTVEC:      rdata = mtvec;
            `CSR_MCOUNTEREN: rdata = mcounteren;
            `CSR_MSCRATCH:   rdata = mscratch;
            `CSR_MEPC:       rdata = mepc;
            `CSR_MCAUSE:     rdata = mcause;
            `CSR_MTVAL:      rdata = mtval;
            `CSR_MIP:        rdata = mip;
            `CSR_MVENDORID:  rdata = MVENDORID;
            `CSR_MARCHID:    rdata = MARCHID;
            `CSR_MIMPID:     rdata = MIMPID;
            `CSR_MHARTID:    rdata = MHARTID;
            // Supervisor
            `CSR_SSTATUS:    rdata = mstatus & 32'h800D_6133;  // S Python V2
            `CSR_SIE:        rdata = mie & mideleg;
            `CSR_STVEC:      rdata = stvec;
            `CSR_SCOUNTEREN: rdata = scounteren;
            `CSR_SSCRATCH:   rdata = sscratch;
            `CSR_SEPC:       rdata = sepc;
            `CSR_SCAUSE:     rdata = scause;
            `CSR_STVAL:      rdata = stval;
            `CSR_SIP:        rdata = mip & mideleg;
            `CSR_SATP:       rdata = satp;
            `CSR_PMPCFG0:    rdata = pmpcfg[0];
            `CSR_PMPCFG1:    rdata = pmpcfg[1];
            `CSR_PMPCFG2:    rdata = pmpcfg[2];
            `CSR_PMPCFG3:    rdata = pmpcfg[3];
            // Counters
            `CSR_CYCLE:      rdata = cycle_cnt[31:0];
            `CSR_TIME:       rdata = cycle_cnt[31:0];
            `CSR_INSTRET:    rdata = instret_read[31:0];
            `CSR_MCYCLE:     rdata = cycle_cnt[31:0];
            `CSR_MINSTRET:   rdata = instret_read[31:0];
            `CSR_MINSTRETH:  rdata = instret_read[63:32];
            `CSR_MCOUNTINHIBIT: rdata = mcountinhibit;
            default:         rdata = (raddr >= 12'h3B0 && raddr <= 12'h3BF)
                                     ? pmpaddr[raddr[3:0]] : 32'h0;
        endcase
    end


    //==========================================================================
    // CSR read EX
    //==========================================================================
    // write MEM in write CSR when returns write
    // csrw csrr read
    // Forwarding exists so a read that follows a write of the same CSR sees
    // the new value instead of the old one. It must not run when the write is
    // suppressed, which is what happens on a trap: csr_write drops but we does
    // not, and forwarding wdata then overwrote the cause and tval the trap had
    // just produced, so the handler read a stale or zero value and took the
    // wrong path.
    wire csr_write  = we && !do_trap;
    wire ex_fwd_hit = csr_write && (addr == raddr);

    always @(*) begin
        case (raddr)
            // Machine
            `CSR_MSTATUS:    rdata_ex = mstatus;
            `CSR_MISA:       rdata_ex = misa;
            `CSR_MEDELEG:    rdata_ex = medeleg;
            `CSR_MIDELEG:    rdata_ex = mideleg;
            `CSR_MIE:        rdata_ex = mie;
            `CSR_MTVEC:      rdata_ex = mtvec;
            `CSR_MCOUNTEREN: rdata_ex = mcounteren;
            `CSR_MSCRATCH:   rdata_ex = mscratch;
            `CSR_MEPC:       rdata_ex = do_trap ? epc_r : mepc;     // `CSR_MCAUSE:     rdata_ex = do_trap ? cause4 : mcause; // `CSR_MTVAL:      rdata_ex = do_trap ? tval_r : mtval;   // trap write read
            `CSR_MIP:        rdata_ex = mip;
            `CSR_MVENDORID:  rdata_ex = MVENDORID;
            `CSR_MARCHID:    rdata_ex = MARCHID;
            `CSR_MIMPID:     rdata_ex = MIMPID;
            `CSR_MHARTID:    rdata_ex = MHARTID;
            // Supervisor
            `CSR_SSTATUS:    rdata_ex = mstatus & 32'h800D_6133;  // S Python V2
            `CSR_SIE:        rdata_ex = mie & mideleg;
            `CSR_STVEC:      rdata_ex = stvec;
            `CSR_SCOUNTEREN: rdata_ex = scounteren;
            `CSR_SSCRATCH:   rdata_ex = sscratch;
            `CSR_SEPC:       rdata_ex = do_trap ? epc_r : sepc;     // `CSR_SCAUSE:     rdata_ex = do_trap ? cause4 : scause; // `CSR_STVAL:      rdata_ex = do_trap ? tval_r : stval;   // `CSR_SIP:        rdata_ex = mip & mideleg;
            `CSR_SATP:       rdata_ex = satp;
            `CSR_PMPCFG0:    rdata_ex = pmpcfg[0];
            `CSR_PMPCFG1:    rdata_ex = pmpcfg[1];
            `CSR_PMPCFG2:    rdata_ex = pmpcfg[2];
            `CSR_PMPCFG3:    rdata_ex = pmpcfg[3];
            // Counters
            `CSR_CYCLE:      rdata_ex = cycle_cnt[31:0];
            `CSR_TIME:       rdata_ex = cycle_cnt[31:0];
            `CSR_INSTRET:    rdata_ex = instret_pre[31:0];
            `CSR_MCYCLE:     rdata_ex = cycle_cnt[31:0];
            `CSR_MINSTRET:   rdata_ex = instret_pre[31:0];
            `CSR_MINSTRETH:  rdata_ex = instret_pre[63:32];
            `CSR_MCOUNTINHIBIT: rdata_ex = mcountinhibit;
            default:         rdata_ex = (raddr >= 12'h3B0 && raddr <= 12'h3BF)
                                        ? pmpaddr[raddr[3:0]] : 32'h0;
        endcase
        if (ex_fwd_hit) rdata_ex = wdata;
    end

    //==========================================================================
    // CSR write
    //==========================================================================

    //==========================================================================
    // op write
    // 2'b01 = CSRRW → write wdata
    //   2'b10 = CSRRS → csr | wdata
    //   2'b11 = CSRRC → csr & ~wdata
    //==========================================================================
    reg [31:0] csr_old;      // current CSR value
    always @(*) begin
        case (addr)
            `CSR_MSTATUS:    csr_old = mstatus;
            `CSR_MEDELEG:    csr_old = medeleg;
            `CSR_MIDELEG:    csr_old = mideleg;
            `CSR_MIE:        csr_old = mie;
            `CSR_MTVEC:      csr_old = mtvec;
            `CSR_MCOUNTEREN: csr_old = mcounteren;
            `CSR_MSCRATCH:   csr_old = mscratch;
            `CSR_MEPC:       csr_old = mepc;
            `CSR_MCAUSE:     csr_old = mcause;
            `CSR_MTVAL:      csr_old = mtval;
            `CSR_SSTATUS:    csr_old = mstatus;
            `CSR_SIE:        csr_old = mie;
            `CSR_STVEC:      csr_old = stvec;
            `CSR_SSCRATCH:   csr_old = sscratch;
            `CSR_SEPC:       csr_old = sepc;
            `CSR_SCAUSE:     csr_old = scause;
            `CSR_STVAL:      csr_old = stval;
            `CSR_SATP:       csr_old = satp;
            `CSR_PMPCFG0:    csr_old = pmpcfg[0];
            `CSR_PMPCFG1:    csr_old = pmpcfg[1];
            `CSR_PMPCFG2:    csr_old = pmpcfg[2];
            `CSR_PMPCFG3:    csr_old = pmpcfg[3];
            default:         csr_old = (addr >= 12'h3B0 && addr <= 12'h3BF)
                                       ? pmpaddr[addr[3:0]] : 32'h0;
        endcase
    end

    wire [31:0] csr_new = (op == 2'b10) ? (csr_old | wdata) :
                          (op == 2'b11) ? (csr_old & ~wdata) :
                                          wdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mstatus   <= 32'h0000_1800;   // MPP=11(M ), MIE=0, MPIE=0
            // misa: MXL=1(32 ) | I(bit8) M(bit12) A(bit0) S(bit18) U(bit20)
            // M/S/U privilege level + Sv32 MMU must S U
            // rv32mi-p-csr misa and
            misa      <= 32'h4014_1101;
            medeleg   <= 32'h0;
            mideleg   <= 32'h0;
            mie       <= 32'h0;
            mtvec     <= 32'h0;
            mcounteren<= 32'h0;
            mscratch  <= 32'h0;
            mepc      <= 32'h0;
            mcause    <= 32'h0;
            mtval     <= 32'h0;
            stvec     <= 32'h0;
            sscratch  <= 32'h0;
            sepc      <= 32'h0;
            scause    <= 32'h0;
            stval     <= 32'h0;
            satp      <= 32'h0;
            scounteren<= 32'h0;
            cycle_cnt <= 64'h0;
            // instret_cnt / mcountinhibit / minstret_we_q / minstreth_we_q
            // counters are owned by their own block and are not reset here
            minstret_wr_req  <= 1'b0;
            minstreth_wr_req <= 1'b0;
            mcinhibit_wr_req <= 1'b0;
            ssip_sw    <= 1'b0;
            // PMP reset Verilog
            for (i_pmp = 0; i_pmp < 16; i_pmp = i_pmp + 1) pmpaddr[i_pmp] <= 32'h0;
            for (i_pmp = 0; i_pmp < 4;  i_pmp = i_pmp + 1) pmpcfg[i_pmp]  <= 32'h0;
        end else begin
            cycle_cnt <= cycle_cnt + 64'd1;
            // clear
            minstret_wr_req  <= 1'b0;
            minstreth_wr_req <= 1'b0;
            mcinhibit_wr_req <= 1'b0;

            // trap / mret / sret in always multiple drivers
            // CSR write
            if (do_trap) begin
                if (trap_to_m) begin
                    mepc   <= epc_r;
                    mcause <= {trap_is_int, 27'd0, cause4};
                    mtval  <= tval_r;
                    mstatus[7]     <= mstatus_mie;   // MPIE = MIE
                    mstatus[3]     <= 1'b0;          // MIE = 0
                    mstatus[12:11] <= priv;          // MPP
                end else begin
                    sepc   <= epc_r;
                    scause <= {trap_is_int, 27'd0, cause4};
                    stval  <= tval_r;
                    mstatus[5]     <= mstatus_sie;   // SPIE = SIE
                    mstatus[1]     <= 1'b0;          // SIE = 0
                    mstatus[8]     <= (priv == `PRV_S);// SPP
                end
            end
            if (mret) begin
                mstatus[3]     <= mstatus_mpie;      // MIE = MPIE
                mstatus[7]     <= 1'b1;              // MPIE = 1
                mstatus[12:11] <= `PRV_U;            // MPP = U
            end
            if (sret) begin
                mstatus[1] <= mstatus_spie;
                mstatus[5] <= 1'b1;
                mstatus[8] <= 1'b0;
            end

            // write mip SSIP(bit1) set /
            if (csr_write && addr == `CSR_MIP) ssip_sw <= csr_new[1];
            // SSIP is interrupt handler
            if (do_trap && trap_is_int && int_cause == 4'd1) ssip_sw <= 1'b0;

            // PMPADDR0-15 write 0x3B0~0x3BF
            // PMP L pmpcfg[i/4][7] after write RISC-V
            // L bad success
            if (csr_write && addr >= 12'h3B0 && addr <= 12'h3BF)
                pmpaddr[addr[3:0]] <= csr_new;

            if (csr_write) begin
                case (addr)
                    `CSR_MSTATUS:    mstatus   <= csr_new;
                    // PMP write
                    // counter write
                    `CSR_MINSTRET:   begin wr_minstret_q  <= csr_new; minstret_wr_req  <= 1'b1; end
                    `CSR_MINSTRETH:  begin wr_minstreth_q <= csr_new; minstreth_wr_req <= 1'b1; end
                    `CSR_MCOUNTINHIBIT: begin wr_mcinhibit_q <= csr_new; mcinhibit_wr_req <= 1'b1; end
                    `CSR_PMPCFG0:    pmpcfg[0] <= csr_new;
                    `CSR_PMPCFG1:    pmpcfg[1] <= csr_new;
                    `CSR_PMPCFG2:    pmpcfg[2] <= csr_new;
                    `CSR_PMPCFG3:    pmpcfg[3] <= csr_new;
                    // sstatus write S Python V2 0x800D6133
                    // sstatus write S MPP=bit12:11 must hold
                    `CSR_SSTATUS:    mstatus   <= (mstatus & ~32'h800D_6133) |
                                                  (csr_new   &  32'h800D_6133);
                    `CSR_MEDELEG:    medeleg   <= csr_new;
                    `CSR_MIDELEG:    mideleg   <= csr_new;
                    `CSR_MIE:        mie       <= csr_new;
                    // mip always write ssip_clr
                    `CSR_MTVEC:      mtvec     <= {csr_new[31:2], csr_new[1:0]};  // vectored
                    `CSR_MCOUNTEREN: mcounteren<= csr_new;
                    `CSR_MSCRATCH:   mscratch  <= csr_new;
                    `CSR_MEPC:       mepc      <= {wdata[31:2], 2'b00};
                    `CSR_MCAUSE:     mcause    <= csr_new;
                    `CSR_MTVAL:      mtval     <= csr_new;
                    `CSR_STVEC:      stvec     <= {csr_new[31:2], csr_new[1:0]};  // vectored
                    `CSR_SCOUNTEREN: scounteren<= wdata;
                    `CSR_SSCRATCH:   sscratch  <= csr_new;
                    `CSR_SEPC:       sepc      <= {wdata[31:2], 2'b00};
                    `CSR_SCAUSE:     scause    <= csr_new;
                    `CSR_STVAL:      stval     <= csr_new;
                    `CSR_SATP:       satp      <= csr_new;
                    default: ;
                endcase
            end
        end
    end

    // retire write minstret/minstreth mcountinhibit[2]=1 when
    reg        minstret_we_q;   // this cycle minstret write
    reg [31:0] minstret_wq;
    reg        minstreth_we_q;
    reg [31:0] minstreth_wq;
    reg        count_halt;      // write minstret after
    // read counter returns " retire before " this cycle +1
    // read counter " before " this cycle 1
    // write this cycle or write when returns write before csrw read
    // read counter " before " this cycle +1
    // write minstret_we_q/minstreth_we_q write write
    assign instret_read =
        minstret_wr_req  ? {instret_cnt_r[63:32], wr_minstret_q}  :   // before write
        minstreth_wr_req ? {wr_minstreth_q, instret_cnt_r[31:0]}  :   // minstret_we_q    ? {instret_cnt_r[63:32], minstret_wq}   :
        minstreth_we_q   ? {minstreth_wq, instret_cnt_r[31:0]}   :
        (instret_cnt_r - ((instret_inc && !mcountinhibit[2] && !count_halt) ? 64'd1 : 64'd0));

    wire instret_wr_pending = 1'b0;
    wire instret_wr_d0      = 1'b0;
    // EX read counter read counter retire → " before " 1
    // counter is 0 when 1
    assign instret_pre = (instret_wr_pending || instret_wr_d0) ? instret_read
                              : ((!mcountinhibit[2] && !count_halt && instret_read != 64'd0 &&
                                  !minstret_we_q && !minstreth_we_q)
                                 ? (instret_read - 64'd1) : instret_read);

    // Vivado else-if when
    wire can_inc = !mcountinhibit[2] && !count_halt && instret_inc &&
                   !minstret_wr_req && !minstreth_wr_req;

    // + output reg Vivado
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            instret_cnt_r  <= 64'h0;
            minstret_we_q  <= 1'b0;
            minstreth_we_q <= 1'b0;
            count_halt     <= 1'b0;
            mcountinhibit  <= 32'h0;
        end else if (minstret_we_q || minstreth_we_q) begin
            instret_cnt_r <= { minstreth_we_q ? minstreth_wq : instret_cnt_r[63:32],
                               minstret_we_q  ? minstret_wq  : instret_cnt_r[31:0] };
            minstret_we_q  <= 1'b0;
            minstreth_we_q <= 1'b0;
            count_halt     <= minstret_we_q && !minstreth_we_q;
        end else if (minstret_wr_req) begin
            // write we write instret_cnt_r
            minstret_we_q  <= 1'b1;
            minstret_wq    <= wr_minstret_q;
        end else if (minstreth_wr_req) begin
            minstreth_we_q <= 1'b1;
            minstreth_wq   <= wr_minstreth_q;
        end else if (mcinhibit_wr_req) begin
            mcountinhibit  <= wr_mcinhibit_q;
        end else if (can_inc) begin
            instret_cnt_r <= instret_cnt_r + 64'd1;
        end
    end

    //--------------------------------------------------------------

    //--------------------------------------------------------------
    assign satp_o   = satp;
    assign mstatus_o= mstatus;
    assign mcause_o = mcause;
    assign mtval_o  = mtval;
    assign scause_o = scause;
    assign stval_o  = stval;

    // in always Vivado ambiguous clock
    assign instret_cnt = instret_cnt_r;

endmodule
