//=============================================================================
// File        : decoder.v
// Description : Instruction decode and control generation
// - ALU immediate
// - illegal instruction
// - CSR
//=============================================================================
`include "riscv_pkg.v"

module decoder (
    input  wire [31:0] inst,

    output wire [6:0]  opcode,
    output wire [4:0]  rd,
    output wire [2:0]  funct3,
    output wire [4:0]  rs1,
    output wire [4:0]  rs2,
    output wire [6:0]  funct7,

    output reg  [31:0] imm,          // immediate sign-extend


    output reg         use_rs1,
    output reg         use_rs2,
    output reg         use_imm,
    output reg  [4:0]  alu_op,
    output reg  [1:0]  wb_sel,       // WB_*
    output reg         reg_we,       // write GPR
    output reg         mem_re,       // read load
    output reg         mem_we,       // write store
    output reg  [2:0]  mem_size,     // LB=0,LH=1,LW=2,LBU=4,LHU=5
    output reg         mem_unsigned,

    // /
    output reg         is_branch,
    output reg         is_jal,
    output reg         is_jalr,
    output reg  [2:0]  br_type,      // BEQ/BNE/BLT/BGE/BLTU/BGEU

    // CSR
    output reg         is_csr,
    output reg  [1:0]  csr_op,       // 01=RW 10=RS 11=RC
    output reg  [11:0] csr_addr,

    // RV32A
    output reg         is_amo,
    output reg  [4:0]  amo_op,


    output reg         is_fence,
    output reg         is_ecall,
    output reg         is_ebreak,
    output reg         is_mret,
    output reg         is_sret,
    output reg         is_wfi,
    output reg         is_sfence_vma,

    output reg         illegal
);

    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    assign opcode = inst[6:0];
    assign rd     = inst[11:7];
    assign funct3 = inst[14:12];
    assign rs1    = inst[19:15];
    assign rs2    = inst[24:20];
    assign funct7 = inst[31:25];

    //--------------------------------------------------------------------------
    // immediate
    //--------------------------------------------------------------------------
    wire [31:0] imm_i = {{20{inst[31]}}, inst[31:20]};
    wire [31:0] imm_s = {{20{inst[31]}}, inst[31:25], inst[11:7]};
    wire [31:0] imm_b = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    wire [31:0] imm_u = {inst[31:12], 12'h0};
    wire [31:0] imm_j = {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};

    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    localparam BR_BEQ  = 3'd0;
    localparam BR_BNE  = 3'd1;
    localparam BR_BLT  = 3'd2;
    localparam BR_BGE  = 3'd3;
    localparam BR_BLTU = 3'd4;
    localparam BR_BGEU = 3'd5;

    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    always @(*) begin

        imm        = 32'h0;
        use_rs1    = 1'b0;
        use_rs2    = 1'b0;
        use_imm    = 1'b0;
        alu_op     = `ALU_ADD;
        wb_sel     = `WB_ALU;
        reg_we     = 1'b0;
        mem_re     = 1'b0;
        mem_we     = 1'b0;
        mem_size   = 3'd2;
        mem_unsigned = 1'b0;
        is_branch  = 1'b0;
        is_jal     = 1'b0;
        is_jalr    = 1'b0;
        br_type    = 3'd0;
        is_csr     = 1'b0;
        csr_op     = 2'd0;
        csr_addr   = 12'h0;
        is_amo     = 1'b0;
        amo_op     = 5'd0;
        is_fence   = 1'b0;
        is_ecall   = 1'b0;
        is_ebreak  = 1'b0;
        is_mret    = 1'b0;
        is_sret    = 1'b0;
        is_wfi     = 1'b0;
        is_sfence_vma = 1'b0;
        illegal    = 1'b0;

        // RV32 C misa.C = 0 when
        // An encoding whose low two bits are not 2'b11 belongs to the
        // compressed space and is always illegal. The `unimp` in
        // rv32mi-p-illegal is only two bytes (0x0000); without this check
        // the following word would be executed as an instruction.
        if (inst[1:0] != 2'b11) begin
            illegal = 1'b1;
        end

        case (opcode)
            //---------------- R-type ----------------
            `OP_RTYPE: begin
                use_rs1 = 1'b1; use_rs2 = 1'b1;
                reg_we  = 1'b1; wb_sel = `WB_ALU;
                case ({funct7, funct3})
                    // RV32I
                    {7'b0000000,3'b000}: alu_op = `ALU_ADD;
                    {7'b0100000,3'b000}: alu_op = `ALU_SUB;
                    {7'b0000000,3'b001}: alu_op = `ALU_SLL;
                    {7'b0000000,3'b010}: alu_op = `ALU_SLT;
                    {7'b0000000,3'b011}: alu_op = `ALU_SLTU;
                    {7'b0000000,3'b100}: alu_op = `ALU_XOR;
                    {7'b0000000,3'b101}: alu_op = `ALU_SRL;
                    {7'b0100000,3'b101}: alu_op = `ALU_SRA;
                    {7'b0000000,3'b110}: alu_op = `ALU_OR;
                    {7'b0000000,3'b111}: alu_op = `ALU_AND;
                    // RV32M
                    {7'b0000001,3'b000}: alu_op = `ALU_MUL;
                    {7'b0000001,3'b001}: alu_op = `ALU_MULH;
                    {7'b0000001,3'b010}: alu_op = `ALU_MULHSU;
                    {7'b0000001,3'b011}: alu_op = `ALU_MULHU;
                    {7'b0000001,3'b100}: alu_op = `ALU_DIV;
                    {7'b0000001,3'b101}: alu_op = `ALU_DIVU;
                    {7'b0000001,3'b110}: alu_op = `ALU_REM;
                    {7'b0000001,3'b111}: alu_op = `ALU_REMU;
                    default: illegal = !is_sfence_vma;
                endcase
            end

            //---------------- I-type ----------------
            `OP_ITYPE: begin
                use_rs1 = 1'b1; use_imm = 1'b1; imm = imm_i;
                reg_we  = 1'b1; wb_sel = `WB_ALU;
                case (funct3)
                    3'b000: alu_op = `ALU_ADD;              // ADDI
                    3'b010: alu_op = `ALU_SLT;              // SLTI
                    3'b011: alu_op = `ALU_SLTU;             // SLTIU
                    3'b100: alu_op = `ALU_XOR;              // XORI
                    3'b110: alu_op = `ALU_OR;               // ORI
                    3'b111: alu_op = `ALU_AND;              // ANDI
                    3'b001: begin                           // SLLI
                        if (funct7 != 7'b0000000) illegal = 1'b1;
                        alu_op = `ALU_SLL;
                    end
                    3'b101: begin                           // SRLI / SRAI
                        if (funct7 == 7'b0100000) alu_op = `ALU_SRA;
                        else if (funct7 == 7'b0000000) alu_op = `ALU_SRL;
                        else illegal = 1'b1;
                    end
                endcase
            end

            //---------------- Load ----------------
            `OP_LOAD: begin
                use_rs1 = 1'b1; use_imm = 1'b1; imm = imm_i;
                reg_we  = 1'b1; wb_sel = `WB_MEM;
                mem_re  = 1'b1;
                case (funct3)
                    3'b000: begin mem_size = 3'd0; mem_unsigned = 1'b0; end // LB
                    3'b001: begin mem_size = 3'd1; mem_unsigned = 1'b0; end // LH
                    3'b010: begin mem_size = 3'd2; mem_unsigned = 1'b0; end // LW
                    3'b100: begin mem_size = 3'd0; mem_unsigned = 1'b1; end // LBU
                    3'b101: begin mem_size = 3'd1; mem_unsigned = 1'b1; end // LHU
                    default: illegal = 1'b1;
                endcase
            end

            //---------------- Store ----------------
            `OP_STORE: begin
                use_rs1 = 1'b1; use_rs2 = 1'b1; use_imm = 1'b1; imm = imm_s;
                mem_we  = 1'b1;
                case (funct3)
                    3'b000: mem_size = 3'd0;                // SB
                    3'b001: mem_size = 3'd1;                // SH
                    3'b010: mem_size = 3'd2;                // SW
                    default: illegal = 1'b1;
                endcase
            end

            //---------------- Branch ----------------
            `OP_BRANCH: begin
                use_rs1 = 1'b1; use_rs2 = 1'b1; use_imm = 1'b1; imm = imm_b;
                is_branch = 1'b1;
                case (funct3)
                    3'b000: br_type = BR_BEQ;
                    3'b001: br_type = BR_BNE;
                    3'b100: br_type = BR_BLT;
                    3'b101: br_type = BR_BGE;
                    3'b110: br_type = BR_BLTU;
                    3'b111: br_type = BR_BGEU;
                    default: illegal = 1'b1;
                endcase
            end

            //---------------- JAL ----------------
            `OP_JAL: begin
                use_imm = 1'b1; imm = imm_j;
                is_jal  = 1'b1;
                reg_we  = 1'b1; wb_sel = `WB_PC4;
            end

            //---------------- JALR ----------------
            `OP_JALR: begin
                use_rs1 = 1'b1; use_imm = 1'b1; imm = imm_i;
                is_jalr = 1'b1;
                reg_we  = 1'b1; wb_sel = `WB_PC4;
                if (funct3 != 3'b000) illegal = 1'b1;
            end

            //---------------- LUI ----------------
            `OP_LUI: begin
                use_imm = 1'b1; imm = imm_u;
                alu_op  = `ALU_PASS_B;
                reg_we  = 1'b1; wb_sel = `WB_ALU;
            end

            //---------------- AUIPC ----------------
            `OP_AUIPC: begin
                use_imm = 1'b1; imm = imm_u;
                alu_op  = `ALU_ADD;               // pc + imm in core
                reg_we  = 1'b1; wb_sel = `WB_ALU;
            end

            //---------------- SYSTEM ----------------
            `OP_SYSTEM: begin
                if (funct3 == 3'b000) begin
                    // inst[31:7] 25
                // sfence.vma is matched separately below because its rs1/rs2
                // fields are operands rather than opcode bits.
                if ((inst[31:25] == 7'h09) && (inst[14:12] == 3'b000) &&
                    (inst[6:0] == 7'h73))
                    is_sfence_vma = 1'b1;

                case (inst[31:7])
                    25'h0000000: is_ecall        = 1'b1;   // ecall       0x00000073
                    25'h0002000: is_ebreak       = 1'b1;   // ebreak      0x00100073
                    25'h0604000: is_mret         = 1'b1;   // mret        0x30200073
                    25'h0204000: is_sret         = 1'b1;   // sret        0x10200073
                    25'h020a000: is_wfi          = 1'b1;   // wfi         0x10500073
                    default: illegal = !is_sfence_vma;
                endcase
                end else begin
                    // CSR
                    is_csr   = 1'b1;
                    csr_addr = inst[31:20];
                    reg_we   = 1'b1;
                    wb_sel   = `WB_CSR;
                    use_rs1  = 1'b1;
                    case (funct3)
                        3'b001: csr_op = 2'b01;   // CSRRW
                        3'b010: csr_op = 2'b10;   // CSRRS
                        3'b011: csr_op = 2'b11;   // CSRRC
                        3'b101: begin             // CSRRWI
                            csr_op = 2'b01; use_rs1 = 1'b0; use_imm = 1'b1;
                            imm = {27'd0, rs1};
                        end
                        3'b110: begin             // CSRRSI
                            csr_op = 2'b10; use_rs1 = 1'b0; use_imm = 1'b1;
                            imm = {27'd0, rs1};
                        end
                        3'b111: begin             // CSRRCI
                            csr_op = 2'b11; use_rs1 = 1'b0; use_imm = 1'b1;
                            imm = {27'd0, rs1};
                        end
                        default: illegal = 1'b1;
                    endcase
                end
            end

            //---------------- FENCE ----------------
            `OP_FENCE: begin
                is_fence = 1'b1;
                if (funct3 != 3'b000 && funct3 != 3'b001) illegal = 1'b1;
            end

            //---------------- RV32A AMO ----------------
            `OP_AMO: begin
                // funct3: 010 = W
                // funct5 (inst[31:27])
                // aq/rl in inst[26:25] hart
                use_rs1 = 1'b1; use_rs2 = 1'b1;
                reg_we  = 1'b1; wb_sel = `WB_MEM;
                is_amo  = 1'b1;
                mem_re  = 1'b1;          // AMO read read
                mem_size = 3'd2;
                if (funct3 != 3'b010) illegal = 1'b1;
                amo_op  = inst[31:27];
            end

            default: illegal = 1'b1;
        endcase
    end

endmodule
