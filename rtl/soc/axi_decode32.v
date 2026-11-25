`timescale 1ns / 1ps
//=============================================================================
// axi_decode32.v
//
// Routes one 32-bit AXI master to three slaves by address:
//
//   0x0000_0000 .. 0x0000_FFFF   BootROM
//   0x1110_0000 .. 0x1110_FFFF   UART
//   0x8000_0000 ..               DDR
//
// Every access is a single beat, so the decode is combinational: the address
// selects which slave sees the request, and the selected slave's response is
// routed back. Unmapped addresses go to the DDR path, which is what the bridge
// produces after aliasing 0x8xxx_xxxx down to 0x0xxx_xxxx.
//=============================================================================
module axi_decode32 (
    input  wire        clk,
    input  wire        aresetn,

    // ---- master coming from the bridge ----
    // Unaliased CPU address of the access in flight, used for region selection.
    input  wire [31:0] m_cpu_addr,

    input  wire [27:0] m_awaddr,
    input  wire [7:0]  m_awlen,
    input  wire [2:0]  m_awsize,
    input  wire [1:0]  m_awburst,
    input  wire        m_awvalid,
    output wire        m_awready,
    input  wire [31:0] m_wdata,
    input  wire [3:0]  m_wstrb,
    input  wire        m_wlast,
    input  wire        m_wvalid,
    output wire        m_wready,
    output wire [1:0]  m_bresp,
    output wire        m_bvalid,
    input  wire        m_bready,

    input  wire [27:0] m_araddr,
    input  wire [7:0]  m_arlen,
    input  wire [2:0]  m_arsize,
    input  wire [1:0]  m_arburst,
    input  wire        m_arvalid,
    output wire        m_arready,
    output wire [31:0] m_rdata,
    output wire [1:0]  m_rresp,
    output wire        m_rlast,
    output wire        m_rvalid,
    input  wire        m_rready,

    // ---- BootROM ----
    output wire [27:0] rom_araddr,
    output wire        rom_arvalid,
    input  wire        rom_arready,
    input  wire [31:0] rom_rdata,
    input  wire        rom_rvalid,
    output wire        rom_rready,
    output wire [27:0] rom_awaddr,
    output wire        rom_awvalid,
    input  wire        rom_awready,
    output wire [31:0] rom_wdata,
    output wire [3:0]  rom_wstrb,
    output wire        rom_wvalid,
    input  wire        rom_wready,
    input  wire [1:0]  rom_bresp,
    input  wire        rom_bvalid,
    output wire        rom_bready,

    // ---- UART ----
    output wire [27:0] uart_araddr,
    output wire        uart_arvalid,
    input  wire        uart_arready,
    input  wire [31:0] uart_rdata,
    input  wire        uart_rvalid,
    output wire        uart_rready,
    output wire [27:0] uart_awaddr,
    output wire        uart_awvalid,
    input  wire        uart_awready,
    output wire [31:0] uart_wdata,
    output wire [3:0]  uart_wstrb,
    output wire        uart_wvalid,
    input  wire        uart_wready,
    input  wire [1:0]  uart_bresp,
    input  wire        uart_bvalid,
    output wire        uart_bready,

    // ---- DDR ----
    output wire [27:0] ddr_awaddr,
    output wire [7:0]  ddr_awlen,
    output wire [2:0]  ddr_awsize,
    output wire [1:0]  ddr_awburst,
    output wire        ddr_awvalid,
    input  wire        ddr_awready,
    output wire [31:0] ddr_wdata,
    output wire [3:0]  ddr_wstrb,
    output wire        ddr_wlast,
    output wire        ddr_wvalid,
    input  wire        ddr_wready,
    input  wire [1:0]  ddr_bresp,
    input  wire        ddr_bvalid,
    output wire        ddr_bready,

    output wire [27:0] ddr_araddr,
    output wire [7:0]  ddr_arlen,
    output wire [2:0]  ddr_arsize,
    output wire [1:0]  ddr_arburst,
    output wire        ddr_arvalid,
    input  wire        ddr_arready,
    input  wire [31:0] ddr_rdata,
    input  wire [1:0]  ddr_rresp,
    input  wire        ddr_rlast,
    input  wire        ddr_rvalid,
    output wire        ddr_rready
);

    //-------------------------------------------------------------------------
    // Regions. The bridge has already aliased 0x8xxx_xxxx to 0x0xxx_xxxx, so
    // DDR arrives here as 0x0xxx_xxxx and the ROM is identified by being below
    // 0x0001_0000 with the UART carved out by its own prefix first.
    //-------------------------------------------------------------------------
    // Region tests use the CPU's full 32-bit address. The 28-bit address below
    // has already had 0x8000_0000 subtracted, so it cannot tell a program being
    // loaded at 0x8000_0000 apart from the BootROM at 0x0000_0000.
    function is_uart(input [31:0] a);
        begin
            is_uart = (a[31:16] == 16'h1110);
        end
    endfunction

    function is_rom(input [31:0] a);
        begin
            is_rom = (a[31:12] == 20'h00000) && !is_uart(a);
        end
    endfunction

    wire wr_uart = is_uart(m_cpu_addr);
    wire wr_rom  = is_rom(m_cpu_addr);
    wire rd_uart = is_uart(m_cpu_addr);
    wire rd_rom  = is_rom(m_cpu_addr);

    wire wr_ddr  = !wr_uart && !wr_rom;
    wire rd_ddr  = !rd_uart && !rd_rom;

    //-------------------------------------------------------------------------
    // Write address channel
    //-------------------------------------------------------------------------
    assign rom_awaddr  = m_awaddr;
    assign rom_awvalid = m_awvalid && wr_rom;
    assign uart_awaddr  = m_awaddr;
    assign uart_awvalid = m_awvalid && wr_uart;
    assign ddr_awaddr  = m_awaddr;
    assign ddr_awlen   = m_awlen;
    assign ddr_awsize  = m_awsize;
    assign ddr_awburst = m_awburst;
    assign ddr_awvalid = m_awvalid && wr_ddr;

    assign m_awready = wr_rom  ? rom_awready  :
                       wr_uart ? uart_awready :
                                 ddr_awready;

    //-------------------------------------------------------------------------
    // Write data channel
    //-------------------------------------------------------------------------
    assign rom_wdata  = m_wdata;
    assign rom_wstrb  = m_wstrb;
    assign rom_wvalid = m_wvalid && wr_rom;
    assign uart_wdata  = m_wdata;
    assign uart_wstrb  = m_wstrb;
    assign uart_wvalid = m_wvalid && wr_uart;
    assign ddr_wdata   = m_wdata;
    assign ddr_wstrb   = m_wstrb;
    assign ddr_wlast   = m_wlast;
    assign ddr_wvalid  = m_wvalid && wr_ddr;

    assign m_wready = wr_rom  ? rom_wready  :
                      wr_uart ? uart_wready :
                                ddr_wready;

    //-------------------------------------------------------------------------
    // Write response
    //-------------------------------------------------------------------------
    assign rom_bready  = m_bready && wr_rom;
    assign uart_bready = m_bready && wr_uart;
    assign ddr_bready  = m_bready && wr_ddr;

    assign m_bresp  = wr_rom  ? rom_bresp  :
                      wr_uart ? uart_bresp :
                                ddr_bresp;
    assign m_bvalid = wr_rom  ? rom_bvalid  :
                      wr_uart ? uart_bvalid :
                                ddr_bvalid;

    //-------------------------------------------------------------------------
    // Read address channel
    //-------------------------------------------------------------------------
    assign rom_araddr  = m_araddr;
    assign rom_arvalid = m_arvalid && rd_rom;
    assign uart_araddr  = m_araddr;
    assign uart_arvalid = m_arvalid && rd_uart;
    assign ddr_araddr  = m_araddr;
    assign ddr_arlen   = m_arlen;
    assign ddr_arsize  = m_arsize;
    assign ddr_arburst = m_arburst;
    assign ddr_arvalid = m_arvalid && rd_ddr;

    assign m_arready = rd_rom  ? rom_arready  :
                       rd_uart ? uart_arready :
                                 ddr_arready;

    //-------------------------------------------------------------------------
    // Read data channel
    //-------------------------------------------------------------------------
    assign rom_rready  = m_rready && rd_rom;
    assign uart_rready = m_rready && rd_uart;
    assign ddr_rready  = m_rready && rd_ddr;

    assign m_rdata  = rd_rom  ? rom_rdata  :
                      rd_uart ? uart_rdata :
                                ddr_rdata;
    assign m_rresp  = rd_rom  ? 2'b00      :
                      rd_uart ? 2'b00      :
                                ddr_rresp;
    assign m_rlast  = rd_rom  ? 1'b1       :
                      rd_uart ? 1'b1       :
                                ddr_rlast;
    assign m_rvalid = rd_rom  ? rom_rvalid  :
                      rd_uart ? uart_rvalid :
                                ddr_rvalid;

endmodule
