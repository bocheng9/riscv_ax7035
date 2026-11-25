#=============================================================================
# soc_ddr_top.xdc
# 板级约束（DDR3 引脚约束见 ip/ddr3/ddr3/user_design/constraints/ddr3.xdc，
#            由 MIG IP 自动带入，此处不重复）
#
# 板卡: ALINX AX7035 (XC7A35T-FGG484-2)
#=============================================================================

set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4  [current_design]
set_property CONFIG_MODE SPIx4                [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50   [current_design]

#-----------------------------------------------------------------------------
# 系统时钟 50MHz (Y18)
#   ★ 200MHz 由 clk_wiz_0 生成，其约束在 IP 自带的 xdc 里
#-----------------------------------------------------------------------------
create_clock -period 20.000 -name sys_clk [get_ports sys_clk]
set_property IOSTANDARD LVCMOS33 [get_ports sys_clk]
set_property PACKAGE_PIN  Y18    [get_ports sys_clk]

#-----------------------------------------------------------------------------
# 复位 (F20, 低有效)
#-----------------------------------------------------------------------------
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PACKAGE_PIN  F20    [get_ports rst_n]

#-----------------------------------------------------------------------------
# UART (FT232HL)
#-----------------------------------------------------------------------------
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]
set_property PACKAGE_PIN  G15    [get_ports uart_rx]

set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]
set_property PACKAGE_PIN  G16    [get_ports uart_tx]

#-----------------------------------------------------------------------------
# LED (★ 共阳极：FPGA 输出 0 时点亮)
#-----------------------------------------------------------------------------
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN  F19    [get_ports {led[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN  E21    [get_ports {led[1]}]

set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN  D20    [get_ports {led[2]}]

set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property PACKAGE_PIN  C20    [get_ports {led[3]}]

#-----------------------------------------------------------------------------
# 时序例外
#-----------------------------------------------------------------------------
set_false_path -from [get_ports uart_rx]
set_false_path -to   [get_ports uart_tx]
set_false_path -to   [get_ports {led[*]}]
set_false_path -from [get_ports rst_n]
