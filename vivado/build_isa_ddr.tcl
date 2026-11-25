#=============================================================================
# build_isa_ddr.tcl
#
# Core + DDR only: no BootROM, no SRAM, no external memory model.
#
#   riscv_core -> cpu_ddr_bridge -> ddr3 (MIG, 32-bit AXI)
#
# The controller is the one from vivado/ddr_sc, the only configuration that has
# been shown to train and to return correct data on this board.
#=============================================================================

set RTL_ROOT "/mnt/d/04.Code/Verilog/RISCV"
set RTL_WIN  "D:/04.Code/Verilog/RISCV"
set PROJ_DIR "$RTL_WIN/vivado/isa_ddr"
set PART     "xc7a35tfgg484-2"
set TOP      "isa_ddr_top"

puts "=============================================="
puts " ISA cases from DDR"
puts "=============================================="

if {[file exists $PROJ_DIR]} {
    puts ">>> Removing old project"
    file delete -force $PROJ_DIR
}

create_project isa_ddr $PROJ_DIR -part $PART -force
set_property target_language Verilog [current_project]

#-----------------------------------------------------------------------------
# Core RTL
#-----------------------------------------------------------------------------
puts ">>> Adding core RTL"
set core_files [glob -nocomplain "$RTL_WIN/rtl/core/*.v"]
puts "    [llength $core_files] core files"
add_files -norecurse $core_files

add_files -norecurse [list \
    $RTL_WIN/rtl/soc/gp_trace.v \
    $RTL_WIN/rtl/mmu/mmu_sv32.v \
    $RTL_WIN/rtl/mmu/tlb.v \
    $RTL_WIN/rtl/mmu/mmu_arb.v \
    "$RTL_WIN/rtl/soc/cpu_ddr_bridge.v" \
    "$RTL_WIN/rtl/soc/axi4_master.v" \
    "$RTL_WIN/rtl/soc/axi_decode32.v" \
    "$RTL_WIN/rtl/soc/bootrom32.v" \
    "$RTL_WIN/rtl/soc/tohost_watch.v" \
    "$RTL_WIN/rtl/soc/uart_mmio32.v" \
    "$RTL_WIN/rtl/periph/uart_rx.v" \
    "$RTL_WIN/rtl/periph/uart_tx.v" \
    "$RTL_WIN/rtl/soc/isa_ddr_top.v" \
]

#-----------------------------------------------------------------------------
# Controller IP, copied from the working project
#-----------------------------------------------------------------------------
puts ">>> Adding the controller"
set src_ip "$RTL_WIN/vivado/ddr3_axi32_src"
set dst_ip "$PROJ_DIR/ip"


create_ip -name clk_wiz -vendor xilinx.com -library ip -module_name clk_wiz_0
set_property -dict [list \
    CONFIG.PRIM_IN_FREQ {50.000} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {200.000} \
    CONFIG.USE_LOCKED {true} \
    CONFIG.USE_RESET {true} \
] [get_ips clk_wiz_0]
generate_target all [get_ips clk_wiz_0]

create_ip -name vio -vendor xilinx.com -library ip -module_name vio_state
set_property -dict [list \
    CONFIG.C_NUM_PROBE_IN {6} \
    CONFIG.C_NUM_PROBE_OUT {2} \
    CONFIG.C_PROBE_IN0_WIDTH {32} \
    CONFIG.C_PROBE_IN1_WIDTH {32} \
    CONFIG.C_PROBE_IN2_WIDTH {32} \
    CONFIG.C_PROBE_IN3_WIDTH {32} \
    CONFIG.C_PROBE_IN4_WIDTH {32} \
    CONFIG.C_PROBE_IN5_WIDTH {32} \
    CONFIG.C_PROBE_OUT0_WIDTH {1} \
    CONFIG.C_PROBE_OUT1_WIDTH {1} \
] [get_ips vio_state]
generate_target all [get_ips vio_state]

# Probes on the receive path: raw pin, synchronised pin, state, bit counter.
create_ip -name ila -vendor xilinx.com -library ip -module_name ila_uart
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES {7} \
    CONFIG.C_PROBE0_WIDTH {32} \
    CONFIG.C_PROBE1_WIDTH {32} \
    CONFIG.C_PROBE2_WIDTH {32} \
    CONFIG.C_PROBE3_WIDTH {32} \
    CONFIG.C_PROBE4_WIDTH {32} \
    CONFIG.C_PROBE5_WIDTH {32} \
    CONFIG.C_PROBE6_WIDTH {32} \
    CONFIG.C_DATA_DEPTH {4096} \
] [get_ips ila_uart]
generate_target all [get_ips ila_uart]

#-----------------------------------------------------------------------------
# Controller: bring in the XCI and regenerate output in this project
#
# Copying the demo's pre-built products does not work: they were generated for
# the demo's own project and the stub leaves the module undefined at
# elaboration. Only the XCI and the constraint file are taken over.
#-----------------------------------------------------------------------------
puts ">>> Copying the controller XCI"
set dst_ddr "$PROJ_DIR/src_ddr3"
file mkdir $dst_ddr
file copy -force "$src_ip/ddr3.xci"     "$dst_ddr/ddr3.xci"
file copy -force "$src_ip/mig_a.prj"    "$dst_ddr/mig_a.prj"
if {[file exists "$src_ip/mig_b.prj"]} {
    file copy -force "$src_ip/mig_b.prj" "$dst_ddr/mig_b.prj"
}
add_files -norecurse "$dst_ddr/ddr3.xci"

puts ">>> Generating the controller"
set ddr3_ip [get_ips ddr3]
if {$ddr3_ip eq ""} {
    puts "!!! controller IP not found after adding its XCI"
    exit 1
}
generate_target all $ddr3_ip

# generate_target already produced the RTL the IP needs; synthesis picks it up
# from the IP's own source list.

#-----------------------------------------------------------------------------
# Constraints
#-----------------------------------------------------------------------------
puts ">>> Adding constraints"

set xdc_data {
create_clock -period 20.000 -name sys_clk [get_ports sys_clk]
set_property IOSTANDARD LVCMOS33 [get_ports sys_clk]
set_property PACKAGE_PIN  Y18    [get_ports sys_clk]

set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PACKAGE_PIN  F20    [get_ports rst_n]

set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]
set_property PACKAGE_PIN  G16    [get_ports uart_tx]

set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]
set_property PACKAGE_PIN  G15    [get_ports uart_rx]

set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN  F19    [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN  E21    [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN  D20    [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property PACKAGE_PIN  C20    [get_ports {led[3]}]

set_false_path -from [get_ports rst_n]

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
}
set fp [open "$PROJ_DIR/pin.xdc" w]
puts $fp $xdc_data
close $fp
add_files -fileset constrs_1 -norecurse "$PROJ_DIR/pin.xdc"

# Pin constraints for the memory interface. Without these the PHY has no
# placement and calibration never completes.
set ddr3_xdc [glob -nocomplain "$src_ip/ddr3/user_design/constraints/ddr3.xdc"]
if {[llength $ddr3_xdc] > 0} {
    puts ">>> Controller constraints: [lindex $ddr3_xdc 0]"
    add_files -fileset constrs_1 -norecurse [lindex $ddr3_xdc 0]
} else {
    puts "!!! ddr3.xdc not found under $src_ip"
    exit 1
}

# mmu_sv32 includes riscv_pkg.v from the core directory.
set_property include_dirs [list "$RTL_WIN/rtl/core" "$RTL_WIN/rtl/soc"] [current_fileset]
set_property top $TOP [current_fileset]
update_compile_order -fileset sources_1

#-----------------------------------------------------------------------------
# Build
#-----------------------------------------------------------------------------
puts ">>> Synthesis"
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    puts "!!! SYNTHESIS FAILED"
    puts "    See: $PROJ_DIR/isa_ddr.runs/synth_1/runme.log"
    exit 1
}
puts ">>> Synthesis OK"

puts ">>> Implementation"
# Physical optimisation is enabled because the remaining timing failures are
# dominated by routing delay rather than logic depth, which is exactly what this
# step improves.
set impl [get_runs impl_1]
reset_run impl_1
set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE Explore $impl
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE Explore $impl
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true $impl
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore $impl
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE Explore $impl

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    puts "!!! IMPLEMENTATION FAILED"
    puts "    See: $PROJ_DIR/isa_ddr.runs/impl_1/runme.log"
    exit 1
}
puts ">>> Implementation OK"

set bit [glob -nocomplain "$PROJ_DIR/isa_ddr.runs/impl_1/*.bit"]
if {[llength $bit] == 0} {
    puts "!!! BITSTREAM MISSING"
    exit 1
}
puts ">BIT_OK [lindex $bit 0]"
