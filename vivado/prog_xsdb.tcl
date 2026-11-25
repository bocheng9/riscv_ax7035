# Program the bitstream over JTAG with xsdb.
#
#   xsdb prog_xsdb.tcl [bitstream]
#
# Vivado's batch mode has no open_hw_manager, so the hardware manager path is
# not available from a script. xsdb does expose fpga -file, which is what this
# uses; it needs an hw_server to be reachable (xsdb starts one if not).
#
# The bitstream path is relative to the repository root, which is derived from
# this script's own location so that the script works from any checkout.

set here [file dirname [file normalize [info script]]]
set root [file dirname $here]

if {[llength $argv] > 0} {
    set bitfile [lindex $argv 0]
} else {
    set bitfile [file join $root "isa_ddr" "isa_ddr.runs" "impl_1" "isa_ddr_top.bit"]
}

# Vivado writes the project under <root>/isa_ddr, so also accept the layout
# where the build output sits next to this script.
if {![file exists $bitfile]} {
    set bitfile [file join $root "vivado" "isa_ddr" "isa_ddr.runs" "impl_1" "isa_ddr_top.bit"]
}

if {![file exists $bitfile]} {
    puts "!!! bitstream not found: $bitfile"
    puts "    run vivado/build_isa_ddr.tcl first, or pass the path as an argument"
    exit 1
}
puts "Bitstream: $bitfile"

connect
set devs [targets -filter {name =~ "*xc7a*"}]
if {[llength $devs] == 0} {
    puts "!!! no FPGA target found"
    puts "    check board power, the USB cable and the JTAG mode switch"
    exit 1
}
target [lindex $devs 0]

fpga -file $bitfile
puts "=== PROGRAMMED ==="
exit 0
