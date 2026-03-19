set_param general.maxThreads 4
create_project -in_memory -part xc7a35tcpg236-1

read_verilog -sv /home/brendan/synthesis_workspace/CORDIC/cordic_rotation_iterative.sv

set xdc_file "/home/brendan/synthesis_workspace/CORDIC/synthesis_logs/clock_cordic_rotation_iterative.xdc"
set fp [open $xdc_file w]
puts $fp "create_clock -period 10.000 -name CLK \[get_ports CLK\]"
close $fp
read_xdc $xdc_file

synth_design -top cordic_rotation_iterative -part xc7a35tcpg236-1

report_utilization -file /home/brendan/synthesis_workspace/CORDIC/synthesis_logs/utilization_cordic_rotation_iterative.rpt
report_timing_summary -file /home/brendan/synthesis_workspace/CORDIC/synthesis_logs/timing_cordic_rotation_iterative.rpt
