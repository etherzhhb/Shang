#############################################################################
#
#############################################################################
# Create timing assignments
#create_base_clock -fmax "100 MHz" -target clk clk

# turn small rams into logic
#set_global_assignment -name AUTO_RAM_TO_LCELL_CONVERSION ON

# prevent DSP blocks from being used
#set_global_assignment -name DSP_BLOCK_BALANCING "LOGIC ELEMENTS"

# minimize circuit area when packing
#set_global_assignment -name INI_VARS "fit_pack_for_density_light=on; fit_report_lab_usage_stats=on"

# extra synthesis options
#set_global_assignment -name SMART_RECOMPILE ON
#set_global_assignment -name OPTIMIZE_MULTI_CORNER_TIMING ON
if {$ENABLE_PHYSICAL_SYNTHESIS == "ON"} {
  set_global_assignment -name PHYSICAL_SYNTHESIS_COMBO_LOGIC ON
  set_global_assignment -name PHYSICAL_SYNTHESIS_REGISTER_RETIMING ON
  set_global_assignment -name PHYSICAL_SYNTHESIS_REGISTER_DUPLICATION ON
  set_global_assignment -name PHYSICAL_SYNTHESIS_ASYNCHRONOUS_SIGNAL_PIPELINING ON
  set_global_assignment -name PHYSICAL_SYNTHESIS_COMBO_LOGIC_FOR_AREA ON
  set_global_assignment -name PHYSICAL_SYNTHESIS_MAP_LOGIC_TO_MEMORY_FOR_AREA ON
  set_global_assignment -name PHYSICAL_SYNTHESIS_EFFORT EXTRA
}
#set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL
#set_global_assignment -name FLOW_ENABLE_IO_ASSIGNMENT_ANALYSIS ON
#set_global_assignment -name CYCLONEII_OPTIMIZATION_TECHNIQUE SPEED
#set_global_assignment -name SYNTH_TIMING_DRIVEN_SYNTHESIS ON
#set_global_assignment -name ADV_NETLIST_OPT_SYNTH_WYSIWYG_REMAP ON
#set_global_assignment -name OPTIMIZE_POWER_DURING_FITTING "NORMAL COMPILATION"
#set_global_assignment -name ROUTER_TIMING_OPTIMIZATION_LEVEL MAXIMUM
#set_global_assignment -name PLACEMENT_EFFORT_MULTIPLIER 4
#set_global_assignment -name ROUTER_EFFORT_MULTIPLIER 4
#set_global_assignment -name FINAL_PLACEMENT_OPTIMIZATION ALWAYS
#set_global_assignment -name FITTER_AGGRESSIVE_ROUTABILITY_OPTIMIZATION AUTOMATICALLY
#set_global_assignment -name ENABLE_DRC_SETTINGS ON
#set_global_assignment -name SAVE_DISK_SPACE OFF
#set_global_assignment -name MUX_RESTRUCTURE OFF
#set_global_assignment -name OPTIMIZE_POWER_DURING_SYNTHESIS "NORMAL COMPILATION"
#set_global_assignment -name STATE_MACHINE_PROCESSING AUTO
#set_global_assignment -name PARALLEL_SYNTHESIS ON
#set_global_assignment -name AUTO_PACKED_REGISTERS_STRATIXII NORMAL
#set_global_assignment -name AUTO_PACKED_REGISTERS_MAXII NORMAL
#set_global_assignment -name AUTO_PACKED_REGISTERS_CYCLONE NORMAL
#set_global_assignment -name AUTO_PACKED_REGISTERS NORMAL
#set_global_assignment -name AUTO_PACKED_REGISTERS_STRATIX NORMAL
#set_global_assignment -name PRE_MAPPING_RESYNTHESIS ON
#set_global_assignment -name SYNTH_PROTECT_SDC_CONSTRAINT ON

#Don't produce too much messages
set_global_assignment -name HDL_MESSAGE_LEVEL LEVEL1
set_global_assignment -name SYNTH_MESSAGE_LEVEL LOW
set_global_assignment -name SYNTH_PROTECT_SDC_CONSTRAINT ON

# Timing report.
#Enable Worst-Case Timing Report
#set_global_assignment -name TIMEQUEST_DO_REPORT_TIMING ON
#set_global_assignment -name TIMEQUEST_REPORT_WORST_CASE_TIMING_PATHS ON
#Report 10 paths per clock domain
#set_global_assignment -name TIMEQUEST_REPORT_NUM_WORST_CASE_TIMING_PATHS 10

# Commit assignments
#export_assignments

# Compile project.
execute_module -tool map
execute_module -tool fit
execute_module -tool asm

