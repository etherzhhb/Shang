#############################################################################
#
#############################################################################
puts $DataJSON "\"les\":\"[get_fitter_resource_usage -resource {Total logic elements}]\","
puts $DataJSON "\"les_wo_reg\":\"[get_fitter_resource_usage -resource {*Combinational with no register}]\","
puts $DataJSON "\"les_w_reg_only\":\"[get_fitter_resource_usage -resource {*Register only}]\","
puts $DataJSON "\"les_and_reg\":\"[get_fitter_resource_usage -resource {*Combinational with a register}]\","

puts $DataJSON "\"lut4\":\"[get_fitter_resource_usage -resource {*4 input functions}]\","
puts $DataJSON "\"lut3\":\"[get_fitter_resource_usage -resource {*3 input functions}]\","
puts $DataJSON "\"lut2\":\"[get_fitter_resource_usage -resource {*2 input functions}]\","

#puts $DataJSON "\"les_normal\":\"[get_fitter_resource_usage -resource {*normal mode}]\","
#puts $DataJSON "\"les_arit\":\"[get_fitter_resource_usage -resource {*arithmetic mode}]\","

puts $DataJSON "\"regs\":\"[get_fitter_resource_usage -resource {*Dedicated logic registers}]\","

puts $DataJSON "\"mult9\":\"[get_fitter_resource_usage -resource {*Embedded Multiplier 9-bit elements}]\","
puts $DataJSON "\"ave_ic\":\"[get_fitter_resource_usage -resource {*Average interconnect usage (total/H/V)}]\","
puts $DataJSON "\"peak_ic\":\"[get_fitter_resource_usage -resource {*Peak interconnect usage (total/H/V)}]\","

#puts $DataJSON "\"max_fanout_node\":\"[get_fitter_resource_usage -resource {*Maximum fan-out node}]\","
#puts $DataJSON "\"max_fanout\":\"[get_fitter_resource_usage -resource {*Maximum fan-out}]\","
puts $DataJSON "\"max_fanout_non-global_node\":\"[get_fitter_resource_usage -resource {*Highest non-global fan-out signal}]\","
puts $DataJSON "\"max_fanout_non-global\":\"[get_fitter_resource_usage -resource {*Highest non-global fan-out}]\","
puts $DataJSON "\"total_fanout\":\"[get_fitter_resource_usage -resource {*Total fan-out}]\","
puts $DataJSON "\"ave_fanout\":\"[get_fitter_resource_usage -resource {*Average fan-out}]\""
puts $DataJSON "\}"
close $DataJSON

################################################################################
# Detail report
set DataJSON [open $BenchmarkReportTmp a+]
puts $DataJSON ",\n\{\"name\":\"$MAIN_RTL_ENTITY\""

# Detail report
set PanelsToReport [list \
{Multiplexer Restructuring Statistics (Restructuring Performed)} \
{Slack Histogram} \
{Timging Path}]

proc panel_to_csv {panel_name DataJSON} {
  puts $DataJSON ","
  puts $DataJSON "\"[string map {{ } _} $panel_name]\": \["

  set num_rows [get_number_of_rows -name "*$panel_name"]

  # Go through all the rows in the report file, including the
  # row with headings, and write out the comma-separated data
  for { set i 0 } { $i < $num_rows } { incr i } {
    set row_data [get_report_panel_row -name "*$panel_name" -row $i]
    if {$i != $num_rows - 1} {
      set endChar ","
    } else {
      set endChar "\]"
    }
    # Store the data in form of array of string
    set rowStr "  \[\"[join $row_data {", "}]\"\]$endChar"
    # Remove the new lines.
    puts $DataJSON [string map {"\n" { }} $rowStr]
  }
}

foreach panel $PanelsToReport {
  panel_to_csv $panel $DataJSON
}
puts $DataJSON "\}"

close $DataJSON
unload_report
