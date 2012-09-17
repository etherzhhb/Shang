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
puts $DataJSON "\"ave_fanout\":\"[get_fitter_resource_usage -resource {*Average fan-out}]\","
