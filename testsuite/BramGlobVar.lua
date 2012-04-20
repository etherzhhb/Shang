RTLGlobalTemplate = [=[
/* verilator lint_off DECLFILENAME */
/* verilator lint_off WIDTH */
/* verilator lint_off UNUSED */

`ifdef quartus_synthesis
#local table_size = # table_name
#for i = 1, table_size do
#_put('\`define gv')
#_put(table_name[i])
#_put(' 32\'d')
#_put((table_num[i])*8)
#_put('\n')
#end

`else
#for k,v in pairs(GlobalVariables) do
import "DPI-C" function chandle vlt_$(escapeNumber(k))();
`define gv$(k) vlt_$(escapeNumber(k))()
#end
`endif
]=]

Misc.RTLGlobalScript = [=[
table_name = {}
table_num = {}
LineTotal = {}
local BramInitFile = assert(io.open (BRAMINIT, "a+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=BlockRAMInitFileGenScript, output=BramInitFile}
if message ~= nil then print(message) end
BramInitFile:close()
local preprocess = require "luapp" . preprocess
RTLGlobalCode, message = preprocess {input=RTLGlobalTemplate}
if message ~= nil then print(message) end
]=]

RunOnDatapath = [=[
$(_put('#')) Slack $(RTLDatapath.Slack)
#local Slack = RTLDatapath.Slack
#local DstName = RTLDatapath.Nodes[1]
#local SrcName = RTLDatapath.Nodes[table.getn(RTLDatapath.Nodes)]
#DstName = '*' .. CurModule:getName() .. '_inst|' .. DstName
#SrcName = '*' .. CurModule:getName() .. '_inst|' .. SrcName
set dst [get_keepers {$(DstName)*}]
set src [get_keepers {$(SrcName)*}]

if { [get_collection_size $src] && [get_collection_size $dst] } {
  $(if Slack == 1 then _put('#') end) set_max_delay -from $src -to $dst $(Slack * PERIOD - PERIOD * 0.05)ns
} elseif {$isInSta && $(Slack) > 1} {
  add_row_to_table -id $missedPanelid [list {$(SrcName)} {$(DstName)} {$(Slack)}]
  #FIXME: Dont save the database every time when a row appended
  save_report_database
}
]=]

SDCHeader = [=[
create_clock -name "clk" -period $(PERIOD)ns [get_ports {clk}]
derive_pll_clocks -create_base_clocks

set isInSta [string match "quartus_sta" $quartus(nameofexecutable)]

if $isInSta {
  # Report the missed constraints.
  load_package report
  load_report
  create_report_panel -folder "Timing Constraints"
  set missedPanelid  [create_report_panel -table "Timing Constraints||Missed Constraints"]
  add_row_to_table -id $missedPanelid [list {src} {dst} {cycles}]
}

]=]
