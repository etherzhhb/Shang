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
#if v.AddressSpace == 0 then
import "DPI-C" function chandle vlt_$(escapeNumber(k))();
`define gv$(k) vlt_$(escapeNumber(k))()
#end
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

--Generate initialize file for block rams.
for k,v in pairs(GlobalVariables) do
  if v.AddressSpace ~= 0 then
    if v.Initializer ~= nil then
      InitFile = io.open (TESTBINARYFLOD .. '/' .. k .. '_init.txt', 'w')
      for i,n in ipairs(v.Initializer) do
        InitFile:write(string.sub(n, 3)..'\n')
      end
      io.close(InitFile)
    end
  end --end addresssapce == 0
end

local preprocess = require "luapp" . preprocess
RTLGlobalCode, message = preprocess {input=RTLGlobalTemplate}
if message ~= nil then print(message) end
]=]

RunOnDatapath = [=[
$(_put('#')) Slack $(RTLDatapath.Slack)
#local Slack = RTLDatapath.Slack
#local DstName = RTLDatapath.Nodes[1].Name
#local SrcName = RTLDatapath.Nodes[table.getn(RTLDatapath.Nodes)].Name
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

set isInSta 0

if $isInSta {
  # Report the missed constraints.
  load_package report
  load_report
  create_report_panel -folder "Timing Constraints"
  set missedPanelid  [create_report_panel -table "Timing Constraints||Missed Constraints"]
  add_row_to_table -id $missedPanelid [list {src} {dst} {cycles}]
}

]=]

IcdelayTclGen = [=[
$(_put('#')) Slack $(RTLDatapath.Slack)
#local Slack = RTLDatapath.Slack
#local DstName = RTLDatapath.Nodes[1].Name
#local SrcName = RTLDatapath.Nodes[table.getn(RTLDatapath.Nodes)].Name
#DstName = '*' .. CurModule:getName() .. '_inst|' .. DstName
#SrcName = '*' .. CurModule:getName() .. '_inst|' .. SrcName
set fileid [open "$(IcDelayTmp)" a+]

set dst [get_keepers {$(DstName)*}]
set src [get_keepers {$(SrcName)*}]
if { [get_collection_size $src] && [get_collection_size $dst] } {
  Write $fileid $dst $src
}
close $fileid
]=]

IcdelayTclHeader = [=[
proc Write {fileid dst src} {
  foreach_in_collection path [get_path -from $src -to $dst -npath 8] {
    set pathSlack [get_path_info $path -slack]
    set from [get_path_info $path -from]
    set to   [get_path_info $path -to]
    #Only record the paths with a negative slack
    set icdelay 0
    set celldelay 0
    set IcFanOutTotal 0.0
    set CellFanOutTotal 0
    set AllFanOutTotal 0
    set RegFanNum 0
    set RegName [get_node_info $from -name]
    set list [get_node_info $from -fanout_edges]
    if {$pathSlack < 0.1} {
    foreach regfannum $list {
      set RegFanNum [expr $RegFanNum + 1]
    }
    foreach_in_collection pt [ get_path_info $path -arrival_points ] {
      set total     [get_point_info $pt -total]
      set incr      [get_point_info $pt -incr]
      set type      [get_point_info $pt -type]
      set number_fanout [get_point_info $pt -number_of_fanout]
      set AllFanOutTotal [expr $AllFanOutTotal + $number_fanout]
      if { $type == "cell" } {
        set CellFanOutTotal [expr $CellFanOutTotal + $number_fanout]
        set celldelay [expr $celldelay + $incr]
      }
      if { $type == "ic" } {
        set IcFanOutTotal [expr $IcFanOutTotal + $number_fanout]
        set icdelay [expr $icdelay + $incr]
      }
    }
    set rate [expr $icdelay/$total]
    set fanoutrate [expr $IcFanOutTotal/$AllFanOutTotal]
    #Write in json file
    puts $fileid ",\{\"slack\":$pathSlack,\
    \"netdelay\":$icdelay,\
    \"celldelay\":$celldelay,\
    \"totaldelay\":$total,\
    \"net_total_rate\":$rate,\
    \"IcFanOutTotal\":$IcFanOutTotal,\
    \"CellFanOutTotal\":$CellFanOutTotal,\
    \"AllFanOutTotal\":$AllFanOutTotal,\
    \"fanout_rate\":$fanoutrate,\
    \"regname\":\"$RegName\",\
    \"RegFanNum\":$RegFanNum\}"
    }
  }
}
]=]