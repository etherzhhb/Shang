RunOnDatapath = [=[
#_put('# Slack ' .. RTLDatapath.Slack .. ' Path nodes: ')
#for i, n in pairs(RTLDatapath.Nodes) do
#  _put(n.Name .. ', ')
#end

#local Slack = RTLDatapath.Slack
#local DstName = RTLDatapath.Nodes[1].Name
#local SrcName = RTLDatapath.Nodes[table.getn(RTLDatapath.Nodes)].Name
#if Functions[FuncInfo.Name] == nil then
#DstName = '*' .. CurModule:getName() .. '_inst|' .. DstName
#SrcName = '*' .. CurModule:getName() .. '_inst|' .. SrcName
#end
set dst [get_keepers {$(DstName)*}]
set src [get_keepers {$(SrcName)*}]
if { [get_collection_size $src] && [get_collection_size $dst] } {
#  for i, n in pairs(RTLDatapath.Nodes) do
#    if n.Name ~= 'n/a' and n.Name ~= DstName and n.Name ~= SrcName then
#      local ThuName = n.Name
#      if Functions[FuncInfo.Name] == nil then
#        ThuName = '*' .. CurModule:getName() .. '_inst|' .. ThuName
#      end
  set thu [get_keepers {$(ThuName)*}]
  if { [get_collection_size $thu]} {
    set_max_delay -from $src -through $thu -to $dst $(Slack * PERIOD)ns
  }

#    end -- End valid thu node.
#  end -- end for
  set_max_delay -from $src -to $dst $(Slack * PERIOD)ns
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

Misc.DatapathScript = [=[
local SlackFile = assert(io.open (MainSDCOutput, "a+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=RunOnDatapath, output=SlackFile}
if message ~= nil then print(message) end
SlackFile:close()
]=]

Misc.TimingConstraintsHeaderScript = [=[
local SlackFile = assert(io.open (MainSDCOutput, "w"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=SDCHeader, output=SlackFile}
if message ~= nil then print(message) end
SlackFile:close()
]=]

SynAttr.ParallelCaseAttr = '/* parallel_case */'
SynAttr.FullCaseAttr = '/* full_case */'
