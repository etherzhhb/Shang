RunOnDatapath = [=[
#local Slack = RTLDatapath.Slack
#local DstName = RTLDatapath.Nodes[1].Name
#local SrcName = RTLDatapath.Nodes[table.getn(RTLDatapath.Nodes)].Name
#_put('#' .. DstName .. '<-' .. SrcName .. ' Slack ' .. Slack .. ' Path nodes: ')
#for i, n in pairs(RTLDatapath.Nodes) do
#  _put(n.Name .. ', ')
#end

#if Functions[FuncInfo.Name] == nil then
#  DstName = '*' .. CurModule:getName() .. '_inst|' .. DstName
#  SrcName = '*' .. CurModule:getName() .. '_inst|' .. SrcName
#end
#local has_thu_node = false
#for i, n in pairs(RTLDatapath.Nodes) do
#  if n.Name ~= 'n/a' and n.Name ~= DstName and n.Name ~= SrcName then
#    local ThuName = n.Name
#    if Functions[FuncInfo.Name] == nil then
#      ThuName = '*' .. CurModule:getName() .. '_inst|' .. ThuName
#    end
set_max_delay -from $(SrcName)* -through $(ThuName)* -to $(DstName)* $(Slack * PERIOD)ns
#  end -- End valid thu node.
#end -- end for

#if (Slack > 1 and RTLDatapath.isCriticalPath == 1) then
  set_max_delay -from $src -to $dst $(Slack * PERIOD)ns
#end
}
]=]

SDCHeader = [=[
create_clock -name "clk" -period $(PERIOD)ns [get_ports {clk}]
derive_pll_clocks -create_base_clocks
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
