RunOnDatapath = [=[
#local Slack = RTLDatapath.Slack
#local DstName = RTLDatapath.Nodes[1].Name
#local SrcName = RTLDatapath.Nodes[table.getn(RTLDatapath.Nodes)].Name

#if (RTLDatapath.isCriticalPath == 1) then
$(_put('#')) $(DstName) <- $(SrcName) Slack $(Slack)
set_multicycle_path -from $(SrcName)* -to $(DstName)* -setup -end $(Slack)
#end
#if Functions[FuncInfo.Name] == nil then
#  DstName = '*' .. CurModule:getName() .. '_inst|' .. DstName
#  SrcName = '*' .. CurModule:getName() .. '_inst|' .. SrcName
#end
#local has_thu_node = false
#for i, n in pairs(RTLDatapath.Nodes) do
#  if n.Name ~= 'n/a' and n.Name ~= DstName and n.Name ~= SrcName then
#    local ThuName = n.Name
#    has_thu_node = true
#    if Functions[FuncInfo.Name] == nil then
#      ThuName = '*' .. CurModule:getName() .. '_inst|' .. ThuName
#    end
$(_put('#')) $(DstName) <- $(ThuName) <- $(SrcName) Slack $(Slack)
set_multicycle_path -from $(SrcName)* -through $(ThuName)* -to $(DstName)* -setup -end $(Slack)
#  end -- End valid thu node.
#end -- end for
]=]

SDCHeader = [=[
create_clock -name "clk" -period $(PERIOD)ns [get_ports {clk}]
derive_pll_clocks -create_base_clocks
set_multicycle_path -from [get_clocks {clk}] -to [get_clocks {clk}] -hold -end 1
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
