AlteraRunOnDatapath = [=[
#local Slack = RTLDatapath.Slack
#local DstName = RTLDatapath.Nodes[1].Name
#local SrcName = RTLDatapath.Nodes[table.getn(RTLDatapath.Nodes)].Name
#if Functions[FuncInfo.Name] == nil then
#  DstName = '*' .. CurModule:getName() .. '_inst|' .. DstName
#  SrcName = '*' .. CurModule:getName() .. '_inst|' .. SrcName
#elseif FuncInfo.Name == 'main' then
#  DstName = '*' .. Functions[FuncInfo.Name].ModName .. '_inst|' .. DstName
#  SrcName = '*' .. Functions[FuncInfo.Name].ModName .. '_inst|' .. SrcName
#end

set src [get_keepers $(SrcName)*]
set dst [get_keepers $(DstName)*]
if {[get_collection_size $src] && [get_collection_size $dst]} {
#if (RTLDatapath.isCriticalPath == 1) then
$(_put('#')) $(DstName) <- $(SrcName) Slack $(Slack)
  set_multicycle_path -from $src -to $dst -setup -end $(Slack)
#end -- Only generate constraits with -though if the current path is not critical.
#for i, n in pairs(RTLDatapath.Nodes) do
#  if (i~=1 and i~= table.getn(RTLDatapath.Nodes)) then
#    local ThuName = n.Name
#    if Functions[FuncInfo.Name] == nil then
#      ThuName = '*' .. CurModule:getName() .. '_inst|' .. ThuName
#    elseif FuncInfo.Name == 'main' then
#      ThuName = '*' .. Functions[FuncInfo.Name].ModName .. '_inst|' .. ThuName
#    end
$(_put('#')) $(DstName) <- $(ThuName) <- $(SrcName) Slack $(Slack)
#    if (RTLDatapath.isCriticalPath ~= 1) then
  set thu [get_nets $(ThuName)*]
  if {[get_collection_size $thu]} {
    set_multicycle_path -from $src -through $thu -to $dst -setup -end $(Slack)
  }
#    end
#  end -- End valid thu node.
#end -- for
}
]=]

AlteraSDCHeader = [=[
create_clock -name "clk" -period $(PERIOD)ns [get_ports {clk}]
derive_pll_clocks -create_base_clocks
set_multicycle_path -from [get_clocks {clk}] -to [get_clocks {clk}] -hold -end 1
]=]

XilinxRunOnDatapath = [=[
#local Slack = RTLDatapath.Slack
#local DstName = RTLDatapath.Nodes[1].Name
#local SrcName = RTLDatapath.Nodes[table.getn(RTLDatapath.Nodes)].Name
#if Functions[FuncInfo.Name] == nil then
#  DstName = '*' .. CurModule:getName() .. '_inst/' .. DstName
#  SrcName = '*' .. CurModule:getName() .. '_inst/' .. SrcName
#elseif FuncInfo.Name == 'main' then
#  DstName = '*' .. Functions[FuncInfo.Name].ModName .. '_inst/' .. DstName
#  SrcName = '*' .. Functions[FuncInfo.Name].ModName .. '_inst/' .. SrcName
#end

INST "$(SrcName)*" TNM = "SRC";
INST "$(DstName)*" TNM = "DST";
#if (RTLDatapath.isCriticalPath == 1) then
$(_put('#')) $(DstName) <- $(SrcName) Slack $(Slack)
TIMESPEC "TS_multi" = FROM "SRC" TO "DST" TS_sys_clk*$(Slack);
#end -- Only generate constraits with -though if the current path is not critical.
#for i, n in pairs(RTLDatapath.Nodes) do
#  if (i~=1 and i~= table.getn(RTLDatapath.Nodes)) then
#    local ThuName = n.Name
#    if Functions[FuncInfo.Name] == nil then
#      ThuName = '*' .. CurModule:getName() .. '_inst/' .. ThuName
#    elseif FuncInfo.Name == 'main' then
#      ThuName = '*' .. Functions[FuncInfo.Name].ModName .. '_inst/' .. ThuName
#    end
$(_put('#')) $(DstName) <- $(ThuName) <- $(SrcName) Slack $(Slack)
#    if (RTLDatapath.isCriticalPath ~= 1) then
NET "$(ThuName)*" TPTHRU = "thuNode"; 
TIMESPEC "TS_mypath" = FROM "SRC" THRU "thuNode" TO "DST" TS_sys_clk*$(Slack);
#    end
#  end -- End valid thu node.
#end -- for
]=]

XilinxSDCHeader = [=[
NET "clk" TNM_NET = "sys_clk";
TIMESPEC "TS_sys_clk" = PERIOD "sys_clk" $(PERIOD) ns HIGH 50%;
]=]

Misc.DatapathScript = [=[
local SlackFileA = assert(io.open (MainSDCOutputA, "a+"))
local SlackFileX = assert(io.open (MainSDCOutputX, "a+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=AlteraRunOnDatapath, output=SlackFileA}
local _, message = preprocess {input=XilinxRunOnDatapath, output=SlackFileX}
if message ~= nil then print(message) end
SlackFileA:close()
SlackFileX:close()
]=]

Misc.TimingConstraintsHeaderScript = [=[
local SlackFileA = assert(io.open (MainSDCOutputA, "w"))
local SlackFileX = assert(io.open (MainSDCOutputX, "w"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=AlteraSDCHeader, output=SlackFileA}
local _, message = preprocess {input=XilinxSDCHeader, output=SlackFileX}
if message ~= nil then print(message) end
SlackFileA:close()
SlackFileX:close()
]=]

SynAttr.ParallelCaseAttr = '/* parallel_case */'
SynAttr.FullCaseAttr = '/* full_case */'
