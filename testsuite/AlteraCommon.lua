FUs.BRam.Prefix = [=[altsyncram:]=]

FUs.BRam.Template=[=[
// Block Ram $(num)
reg  [$(datawidth - 1):0]  bram$(num)arrayout;
(* ramstyle = "M4K, no_rw_check" *) reg  [$(datawidth - 1):0]  bram$(num)array[0:$(size - 1)];

#if filename ~= [[]] then
initial
    $(_put('$'))readmemh("$(FUs.BRam.InitFileDir .. '/' .. filename)", bram$(num)array);
#end
]=]

RunOnDatapath = [=[
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
