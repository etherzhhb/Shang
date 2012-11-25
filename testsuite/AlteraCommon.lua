FUs.BRam.Prefix = [=[altsyncram:]=]

FUs.BRam.Template=[=[
// Block Ram $(num)
reg  [$(datawidth - 1):0]  bram$(num)arrayout;
(* ramstyle = "no_rw_check" *) reg  [$(datawidth - 1):0]  bram$(num)array[0:$(size - 1)];

#if filename ~= [[]] then
initial
    $(_put('$'))readmemh("$(filepath .. '/' .. filename)", bram$(num)array);
#end
]=]

RunOnDatapath = [=[
#local Slack = RTLDatapath.Slack
#local DstNameSet = RTLDatapath.Nodes[1].NameSet
#local SrcNameSet = RTLDatapath.Nodes[table.getn(RTLDatapath.Nodes)].NameSet

foreach DstPattern $(DstNameSet) {
#if Functions[FuncInfo.Name] == nil then
  set dst [get_keepers "*$(CurModule:getName())_inst|$DstPattern*"]
#elseif FuncInfo.Name == 'main' then
  set dst [get_keepers "*$(Functions[FuncInfo.Name].ModName)_inst|$DstPattern*"]
#end
  if { [get_collection_size $dst] } { break }
}

foreach SrcPattern $(SrcNameSet) {
#if Functions[FuncInfo.Name] == nil then
  set src [get_keepers "*$(CurModule:getName())_inst|$SrcPattern*"]
#elseif FuncInfo.Name == 'main' then
  set src [get_keepers "*$(Functions[FuncInfo.Name].ModName)_inst|$SrcPattern*"]
#end
  if { [get_collection_size $src] } { break }
}

if {[get_collection_size $src] && [get_collection_size $dst]} {
#if (RTLDatapath.isCriticalPath == 1) then
$(_put('#')) $(DstNameSet) <- $(SrcNameSet) Slack $(Slack)
  set_multicycle_path -from $src -to $dst -setup -end $(Slack)
#end -- Only generate constraits with -though if the current path is not critical.
#for i, n in pairs(RTLDatapath.Nodes) do
#  if (i~=1 and i~= table.getn(RTLDatapath.Nodes)) then
#local ThuNameSet = n.NameSet
$(_put('#')) $(DstNameSet) <- $(ThuNameSet) <- $(SrcNameSet) Slack $(Slack)
#    if (RTLDatapath.isCriticalPath ~= 1) then
  foreach ThuPattern $(ThuNameSet) {
#if Functions[FuncInfo.Name] == nil then
    set thu [get_keepers "*$(CurModule:getName())_inst|$ThuPattern*"]
#elseif FuncInfo.Name == 'main' then
    set thu [get_keepers "*$(Functions[FuncInfo.Name].ModName)_inst|$ThuPattern*"]
#end
    if { [get_collection_size $thu] } { break }
  }

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
derive_clock_uncertainty
set_multicycle_path -from [get_clocks {clk}] -to [get_clocks {clk}] -hold -end 0
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
