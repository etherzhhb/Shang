ModelsimGenTemplate = [=[
$('#')!/bin/sh
PATH=~/altera/10.1/modelsim_ase/bin/:~/altera/modelsim_ase/bin/:$PATH
vlib work
vlog +define+quartus_synthesis -sv $(RTLModuleName).v
vlog -sv INTF_$(RTLModuleName).v
vlog -sv DUT_TOP_tb.sv
vlog -sv BRAM.sv
vsim -t 1ps work.DUT_TOP_tb -c -do "run -all;vcd flush;quit -f"

]=]

Passes.ModelsimGen = { FunctionScript = [=[
if Functions[FuncInfo.Name] ~= nil then
end
]=], GlobalScript =[=[
local ModelDoFile = assert(io.open (MODELDOFILE, "w+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=ModelsimGenTemplate, output=ModelDoFile}
if message ~= nil then print(message) end
ModelDoFile:close()
]=]}
