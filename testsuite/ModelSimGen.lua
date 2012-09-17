ModelsimGenTemplate = [=[
$('#')!/bin/bash
PATH=~/altera/10.1/modelsim_ase/bin/:~/altera/modelsim_ase/bin/:$PATH
export CYCLONEII_SIM=$QUARTUS_ROOT/eda/sim_lib/cycloneii_atoms.v
cd ./simulation/modelsim
vdel -lib work *
vlib work
vlog -work work $CYCLONEII_SIM
vlog DUT_TOP.vo
vlog -sv ../../DUT_TOP_tb.sv
vsim -t 1ps -L cycloneii_ver work.DUT_TOP_tb -c -do "do DUT_TOP_dump_all_vcd_nodes.tcl;run -all;vcd flush;quit -f"

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
