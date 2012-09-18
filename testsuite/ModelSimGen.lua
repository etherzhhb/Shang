ModelsimGenTemplate = [=[
$('#')!/bin/bash
PATH=~/altera/10.1/modelsim_ase/bin/:~/altera/modelsim_ase/bin/:$PATH
vdel -lib work -all
vlib work
vlog +define+quartus_synthesis -sv $(RTLModuleName).v
vlog -sv INTF_$(RTLModuleName).v
sed -i -e 's/<half-period>/5/' DUT_TOP_tb.sv
vlog -sv DUT_TOP_tb.sv
vlog -sv BRAM.sv
vsim -t 1ps work.DUT_TOP_tb -c -do "run -all;quit -f"

]=]

ModelsimGenTemplatePostRoutedSim = [=[
$('#')!/bin/bash
PATH=~/altera/10.1/modelsim_ase/bin/:~/altera/modelsim_ase/bin/:$PATH
export CYCLONEII_SIM=$QUARTUS_ROOT/eda/sim_lib/cycloneii_atoms.v
cd ./simulation/modelsim
vdel -lib work -all
vlib work
vlog -work work $CYCLONEII_SIM
vlog DUT_TOP.vo
vlog -sv ../../DUT_TOP_tb.sv
vsim -t 1ps -L cycloneii_ver work.DUT_TOP_tb -c -do "do DUT_TOP_dump_all_vcd_nodes.tcl;run <run-time>ns;vcd flush;quit -f"

]=]

Passes.ModelsimGen = { FunctionScript = [=[
if Functions[FuncInfo.Name] ~= nil then
end
]=], GlobalScript =[=[
local preprocess = require "luapp" . preprocess

local ModelDoFile = assert(io.open (MODELDOFILE, "w+"))

local _, message = preprocess {input=ModelsimGenTemplate, output=ModelDoFile}
if message ~= nil then print(message) end
ModelDoFile:close()

ModelDoFile = assert(io.open (MODELDOFILE .. "_post_routed", "w+"))
local _, post_routed_message = preprocess {input=ModelsimGenTemplatePostRoutedSim, output=ModelDoFile}
if post_routed_message ~= nil then print(post_routed_message) end
ModelDoFile:close()
]=]}
