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
local BramInitFile = assert(io.open (BRAMINIT, "w+"))
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
