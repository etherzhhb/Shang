--Generate initialize file for block rams.
for k,v in pairs(GlobalVariables) do
  if v.AddressSpace ~= 0 then
    if v.Initializer ~= nil then
      InitFile = io.open (test_binary_root .. '/' .. k .. '_init.txt', 'w')
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
