--Return type with given size.
function getType(Size)
  if Size == 0 then        return "void"
  elseif Size == 1 then    return "bool"
  elseif Size == 8 then    return "unsigned char"
  elseif Size == 16 then  return "unsigned short"
  elseif Size == 32 then  return "unsigned int"
  elseif Size == 64 then  return "unsigned long long"
  else                    return "bad-type"
  end
end

--Escape numbers for Verilator DPI-C functions.
function number2Alpha(w)
  if w == '_' then
    return 's'
  else
    return string.char(string.byte(w) + 17)
  end
end

function escapeNumber(s)
  return string.gsub(s, "([%d\_])", number2Alpha)
end

--Decide the bit widths of return value
function getRetPort(Size)
  if Size == 0 then        return " "
  elseif Size == 64 then  return "sc_signal<uint64_t>return_value"
  else                    return "sc_signal<uint32_t>return_value"
  end
end

--Giving the arguments type in SystemC
function getBitWidth(Size)
  if Size == 1 then        return "bool "
  elseif Size == 8 then    return "unsigned int"
  elseif Size == 16 then  return "unsigned int"
  elseif Size == 32 then  return "uint32_t"
  elseif Size == 64 then  return "uint64_t"
  else                    return "uint32_t"
  end
end
