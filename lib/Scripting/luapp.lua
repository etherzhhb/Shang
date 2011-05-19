-- luapp.lua

module("luapp", package.seeall)

-- Count number of chars c in string s.
local function countchar(s, c)
  local count = 0
  local i = 1
  while true do
    i = string.find(s, c, i)
    if i then count = count + 1; i = i + 1 else break end
  end
  return count
end

-- In error message string, translate line numbers from
-- processed file to source file.
-- linenums is translation array (processed line number ->
-- source line number) or source line number.
local function fix_linenums(message, linenums)
  message = message:gsub("(%b[]:)(%d+)", function(a,n)
    n = tonumber(n)
    local source_linenum =
      type(linenums) == "table" and (linenums[n] or '?') or
      type(linenums) == "number" and linenums + n - 1 or
      '?'
    return a .. source_linenum
  end)
  return message
end


-- Expands $(...) syntax.
local function parse_dollar_paren(pieces, chunk, name, linenum)
  local is = 1
  for ibegin, iend in chunk:gmatch("()$%b()()") do
    local text = chunk:sub(is, ibegin - 1)
    local executed = chunk:sub(ibegin+2, iend-2) -- remove parens

    local name2 = name .. ":" .. executed
    linenum = linenum + countchar(text, '\n')
    local may_have_comment = executed:find("%-%-")
    local nl = may_have_comment and "\n" or ""

    pieces[#pieces+1] = ("_put(%q)"):format(text)
    if loadstring("return " .. executed, name2) then -- is expression list
      pieces[#pieces+1] = "_put(" .. executed .. nl .. ")"
    else -- assume chunk
      local status, message = loadstring(executed, name2)
      if not status then -- unrecognized
        if message then
          message = fix_linenums(message, linenum)
        end
        return status, message
      end
      pieces[#pieces+1] = " " .. executed .. nl .. " "
      linenum = linenum + countchar(executed, '\n')
    end
    is = iend
  end
  pieces[#pieces+1] = ("_put(%q)"):format(chunk:sub(is))
  return true
end

-- Expands #... syntax.
local function parse_hash_lines(chunk, name)
  local pieces = {}

  local luas = {} -- for improved error reporting
  local linenums = {}
  local linenum = 1

  pieces[#pieces+1] = "local _put = ... "

  local is = 1
  while true do
    local _, ie, lua = chunk:find("^#+([^\n]*\n?)", is)
    if not ie then
      local iss; iss, ie, lua = chunk:find("\n#+([^\n]*\n?)", is)
      local text = chunk:sub(is, iss)
      local status, message = parse_dollar_paren(pieces, text, name, linenum)
      if not status then return status, message end
      if not ie then break end
      linenum = linenum + countchar(text, '\n')
    end

    luas[#luas+1] = lua
    linenums[#linenums+1] = linenum
    linenum = linenum + 1

    pieces[#pieces+1] = ' ' .. lua .. ' '

    is = ie + 1
  end

  local code = table.concat(pieces, ' ')

  -- Attempt to compile.
  local f, message = loadstring(code, name)
  if not f then
    -- Attempt to compile only user-written Lua
    -- (for cleaner error message)
    local lua = table.concat(luas)
    local f2, message2 = loadstring(lua, name)
    if not f2 then
      message = fix_linenums(message2, linenums)
    else -- unexpected
      message = fix_linenums(message, nil)
    end
  end

  return f, message
end

-- Abstraction of string output stream.
local function string_writer()
  local t = {}
  local function write(...)
    local n = select('#', ...)
    if n > 0 then
      t[#t+1] = tostring((...))
      write(select(2, ...))
    end
  end
  local function close()
    return table.concat(t)
  end
  return {write=write, close=close}
end

-- Abstraction of file output stream.
local function file_writer(fh, is_close)
  local function write(...)
    local n = select('#', ...)
    if n > 0 then
      fh:write(tostring((...)))
      write(select(2, ...))
    end
  end
  local function close()
    if is_close then fh:close() end
  end
  return {write=write, close=close}
end

-- Convert output specification to output stream.
-- A helper function for C<preprocess>.
local function make_output(output)
  if type(output) == 'string' then
    output = string_writer()
  elseif type(output) == 'table' then
    assert(#output == 1, 'table size must be 1')
    local filename = output[1]
    local fh, message = io.open(filename, 'w')
    if not fh then return false, message end
    output = file_writer(fh, true)
  elseif io.type(output) == 'file' then
    output = file_writer(output, false)
  else
    error('unrecognized', 2)
  end
  return output
end

-- Convert input specification to input stream.
-- A helper function for C<preprocess>.
local function make_input(input)
  if type(input) == 'string' then
    input = {text = input, name = 'source'}
  elseif type(input) == 'table' then
    assert(#input == 1, 'table size must be 1')
    local filename = input[1]
    local fh, message = io.open(filename)
    if not fh then return false, message end
    input = {text = fh:read'*a', name = filename}
    fh:close()
  elseif io.type(input) == 'file' then
    input = {text = input:read'*a', name = nil}
  else
    error('unrecognized', 2)
  end
  return input
end

function preprocess(t)
  if type(t) == 'string' then t = {input = t} end
  local input = t.input or io.stdin
  local output = t.output or
             (type(input) == 'string' and 'string') or io.stdout
  local lookup = t.lookup or _G
  local strict = t.strict; if strict == nil then strict = true end

  input = make_input(input)
  local name = input.name or "<source>"

  local f, message = parse_hash_lines(input.text, name)
  if not f then return f, message end

  local mt = {}
  if strict then
    function mt.__index(t,k)
      local v = lookup[k]
      if v == nil then
        error("Undefined global variable " .. tostring(k), 2)
      end
      return v
    end
  else
    mt.__index = lookup
  end

  local env = {}
  setmetatable(env, mt)
  setfenv(f, env)


  output = make_output(output)

  local status, message = pcall(f, output.write)

  local result = output.close()
  if not result then result = true end

  if not status then
    return false, message
  else
    return result
  end
end



local function command(...)
  local t = {...}

  if t[1] == '-t' then
    os.exit(M.testsuite() and 0 or 1)
  elseif t[1] == '-d' then
    print(M.DOC)
    return
  elseif t[1] == '-v' then
    print(M.VERSION)
    return
  end

  local input, output
  local i=1; while i <= #t do
    if t[i] == '-e' then
      i = i + 1
      input = assert(t[i])
    elseif t[i] == '-' and not input then
      input = io.stdin
    elseif t[i] == '-' and not output then
      output = io.stdout
    elseif not input then
      input = {t[i]}
    elseif not output then
      output = {t[i]}
    else
      error("unrecognized command-line arg " .. tostring(t[i]))
    end
    i = i + 1
  end
  if not input then
    io.stderr:write(
      "usage: luapp [options] [input] [output]\n\n" ..
      "  -e string  input as command-line expression\n" ..
      "  -c command special command ('test' or 'doc')\n" ..
      "  -d         print full documentation\n" ..
      "  -t         run test suite\n" ..
      "  -v         print version\n")
    os.exit(1)
  end
  output = output or io.stdout
  local status, message = M.preprocess{input=input, output=output, lookup=_G}
  if not status then
    io.stderr:write(message .. "\n")
    os.exit(1)
  end
end


-- TEST SUITE
function testsuite()

  local preprocess = require "luapp" . preprocess

  local check = {}
  check['='] = function(a, b, message)
    message = message or ''
    if not(a == b) then
      error(string.format('FAIL: [%s] == [%s] %s',
        tostring(a), tostring(b), message), 2)
    end
  end
  function check.fail(f)
    if pcall(f) then
      error(string.format('FAIL: did not raise'), 2)
    end
  end
  function check.pass(f)
    local status, message = pcall(f)
    if not status then
      error(string.format('FAIL: raised ' .. message), 2)
    end
  end

  check['='](preprocess'', '')
  check['='](preprocess'$', '$')
  check['='](preprocess'$("$")', '$')
  check['='](preprocess'$("$")(', '$(')
  check['='](preprocess' $ $ $ ', ' $ $ $ ')
  check['='](preprocess'$()', '')
  check['='](preprocess'$(\n)', '')
  check['='](preprocess'$(false)', 'false')
  check['='](preprocess'$(nil)', 'nil')
  check['='](preprocess'$(1,2)', '12')
  check['='](preprocess'$(_put(1,2))', '12')
  --check.fail(function() preprocess'$(' end)
  --check.fail(function() preprocess'$(()' end)

  check['='](preprocess'$(1+2)', '3')
  check['='](preprocess'$((1+2)*2)', '6')
  check['='](preprocess'a$(1)$(2)b$(3)c', 'a12b3c')

  check['='](preprocess'$(local x=2)$(x)$(local x=3)$(x)', '23')
  check['='](preprocess'$(for n=1,3 do _put(n) end)', '123')
  check['='](preprocess'$(local function test(x) return x+1 end)$(test(2))', '3')

  check['='](preprocess'$("$")', '$')

  check['='](preprocess'#', '')
  check['='](preprocess'#_put(2)', '2')
  check['='](preprocess'#x=2\n$(x)', '2')
  check['='](preprocess'#for x=1,2 do\n$(x)\n#end', '1\n2\n')
  check['='](preprocess'$("#")', '#')

  local t = {a=5}
  check['=']('5', preprocess {input='$(a)', lookup=t})
  check['=']('nil', preprocess {input='$(b)', lookup=t, strict=false})
  check.fail(function() assert(preprocess {input='$(b)', lookup=t}) end)



  -- preprocess {input = {'input.txt'}, output = io.stdout, lookup = _G}

  check['='](preprocess[[$(local x=5)$("$(x)")]], '$(x)')

  check['=']([[
testfalsenil16
  1
  2
  3
123
10
nil4
k=1
k=2
6
]],preprocess[[
test$(false)$(nil)$(1)$(local y=6)$(y)
#for n=1,3 do
  $(n)
#end
$(for n=1,3 do _put(n) end)
#function make(n)
#  for k=1,n do
k=$(k)
#  end
#end
#local function inc(n) return n+1 end
#local x
#do local x=10
$(x)
#end
$(x)$(local x = 4)$(x)
$(make(2))$(inc(5))
]])

  -- docs
  check['=']([[
x is now 1
y is now 1
y is now 2
x is now 2
y is now 1
y is now 2
x and y are now nil and nil
]], preprocess[[
#local x,y
#for x=1,2 do
x is now $(x)
#  for y=1,2 do
y is now $(y)
#  end
#end
x and y are now $(x) and $(y)
]])

  check['='](
  [[ASDF]],
  preprocess{input=[[$(
    local function ucase(s) return (s:gsub("%l", string.upper)) end
  )$(ucase("aSdF"))]], lookup=_G}
  )

  -- check line numbers in error messages
  local _,message = preprocess"$(x=1)$(x = =)"
  assert(message:find(":1:"))
  local _,message = preprocess"$(x=1 --)$(x = =)"
  assert(message:find(":1:"))
  local _,message = preprocess"$(x=1 --)\n$(x = =)"
  assert(message:find(":2:"))
  local _,message = preprocess"$(x=1 --)\n#x=2\n$(x = =)"
  assert(message:find(":3:"))
  local _,message = preprocess"$(x=1 --)$(\nx = =)"
  assert(message:find(":2:"))
  local _,message = preprocess"$(x=1 --)$(\nx = 3)\n#x= ="
  assert(message:find(":3:"))

  -- test of input/output methods
  -- should output "1+2=3" twice
  preprocess {input='1+2=$(1+2)\n', output=io.stdout}
  preprocess {input='1+2=$("$")(1+2)\n', output={'tmp.txt'}}
  preprocess {input={'tmp.txt'}, output=io.stdout}

  print 'done'

  return true
end

-- DOCUMENTATION
DOC = [=[

=NAME

Luapp - A preprocessor based on Lua.

=DESCRIPTION

This module is a simple macro preprocessor[1] implemented in Lua.

=DESIGN QUALITIES

This module has the following characteristics:

* This module is intended to be robust and fully tested.
* It is implemented entirely in Lua.
* For any string C<x> there exist at least one C<y> such that
  C<preprocess(y) == x>.
* The syntax is quite simple and unambiguous.
  There are two syntaxes available for embedding Lua preprocessor
  code in your text: $(...) or "#...".  The former resembles the "Makefile",
  M4, or Perl style.  The latter resembles the C preprocessor style.

    $(for x=1,3 do _put(x) end)

    #for x=1,3 do   -- not identical due to spacing differences
      $(x)
    #end

* The C<"#..."> style allows text to be nested (lexically) in Lua code
  to be nested to text to be nested in Lua code, etc.  For example:

    #for x=1,2 do
    x is now $(x)
    #  for y=1,2 do
    y is now $(y)
    #  end
    #end
    x and y are now $(x) and $(y)

  Outputs:

    x is now 1
    y is now 1
    y is now 2
    x is now 2
    y is now 1
    y is now 2
    x and y are now nil and nil

* The module will try to report an meangingful error if syntax is bad:
  C<$(if x then then)>.  However, there are probaby cases where it
  fails in this.
* It is possible to run the preprocessor on untrusted source.  Just set
  the lookup table to C<nil> or to a custom table.
* Currently, the processor loads the entire source into memory.  For
  very large files that exceed available RAM, this might not be
  suitable.
* Speed should be reasonably good, though probabily not optimal due to
  checks (it has not been performance tested).  There may be room for
  some optimization.

=SYNTAX

* C<$(chunk)> where I<chunk> is a chunk of Lua code will evalute the
  chunk output nothing.  I<chunk> must NOT call C<return> (not
  supported--should it be?)
* C<$(explist)> where I<explist> is a Lua expression list will
  evaluate the expression list and output each element of the
  expression list as a string (via C<tostring>). Note: if I<x> in
  C<$(x)> can be interpreted as both a chunk and an expression list,
  it is interpreted as an expression list.  This allows function
  calls: C<$(f())>.
* C<$('$')> allows a C<$> to be outputted literally. Example:
  C<$('$')(1+2)> outputs C<$(1+2)>. C<$('#')> allows a C<#> the be
  outputted literally in the first column. Example: C<$('#')if>
  outputs C<#if>.
* C<$(chunk)> may contain calls to the function C<_put>, which
  stringifies all its arguments and outputs them.  For example,
  C<$(_put(explist))> is the same as C<$(explist)>.  This can be
  useful for things like C<$(for n=1,10 do _put(n, ' ') end)>.
* C<$(x)> where I<x> is not a valid Lua expression or statement
  generates an error.
* Any line having C<'#'> in the first column is treated as Lua code.

  #if DEBUG
    Debug $(x).
  #else
    Release $(x).
  #end

=INTERFACE

==IMPORT

  local preprocess = require "luapp" . preprocess

==FUNCTION preprocess

  result, message = preprocess(t)
  where t = {input=input, output=output, lookup=lookup,
             strict=strict} or input

Preprocesses text.

* C<input> - input source.  This can be the text itself (as a string),
    a readable file handle, or a filename (an array with first element
    being the file name).  If omitted, this will be C<io.stdin>.
* C<output> - output destination.  This can be 'string' (the processed
    text is returned as a string in result), a writable file handle,
    or a filename (an array with the first element being the file
    name). If omitted, this will be 'string' (if input is a string) or
    io.stdout.
* C<lookup> - a lookup table used for retrieving the
    values of global variables referenced by the preprocessed file.
    Global writes in the preprocessed file are not written to this
    table.  If omitted, all global accesses will have the value
    C<nil>.  Often, this value is set to C<_G> (the global table).
* C<strict> - enable strict-like mode on global variables.
    Accessing global variables with value C<nil> triggers
    an error.  C<true> or C<false>.  Default C<true>.
* C<result> - the result.  The is normally the processed text (if
    output is set to 'string') or true.  On failure, this is set to
    false and message is set.
* C<message> - the error message string.  This is set only if result
    is C<false>.

==FIELD VERSION

  version = luapp.VERSION

==Command Line Usage

  lua luapp.lua [option] [input] [output]

Examples:

  cat in.txt | luapp.lua - > out.txt

  luapp.lua in.txt out.txt

  luapp.lua -e '$(1+2)'

Version number

=EXAMPLES

  $(local function ucase(s) return s:gsub("^%l", string.upper) end)
  $(ucase("aSdF"))       ($-- outputs "ASDF")

=HISTORY

0.3 - 2007-09-04
  preprocess - default lookup to _G
  preprocess - new "strict" argument.
  preprocess - remove undocumented #ARGS(...)
  preprocess - improved error reporting
  merged into single file.

0.1 - 2007-08-30
  initial version adapted from rici's code

=AUTHOR NOTES

This documentation is formatted in a loose POD[2] style.

=REFERENCES

[1] http://en.wikipedia.org/wiki/Preprocessor
[2] http://en.wikipedia.org/wiki/Plain_Old_Documentation

=COPYRIGHT/LICENSE

Licensed under the same terms as Lua itself--That is, the MIT license:

(c) 2007 David Manura.  Derived from previous
http://lua-users.org/wiki/SlightlyLessSimpleLuaPreprocessor (SLSLPP)
code by RiciLake, which in turn was loosely based on
http://lua-users.org/wiki/SimpleLuaPreprocessor .

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

]=]

-- this hack detects whether the module is run from the command-line.
-- also see http://lua-users.org/lists/lua-l/2007-02/msg00125.html
-- local is_run = not pcall(function() return getfenv(5) end)
-- if is_run then command(...) end

-- return M
