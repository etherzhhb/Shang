#!/usr/bin/env lua
--
-- lua2c.lua: Embedder of Lua scripts into C programs.
--
-- Copyright (C)2010 Valeriu Paloş. All rights reserved!
--
-- This program is an adapted version of the bin2c.lua script written
-- by Mark Edgar. It is licensed the same as Lua, i.e. MIT license. A
-- worthy mention to Leg (http://leg.luaforge.net) which provided the
-- better heredoc string parsing grammar.
--
-- Dependencies: lua(5.1), and libraries: 'lpeg', 'bit'(bitlib/bitop)
--

-- Requirements.
require'bit'
require'lpeg'

-- Aliases.
local P, R, V, S, C, Cs, Cb, Cg, Cmt =
      lpeg.P, lpeg.R, lpeg.V, lpeg.S,
      lpeg.C, lpeg.Cs, lpeg.Cb, lpeg.Cg, lpeg.Cmt

-- Manual.
local description = [[
Usage: lua lua2c.lua <options> filename.lua

Options:
  -h shows a brief informative text on this utility

  -c embeds the source as precompiled bytecode for maximizing loading
     speeds (although the difference is small); note that LuaJIT does
     not load Lua bytecode so it won't work with this option!

  -s strips comments and whitespace from the original code before the
     C code generation to reduce the size of the generated file. This
     option works with LuaJIT.

  -e applies a bitwise XOR on the embedded data to prevent the source
     from being directly visible inside the executable; this is *not*
     a secure encryption method, only a mild obfuscation of the bytes
     that would end up (as plain text) in the binary.

  -u generates an unprotected (void) function which does not return a
     result status, but instead terminates the program on error. This
     behaviour is desirable in many cases when it is unacceptable for
     an embedded file not to load successfully.

Compiles a Lua source into a C header file which can be included into
an existing C program via the '#include' directive. This file defines
a function named 'load_<name>_lua()', where <name> is, basically, the
lowercase name of the given file (without directory) with all the non
alphanumeric characters replaced by underscores. The resulting source
code is as follows:

    int load_<name>_lua(lua_State* L) {
        /* ... */
    }

Calling this function in C has the same effect as loading and running
the original Lua file in-place and on the given Lua state. The result
is returned by the function as a signed integer. Note that the output
produced by this tool is *incompatible* with the original generators!

http://valeriu.palos.ro/669/lua-to-cee/

Copyright (C)2010 Valeriu Paloş. All rights reserved.
Licensed under the same terms as Lua (i.e. MIT license).
]]

-- Arguments.
local source
local do_help
local do_compile
local do_minify
local do_encrypt
local do_unsafe
for i, v in ipairs(arg or {}) do
    if v:find('^-.+$') then
        do_help     = v:find('h', 1, true)
        do_compile  = v:find('c', 1, true)
        do_minify   = v:find('s', 1, true)
        do_encrypt  = v:find('e', 1, true)
        do_unsafe   = v:find('u', 1, true)
    else
        source = arg[i]
    end
end
do_help = do_help or not source

-- Help.
if do_help then
    io.stderr:write(description)
    return
end

-- Read file.
local identity = source:lower():gsub('^.+/', ''):gsub('[^%w_]', '_')
local file     = assert(io.open(source, 'rb'))
local content  = file:read'*a'
local length   = content:len()
file:close()

-- Minify.
if do_minify then
    local grammar = P{ 'entry';
        entry     = Cs((V'space'/'' + V'comments'/'' +
                        V'strings' + V'words' + 1)^0),

        space     = S' \n\r\t\f',
        heredoc   = #(P'[' * P'='^0 * P'[') * function (text, start)
            local level = assert(text:match('^%[(=*)%[', start))
            level = assert(text:find(']'..level..']', start, true),
                           'unclosed long brackets')
            return level + 2
        end,
        string    = ('"' * ((P'\\\\' + P'\\"' + 1) - P'"')^0 * '"') +
                    ("'" * ((P'\\\\' + P"\\'" + 1) - P"'")^0 * "'"),
        strings   = V'heredoc' + V'string',
        comments  = P'--' * V'heredoc' +
                    P'--' * (1 - P'\n')^0 * (P'\n'^0 + -1),
        word      = R('az', 'AZ', '09', '__')^1,
        words     = V'word' *
                    ((V'space' + V'comments')^1/'' * (V'word'/' %0'))^1
    }

    content = lpeg.match(grammar, content)
    length  = content:len()
end

-- Compile.
if do_compile then
    content = string.dump(assert(loadstring(content, source)))
    length  = content:len()
end

-- Prepare.
math.randomseed(os.time())
local seed = math.random(255)
local step = math.random(254)

-- Assemble bytes.
local mask = seed
local grammar = P{ 'entry';
    entry     = Cs(V'character'^0),
    character = P(1) / function (c)
        local byte = string.byte(c)

        if do_encrypt then
            byte = bit.bxor(byte, mask)
            mask = (mask + step) % 256
        end

        return ('%3d,'):format(byte)
    end
}
content = lpeg.match(grammar, content)
content = content:gsub(('.'):rep(68), '%0\n        ')

-- Assemble main code.
local output = string.format([[
/*
 * LUA SCRIPT %q EMBEDDED AS A C %s.
 *
 * C SOURCE CODE AUTO-GENERATED FROM THE LUA FILE. DON'T EDIT!
 * INCLUDE THIS HEADER FILE USING THE '#include' DIRECTIVE AND
 * THEN CALL THE DEFINED FUNCTION GIVING IT A VALID LUA STATE!
 */

#ifndef __lua2c_%s__
#define __lua2c_%s__

#include <lua.h>
#include <lauxlib.h>

/*
 * Load the file %q inside the given Lua state.
 * Returns zero on success and non-zero on errors.
 */
%s load_%s(lua_State* L) {
    static unsigned char content[%u] = {
        %s
    };

]]..(do_encrypt and string.format([[
    static int i = 0;
    int mask = %u, step = %u;
    while (i < %u) {
        content[i++] ^= mask %%%% 256;
        mask += step;
    }

]], seed, step, length) or '')..[[
    int result = luaL_loadbuffer(L, (const char*)content,
                                 %u, %q);
    if (!result) {
        result = lua_pcall(L, 0, 0, 0);
    }

]]..(do_unsafe and [[
    if (result) {
        lua_error(L);
    }
]] or [[
    return result;
]])..[[
}

#endif
]], source, do_unsafe and 'PROCEDURE' or 'FUNCTION',
    identity, identity, source, do_unsafe and 'void' or 'int',
    identity, length, content, length, source)

-- Submit.
io.write(output)
