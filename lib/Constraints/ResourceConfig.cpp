/*
* Copyright: 2008 by Nadav Rotem. all rights reserved.
* IMPORTANT: This software is supplied to you by Nadav Rotem in consideration
* of your agreement to the following terms, and your use, installation, 
* modification or redistribution of this software constitutes acceptance
* of these terms.  If you do not agree with these terms, please do not use, 
* install, modify or redistribute this software. You may not redistribute, 
* install copy or modify this software without written permission from 
* Nadav Rotem. 
*/

#include "vbe/ResourceConfig.h"

using namespace xVerilog;

namespace {
struct UnitNumParser : public cl::basic_parser<unsigned> {
// parse - Return true on error.
  bool parse(cl::Option &O, StringRef ArgName,
            const std::string &ArgValue, unsigned &Val);
};
}

typedef cl::opt<unsigned, false, UnitNumParser> UnitNumParserOption;

// the parser method for our integer parser 
bool UnitNumParser::parse(cl::Option &O, StringRef ArgName,
        const std::string &Arg, unsigned &Val) {
    const char *ArgStart = Arg.c_str();
    char *End;
    // Parse integer part, leaving 'End' pointing to the first non-integer char
    Val = (unsigned)strtol(ArgStart, &End, 0);
    //cerr<<"Parsing "<<ArgName<<" as "<<Val<<"\n";
    return false; // no error    
}


// TODO: Read these by xml reader
static UnitNumParserOption 
param_mem_num("units_memport",
             cl::desc("vbe - the number of memory ports"),
             cl::value_desc("num"));

static UnitNumParserOption 
param_mul_num("units_mul",
             cl::desc("vbe - number of multiply units"),
             cl::value_desc("num"));

static UnitNumParserOption
param_div_num("units_div",
             cl::desc("vbe - number of division units"),
             cl::value_desc("num"));

static UnitNumParserOption
param_shl_num("units_shl", 
             cl::desc("vbe - number of shift units"), 
             cl::value_desc("num"));

static UnitNumParserOption
delay_mem_num("delay_memport", 
             cl::desc("vbe - delay cycles of memory ports"), 
             cl::value_desc("num"));

static UnitNumParserOption
mem_wordsize("mem_wordsize", 
            cl::desc("vbe - the word size of the memory port (64bits to 8bits) "), 
            cl::value_desc("num"));

static UnitNumParserOption
membus_size("membus_size", 
           cl::desc("vbe - the size of pointers (64bits to 8bits) "),
           cl::value_desc("num"));

static UnitNumParserOption
delay_mul_num("delay_mul", 
             cl::desc("vbe - delay cycles of multiply units"),
             cl::value_desc("num"));

static UnitNumParserOption
delay_div_num("delay_div",
             cl::desc("vbe - delay cycles of division units"),
             cl::value_desc("num"));

static UnitNumParserOption
delay_shl_num("delay_shl",
             cl::desc("vbe - delay cycles of shift units"),
             cl::value_desc("num"));

static UnitNumParserOption
inline_wire("inline_op_to_wire",
           cl::desc("vbe - inline operations smaller then this bit number "
                    "to operations which are done in the same cycle as wires"),
           cl::value_desc("num"));

//TODO:
//Change these parameters to a true/false parameters rather
//then numbers (zero or one)
static UnitNumParserOption
include_freq("include_freq",
             cl::desc("include frequency when considering the design score"),
             cl::value_desc("num"));

static UnitNumParserOption 
include_size("include_size",
             cl::desc("include size when considering the design score"), 
             cl::value_desc("num"));

static UnitNumParserOption 
include_clocks("include_clocks",
               cl::desc("include clocks when considering the design score"),
               cl::value_desc("num"));

void ResourceConfig::initializePass() {
  // Get target data

  ResTab["memport"] = param_mem_num;
  ResTab["mul"] = param_mul_num;
  ResTab["div"] = param_div_num;
  ResTab["shl"] = param_shl_num;

  ResTab["mem_wordsize"] = mem_wordsize;
  ResTab["delay_memport"] = delay_mem_num;
  ResTab["delay_mul"] = delay_mul_num;
  ResTab["delay_div"] = delay_div_num;
  ResTab["delay_shl"] = delay_shl_num;
  ResTab["membus_size"] = membus_size;

  ResTab["inline_op_to_wire"] = inline_wire;
  ResTab["include_size"] = include_size;
  ResTab["include_freq"] = include_freq;
  ResTab["include_clocks"] = include_clocks;
}

char ResourceConfig::ID = 0;

ResourceConfig::ResTabTy ResourceConfig::ResTab;

static RegisterPass<ResourceConfig>
X("vbe-resource-config", "vbe - resource config", false, true);
