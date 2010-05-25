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

#include "vbe/params.h"

namespace xVerilog {

 // the parser method for our integer parser 
    bool UnitNumParser::parse(cl::Option &O, const char *ArgName,
            const std::string &Arg, unsigned &Val) {
        const char *ArgStart = Arg.c_str();
        char *End;
        // Parse integer part, leaving 'End' pointing to the first non-integer char
        Val = (unsigned)strtol(ArgStart, &End, 0);
        //cerr<<"Parsing "<<ArgName<<" as "<<Val<<"\n";
        return false; // no error    
    }

    UnitNumParserOption 
        machineResourceConfig::param_mem_num("units_memport", cl::desc(" the number of memory ports"), cl::value_desc("num"));
    UnitNumParserOption 
        machineResourceConfig::param_mul_num("units_mul", cl::desc("number of multiply units"), cl::value_desc("num"));
    UnitNumParserOption  
        machineResourceConfig::param_div_num("units_div", cl::desc("number of division units"), cl::value_desc("num"));
    UnitNumParserOption 
        machineResourceConfig::param_shl_num("units_shl", cl::desc("number of shift units"), cl::value_desc("num"));

    UnitNumParserOption 
        machineResourceConfig::delay_mem_num("delay_memport", cl::desc("delay cycles of memory ports"), cl::value_desc("num"));

    UnitNumParserOption 
        machineResourceConfig::mem_wordsize("mem_wordsize", cl::desc("the word size of the memory port  (64bits to 8bits) "), cl::value_desc("num"));
    
    UnitNumParserOption 
        machineResourceConfig::membus_size("membus_size", cl::desc("the size of pointers (64bits to 8bits) "), cl::value_desc("num"));

    UnitNumParserOption 
        machineResourceConfig::delay_mul_num("delay_mul", cl::desc("delay cycles of multiply units"), cl::value_desc("num"));
    UnitNumParserOption  
        machineResourceConfig::delay_div_num("delay_div", cl::desc("delay cycles of division units"), cl::value_desc("num"));
    UnitNumParserOption 
        machineResourceConfig::delay_shl_num("delay_shl", cl::desc("delay cycles of shift units"), cl::value_desc("num"));

    UnitNumParserOption 
        machineResourceConfig::inline_wire("inline_op_to_wire", cl::desc("inline operations smaller then this bit number to operations which are done in the same cycle as wires"), cl::value_desc("num"));

    //TODO:
    //Change these parameters to a true/false parameters rather
    //then numbers (zero or one)
    UnitNumParserOption machineResourceConfig::include_freq("include_freq", cl::desc("include frequency when considering the design score"), cl::value_desc("num"));

    UnitNumParserOption machineResourceConfig::include_size("include_size", cl::desc("include size when considering the design score"), cl::value_desc("num"));

    UnitNumParserOption machineResourceConfig::include_clocks("include_clocks", cl::desc("include clocks when considering the design score"), cl::value_desc("num"));

    map<string, unsigned int> machineResourceConfig::getResourceTable() {
        map<string,unsigned int> myMap;
        myMap["memport"] = param_mem_num;
        myMap["mul"] = param_mul_num;
        myMap["div"] = param_div_num;
        myMap["shl"] = param_shl_num;

        myMap["mem_wordsize"] = mem_wordsize;
        myMap["delay_memport"] = delay_mem_num;
        myMap["delay_mul"] = delay_mul_num;
        myMap["delay_div"] = delay_div_num;
        myMap["delay_shl"] = delay_shl_num;
        myMap["membus_size"] = membus_size;

        myMap["inline_op_to_wire"] = inline_wire;
        myMap["include_size"] = include_size;
        myMap["include_freq"] = include_freq;
        myMap["include_clocks"] = include_clocks;
        return myMap;
    }

} // namespace
