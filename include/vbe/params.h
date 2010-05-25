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

#ifndef LLVM_PARAMS_H
#define LLVM_PARAMS_H

#include "llvm/Pass.h"
#include "llvm/Function.h"
#include "llvm/Instructions.h"
#include "llvm/Constants.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/Streams.h"
#include "llvm/Module.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/CFG.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Support/CommandLine.h"

#include <map>
#include <algorithm>
#include <sstream>


using std::map;
using std::string;
using std::stringstream;

using namespace llvm;

namespace xVerilog {

    // define the integer parser
    struct UnitNumParser : public cl::basic_parser<unsigned> {
        // parse - Return true on error.
        bool parse(cl::Option &O, const char *ArgName, const std::string &ArgValue,
                unsigned &Val);
    };
   
    typedef cl::opt<unsigned, false, UnitNumParser> UnitNumParserOption;

    class machineResourceConfig {
        public:
            /*
             * Load all of the values into a structure which will be used by the scheduler
             * to build the hardware description table.
             */
            static map<string, unsigned int> getResourceTable();
    	    static string  chrsubst(string str , int ch, int ch2) { //JAWAD
		char *s1   = new char [str.size()+1];  
		strcpy (s1, str.c_str());
		int count = 0; /* The count to return */
		char *wrk = strchr(s1, ch); /* Find first char in s1 */
		while (wrk) { /* While we have matches */
			*wrk = (char) ch2; /* Replace the character */
			count++, wrk++; /* Increment the count & pointer */
			wrk = strchr(wrk, ch); /* Search for next occurance */
		}
		//return count; /* Return the count */
 		return  string(s1);
            }   
        private:
            static UnitNumParserOption param_mem_num; 
            static UnitNumParserOption param_mul_num;
            static UnitNumParserOption param_div_num;
            static UnitNumParserOption param_shl_num;
            static UnitNumParserOption delay_mem_num;
            static UnitNumParserOption delay_mul_num;
            static UnitNumParserOption delay_div_num;
            static UnitNumParserOption delay_shl_num;
            static UnitNumParserOption mem_wordsize;
            static UnitNumParserOption membus_size;
            static UnitNumParserOption inline_wire;
            static UnitNumParserOption include_size;
            static UnitNumParserOption include_freq;
            static UnitNumParserOption include_clocks;
    }; //class

} // namespace

#endif // h guard

