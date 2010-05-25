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
#include "intrinsics.h"
#include <sstream>
namespace xVerilog {


    string getIntrinsicForInstruction(CallInst *intrin, vector<string> &params) {
         std::stringstream ss;
         string macro_name =  intrin->getCalledFunction()->getNameStr(); 
         if (macro_name == "rotate_left") {
            assert(params.size()==3 && "rotate_left needs to have two params");
            string var = params[1];
            string offset = params[2];
            ss<<"{" << var<<"[31-"<<offset<<":0]"<< "," << var<<"[31:32-"<<offset<<"]}";
         }   
       return ss.str();
    }


} // namespace
