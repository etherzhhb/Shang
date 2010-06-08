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

#ifndef LLVM_SCHED_UTILS_H
#define LLVM_SCHED_UTILS_H

#include "llvm/Target/Mangler.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <set>

using namespace llvm;

using std::set;
using std::vector;
using std::pair;
using std::string;
using std::stringstream;

namespace {
   
    /*
     * Set the terminal to color mode. Each line from this point will
     * be printed in color (white over blue).
     */
    static inline std::string xBlue() {
        return "\033[44;37;5m";
    }

    /*
     * Set the terminal to color mode. Each line from this point will
     * be printed in color (white over blue).
     */
    static inline std::string xRed() {
        return "\033[22;31;5m";
    }

    /*
     * Reset the terminal to it's regular color mode
     */ 
     std::string xReset() {
        return "\033[0m";
    }

    static inline void logPassMessage(std::string passName,int line ,std::string message, bool good=true) {
        if (good) {
            errs()<<xBlue();
        } else {
            errs()<<xRed();
        }
        errs() << "OPT:"<< passName << "," << line << ":" << xReset() << message <<"\n";
    }
} //end of namespace
#endif // h guard
