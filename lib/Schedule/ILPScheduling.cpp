//===---------- IPLScheduling.cpp - IPL Scheduler ---------------*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
// IMPORTANT: This software is supplied to you by Hongbin Zheng in consideration
// of your agreement to the following terms, and your use, installation, 
// modification or redistribution of this software constitutes acceptance
// of these terms.  If you do not agree with these terms, please do not use, 
// install, modify or redistribute this software. You may not redistribute, 
// install copy or modify this software without written permission from 
// Hongbin Zheng. 
//
//===----------------------------------------------------------------------===//
//
// This file implement the Scheduler with Integer Linear Programming approach.
// Please reference paper:
// Hwang, C.-T.; Lee, J.-H.; Hsu, Y.-C.; ,
// "A formal approach to the scheduling problem in high level synthesis ,
// " Computer-Aided Design of Integrated Circuits and Systems,
//   IEEE Transactions on , vol.10, no.4, pp.464-475, Apr 1991
// for more detail.
//
//===----------------------------------------------------------------------===//

#include "SchedulingBase.h"

#include "lp_solve/lp_lib.h"


#define DEBUG_TYPE "vbe-fdls"
#include "llvm/Support/Debug.h"

using namespace llvm;

bool IPLScheduler::scheduleState() {
  lprec *lp = make_lp(0,4);

  delete_lp(lp);
  return true;
}
