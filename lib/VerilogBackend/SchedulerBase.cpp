//===------ ScheduleBase.cpp - The BaseClass of all scheduler  ----*- C++ -*-===//
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
// This file implement the SchedulerBase class, which provide common service for
// schedulers.
//
//===----------------------------------------------------------------------===//
//

#include "vbe/SchedulerBase.h"

#include "HWAtomInfo.h"
#include "llvm/Support/Debug.h"


using namespace llvm;
using namespace esyn;

//===----------------------------------------------------------------------===//
void Scheduler::clearSchedulerBase() {
  ResCycMap.clear();
}

Scheduler::~Scheduler() {
  clearSchedulerBase();
}

unsigned Scheduler::getReadyCycle(HWResource::ResIdType ResId) {
  return ResCycMap[ResId];
}

void Scheduler::rememberReadyCycle(HWResource::ResIdType ResId,
                                   unsigned ReadyCycle) {
  ResCycMap[ResId] = ReadyCycle;
}
