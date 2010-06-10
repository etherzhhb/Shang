//===------------ HWAtom.cpp - Translate LLVM IR to HWAtom  -----*- C++ -*-===//
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
// This file implement the HWAtom class, which represent the basic atom
// operation in hardware. and the HWAtomInfo pass, which construct the HWAtom
// from LLVM IR.
//
//===----------------------------------------------------------------------===//

#include "HWAtom.h"

#include "llvm/Assembly/Writer.h"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;


HWAtom::~HWAtom() {}

void HWAtom::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

void HWASigned::print(raw_ostream &OS) const {
  OS << "signed (";
  getOperand()->print(OS);
  OS << ")";
}

void HWABitCast::print(raw_ostream &OS) const {
  OS << "Bitcast form: ";
  getOperand()->print(OS);
  OS << " to ";
  WriteAsOperand(OS, &Inst, false);
}


//===----------------------------------------------------------------------===//

char HWAtomInfo::ID = 0;
RegisterPass<HWAtomInfo> X("vbe-hw-atom-info",
                           "vbe - Construct the Hardware atom respresent"
                           " on llvm IR");

bool HWAtomInfo::runOnFunction(Function &F) {
  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
    for (BasicBlock::iterator BI = I->begin(), BE = I->end(); I != E; ++I) {
      visit(*BI);
    }
  }
 return false;
}

void HWAtomInfo::clear() {
  HWAtomAllocator.Reset();
  UniqiueHWAtoms.clear();
  InstToHWAtoms.clear();
}