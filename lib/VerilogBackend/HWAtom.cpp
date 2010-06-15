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
// operation in hardware.
//
//===----------------------------------------------------------------------===//

#include "vbe/HWAtom.h"

#include "llvm/Assembly/Writer.h"
#include "llvm/Support/Debug.h"
#include "llvm/Analysis/LoopInfo.h"

#include "vbe/ResourceConfig.h"

using namespace llvm;
using namespace esyn;


HWAtom::~HWAtom() {}

void HWAtom::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

void HWASigned::print(raw_ostream &OS) const {
  OS << "signed (";
  WriteAsOperand(OS, &getDep(0)->getValue(), false);
  OS << ")";
}


void HWARegister::print( raw_ostream &OS ) const {
  OS << "Register: ";
  WriteAsOperand(OS, &Val, false);
  OS << " using ";
  WriteAsOperand(OS, &getDep(0)->getValue(), false);
}

void HWAWireOp::print(raw_ostream &OS) const {
  OS << "Wire Op ";
  WriteAsOperand(OS, &getDep(0)->getValue(), false);
  OS << " to ";
  WriteAsOperand(OS, &Val, false);
}

void HWAStateEnd::print(raw_ostream &OS) const {
  OS << "State Transfer: " << Val;
}

void HWAStateBegin::print(raw_ostream &OS) const {
  OS << "State: ";
  WriteAsOperand(OS, &getValue(), false);
  OS << "\n";
  for (HWAStateBegin::const_iterator I = begin(), E = end(); I != E; ++I) {
    (*I)->print(OS.indent(2));
    OS << "\n";
  }
  
}

void HWAOpPostAllRes::print(raw_ostream &OS) const {
  WriteAsOperand(OS, &getValue(), false);
  OS << " PostAllRes: " << getUsedResource().getName();
}

void HWAOpPreAllRes::print(raw_ostream &OS) const {
  WriteAsOperand(OS, &getValue(), false);
  OS << " PreAllRes: " << getUsedResource().getName();
}

//===----------------------------------------------------------------------===//
void HWResTable::clear() {
  ResSet.clear();
}

HWResTable::~HWResTable() {
  clear();
}

HWResource *HWResTable::initResource(std::string Name){
  HWResource *HR = RC.getResource(Name);
  assert(HR && "Can not init resource!");
  ResSet.insert(HR);
  return HR;
}
