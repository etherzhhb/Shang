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


void esyn::HWADrvReg::print(raw_ostream &OS) const {

}

void HWMemDep::print(raw_ostream &OS) const {

}

void HWCtrlDep::print(raw_ostream &OS) const {
  //OS << "Value dep: ";
  //WriteAsOperand(OS, &Val, false);
  //OS << " -> ";
  //WriteAsOperand(OS, &getDep(0)->getValue(), false);
}

void HWValDep::print(raw_ostream &OS) const {
  //OS << "Control dep: ";
  //WriteAsOperand(OS, &Val, false);
  //OS << " -> ";
  //WriteAsOperand(OS, &getDep(0)->getValue(), false);
}

void FSMState::getScheduleMap(ScheduleMapType &Atoms) const {
  for (const_usetree_iterator I = usetree_begin(), E = usetree_end(); I != E; ++I) {
    HWAtom *A = const_cast<HWAtom*>(*I);
    Atoms.insert(std::make_pair(A->getSlot(), A));
  }
}

void FSMState::print(raw_ostream &OS) const {
  OS << "State: ";
  WriteAsOperand(OS, getBasicBlock(), false);
  OS << "\n";
  unsigned oldSlot = 0;

  std::multimap<unsigned, HWAtom*> Atoms;
  getScheduleMap(Atoms);
  for (std::multimap<unsigned, HWAtom*>::iterator I = Atoms.begin(),
      E = Atoms.end(); I != E; ++I) {
    HWAtom *A = I->second;
    if (A->getSlot() != oldSlot) {
      oldSlot = A->getSlot();
      OS << "Cycle: " << oldSlot << "\n";
    }
    A->print(OS.indent(2));
    OS << " at "<< A->getSlot() << "\n";
  }
}

void HWAPreBind::print(raw_ostream &OS) const {
  if (getValue().getType()->isVoidTy())
    OS << getValue() << '\n';
  else
    WriteAsOperand(OS, &getValue(), false);
  OS << " Res: " << SubClassData;
}

void HWAPostBind::print(raw_ostream &OS) const {
  if (getValue().getType()->isVoidTy())
    OS << getValue() << '\n';
  else
    WriteAsOperand(OS, &getValue(), false);
  OS << " PostBind: " << SubClassData;
}

void HWAVRoot::print(raw_ostream &OS) const {
  WriteAsOperand(OS, &getValue(), false);
  OS << " Entry";
}

HWAtom::HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V)
: FastID(ID), HWAtomType(HWAtomTy), Val(V), SchedSlot(0) {}


HWAtom::HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy,
               Value &V, HWEdge *Dep0) : FastID(ID),
               HWAtomType(HWAtomTy), Val(V), SchedSlot(0)  {
  Deps.push_back(Dep0);
  Dep0->getSrc()->addToUseList(this);
}


void HWAtom::scheduledTo(unsigned slot) {
  SchedSlot = slot;
}


void HWAtom::replaceAllUseBy(HWAtom *A) {
  while (!use_empty()) {
    HWAtom *U = use_back();

    U->setDep(U->getDepIdx(this), A);
  }
}

HWAPreBind::HWAPreBind(const FoldingSetNodeIDRef ID, HWAPostBind &PostBind,
                       unsigned Instance)
  : HWAOpInst(ID, atomPreBind, PostBind.getInst<Instruction>(),
  PostBind.getLatency(), PostBind.edge_begin(), PostBind.edge_end(),
  PostBind.getInstNumOps(),
  HWResource::createResId(PostBind.getResClass(), Instance)) {
  // Remove the PostBind atom from the use list of its dep.
  for (dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
    I->removeFromList(&PostBind);

  PostBind.replaceAllUseBy(this);

  // Setup the step
  scheduledTo(PostBind.getSlot());
}
