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

void HWAWrStg::print(raw_ostream &OS) const {
  OS << "Write Storage "
     << (Reg->isFuReg() ? Reg->getFUnit().getRawData() : Reg->getRegNum());
}

void HWAImpStg::print(raw_ostream &OS) const {
  OS << "Import Storage "
     << (Reg->isFuReg() ? Reg->getFUnit().getRawData() : Reg->getRegNum());
}

void HWADelay::print(raw_ostream &OS) const {
  OS << "Delay: " << getLatency();
}

void HWMemDep::print(raw_ostream &OS) const {

}

void HWCtrlDep::print(raw_ostream &OS) const {
  //OS << "Value dep: ";
  //WriteAsOperand(OS, &Val, false);
  //OS << " -> ";
  //WriteAsOperand(OS, &getDep(0)->getValue(), false);
}

void HWConst::print(raw_ostream &OS) const {
  //OS << "Control dep: ";
  //WriteAsOperand(OS, &Val, false);
  //OS << " -> ";
  //WriteAsOperand(OS, &getDep(0)->getValue(), false);
}

void HWValDep::print(raw_ostream &OS) const {

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

void HWAOpInst::print(raw_ostream &OS) const {
  if (getValue().getType()->isVoidTy())
    OS << getValue() << '\n';
  else
    WriteAsOperand(OS, &getValue(), false);
  OS << " Res: " << getFunUnitID();
}

void HWAVRoot::print(raw_ostream &OS) const {
  WriteAsOperand(OS, &getValue(), false);
  OS << " Entry";
}

HWAtom::HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V,
               unsigned latancy)
: FastID(ID), HWAtomType(HWAtomTy), Val(V), SchedSlot(0), Latancy(latancy) {}


HWAtom::HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy,
               Value &V, HWEdge *Dep0, unsigned latancy) : FastID(ID),
               HWAtomType(HWAtomTy), Val(V), SchedSlot(0), Latancy(latancy) {
  Deps.push_back(Dep0);
  Dep0->getDagSrc()->addToUseList(this);
}


void HWAtom::scheduledTo(unsigned slot) {
  SchedSlot = slot;
}


void HWAtom::replaceAllUseBy(HWAtom *A) {
  while (!use_empty()) {
    HWAtom *U = use_back();

    U->setDep(U->getDepIt(this), A);
  }
}

HWAPreBind::HWAPreBind(const FoldingSetNodeIDRef ID, HWAPostBind &PostBind,
                       HWFUnit FUID)
  : HWAOpInst(ID, atomPreBind, PostBind.getInst<Instruction>(),
              PostBind.edge_begin(), PostBind.edge_end(),
              PostBind.getInstNumOps(), FUID) {
  // Remove the PostBind atom from the use list of its dep.
  for (dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
    I->removeFromList(&PostBind);
    // This is done in the constructor of HWAtom.
    //I->addToUseList(this);

  PostBind.replaceAllUseBy(this);
  // Setup the step
  scheduledTo(PostBind.getSlot());
  setParent(PostBind.getParent());
}

HWAtom *HWMemDep::getSCCSrc() const {
  return Data.getPointer();
}

void esyn::FSMState::dump() const {
  print(dbgs());
}

HWValDep::HWValDep(HWAtom *Src, bool isSigned, bool isImport)
: HWEdge(edgeValDep, Src, 0), IsSigned(isSigned), IsImport(isImport) {
  assert((!IsImport || isa<HWAVRoot>(Src)) && "Bad import edge!");
}