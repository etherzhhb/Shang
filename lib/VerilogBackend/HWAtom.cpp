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

void HWAWrReg::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] " << "Write Storage "
     << Reg->getResType() << "$" << Reg->getRegNum();
}

void HWARdReg::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] " << "Import Storage "
     << Reg->getResType() << "$" << Reg->getRegNum();
}

void HWADelay::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] " << "Delay: " << getLatency();
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
  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    HWAtom *A = const_cast<HWAtom*>(*I);
    Atoms.insert(std::make_pair(A->getSlot(), A));
  }
}

void FSMState::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] ";
  WriteAsOperand(OS, &getValue(), false);
  OS << " Entry";

  //OS << "State: ";
  //WriteAsOperand(OS, getBasicBlock(), false);
  //OS << "\n";
  //unsigned oldSlot = 0;

  //std::multimap<unsigned, HWAtom*> Atoms;
  //getScheduleMap(Atoms);
  //for (std::multimap<unsigned, HWAtom*>::iterator I = Atoms.begin(),
  //    E = Atoms.end(); I != E; ++I) {
  //  HWAtom *A = I->second;
  //  if (A->getSlot() != oldSlot) {
  //    oldSlot = A->getSlot();
  //    OS << "Cycle: " << oldSlot << "\n";
  //  }
  //  A->print(OS.indent(2));
  //  OS << " at "<< A->getSlot() << "\n";
  //}
}

void HWAOpInst::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] ";
  if (getValue().getType()->isVoidTy())
    OS << getValue() << '\n';
  else
    WriteAsOperand(OS, &getValue(), false);
  OS << " Res: " << *getFunUnit();
}

HWAtom::HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V,
               unsigned latancy, unsigned short Idx)
: FastID(ID), HWAtomType(HWAtomTy), Val(V), SchedSlot(0), Latancy(latancy),
InstIdx(Idx) {}


HWAtom::HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy,
               Value &V, HWEdge *Dep0, unsigned latancy, unsigned short Idx) : FastID(ID),
               HWAtomType(HWAtomTy), Val(V), SchedSlot(0), Latancy(latancy),
               InstIdx(Idx) {
  Deps.push_back(Dep0);
  Dep0->getSrc()->addToUseList(this);
}


void HWAtom::scheduledTo(unsigned slot) {
  assert(slot && "Can not schedule to slot 0!");
  SchedSlot = slot;
}

void HWAtom::dropAllReferences() {
  for (dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
    I->removeFromList(this);
}

void HWAtom::replaceAllUseBy(HWAtom *A) {
  while (!use_empty()) {
    HWAtom *U = use_back();

    U->setDep(U->getDepIt(this), A);
  }

  // The Atom dead.
  getParent()->eraseAtom(this);
}

HWAPreBind::HWAPreBind(const FoldingSetNodeIDRef ID, HWAPostBind &PostBind,
                       unsigned id)
  : HWAOpInst(ID, atomPreBind, PostBind.getInst<Instruction>(),
              PostBind.edge_begin(), PostBind.edge_end(),
              PostBind.getInstNumOps(), PostBind.getFunUnit(), PostBind.getIdx()),
              FUID(id) {
  // Remove the PostBind atom from the use list of its dep.
  for (dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
    I->removeFromList(&PostBind);
    // This is done in the constructor of HWAtom.
    //I->addToUseList(this);

  // Setup the step
  scheduledTo(PostBind.getSlot());

  setParent(PostBind.getParent());
  PostBind.replaceAllUseBy(this);
}

void FSMState::dump() const {
  print(dbgs());
}

void FSMState::setExitRoot(HWAOpInst *Exit) {
  ExitRoot = Exit;

  for (usetree_iterator I = usetree_iterator::begin(this),
    E = usetree_iterator::end(this); I != E; ++I) {
      (*I)->setParent(this);
  }

  std::sort(Atoms.begin(), Atoms.end(), HWAtom::top_sort());
}

HWValDep::HWValDep(HWAtom *Src, bool isSigned, enum ValDepTypes T)
: HWEdge(edgeValDep, Src, 0), IsSigned(isSigned), DepType(T) {}

void HWAtom::setParent(FSMState *State) {
  Parant = State;
  State->addAtom(this);
}

bool HWAWrReg::writeFUReg() const {
  return Reg->isFuReg();
}