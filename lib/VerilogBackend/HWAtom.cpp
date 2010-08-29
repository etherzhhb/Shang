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


void HWRegister::print(raw_ostream &OS) const {
  OS << getRegNum() << "(" << getResType() << ")";
}

void HWAWrReg::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] " << "Write Storage ";
  Reg->print(OS);
}

void HWALIReg::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] " << "Live in register for ";
  WriteAsOperand(OS, &getValue(), false);  
}

void HWADelay::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] " << "Delay: " << getLatency();
}

void HWMemDep::print(raw_ostream &OS) const {

}

void HWCtrlDep::print(raw_ostream &OS) const {
}

void HWConst::print(raw_ostream &OS) const {
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
}

void HWAOpFU::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] ";
  if (getValue().getType()->isVoidTy())
    OS << getValue() << '\n';
  else
    WriteAsOperand(OS, &getValue(), false);
  OS << " Res: " << *getFUnit();
}

HWAtom::HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V,
               uint8_t latancy, uint8_t bitWidth, unsigned short Idx)
  : FastID(ID), HWAtomType(HWAtomTy), Val(V), SchedSlot(0), Latancy(latancy),
  BitWidth(bitWidth), InstIdx(Idx), Parent(0) {}


HWAtom::HWAtom(const FoldingSetNodeIDRef ID, unsigned HWAtomTy, Value &V,
               HWEdge *Dep0, uint8_t latancy, uint8_t bitWidth, unsigned short Idx)
  : FastID(ID), HWAtomType(HWAtomTy), Val(V), SchedSlot(0), Latancy(latancy),
  BitWidth(bitWidth), InstIdx(Idx), Parent(0) {
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

void FSMState::dump() const {
  print(dbgs());
}

static inline bool top_sort(const HWAtom* LHS, const HWAtom* RHS) {
  return LHS->getIdx() < RHS->getIdx();
}

void FSMState::setExitRoot(HWAOpFU *Exit) {
  ExitRoot = Exit;
  std::sort(Atoms.begin(), Atoms.end(), top_sort);
}

HWValDep::HWValDep(HWAtom *Src, bool isSigned, enum ValDepTypes T)
: HWEdge(edgeValDep, Src, 0), IsSigned(isSigned), DepType(T) {}

void HWAtom::setParent(FSMState *State) {
  Parent = State;
}

bool HWAWrReg::writeFUReg() const {
  return Reg->isFuReg();
}

HWADelay::HWADelay(const FoldingSetNodeIDRef ID, HWCtrlDep &Edge, unsigned Delay,
                   unsigned Idx )
  : HWAtom(ID, atomDelay, Edge->getValue(), &Edge,Delay, 0, Idx) {
  Edge->getParent()->addAtom(this);
}

HWAWrReg::HWAWrReg( const FoldingSetNodeIDRef ID, HWEdge &Edge, HWRegister *reg,
                   unsigned short Slot, unsigned short Idx)
  : HWAtom(ID, atomWrReg, Edge->getValue(), &Edge, 1, Edge->getBitWidth(), Idx),
  Reg(reg) {
  if (Slot)
    scheduledTo(Slot);
  Edge->getParent()->addAtom(this);
}

HWALIReg::HWALIReg(const FoldingSetNodeIDRef ID, Value &V, HWEdge *VEdge,
                   uint8_t bitWidth, unsigned short Idx )
  : HWAtom(ID, atomLIReg, V, VEdge, 0, bitWidth, Idx) {
  (*VEdge)->getParent()->addAtom(this);
}

HWConst::HWConst(FSMState *Src, Constant *Const)
: HWEdge(edgeConst, Src, 0), C(Const) {}
