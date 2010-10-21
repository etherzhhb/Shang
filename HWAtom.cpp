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

#include "esly/HWAtom.h"

#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"
#include "llvm/Analysis/LoopInfo.h"

using namespace llvm;


HWAtom::~HWAtom() {}

void HWAtom::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

void HWRegister::print(raw_ostream &OS) const {
  OS << getRegNum() << "(" << getResType() << ")";
}

void HWADelay::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] " << "Delay: " << getLatency();
}

void HWMemDep::print(raw_ostream &OS) const {

}

void HWCtrlDep::print(raw_ostream &OS) const {
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
  OS << "[" << getIdx() << "] " << getMBB()->getName()
     << " Entry";
}

void HWAOpFU::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] " << *getMInst() << '\t'
     << " Res: " << getResType() << " ID: " << FUID;
}

HWAtom::HWAtom(unsigned HWAtomTy, const MachineInstr *MI,
               unsigned short latancy, unsigned short Idx)
  : HWAtomType(HWAtomTy), Latancy(latancy), SchedSlot(0), InstIdx(Idx),
  Parent(0), MInst(MI) {}


HWAtom::HWAtom(unsigned HWAtomTy, const MachineInstr *MI, HWEdge *Dep0,
               unsigned short latancy, unsigned short Idx)
  : HWAtomType(HWAtomTy), Latancy(latancy), SchedSlot(0), InstIdx(Idx),
  Parent(0), MInst(MI) {
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



// DOTWriter for FSMState.
#include "llvm/Support/GraphWriter.h"

template<>
struct DOTGraphTraits<FSMState*> : public DefaultDOTGraphTraits {

  DOTGraphTraits(bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  static std::string getGraphName(const FSMState *G) {
    return G->getMBB()->getName();
  }

  /// If you want to override the dot attributes printed for a particular
  /// edge, override this method.
  static std::string getEdgeAttributes(const HWAtom *Node,
                                       HWAtom::use_iterator EI) {
    return "";
  }

  std::string getNodeLabel(const HWAtom *Node, const FSMState *Graph) {
    std::string Str;
    raw_string_ostream ss(Str);
    Node->print(ss);
    return ss.str();
  }
};

void FSMState::viewGraph() {
  ViewGraph(this, this->getMBB()->getName());
}
