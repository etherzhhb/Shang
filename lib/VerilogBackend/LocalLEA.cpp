//===- ForceDirectedScheduler.cpp - The ForceDirected Scheduler  -*- C++ -*-===//
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
// This file implement the Local left edge algorithm to perform register merging.
//
//===----------------------------------------------------------------------===//

#include "HWAtomInfo.h"
#include "HWAtomPasses.h"

#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/Allocator.h"

#define DEBUG_TYPE "vbe-local-lea"
#include "llvm/Support/Debug.h"

using namespace llvm;
using namespace esyn;

namespace {
struct RegChann {
  unsigned short Start, End;
  SmallVector<HWRegister*, 4> Regs;

  RegChann(HWRegister *R, unsigned short start, unsigned short end)
    : Start(start), End(end) { Regs.push_back(R); }

  unsigned short getStart() const { return Start; }
  unsigned short getEnd() const { return End; }

  HWRegister *getFirstRegister() const { return Regs.front(); }
  // Forward function
  unsigned getBitWidth() const { return getFirstRegister()->getBitWidth(); }
  typedef SmallVector<HWRegister*, 4>::iterator reg_iterator;
  typedef SmallVector<HWRegister*, 4>::const_iterator const_reg_iterator;

  reg_iterator begin() { return Regs.begin(); }
  const_reg_iterator begin() const { return Regs.begin(); }

  reg_iterator end() { return Regs.end(); }
  const_reg_iterator end() const { return Regs.end(); }

  void mergeChann(RegChann *RegCh) {
    assert((RegCh->getStart() > getEnd() || getStart() > RegCh->getEnd())
            && "Can not merge channel!");
    // Merge interval.
    Start = std::min(RegCh->getStart(), getStart());
    End = std::max(RegCh->getEnd(), getEnd());
    // Merge the registers.
    Regs.append(RegCh->begin(), RegCh->end());
  }

  void print(raw_ostream &OS) const {
    for (const_reg_iterator I = begin(), E = end(); I != E; ++I) {
      (*I)->print(OS);
      OS << "; ";
    }

    OS << "\t{" << Start << ", " << End << "}\n";
  }

  void dump() const { print(dbgs()); }
};

struct LocalLEA : public BasicBlockPass {
  static char ID;
  LocalLEA() : BasicBlockPass(&ID) {}

  bool runOnBasicBlock(BasicBlock &BB);
  void getAnalysisUsage(AnalysisUsage &AU) const;
  void releaseMemory();

  BumpPtrAllocator ChannAllocator;

  RegChann *createChann(HWRegister *R, unsigned short start, unsigned short end);
};
} // end namespace


inline bool chann_sort(const RegChann *LHS, const RegChann *RHS) {
    return LHS->getStart() < RHS->getStart();
}

//===----------------------------------------------------------------------===//

char LocalLEA::ID = 0;

RegChann *LocalLEA::createChann(HWRegister *R,
                                unsigned short start, unsigned short end) {
  return new (ChannAllocator) RegChann(R, start, end);
}

void LocalLEA::releaseMemory() {
  ChannAllocator.Reset();
}

void LocalLEA::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.setPreservesAll();
}

bool LocalLEA::runOnBasicBlock(BasicBlock &BB) {
  HWAtomInfo &HI = getAnalysis<HWAtomInfo>();

  FSMState *State = HI.getStateFor(BB);
  unsigned II = State->getII();
  DEBUG(dbgs() << "\nState " << BB.getName() << " {" << State->getSlot() << ", "
               << State->getEndSlot() << "} II: " << II << "\n");
  // First of all, build live interval for
  // Live-in register and write register atoms.
  SmallVector<RegChann*, 64> Channs;
  SmallVector<HWAWrReg*, 8> WrToPHI;

  for (FSMState::iterator I = State->begin(), E = State->end(); I != E; ++I) {
    HWAtom *A = *I;
    HWRegister *R = 0;

    unsigned Start = A->getFinSlot();

    if (HWAWrReg *WR = dyn_cast<HWAWrReg>(A)) {
      // Function unit register merging is not perform in this stage.
      if (WR->writeFUReg())
        continue;
      // Ignore the write to PHINode atom.
      if (HWValDep *VD = dyn_cast<HWValDep>(WR->use_back()->getEdgeFrom(WR)))
        if (VD->getDepType() == HWValDep::PHI) {
          // Write to PHINode atom may become a nop after registers merged.
          WrToPHI.push_back(WR);
          continue;
        }
        

      R = WR->getReg();
    } else if (HWALIReg *LI = dyn_cast<HWALIReg>(A)) {
      R = HI.lookupRegForValue(&LI->getValue());
      // Live-in register start when the state begin.
      Start = State->getSlot();
    } else
      continue;

    unsigned End = Start;
    DEBUG(A->print(dbgs()));
    DEBUG(dbgs() << " refering reg: ");
    DEBUG(R->print(dbgs()));
    DEBUG(dbgs() << " used by :");


    for (HWAtom::use_iterator UI = A->use_begin(), UE = A->use_end();
         UI != UE; ++UI) {
      HWAtom *U = *UI;
      End = std::max(End, U->getFinSlot());
      DEBUG(dbgs() << "\n\tat slot " << U->getFinSlot() << " >");
      DEBUG(U->print(dbgs()));
    }
    DEBUG(dbgs() << '\n');
    if (II) {
      assert((End - A->getSlot() <= II || State->getExitRoot()->isDepOn(A))
             && "Anti dependence Find!");
      unsigned StateStart = State->getSlot();
      unsigned Length = End - Start;
      // TODO: In fact the live time of live in register is not that long.
      // Move Start to the first iteration.
      Start = StateStart + (Start - StateStart) % II;
      // We get a live-in register or a export value.
      End = Start + std::max(II, Length);
    }
    
    RegChann *RegCh = createChann(R, Start, End);
    Channs.push_back(RegCh);
  }

  // Sort the channels.
  std::sort(Channs.begin(), Channs.end(), chann_sort);
  DEBUG(
    for (SmallVector<RegChann*, 64>::iterator I = Channs.begin(),
         E = Channs.end(); I != E; ++I)
      (*I)->dump();    
  );
  // And perform LEA merging.
  for (unsigned i = 0, e = Channs.size(); i != e; ++i) {
    RegChann *Chann = Channs[i];
    if (!Chann)
      continue;

    unsigned MergeIdx = i + 1;
    unsigned CurEnd = Chann->getEnd();
    for (;MergeIdx < e; ++MergeIdx) {
      RegChann *NextChann = Channs[MergeIdx];
      if (!NextChann)
        continue;
      
      if (NextChann->getStart() > CurEnd
          // Only merge the register with the same type.
          && NextChann->getBitWidth() == Chann->getBitWidth()) {
        Chann->mergeChann(NextChann);
        // Update end.
        CurEnd = Chann->getEnd();
        //
        Channs[MergeIdx] = 0;
      }
    }
  }

  DEBUG(
    dbgs() << "After channel merge:\n";
    for (SmallVector<RegChann*, 64>::iterator I = Channs.begin(),
        E = Channs.end(); I != E; ++I)
      if (RegChann *Ch = *I)
        Ch->dump();          
  );
  
  // Re-assign register number.
  for (SmallVector<RegChann*, 64>::iterator I = Channs.begin(),
      E = Channs.end(); I != E; ++I)
    if (RegChann *Ch = *I) {
      unsigned NewNum = Ch->getFirstRegister()->getRegNum();
      for (RegChann::reg_iterator I = Ch->begin() + 1, E = Ch->end(); I != E; ++I)
        (*I)->setRegNum(NewNum);
    }

  DEBUG(
    dbgs() << "After merge:\n";
  for (SmallVector<RegChann*, 64>::iterator I = Channs.begin(),
    E = Channs.end(); I != E; ++I)
    if (RegChann *Ch = *I)
      Ch->dump();          
  );

  // Eliminate Nops.
  while (!WrToPHI.empty()) {
    HWAWrReg *WritePHI = WrToPHI.back();
    WrToPHI.pop_back();

    HWAtom *Src = WritePHI->getSrc();
    // Writing a constant.
    if (!Src) continue;

    HWRegister *SrcReg = HI.lookupRegForValue(&Src->getValue()),
               *PHIReg = WritePHI->getReg();
    // This Write atom is not necessary.
    if (SrcReg->getRegNum() ==  PHIReg->getRegNum()) {
      WritePHI->dropAllReferences();
      WritePHI->replaceAllUseBy(Src);
    }
  }
  

  return false;
}

Pass *esyn::createLocalLEAPass() {
  return new LocalLEA();
}
