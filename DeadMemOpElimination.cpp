//===- AdjustLIForBundles.cpp - Adjust live intervals for bundles -*- C++ -*-=//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
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
// This file implement the pass that eliminate the dead memory operations
// at machine-code level.
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Passes.h"
#include "vtm/Utilities.h"

#include "llvm/Analysis/AliasSetTracker.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/ADT/SetOperations.h"
#include "llvm/ADT/Statistic.h"
#define DEBUG_TYPE "vtm-dead-memop-elimination"
#include "llvm/Support/Debug.h"

using namespace llvm;

STATISTIC(DeadStoreEliminated,
          "Number of dead stores eliminated in machine code.");
STATISTIC(DeadLoadEliminated,
          "Number of dead loads eliminated in machine code.");

namespace {
struct DeadMemOpElimination : public MachineFunctionPass {
  static char ID;

  DeadMemOpElimination() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<AliasAnalysis>();
    AU.addPreserved<AliasAnalysis>();
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  typedef DenseMap<AliasSet*, MachineInstr*> DefMapTy;

  void updateReachingDefByCallInst(MachineInstr *MI, DefMapTy &Defs);

  typedef MachineBasicBlock::instr_iterator instr_iterator;
  instr_iterator handleMemOp(instr_iterator I, DefMapTy &Defs,
                             AliasSetTracker &AST);

  bool runOnMachineBasicBlock(MachineBasicBlock &MBB, AliasAnalysis &AA);

  bool runOnMachineFunction(MachineFunction &MF) {
    AliasAnalysis &AA = getAnalysis<AliasAnalysis>();
    bool changed = false;

    for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
      changed |= runOnMachineBasicBlock(*I, AA);

    return changed;
  }
};
}

Pass *llvm::createDeadMemOpEliminationPass() {
  return new DeadMemOpElimination();
}

char DeadMemOpElimination::ID = 0;

void DeadMemOpElimination::updateReachingDefByCallInst(MachineInstr *MI,
                                                       DefMapTy &Defs) {
  assert(MI->getOpcode() == VTM::VOpInternalCall && "Bad Instruction type!");

  typedef DefMapTy::iterator def_it;
  for (def_it I = Defs.begin(), E = Defs.end(); I != E; ++I) {
    if (I->first->isForwardingAliasSet()) continue;

    // We assume submodule call modify all memory locations at the moment.
    I->second = MI;
  }
}

static inline bool isPredIdentical(const MachineInstr *LHS,
                                   const MachineInstr *RHS) {
  const MachineOperand *LHSPred = VInstrInfo::getPredOperand(LHS);
  const MachineOperand *RHSPred = VInstrInfo::getPredOperand(RHS);

  return LHSPred->isIdenticalTo(*RHSPred);
}

DeadMemOpElimination::instr_iterator
DeadMemOpElimination::handleMemOp(instr_iterator I, DefMapTy &Defs,
                                  AliasSetTracker &AST) {
  MachineInstr *MI = I;

  MachineMemOperand *MO = *MI->memoperands_begin();
  // AliasAnalysis cannot handle offset right now, so we pretend to write a
  // a big enough size to the location pointed by the base pointer.
  uint64_t Size = MO->getSize() + MO->getOffset();
  AliasSet *AA = &AST.getAliasSetForPointer(const_cast<Value*>(MO->getValue()),
                                            Size, 0);

  MachineInstr *&LastMI = Defs[AA];

  bool canHandleLastStore =
    LastMI && LastMI->getOpcode() != VTM::VOpInternalCall
    && !(*LastMI->memoperands_begin())->isVolatile() && AA->isMustAlias()
    // FIXME: We may need to remember the last definition for all predicates.
    && isPredIdentical(LastMI, MI);

  // FIXME: These elimination is only valid if we are in single-thread mode!
  if (VInstrInfo::mayStore(MI)) {
    if (canHandleLastStore) {
      // Dead store find, remove it.
      LastMI->eraseFromParent();
      ++DeadStoreEliminated;
    }

    // Update the definition.
    LastMI = MI;
    return I;
  }

  // Now MI is a load.
  if (!canHandleLastStore) return I;

  MachineMemOperand *LastMO = *LastMI->memoperands_begin();
  // We can only handle last store if and only if their memory operand have
  // the must-alias address and the same size.
  if (LastMO->getSize() != MO->getSize()) return I;

  // Loading the value that just be stored, the load is not necessary.
  MachineOperand LoadedMO = MI->getOperand(0);
  MachineOperand StoredMO = LastMI->getOperand(2);

  // Simply replace the load by a copy.
  DebugLoc dl = MI->getDebugLoc();
  I = *BuildMI(*MI->getParent(), I, dl, VInstrInfo::getDesc(VTM::VOpMove))
        .addOperand(LoadedMO).addOperand(StoredMO).
        addOperand(*VInstrInfo::getPredOperand(MI)).
        addOperand(*VInstrInfo::getTraceOperand(MI));

  MI->eraseFromParent();
  ++DeadLoadEliminated;
  return I;
}

bool DeadMemOpElimination::runOnMachineBasicBlock(MachineBasicBlock &MBB,
                                                  AliasAnalysis &AA) {
  AliasSetTracker AST(AA);
  DefMapTy ReachingDefMap;

  typedef AliasSetTracker::iterator ast_iterator;

  for (instr_iterator I = MBB.instr_begin(), E = MBB.instr_end(); I != E; ++I) {
    unsigned Opcode = I->getOpcode();

    if (Opcode == VTM::VOpInternalCall) {
      updateReachingDefByCallInst(I, ReachingDefMap);
      continue;
    }

    if (Opcode != VTM::VOpMemTrans && Opcode != VTM::VOpBRAMTrans) continue;

    I = handleMemOp(I, ReachingDefMap, AST);
  }
  
  return true;
}
