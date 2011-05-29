//====-- CopyElimination.cpp - Fuse the copys into micro state --*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement the CopyElimination pass, which move the copy instructions
// into the microstates.
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "vtm/FUInfo.h"
#include "vtm/MicroState.h"
#include "vtm/BitLevelInfo.h"

#include "llvm/Function.h"

#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Target/TargetInstrInfo.h"

#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "eliminate-regietsr-copy"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
struct CopyElimination : public MachineFunctionPass{
  static char ID;
  BitLevelInfo *BLI;
  MachineRegisterInfo *MRI;
  const TargetInstrInfo *TII;

  CopyElimination() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    // Diry hack: Force re-run the bitlevel info.
    AU.addRequired<BitLevelInfo>();
    AU.addPreserved<BitLevelInfo>();
  }

  void EliminateCopy(MachineInstr &Copy, MachineBasicBlock *TargetBB = 0);

  bool runOnMachineBasicBlock(MachineBasicBlock &MBB, MachineFunction &MF);

  bool runOnMachineFunction(MachineFunction &MF) {
    bool Changed = false;

    BLI = &getAnalysis<BitLevelInfo>();
    TII = MF.getTarget().getInstrInfo();

    for (MachineFunction::iterator I = MF.begin(), E = MF.end();
         I != E; ++I)
      Changed |= runOnMachineBasicBlock(*I, MF);

    return Changed;
  }
};
}

char CopyElimination::ID = 0;

Pass *llvm::createCopyEliminationPass() {
  return new CopyElimination();
}

static MachineInstr &findPrevControl(MachineInstr &I) {
  MachineBasicBlock *MBB = I.getParent();
  MachineBasicBlock::iterator It = I;
  assert(I.getNextNode()->getOpcode() != VTM::Datapath
         && I.getNextNode()->getOpcode() != VTM::Control
         && "Can not handle copy in the middle of the block!");
  do {
    --It;
    if (It->getOpcode() == VTM::Control)
      return *It;
  } while (It != MBB->begin());

  assert(0 && "Can not find prior control!");
  return I;
}

void CopyElimination::EliminateCopy(MachineInstr &Copy,
                                    MachineBasicBlock *TargetBB) {
  ucState Ctrl(findPrevControl(Copy));

  ucOperand SrcOp = Copy.getOperand(1),
            DstOp = Copy.getOperand(0),
            PredOp = ucOperand::CreatePredicate();

  unsigned SrcReg = SrcOp.getReg(),
           DstReg = DstOp.getReg();

  // Try to set the bit width for new instert copy instruction.
  BLI->updateBitWidth(SrcOp, BLI->getBitWidth(SrcReg));
  BLI->updateBitWidth(DstOp, BLI->getBitWidth(SrcReg));

  for (ucState::iterator I = Ctrl.begin(), E = Ctrl.end(); I != E; ++I) {
    ucOp Op = *I;
    if (Op->getOpcode() == VTM::COPY) {
      ucOperand &MO = Op.getOperand(0);
      assert((!MO.isUse() || MO.getReg() != DstReg || MO.isKill())
             && "Can not fuse instruction!");
      // Forward the wire value if necessary.
      if (MO.isDef() && MO.getReg() == SrcReg) {
        assert(Op->getOpcode() == VTM::COPY && "Can only forward copy!");
        SrcOp = Op.getOperand(1);
        continue;
      }
    }

    if (TargetBB && Op->getOpcode() == VTM::VOpToState
        && Op.getOperand(0).getMBB() == TargetBB)
      PredOp = Op.getPredicate();
  }

  // Transfer the operands.
  Copy.RemoveOperand(1);
  Copy.RemoveOperand(0);
  MachineInstrBuilder MIB(&*Ctrl);
  // Diry hack: Temporary use the slot of the micro state.
  MIB.addOperand(ucOperand::CreateOpcode(VTM::COPY, Ctrl.getSlot()));
  MIB.addOperand(PredOp).addOperand(DstOp).addOperand(SrcOp);

  // Discard the operand.
  Copy.eraseFromParent();
}

bool CopyElimination::runOnMachineBasicBlock(MachineBasicBlock &MBB,
                                             MachineFunction &MF) {
  DEBUG(dbgs() << MBB.getName() << " After register allocation:\n";
        printVMBB(dbgs(), MBB));
  typedef SmallVector<MachineInstr*, 16> IListTy;
  IListTy Worklist;

  for (MachineBasicBlock::iterator I = MBB.begin(), E = MBB.end();
       I != E; ++I) {
    MachineInstr *Instr = I;

    if (Instr->isCopy()) Worklist.push_back(Instr);
  }

  if (Worklist.empty()) return false;

  for (IListTy::iterator I = Worklist.begin(), E = Worklist.end(); I != E; ++I){
    MachineInstr *Copy = *I;
    if (Copy != MBB.begin()) {
      EliminateCopy(*Copy);
      continue;
    }

    // Move the copy at the beginning of the block to the end of the
    // predecessors of the current block.
    typedef MachineBasicBlock::pred_iterator pred_it;
    for (pred_it PI = MBB.pred_begin(), PE = MBB.pred_end(); PI != PE; ++PI) {
      MachineBasicBlock *PredBB = *PI;
      MachineInstr *NewCopy = MF.CloneMachineInstr(Copy);
      PredBB->insert(PredBB->getFirstTerminator(), NewCopy);
      EliminateCopy(*NewCopy, &MBB);
    }

    Copy->eraseFromParent();
  }

  DEBUG(dbgs() << MBB.getName() << " After copy fixed:\n";
        printVMBB(dbgs(), MBB));
  return true;
}
