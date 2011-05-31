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
#include "llvm/Support/ErrorHandling.h"
#define DEBUG_TYPE "eliminate-regietsr-copy"
#include "llvm/Support/Debug.h"

#include <map>

using namespace llvm;

namespace {
struct CopyElimination : public MachineFunctionPass{
  static char ID;
  BitLevelInfo *BLI;
  MachineRegisterInfo *MRI;
  const TargetInstrInfo *TII;

  // Remember the instruction containing the VOpToState branching to others.
  // For normal BB, it should be the last control instruction.
  // For pipelined BB, it maybe located in the middle of the BB.
  std::map<MachineBasicBlock*, MachineInstr*> BrCtrlMap;
  MachineInstr *getBrCrl(MachineBasicBlock *MBB) {
    std::map<MachineBasicBlock*, MachineInstr*>::iterator at
      = BrCtrlMap.find(MBB);

    if (at != BrCtrlMap.end()) return at->second;

    // Otherwise, find the br now.
    for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end();
         I != E; ++I) {
      MachineInstr &MI = *I;

      if (MI.getFlag((MachineInstr::MIFlag)ucState::hasTerm)) {
        BrCtrlMap.insert(std::make_pair(MBB, &MI));
        return &MI;
      }
    }

    llvm_unreachable("Cannot find br control in a MachineBasicBlock!");
    return 0;
  }

  CopyElimination() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    // Diry hack: Force re-run the bitlevel info.
    AU.addRequired<BitLevelInfo>();
    AU.addPreserved<BitLevelInfo>();
  }

  MachineInstr *getBrInst();

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

  void releaseMemory() { BrCtrlMap.clear(); }
};
}

char CopyElimination::ID = 0;

Pass *llvm::createCopyEliminationPass() {
  return new CopyElimination();
}

void CopyElimination::EliminateCopy(MachineInstr &Copy,
                                    MachineBasicBlock *TargetBB) {
  assert(0 && "Unexpected copy!");
  MachineBasicBlock *CurBB = Copy.getParent();
  assert(CurBB != TargetBB && "Copy should be eliminated by former passes!");

  // Insert the copy to the control instruction in *execution sequence*, for a
  // normal BB, the appearance sequence and execution sequence of instructions
  // are the same, but in a pipelined BB, it is not true.
  // In a pipelined BB, we should insert the copy to the last control
  // instruction in the execution sequence, which is also the last control
  // instruction in the appearance sequence of the *kernel*. Becasue the copy
  // instruction is coping register value from previous iteration to next
  // iteration of the *kenerl*.
  ucState Ctrl(*getBrCrl(CurBB));

  ucOperand SrcOp = Copy.getOperand(1),
            DstOp = Copy.getOperand(0),
            PredOp = ucOperand::CreatePredicate();

  unsigned SrcReg = SrcOp.getReg(),
           DstReg = DstOp.getReg();

  unsigned Slot = 0;

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
    } else if (Op->getOpcode() == VTM::VOpToState
               && Op.getOperand(0).getMBB() != CurBB) {
      Slot = Op->getPredSlot();

      if (TargetBB && Op.getOperand(0).getMBB() == TargetBB)
        PredOp = Op.getPredicate();
    }
  }

  // Transfer the operands.
  Copy.RemoveOperand(1);
  Copy.RemoveOperand(0);
  MachineInstrBuilder MIB(&*Ctrl);
  // Diry hack: Temporary use the slot of the micro state.
  assert(Slot && "VOpToState not found!");
  MIB.addOperand(ucOperand::CreateOpcode(VTM::COPY, Slot));
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

    if (Instr->getFlag((MachineInstr::MIFlag)ucState::hasTerm))
      BrCtrlMap.insert(std::make_pair(&MBB, Instr));

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
