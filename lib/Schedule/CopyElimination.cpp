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
#include "vtm/VFInfo.h"
#include "vtm/MicroState.h"
#include "vtm/BitLevelInfo.h"

#include "llvm/Function.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
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
  LiveIntervals *LI;
  VFInfo *VFI;

  CopyElimination() : MachineFunctionPass(ID) {
    initializeCopyEliminationPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.addRequired<LiveVariables>();
    AU.addPreserved<LiveVariables>();
    AU.addRequiredID(PHIEliminationID);
    AU.addPreservedID(PHIEliminationID);
    AU.addRequired<SlotIndexes>();
    AU.addPreserved<SlotIndexes>();
    AU.addRequired<LiveIntervals>();
    AU.addPreserved<LiveIntervals>();
    AU.addRequired<BitLevelInfo>();
    AU.addPreserved<BitLevelInfo>();
    AU.setPreservesCFG();
  }

  MachineInstr *getBrInst();

  void EliminatePHI(MachineInstr *Copy);

  bool runOnMachineBasicBlock(MachineBasicBlock &MBB, MachineFunction &MF);

  bool runOnMachineFunction(MachineFunction &MF) {
    bool Changed = false;

    BLI = &getAnalysis<BitLevelInfo>();
    TII = MF.getTarget().getInstrInfo();
    VFI = MF.getInfo<VFInfo>();

    for (MachineFunction::iterator I = MF.begin(), E = MF.end();
         I != E; ++I)
      Changed |= runOnMachineBasicBlock(*I, MF);

    return Changed;
  }
};
}

char CopyElimination::ID = 0;
char &llvm::CopyEliminationID = CopyElimination::ID;

Pass *llvm::createCopyEliminationPass() {
  return new CopyElimination();
}
INITIALIZE_PASS_BEGIN(CopyElimination, "vtm-copy-elim",
                      "Eliminate copies produced by PHIElimination.",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(SlotIndexes)
INITIALIZE_PASS_DEPENDENCY(LiveIntervals)
INITIALIZE_PASS_END(CopyElimination, "vtm-copy-elim",
                    "Eliminate copies produced by PHIElimination.",
                    false, false)

void CopyElimination::EliminatePHI(MachineInstr *PN) {
  //MachineBasicBlock *CurBB = PN->getParent();
  //
  //ucOperand SrcOp = PN->getOperand(1),
  //          DstOp = PN->getOperand(0),
  //          PredOp = ucOperand::CreatePredicate();

  //unsigned SrcReg = SrcOp.getReg(),
  //         DstReg = DstOp.getReg();

  //MachineBasicBlock::iterator InsertPos = PN;
  //// Find the insert position.
  //if (PN == CurBB->begin())
  //  while ((++InsertPos)->getOpcode() != VTM::Control)
  //    assert(InsertPos->getOpcode() == VTM::COPY && "Unexpected Instruction!");
  //else
  //  while ((--InsertPos)->getOpcode() != VTM::Control)
  //    assert(InsertPos->getOpcode() == VTM::COPY && "Unexpected Instruction!");
  //
  //unsigned Slot = VFI->lookupPhiSlot(SrcReg, DstReg);
  //
  //if (Slot){
  //  unsigned II = VFI->getIIFor(CurBB);
  //  unsigned StartSlot = VFI->getStartSlotFor(CurBB);
  //  unsigned Offset = Slot - StartSlot;

  //  for (;;) {
  //    unsigned CtrlOffset = ucState(*InsertPos).getSlot() - StartSlot;
  //    if (Offset == CtrlOffset || (Offset - CtrlOffset) % II == 0)
  //      break;
  //    ++InsertPos;
  //    ++InsertPos;
  //  }
  //} else
  //  Slot = ucState(*InsertPos).getSlot();

  //ucState Ctrl(*InsertPos);

  //// Try to set the bit width for new instert copy instruction.
  //BLI->updateBitWidth(SrcOp, BLI->getBitWidth(SrcReg));
  //BLI->updateBitWidth(DstOp, BLI->getBitWidth(SrcReg));

  //for (ucState::iterator I = Ctrl.begin(), E = Ctrl.end(); I != E; ++I) {
  //  ucOp Op = *I;
  //  if (Op->getOpcode() != VTM::COPY) continue;

  //  ucOperand &MO = Op.getOperand(0);
  //  assert((!MO.isUse() || MO.getReg() != DstReg || MO.isKill())
  //          && "Can not fuse instruction!");
  //  // Forward the wire value if necessary.
  //  if (MO.isDef() && MO.getReg() == SrcReg) {
  //    SrcOp = Op.getOperand(1);
  //    break;
  //  }
  //}

  //// Transfer the operands.
  //PN->RemoveOperand(1);
  //PN->RemoveOperand(0);
  //MachineInstrBuilder MIB(InsertPos);
  //// Diry hack: Temporary use the slot of the micro state.
  //assert(Slot && "VOpToState not found!");
  //MIB.addOperand(ucOperand::CreateOpcode(VTM::COPY, Slot));
  //MIB.addOperand(PredOp).addOperand(DstOp).addOperand(SrcOp);
  //// Discard the operand.
  //PN->eraseFromParent();
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

  return true;
  if (Worklist.empty()) return false;

  for (IListTy::iterator I = Worklist.begin(), E = Worklist.end(); I != E; ++I)
    EliminatePHI(*I);

  DEBUG(dbgs() << MBB.getName() << " After copy fixed:\n";
        printVMBB(dbgs(), MBB));
  return true;
}
