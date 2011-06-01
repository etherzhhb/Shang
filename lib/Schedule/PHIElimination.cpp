//====-- PHIElimination.cpp - Fuse the copys into micro state ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement the PHIElimination pass, which move the copy instructions
// into the microstates.
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/MicroState.h"

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
struct PHIElimination : public MachineFunctionPass{
  static char ID;
  VFInfo *VFI;
  MachineRegisterInfo *MRI;

  std::map<uint64_t, MachineInstr*> BrCtrlMap;
  MachineInstr *getBrCrl(MachineBasicBlock *SrcMBB, MachineBasicBlock *DstMBB){
    uint64_t key = (uint64_t(SrcMBB->getNumber()) & 0xffff)
                   | ((uint64_t(DstMBB->getNumber()) & 0xffff) << 32);

    std::map<uint64_t, MachineInstr*>::iterator at = BrCtrlMap.find(key);

    if (at != BrCtrlMap.end()) return at->second;

    // Otherwise, find the br now.
    for (MachineBasicBlock::iterator I = SrcMBB->begin(), E = SrcMBB->end();
         I != E; ++I) {
      MachineInstr &MI = *I;

      if (!MI.getFlag((MachineInstr::MIFlag)ucState::hasTerm)) continue;

      typedef MachineInstr::mop_iterator mop_iterator;
      for (mop_iterator OI = MI.operands_begin(), OE = MI.operands_end();
           OI != OE; ++OI) {
        MachineOperand &MO = *OI;
        if (MO.isMBB() && MO.getMBB() == DstMBB) {
          BrCtrlMap.insert(std::make_pair(key, &MI));
          return &MI;
        }
      }
    }

    llvm_unreachable("Cannot find br control in a MachineBasicBlock!");
    return 0;
  }

  PHIElimination() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  void EliminatePHI(MachineInstr *PN);

  bool runOnMachineBasicBlock(MachineBasicBlock &MBB, MachineFunction &MF);

  bool runOnMachineFunction(MachineFunction &MF) {
    bool Changed = false;
    VFI = MF.getInfo<VFInfo>();
    MRI = &MF.getRegInfo();

    for (MachineFunction::iterator I = MF.begin(), E = MF.end();
         I != E; ++I)
      Changed |= runOnMachineBasicBlock(*I, MF);

    return Changed;
  }

  void releaseMemory() { BrCtrlMap.clear(); }
};
}

char PHIElimination::ID = 0;

Pass *llvm::createPHIEliminationPass() {
  return new PHIElimination();
}

void PHIElimination::EliminatePHI(MachineInstr *PN) {
  MachineBasicBlock *CurBB = PN->getParent();

  typedef SmallVector<MachineOperand, 8> OperandVector;
  OperandVector Ops;

  ucState FirstCtrl(*CurBB->getFirstNonPHI());
  MachineInstrBuilder MIB(&*FirstCtrl);

  unsigned startSlot = FirstCtrl.getSlot();
  unsigned Slot = startSlot + PN->getFlags();

  MIB.addOperand(ucOperand::CreateOpcode(VTM::PHI, Slot));
  MIB.addOperand(ucOperand::CreatePredicate());
  MachineOperand Def = PN->getOperand(0);
  MIB.addOperand(Def);

  for (unsigned i = 1; i != PN->getNumOperands(); i += 2) {
    MachineOperand SrcOp = PN->getOperand(i);
    unsigned SrcReg = SrcOp.getReg();
    MachineOperand SrcBBOp = PN->getOperand(i + 1);
    MachineBasicBlock *SrcBB = SrcBBOp.getMBB();

    // Find the opcode of the ucOp defining the register.
    MachineRegisterInfo::def_iterator DI = MRI->def_begin(SrcReg);
    assert (++MRI->def_begin(SrcReg) == MRI->def_end() && "Not in SSA From!");
    MachineInstr &DefInst = *DI;
    MachineInstr::mop_iterator OI = DefInst.operands_begin() + DI.getOperandNo();
    MachineInstr::mop_iterator OpcodeI = OI;
    while (!cast<ucOperand>(*(--OpcodeI)).isOpcode())
      assert(OI != DefInst.operands_begin() && "Broken ucState!");

    ucOperand &Opcode = cast<ucOperand>(*OpcodeI);
    // If the source value not defined in source BB, no need to try to forward
    // the value.
    if (Opcode.getParent()->getParent() == SrcBB) {
      unsigned DefSlot = Opcode.getPredSlot();
      unsigned EndSlot = VFI->getStartSlotFor(SrcBB)
                         + VFI->getTotalSlotFor(SrcBB);

      // If we reading a value from others BB, make sure we are reading the last
      // value from the predecessor.
      // Or if we are reading a value frome the same BB, We need match the slot
      // for pipeline register, otherwise the end control state of source BB and
      // the first control state of current BB is overlap Because the PHIs read
      // values from previous iteration, adjust the write slot.
      if ((SrcBB == CurBB && DefSlot == Slot)
          || (SrcBB != CurBB && DefSlot == EndSlot)) {
        switch(Opcode.getOpcode()) {
          // Forward the wire value if necessary.
        case VTM::COPY:
          SrcOp = *(OI + 1);
          break;
        case VTM::IMPLICIT_DEF:
          SrcOp.ChangeToImmediate(0);
          break;
        case VTM::PHI: // Do not forward the PHI value.
          break;
        default:
          llvm_unreachable("Unexpected ucOp type!");
        }
      }
    }

    MIB.addOperand(SrcOp).addOperand(SrcBBOp);
  }

  PN->eraseFromParent();
}

bool PHIElimination::runOnMachineBasicBlock(MachineBasicBlock &MBB,
                                             MachineFunction &MF) {
  DEBUG(dbgs() << MBB.getName() << " After Schedule:\n";
        printVMBB(dbgs(), MBB));
  typedef SmallVector<MachineInstr*, 16> IListTy;
  IListTy Worklist;

  MachineBasicBlock::iterator I = MBB.begin();
  while (I->isPHI())
    EliminatePHI(I++);

  DEBUG(dbgs() << MBB.getName() << " After phi eliminated:\n";
        printVMBB(dbgs(), MBB));
  return true;
}
