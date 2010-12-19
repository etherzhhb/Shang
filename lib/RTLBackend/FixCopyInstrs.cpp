//====-- FixCopyInstrs.cpp - Fuse the copys into micro state ----*- C++ -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement the FixCopyInstrs pass, which move the copy instructions
// into the microstates.
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "vtm/FUInfo.h"
#include "vtm/MicroState.h"
#include "vtm/BitLevelInfo.h"

#include "llvm/Function.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Target/TargetInstrInfo.h"

#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "fix-regietsr-copy"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
struct FixCopy : public MachineFunctionPass{
  static char ID;
  BitLevelInfo *BLI;
  const TargetInstrInfo *TII;

  FixCopy() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    // Diry hack: Force re-run the bitlevel info.
    AU.addRequired<BitLevelInfo>();
    AU.setPreservesAll();
  }


  void FuseCopyInstr(MachineInstr &Copy, LLVMContext &Context);

  void runOnMachineBasicBlock(MachineBasicBlock &MBB, LLVMContext &Context);

  bool runOnMachineFunction(MachineFunction &MF) {
    BLI = &getAnalysis<BitLevelInfo>();
    TII = MF.getTarget().getInstrInfo();

    LLVMContext &Context = MF.getFunction()->getContext();

    for (MachineFunction::iterator I = MF.begin(), E = MF.end();
         I != E; ++I)
      runOnMachineBasicBlock(*I, Context);

    return false;
  }

};
}

char FixCopy::ID = 0;

Pass *llvm::createFixCopyPass() {
  return new FixCopy();
}

static MachineInstr &findPrevControl(MachineInstr &I) {
  MachineBasicBlock *MBB = I.getParent();
  MachineBasicBlock::iterator It = I;
  assert(I.getNextNode()->getOpcode() == VTM::EndState
         && "Can not handle copy in the middle of the block!");
  do {
    --It;
    if (It->getOpcode() == VTM::Control)
      return *It;
  } while (It != MBB->begin());

  assert(0 && "Can not find prior control!");
  return I;
}

void FixCopy::FuseCopyInstr(MachineInstr &Copy, LLVMContext &Context) {
  ucState Ctrl(findPrevControl(Copy));

  MachineOperand &SrcOp = Copy.getOperand(1),
                 &DstOp = Copy.getOperand(0);

  unsigned SrcWire = 0;

  unsigned SrcReg = SrcOp.getReg(),
           DstReg = DstOp.getReg();

  for (ucState::iterator I = Ctrl.begin(), E = Ctrl.end(); I != E; ++I) {
    ucOp Op = *I;
    for (ucOp::op_iterator MI = Op.op_begin(), ME = Op.op_end();
         MI != ME; ++MI) {
      MachineOperand &MO = *MI;
      if (!MO.isReg()) continue;
      // TODO: Overcome this!
      assert((!MO.isUse() || MO.getReg() != DstReg)
              && "Can not fuse instruction!");
      // Forward the wire value if necessary.
      if (MO.isDef() && MO.getReg() == SrcReg) {
        MachineOperand &SrcOperand = Op.getOperand(1);
        switch (SrcOperand.getType()) {
        case MachineOperand::MO_Metadata: {
          MetaToken ReadWire(SrcOperand.getMetadata());
          assert(ReadWire.isReadWire() && "Bad operand!");
          SrcWire = ReadWire.getWireNum();
          break;
        }
        case MachineOperand::MO_Register: {
          // TODO: Copy the operand.
          assert(0 && "Forwarding register value not support yet!");
          break;
        }
        // TODO: Immediate for SetI operation.
        default:
          assert(0 && "Unknown Operand type!");
          break;
        }
      }
    }
  }

  // Transfer the operands.
  Copy.RemoveOperand(1);
  Copy.RemoveOperand(0);
  MachineInstrBuilder MIB(&*Ctrl);
  // Diry hack: Temporary use the slot of the micro state.
  MDNode *OpCode = MetaToken::createInstr(MetaToken::GeneralSlot,
                                          TII->get(VTM::COPY), Context);
  MIB.addMetadata(OpCode).addOperand(DstOp);
  if (SrcWire)
    MIB.addMetadata(MetaToken::createReadWire(SrcWire, Context));
  else
    MIB.addOperand(SrcOp);

  // Discard the operand.
  Copy.eraseFromParent();
}

void FixCopy::runOnMachineBasicBlock(MachineBasicBlock &MBB,
                                     LLVMContext &Context) {
  DEBUG(dbgs() << MBB.getName() << " After register allocation:\n");
  SmallVector<MachineInstr*, 8> Copys;
  for (MachineBasicBlock::iterator I = MBB.begin(), E = MBB.end();
      I != E; ++I) {
    MachineInstr *Instr = I;
    switch (Instr->getOpcode()) {
    case VTM::Control:
    case VTM::Datapath:
      DEBUG(ucState(*Instr).dump());
      break;
    case VTM::COPY:
      Copys.push_back(Instr);
      // fall through
    case VTM::EndState:
      DEBUG(Instr->dump());
      break;
    default:
      DEBUG(Instr->dump());
      assert(0 && "Unexpected instruction!");
      break;
    }
  }

  while (!Copys.empty())
    FuseCopyInstr(*Copys.pop_back_val(), Context);

  DEBUG(
  dbgs() << MBB.getName() << " After copy fixed:\n";
  for (MachineBasicBlock::iterator I = MBB.begin(), E = MBB.end();
    I != E; ++I) {
      MachineInstr *Instr = I;
      switch (Instr->getOpcode()) {
    case VTM::Control:
    case VTM::Datapath:
      ucState(*Instr).dump();
      break;
    case VTM::EndState:
      Instr->dump();
      break;
    default:
      Instr->dump();
      assert(0 && "Unexpected instruction!");
      break;
      }
  });
}
