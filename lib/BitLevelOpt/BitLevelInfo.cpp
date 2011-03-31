//====- BitLevelInfo.cpp - Verilog target machine bit level info -*- C++ -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file implement Verilog target machine-specific bit level information
// analyze pass, this analyze pass will compute the bit width for each register
// and immediate base on the register width, and the result of bitwise operation
// such as bitslice selection.
//
//===----------------------------------------------------------------------===//

#include "vtm/BitLevelInfo.h"
#include "vtm/VInstrInfo.h"
#include "vtm/VFuncInfo.h"
#include "vtm/Passes.h"
#include "vtm/VTM.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#define DEBUG_TYPE "vtm-bli"
#include "llvm/Support/Debug.h"

using namespace llvm;

INITIALIZE_PASS(BitLevelInfo, "vtm-bli", "Verilog Target Machine - "
                "Bit Level Information Analysis", false, true)

Pass *llvm::createBitLevelInfoPass() {
  return new BitLevelInfo();
}

char BitLevelInfo::ID = 0;

BitLevelInfo::BitLevelInfo() : MachineFunctionPass(ID) {
  initializeBitLevelInfoPass(*PassRegistry::getPassRegistry());
}

void BitLevelInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.setPreservesAll();
}

bool BitLevelInfo::runOnMachineFunction(MachineFunction &MF) {
  MRI = &MF.getRegInfo();

  // Anotate the bit width information to target flag.
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();
       BI != BE; ++BI)
    for (MachineBasicBlock::iterator I = BI->begin(), E =  BI->end();
         I != E; ++I) {
      MachineInstr &Instr = *I;
      switch (Instr.getOpcode()) {
      default: break;
      // Always fix the bit width of VOpToState.
      case VTM::VOpToState:
        updateBitWidth(Instr.getOperand(0), 1);
        // Fall through
      case VTM::COPY:     case VTM::PHI:
        // Fall through
      case VTM::Control:  case VTM::Datapath:
        continue;
      }

      BitWidthOperand BWO(Instr);
      for (unsigned i = 0, e = Instr.getNumOperands() - 1; i < e; ++i) {
        MachineOperand &MO = Instr.getOperand(i);
        if (!MO.isReg() && !MO.isImm()) continue;

        bool Changed = updateBitWidth(MO, BWO.getBitWidth(i));
        if (MO.isReg() && MO.isDef() && Changed)
          propagateBitWidth(MO);
      }
    }

  DEBUG(dbgs() << "---------- After bit width annotation.\n");
  DEBUG(MF.dump());
  return false;
}

void BitLevelInfo::computeBitWidth(MachineInstr *Instr) {
  SmallVector<MachineOperand*, 2> Defs;
  switch (Instr->getOpcode()) {
  // Copy instruction may inserted during register allocation, in this case
  // its operand will not come with any bit width information.
  case VTM::COPY: {
    MachineOperand &Result = Instr->getOperand(0),
                   &Operand = Instr->getOperand(1);
    assert (Operand.isReg()
            && TargetRegisterInfo::isVirtualRegister(Operand.getReg())
            && "Not support Physics register yet!");
    
    unsigned Width = getBitWidthInternal(Operand);

    if (updateBitWidth(Result, Width))
      Defs.push_back(&Result);
    break;
  }
  case VTM::PHI: {
    MachineOperand &Result = Instr->getOperand(0);
    if (updateBitWidth(Result, computePHI(Instr)))
      Defs.push_back(&Result);
    break;
  }
  // We can not check these instructions at this moment.
  case VTM::Control:
  case VTM::Datapath:
  // Not necessary to compute the bitwitdh information of these instructions.
  case VTM::VOpArg:
  case VTM::VOpMemAccess:
  // These intructions do not define anything.
  case VTM::VOpToState:
  case VTM::EndState:
  case VTM::VOpRet:
  case VTM::VOpRetVal:
    // Do nothing for these instructions as they do not define anything.
    return;
  // Bit level instructions, the most importance instructions
  // for bit level information.
  case VTM::VOpBitSlice:
    // BitSlice's width never change.
    //assert(!updateBitWidth(Instr->getOperand(0), computeBitSliceWidth(Instr))
    //       && "BitSlice's width changed!");
    return;
  case VTM::VOpBitRepeat:
    assert(!updateBitWidth(Instr->getOperand(0), computeBitRepeatWidth(Instr))
           && "BitRepeat's width changed!");
    return;
  case VTM::VOpBitCat:
    assert(!updateBitWidth(Instr->getOperand(0), computeBitCatWidth(Instr))
           && "BitCat's width changed!");
    return;
  // Operations with Fixed bit width.
  case VTM::VOpROr:
  case VTM::VOpRAnd:
  case VTM::VOpRXor: {
    MachineOperand &Result = Instr->getOperand(0);
    if (updateBitWidth(Result, 1))
      Defs.push_back(&Result);
    break;
  }
  // Leaves.
  // FIXME
  // Dirty Hack: this appear in bugpoint.
  // case VTM::IMPLICIT_DEF:
  // Other Instructions.
  case VTM::VOpAdd: {
    MachineOperand &Carry = Instr->getOperand(1);
    if (updateBitWidth(Carry, 1))
      Defs.push_back(&Carry);
    
    MachineOperand &Result = Instr->getOperand(0);
    unsigned Width = computeByOpWithSameWidth(Instr->operands_begin() + 2,
                                              Instr->operands_begin() + 4);
    if (updateBitWidth(Result, Width))
      Defs.push_back(&Result);
    break;
  }
  case VTM::VOpMult:
  case VTM::VOpOr:
  case VTM::VOpAnd:
  case VTM::VOpXor: {
    MachineOperand &Result = Instr->getOperand(0);
    unsigned Width = computeByOpWithSameWidth(Instr->operands_begin() + 1,
                                              Instr->operands_begin() + 3);
    if (updateBitWidth(Result, Width))
      Defs.push_back(&Result);
    break;
  }
  // The bitwidth determinate by its first operand.
  case VTM::VOpSetRI:
  case VTM::VOpNot:
  case VTM::VOpSRA:
  case VTM::VOpSRL:
  case VTM::VOpSHL: {
    MachineOperand &Result = Instr->getOperand(0);
    unsigned Width = getBitWidth(Instr->getOperand(1));
    if (updateBitWidth(Result, Width))
      Defs.push_back(&Result);
    break;
  }
  default: assert(0 && "Unknown instruction!");
  }

  // FIXME: Implement a iterative bit witdh update approach.
  // Update bit widths.
  while (!Defs.empty())
    propagateBitWidth(*Defs.pop_back_val());
}

void BitLevelInfo::propagateBitWidth(MachineOperand &MO) {
  assert(MO.isReg() && "Wrong operand type!");

  unsigned char BitWidth = getBitWidth(MO);
  assert(BitWidth && "Bit width not available!");

  for (MachineRegisterInfo::reg_iterator I = MRI->reg_begin(MO.getReg()),
       E = MRI->reg_end(); I != E; ++I) {
    MachineOperand &MO = I.getOperand();

    // Propagate bit width information through the def-use chain.
    if (updateBitWidth(MO, BitWidth) && (I->isCopy() || I->isPHI()))
      computeBitWidth(&*I);
  }
}

// TODO: Verify the bit width information.
void BitLevelInfo::verifyAnalysis() const {

}

unsigned BitLevelInfo::getBitWidth(MachineOperand &MO) const  {
  unsigned BitWidth = getBitWidthInternal(MO);
  assert(BitWidth && "Bit width information not available!");
  return BitWidth;
}

unsigned BitLevelInfo::getBitWidth(unsigned R) const {
  unsigned Size = 0;
  for (MachineRegisterInfo::def_iterator I = MRI->def_begin(R),
       E = MRI->def_end(); I != E; ++I) {
    unsigned S = getBitWidthInternal(I.getOperand());
    if (S == 0) { // Get the bit width from source operand.
      assert(I->isCopy() && "Can not get register bit width!");
      S = getBitWidth(I->getOperand(1).getReg());
    }

    Size = std::max(Size, S);
  }

  return Size;
}
