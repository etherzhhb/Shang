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
#include "vtm/Passes.h"
#include "vtm/VFuncInfo.h"
#include "vtm/VTM.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

using namespace llvm;

INITIALIZE_PASS(BitLevelInfo, "vtm-bli", "Verilog Target Machine - "
                "Bit Level Information Analysis", false, true);

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
  std::vector<MachineInstr*> WorkStack;

  VFI = MF.getInfo<VFuncInfo>();
  MRI = &MF.getRegInfo();

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();
       BI != BE; ++BI)
    for (MachineBasicBlock::iterator I = BI->begin(), E =  BI->end();
         I != E; ++I)
      computeBitWidth(I);

  return false;
}

unsigned BitLevelInfo::computeWidthByRC(MachineOperand &MO) {
  unsigned Reg = MO.getReg();

  const TargetRegisterClass *RC = MRI->getRegClass(Reg);
  // Just return the register bitwidth.
  return RC->vt_begin()->getSizeInBits();
}

void BitLevelInfo::computeBitWidth(MachineInstr *Instr) {
  SmallVector<MachineOperand*, 2> Defs;
  switch (Instr->getOpcode()) {
  default:
    assert(0 && "Unknown instruction!");
  // We can not check these instructions at this moment.
  case VTM::Control:
  case VTM::Datapath:
  case VTM::Terminator:
  // These intructions do not define anything.
  case VTM::VOpToState:
  case VTM::VOpRet:
  case VTM::VOpRetVal:
    // Do nothing for these instructions as they do not define anything.
    return;
  // Bit level instructions, the most importance instructions
  // for bit level information.
  case VTM::VOpBitSlice: {
    MachineOperand &Result = Instr->getOperand(0);
    if (updateBitWidth(Result, computeBitSliceWidth(Instr)))
      Defs.push_back(&Result);
    break;
  }
  case VTM::VOpBitRepeat: {
    MachineOperand &Result = Instr->getOperand(0);
    if (updateBitWidth(Result, computeBitRepeatWidth(Instr)))
      Defs.push_back(&Result);
    break;
  }
  case VTM::VOpBitCat: {
    MachineOperand &Result = Instr->getOperand(0);
    if (updateBitWidth(Result, computeBitCatWidth(Instr)))
      Defs.push_back(&Result);
    break;
  }
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
  case VTM::VOpArg: {
    MachineOperand &Result = Instr->getOperand(0);
    if (updateBitWidth(Result, computeWidthByRC(Result)))
      Defs.push_back(&Result);
    break;
  }
  // FIXME: The bit width of result of memory access is deteminated by
  // constraints.
  case VTM::VOpMemAccess: {
    MachineOperand &Result = Instr->getOperand(0);
    if (updateBitWidth(Result, computeWidthByRC(Result)))
      Defs.push_back(&Result);
    break;
  }
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
  case VTM::COPY:
  case VTM::VOpNot:
  case VTM::VOpSRA:
  case VTM::VOpSHL: {
    MachineOperand &Result = Instr->getOperand(0);
    unsigned Width = getBitWidth(Instr->getOperand(1));
    if (updateBitWidth(Result, Width))
      Defs.push_back(&Result);
    break;
  }
  }

  // Update bitwidths.
  while (!Defs.empty())
    updateUsesBitwidth(*Defs.pop_back_val());
}

void BitLevelInfo::updateUsesBitwidth(MachineOperand &MO) {
  assert(MO.isReg() && "Wrong operand type!");

  unsigned char BitWidth = getBitWidth(MO);
  assert(BitWidth && "Bit width not available!");

  for (MachineRegisterInfo::reg_iterator I = MRI->reg_begin(MO.getReg()),
       E = MRI->reg_end(); I != E; ++I)
    updateBitWidth(I.getOperand(), BitWidth);
}

// TODO: Verify the bitwidth information.
void BitLevelInfo::verifyAnalysis() const {

}
