//====--- BitLevelInfo.h - Verilog target machine bit level info -*- C++ -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file declares Verilog target machine-specific bit level information
// analyze pass, this analyze pass will compute the bit width for each register
// and immediate base on the register width, and the result of bitwise operation
// such as bitslice selection.
//
// TODO:
//  1. Support analysis the bit width of ucOp.
//  2. Use list driven bitwidth computation algorithm to allow bitwidth
//     information propagates through the use-def chain.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_BIT_LEVEL_INFO
#define VTM_BIT_LEVEL_INFO

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"

namespace llvm {

class BitLevelInfo : public MachineFunctionPass {
  unsigned getBitWidthInternal(MachineOperand &MO) const {
    assert((MO.isImm() || MO.isReg()) && "Unsupported operand type!");
    return MO.getTargetFlags();
  }

  void computeBitWidth(MachineInstr *Instr);
  void propagateBitWidth(MachineOperand &MO);
  
  unsigned computeBitSliceWidth(MachineInstr *BitSlice) {
    unsigned UB = BitSlice->getOperand(2).getImm(),
      LB = BitSlice->getOperand(3).getImm();
    return UB - LB;
  }

  unsigned computeBitRepeatWidth(MachineInstr *BitRepeat) const {
      unsigned EltWidth = getBitWidth(BitRepeat->getOperand(1)),
        Times = BitRepeat->getOperand(2).getImm();
      return EltWidth * Times;
  }

  unsigned computeBitCatWidth(MachineInstr *BitCat) const {
      unsigned BitWidth = 0;
      for (MachineInstr::mop_iterator I = BitCat->operands_begin() + 1,
        E = BitCat->operands_end(); I != E; ++I)
        BitWidth += getBitWidth(*I);

      return BitWidth;
  }

  unsigned computeByOpWithSameWidth(MachineInstr::mop_iterator I,
                                    MachineInstr::mop_iterator E) {
    assert(I != E && "The range is empty!");
    unsigned BitWidth = getBitWidthInternal(*I);
    while (++I != E)
      if (unsigned Width = getBitWidthInternal(*I)) {
        assert ((BitWidth == 0 || BitWidth == Width)
                 && "Bit width of PHINode not match!");
        BitWidth = Width;
      }

    return BitWidth;
  }

  unsigned computePHI(MachineInstr *PN) {
    assert(PN->isPHI() && "Wrong Instruction type!");
    unsigned BitWidth = 0;
    
    for (unsigned i = 1; i != PN->getNumOperands(); i += 2)
      if (unsigned Width = getBitWidthInternal(PN->getOperand(i))) {
        assert ((BitWidth == 0 || BitWidth == Width)
                 && "Bit width of PHINode not match!");
        BitWidth = Width;
      }

    return BitWidth;
  }

  MachineRegisterInfo *MRI;

public:
  static char ID;
  BitLevelInfo();

  void getAnalysisUsage(AnalysisUsage &AU) const;
  bool runOnMachineFunction(MachineFunction &MF);

  void verifyAnalysis() const;

  unsigned getBitWidth(unsigned R) const;

  unsigned getBitWidth(MachineOperand &MO) const;

  bool updateBitWidth(MachineOperand &MO, unsigned char BitWidth) {
    unsigned char OldBitWidth = getBitWidthInternal(MO);
    assert((OldBitWidth == 0 || OldBitWidth >= BitWidth)
            && "Bit width not convergent!");
    assert(BitWidth && "Invalid bit width!");
    MO.setTargetFlags(BitWidth);

    return OldBitWidth != BitWidth;
  }
};
}

#endif
