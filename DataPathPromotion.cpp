//====- DataPathPromotion.cpp - Perpare for RTL code generation -*- C++ -*-===//
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
// This file implement the DataPathPromotion pass, promote the operation in 
// DataPath from ChainedOpc to ControlOpc according to the restraint.
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/VInstrInfo.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"

using namespace llvm;
namespace {
struct DataPathPromotion : public MachineFunctionPass {
  // Mapping the PHI number to accutally register.
  std::map<unsigned, unsigned> PHIsMap;
  static char ID;

  DataPathPromotion() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) {
    MachineRegisterInfo &MRI = MF.getRegInfo();

    for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
      for (MachineBasicBlock::instr_iterator II = I->instr_begin(),
           IE = I->instr_end(); II != IE; ++II) {
        MachineInstr *MI = II;   
          switch(MI->getOpcode()) {
          default: 
            break;
          case VTM::VOpAdd_c: {
            unsigned Size = VInstrInfo::getBitWidth(MI->getOperand(0));
            if (!getFUDesc(VFUs::AddSub)->shouldBeChained(Size-1)) 
              MI->setDesc(VInstrInfo::getDesc(VTM::VOpAdd));            
            break;
          }
          case VTM::VOpICmp_c: {
            unsigned Size = VInstrInfo::getBitWidth(MI->getOperand(0));
            if (!getFUDesc(VFUs::ICmp)->shouldBeChained(Size)) 
              MI->setDesc(VInstrInfo::getDesc(VTM::VOpICmp));           
            break;
          }
          case VTM::VOpSHL_c: {
            unsigned Size = VInstrInfo::getBitWidth(MI->getOperand(0));
            if (!getFUDesc(VFUs::Shift)->shouldBeChained(Size)) 
              MI->setDesc(VInstrInfo::getDesc(VTM::VOpSHL));           
            break;
          }
          case VTM::VOpSRA_c: {
            unsigned Size = VInstrInfo::getBitWidth(MI->getOperand(0));
            if (!getFUDesc(VFUs::Shift)->shouldBeChained(Size)) 
              MI->setDesc(VInstrInfo::getDesc(VTM::VOpSRA));           
            break;
          }
          case VTM::VOpSRL_c: {
            unsigned Size = VInstrInfo::getBitWidth(MI->getOperand(0));
            if (!getFUDesc(VFUs::Shift)->shouldBeChained(Size)) 
              MI->setDesc(VInstrInfo::getDesc(VTM::VOpSRL));
            break;
          }
          case VTM::VOpMultLoHi_c:             
          case VTM::VOpMult_c:  {
            unsigned Size = VInstrInfo::getBitWidth(MI->getOperand(0));
            if (!getFUDesc(VFUs::Mult)->shouldBeChained(Size)) 
              MI->setDesc(VInstrInfo::getDesc(VTM::VOpMult));
            break;  
          }
      }
    }    
    return true;
  }

  const char *getPassName() const {
    return "Data Path Promotion Pass";
  }
};
}

char DataPathPromotion::ID = 0;

Pass *llvm::createDataPathPromotionPass() {
  return new DataPathPromotion();
}
