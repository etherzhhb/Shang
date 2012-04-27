//===----- BundleTokens.h - Tokens for operation in a FSM state  -*- C++ -*-===//
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
//
//
//===----------------------------------------------------------------------===//


#ifndef BUNDLE_TOKENS_H
#define BUNDLE_TOKENS_H

#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/FUInfo.h"
#include "vtm/VInstrInfo.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBundle.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/STLExtras.h"

namespace llvm {
class MCInstrDesc;
class ucOpIterator;
class ucState;
class ucOp;
struct ucOpExpressionTrait;

//uc Operand
class ucOperand : public MachineOperand {
public:
  /*implicit*/ ucOperand(const MachineOperand &O) : MachineOperand(O) {}
  ucOperand() : MachineOperand(MachineOperand::CreateReg(0, false)) {}

  static bool classof(const MachineOperand *) { return true; }


  struct Mapper {
    typedef ucOperand &result_type;

    ucOperand &operator()(MachineOperand &Op) const {
      return cast<ucOperand>(Op);
    }
  };
};
}

#endif
