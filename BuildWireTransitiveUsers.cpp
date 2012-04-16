//===- BuildWireTransitiveUsers.cpp - Forward the operands use by wire ops-*- C++ -*-=//
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
// This file implement the pass that forward the use operand of wireops so we
// can compute a correct live interval for the use operands of wireops. for
// example if we have:
// a = bitslice b, ...
// ...
// ... = a ...
// and we will add the operand of wireops to the instructions that using its
// results as implicit use:
// a = bitslice b, ...
// ...
// ... = a ..., imp use b
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/MicroState.h"
#include "vtm/VFInfo.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SetOperations.h"
#define DEBUG_TYPE "vtm-fix-machine-code"
#include "llvm/Support/Debug.h"
#include <map>

using namespace llvm;

namespace {
struct BuildWireTransitiveUsers : public MachineFunctionPass {
  static char ID;

  typedef std::set<unsigned> RegSet;
  // Mapping wire number to the register use by this wire.
  typedef std::map<unsigned,  RegSet> WireUseMapTy;
  typedef WireUseMapTy::iterator WireMapIt;
  WireUseMapTy WireUse;
  RegSet &getRegsUseBy(unsigned WireNum) {
    WireUseMapTy::iterator at = WireUse.find(WireNum);
    assert(at != WireUse.end() && "Wire not found!");
    return at->second;
  }

  BuildWireTransitiveUsers() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF);

  void buildWireUseMap(MachineFunction &MF);

  void buildUseMapForDatapath(MachineInstr *Inst, MachineRegisterInfo &MRI);

  void addUseToMap(unsigned RegNum, SmallVectorImpl<WireMapIt> &WireDefs,
                   MachineRegisterInfo &MRI);

  void forwardWireUses(MachineFunction &MF);
  void addUseToBundle(MachineInstr *Bundle, RegSet &ImpUses, RegSet &ExpUse);


  const char *getPassName() const { return "Forward Registers Used by Wires"; }
};
}

char BuildWireTransitiveUsers::ID = 0;

bool BuildWireTransitiveUsers::runOnMachineFunction(MachineFunction &MF) {
  WireUse.clear();

  buildWireUseMap(MF);

  forwardWireUses(MF);

  return true;
}

void BuildWireTransitiveUsers::buildWireUseMap(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    typedef MachineBasicBlock::instr_iterator instr_it;
    bool IsInDataPath = false;
    for (instr_it II = instr_it(&*BI->getFirstNonPHI()), IE = BI->instr_end();
         II != IE; ++II) {
      MachineInstr *MI = II;
      if (MI->getOpcode() == VTM::CtrlStart) IsInDataPath = false;
      else if (MI->getOpcode() == VTM::Datapath) IsInDataPath = true;
      else if (IsInDataPath && MI->isInsideBundle())
        buildUseMapForDatapath(MI, MRI);
    }
  }

  DEBUG(
    for (WireMapIt I = WireUse.begin(), E = WireUse.end(); I != E; ++I) {
      dbgs() << "Wire " << TargetRegisterInfo::virtReg2Index(I->first)
             << " Using ";

      RegSet &RSet = I->second;
      for (RegSet::iterator SI = RSet.begin(), SE = RSet.end(); SI != SE; ++SI)
        dbgs() << TargetRegisterInfo::virtReg2Index(*SI) << ", ";

      dbgs() << '\n';
    }
  );
}

void BuildWireTransitiveUsers::addUseToMap(unsigned RegNum,
                                   SmallVectorImpl<WireMapIt> &WireDefs,
                                   MachineRegisterInfo &MRI) {
  if (MRI.getRegClass(RegNum) != VTM::WireRegisterClass) {
    // Remember the wire operand use this register.
    for (SmallVectorImpl<WireMapIt>::iterator WI = WireDefs.begin(),
         WE = WireDefs.end(); WI != WE; ++WI)
      (*WI)->second.insert(RegNum);
  } else {
    // If a wire operation using a wire, this operation also use the
    // registers used by that wire operation.
    WireMapIt src = WireUse.find(RegNum);
    if (src == WireUse.end()) {
      // Dirty Hack: Build the wire map not if we not visited it yet.
      if (MachineInstr *SrcMI = MRI.getVRegDef(RegNum)) {
        assert(isDatapathBundle(SrcMI) && "Unexpected opcode!");
        buildUseMapForDatapath(SrcMI, MRI);
        src = WireUse.find(RegNum);
      }
    }

    assert(src != WireUse.end() && "Source wire not defined?");
    for (SmallVectorImpl<WireMapIt>::iterator WI = WireDefs.begin(),
         WE = WireDefs.end(); WI != WE; ++WI)
      (*WI)->second.insert(src->second.begin(), src->second.end());
  }
}

void
BuildWireTransitiveUsers::forwardWireUses(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  RegSet ImpUses, ExpUse;

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    typedef MachineBasicBlock::instr_iterator instr_it;
    // F
    for (instr_it II = instr_it(BI->getFirstNonPHI()), IE = BI->instr_end();
         II != IE; ++II) {
      MachineInstr *Inst = II;

      // Flush the forwarded uses to the ctrl end.
      if (Inst->getOpcode() == VTM::CtrlEnd) {
        addUseToBundle(Inst, ImpUses, ExpUse);
        continue;
      }

      // Iterate over the machine operands to build the current implicit use
      // set.
      for (MachineInstr::mop_iterator OI = Inst->operands_begin(),
           OE = Inst->operands_end(); OI != OE; ++OI) {
        MachineOperand &MO = *OI;

        if (!MO.isReg() || MO.isDef() || MO.getReg() == 0) continue;

        MO.setIsKill(false);

        unsigned RegNum = MO.getReg();
        if (MRI.getRegClass(RegNum) != VTM::WireRegisterClass) {
          ExpUse.insert(RegNum);
          continue;
        }

        WireMapIt src = WireUse.find(RegNum);
        if (src == WireUse.end()) {
          assert((MRI.getVRegDef(RegNum)->isImplicitDef()
                  || MRI.getVRegDef(RegNum)->isPHI()) && "Wire not define?");
          continue;
        }

        ImpUses.insert(src->second.begin(), src->second.end());
      }
    }
  }

  DEBUG(MF.dump());
}

void BuildWireTransitiveUsers::addUseToBundle(MachineInstr *Bundle,
                                      RegSet &ImpUses, RegSet &ExpUse) {
  // Do not add the register to implicit use if it is explicit used.
  set_subtract(ImpUses, ExpUse);

  if (!ImpUses.empty()) {
    MachineInstrBuilder Builder(Bundle);
    for (RegSet::iterator UI = ImpUses.begin(), UE = ImpUses.end();
         UI != UE; ++UI)
      Builder.addReg(*UI, RegState::Implicit);

    ImpUses.clear();
  }

  ExpUse.clear();
}

void BuildWireTransitiveUsers::buildUseMapForDatapath(MachineInstr *Inst,
                                              MachineRegisterInfo &MRI) {
  SmallVector<WireMapIt, 2> WireDefs;

  typedef MachineInstr::mop_iterator op_it;
  for (op_it OI = Inst->operands_begin(), OE = Inst->operands_end();
       OI != OE; ++OI) {
    MachineOperand &MO = *OI;

    if (!MO.isReg() || MO.getReg() == 0) continue;

    unsigned RegNum = MO.getReg();
    if (MO.isDef()) {
      assert(MRI.getRegClass(RegNum) == VTM::WireRegisterClass
              && "Datapath defines register?");

      WireMapIt at;
      bool inserted;
      tie(at, inserted) = WireUse.insert(std::make_pair(RegNum, RegSet()));
      WireDefs.push_back(at);
      // Seems that we had already visited this instruction?
      if (!inserted) return;
      // assert(inserted && "Wire already existed!");
    } else {
      assert(!WireDefs.empty() && "Datapath dose not defines wire?");
      addUseToMap(RegNum, WireDefs, MRI);
    }
  }
}

Pass *llvm::createBuildWireTransitiveUsersPass() {
  return new BuildWireTransitiveUsers();
}
