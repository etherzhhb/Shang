//===- ForwardWireUsers.cpp - Forward the operands use by wire ops-*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
#include "vtm/VTM.h"
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
struct ForwardWireUsers : public MachineFunctionPass {
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

  ForwardWireUsers() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF);

  void buildWireUseMap(MachineFunction &MF);

  void forwardPHIUse(MachineFunction &MF);

  void buildUseMapForState(MachineInstr *Inst, MachineRegisterInfo &MRI);

  void addUseToMap(unsigned RegNum, SmallVectorImpl<WireMapIt> &WireDefs,
                   MachineRegisterInfo &MRI);

  void forwardWireUses(MachineFunction &MF);
};
}

char ForwardWireUsers::ID = 0;

bool ForwardWireUsers::runOnMachineFunction(MachineFunction &MF) {
  WireUse.clear();

  buildWireUseMap(MF);

  forwardWireUses(MF);

  forwardPHIUse(MF);

  return true;
}

void
ForwardWireUsers::buildWireUseMap(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    for (MachineBasicBlock::iterator II = BI->getFirstNonPHI(), IE = BI->end();
         II != IE; ++II) {
      if (II->getOpcode() == VTM::Datapath)
        buildUseMapForState(II, MRI);
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

void ForwardWireUsers::addUseToMap(unsigned RegNum,
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
        assert(SrcMI->getOpcode() == VTM::Datapath && "Unexpected opcode!");
        buildUseMapForState(SrcMI, MRI);
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
ForwardWireUsers::forwardWireUses(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  RegSet ImpUses, ExpUse;

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI)
    for (MachineBasicBlock::iterator II = BI->begin(), IE = BI->end();
         II != IE; ++II) {
      MachineInstr *Inst = II;
      unsigned OpC = Inst->getOpcode();
      if (OpC != VTM::Datapath && OpC != VTM::Control) continue;

      ImpUses.clear();
      ExpUse.clear();


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
          assert(MRI.getVRegDef(RegNum)->isPHI() && "Wire not define?");
          continue;
        }

        ImpUses.insert(src->second.begin(), src->second.end());
      }

      // Do not add the register to implicit use if it is explicit used.
      set_subtract(ImpUses, ExpUse);

      if (ImpUses.empty()) continue;

      // Flush the current implicit use to the machine code.
      MachineInstrBuilder Builder(Inst);
      unsigned PredSlot = ucState(Inst).getSlot();
      Builder.addOperand(ucOperand::CreateOpcode(VTM::ImpUse, PredSlot));
      // DirtyHack: Add the dummy predicate to the control op.
      // FIXME: Also add predicate operand for implicit use?
      if (OpC == VTM::Control) Builder.addOperand(ucOperand::CreatePredicate());

      for (RegSet::iterator UI = ImpUses.begin(), UE = ImpUses.end();
           UI != UE; ++UI)
        Builder.addReg(*UI, RegState::Implicit);
    }

  DEBUG(
    printVMF(dbgs(), MF);
  );
}

void ForwardWireUsers::buildUseMapForState(MachineInstr *Inst,
                                           MachineRegisterInfo &MRI) {
  SmallVector<WireMapIt, 2> WireDefs;
  ucState S(Inst);

  for (ucState::iterator I = S.begin(), E = S.end(); I != E; ++I) {
    ucOp Op = *I;
    WireDefs.clear();

    for (ucOp::op_iterator OI = Op.op_begin(), OE = Op.op_end(); OI != OE;
         ++OI) {
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
}

void ForwardWireUsers::forwardPHIUse(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  RegSet ReadByPhi;

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock::iterator II = BI->begin();
    // Collect the registers read by PHI.
    while (II->isPHI()) {
      MachineInstr *PN = II;
      ++II;

      for (unsigned i = 1, e = PN->getNumOperands(); i != e; i += 2) {
        unsigned RegNum = PN->getOperand(i).getReg();
        MachineBasicBlock *SrcBB = PN->getOperand(i + 1).getMBB();
        MachineInstr *LastMI = SrcBB->getFirstTerminator();
        assert(LastMI->getOpcode() == VTM::EndState && "Unexpected terminator!");
        // Dirty Hack: Insert the implicit read at the EndState instruction.
        MachineInstrBuilder Builder(LastMI);

        if (MRI.getRegClass(RegNum) != VTM::WireRegisterClass) {
          if (!LastMI->readsVirtualRegister(RegNum))
            Builder.addReg(RegNum, RegState::Implicit);
          ReadByPhi.insert(RegNum);
        } else {
          // Skip the implicit defines because it do not read any register.
          if (MRI.getVRegDef(RegNum)->isImplicitDef())
            continue;

          RegSet &ReadByWire = getRegsUseBy(RegNum);
          for (RegSet::iterator RI = ReadByWire.begin(), RE = ReadByWire.end();
               RI != RE; ++RI) {
            unsigned R = *RI;
            if (!LastMI->readsVirtualRegister(R))
              Builder.addReg(R, RegState::Implicit);
            ReadByPhi.insert(R);
          }
        }
      }
    }

    // Nothing to read.
    if (ReadByPhi.empty()) continue;

    // Skip all implicit defines.
    while (II->isImplicitDef()) ++II;

    // The register/wire read by PHI will be read at the first slot of the MBB.
    assert(II != BI->end() && "Empty block?");
    MachineInstr *MI = II;
    assert(MI->getOpcode() == VTM::Control && "First MI not control?");
    MachineInstrBuilder Builder(MI);
    unsigned PredSlot = ucState(MI).getSlot();
    Builder.addOperand(ucOperand::CreateOpcode(VTM::ImpUse, PredSlot));
    // PHIs are always execute.
    Builder.addOperand(ucOperand::CreatePredicate());
    for (RegSet::iterator UI = ReadByPhi.begin(), UE = ReadByPhi.end();UI != UE;
         ++UI)
      Builder.addReg(*UI, RegState::Implicit);

    ReadByPhi.clear();
  }
}

Pass *llvm::createForwardWireUsersPass() {
  return new ForwardWireUsers();
}
