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

  // Mapping the slot number to the wire read at this slot by phis.
  typedef std::map<unsigned,  RegSet> PHIUseMapTy;
  typedef PHIUseMapTy::iterator PHIUseMapIt;
  PHIUseMapTy PhiUse;

  ForwardWireUsers() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF);

  void buildWireUseMap(MachineFunction &MF);

  void buildPHIUseMap(MachineInstr *PN, MachineBasicBlock *MBB,
                      MachineRegisterInfo &MRI, VFInfo *VFI);

  void buildUseMapForState(MachineInstr *Inst, MachineRegisterInfo &MRI);

  void addUseToMap(unsigned RegNum, SmallVectorImpl<WireMapIt> &WireDefs,
                   MachineRegisterInfo &MRI);

  void forwardWireUses(MachineFunction &MF);
};
}

char ForwardWireUsers::ID = 0;

bool ForwardWireUsers::runOnMachineFunction(MachineFunction &MF) {
  WireUse.clear();
  PhiUse.clear();

  buildWireUseMap(MF);

  forwardWireUses(MF);

  return true;
}

void
ForwardWireUsers::buildWireUseMap(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  VFInfo *VFI = MF.getInfo<VFInfo>();

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock::iterator II = BI->begin(), IE = BI->end();
    while (II->isPHI()) {
      buildPHIUseMap(II, BI, MRI, VFI);
      ++II;
    }

    while (II != IE) {
      if (II->getOpcode() == VTM::Datapath)
        buildUseMapForState(II, MRI);
      ++II;
    }
  }

  DEBUG(
    for (PHIUseMapIt I = PhiUse.begin(), E = PhiUse.end(); I != E; ++I) {
      dbgs() << "Slot" << I->first << " Using ";

      RegSet &RSet = I->second;
      for (RegSet::iterator SI = RSet.begin(), SE = RSet.end(); SI != SE; ++SI)
        dbgs() << TargetRegisterInfo::virtReg2Index(*SI) << ", ";

      dbgs() << '\n';
    }

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
  typedef std::set<unsigned> UseSet;
  UseSet ImpUses, ExpUse;

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI)
    for (MachineBasicBlock::iterator II = BI->begin(), IE = BI->end();
         II != IE; ++II) {
      MachineInstr *Inst = II;
      unsigned OpC = Inst->getOpcode();
      if (OpC != VTM::Datapath && OpC != VTM::Control) continue;

      ImpUses.clear();
      ExpUse.clear();

      unsigned PredSlot = ucState(Inst).getSlot();

      if (OpC == VTM::Control) {
        PHIUseMapIt at = PhiUse.find(PredSlot);
        if (at != PhiUse.end()) {
          RegSet &UseWire = at->second;
          for (RegSet::iterator RI = UseWire.begin(), RE = UseWire.end();
               RI != RE; ++RI) {
            unsigned WireNum = *RI;
            ucState S(MRI.getVRegDef(WireNum));
            set_union(ImpUses, getRegsUseBy(WireNum));
          }
        }
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
      Builder.addOperand(ucOperand::CreateOpcode(VTM::ImpUse, PredSlot));
      // DirtyHack: Add the dummy predicate to the control op.
      // FIXME: Also add predicate operand for implicit use?
      if (OpC == VTM::Control) Builder.addOperand(ucOperand::CreatePredicate());

      for (UseSet::iterator UI = ImpUses.begin(), UE = ImpUses.end();
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

void ForwardWireUsers::buildPHIUseMap(MachineInstr *PN, MachineBasicBlock *MBB,
                                      MachineRegisterInfo &MRI, VFInfo *VFI ) {
  for (unsigned i = 1, e = PN->getNumOperands(); i != e; i += 2) {
    unsigned RegNum = PN->getOperand(i).getReg();

    if (MRI.getRegClass(RegNum) != VTM::WireRegisterClass)
      continue;

    MachineBasicBlock *SrcBB = PN->getOperand(i + 1).getMBB();

    unsigned PHISlot = 0;
    if (SrcBB != MBB)
      // A PHI using a wire from other BB, that means the wire will be read
      // at the end of the src BB.
      PHISlot = VFI->getEndSlotFor(SrcBB);
    else
      // A PHI using a wire from the same BB as the parent of the PHI, we
      // need to get the scheduling information of the PHI.
      PHISlot = abs(VFI->lookupPHISlot(PN).first);

    unsigned StartSlot = VFI->getStartSlotFor(SrcBB),
      II = VFI->getIIFor(SrcBB);
    unsigned ModuloSlot = (PHISlot - StartSlot) % II + StartSlot;
    // Because the copy is run at the next iteration, so translate the slot
    // to base on the first iteration.
    if (ModuloSlot == StartSlot) ModuloSlot = StartSlot + II;

    PhiUse[ModuloSlot].insert(RegNum);
  }
}

Pass *llvm::createForwardWireUsersPass() {
  return new ForwardWireUsers();
}
