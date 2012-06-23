//===- AdjustLIForBundles.cpp - Adjust live intervals for bundles -*- C++ -*-=//
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
// This file implement the pass that bundle the instructions and move all its
// live ranges into the SlotIndex of the bundle start.
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Passes.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#define DEBUG_TYPE "vtm-adjust-li"
#include "llvm/Support/Debug.h"

using namespace llvm;

typedef MachineBasicBlock::instr_iterator inst_it;
typedef std::set<unsigned> RegSet;
// Mapping wire number to the register use by this wire.
typedef std::map<unsigned,  RegSet> WireDepMapTy;
typedef WireDepMapTy::iterator WireMapIt;

namespace {
  struct AdjustLIForBundles : public MachineFunctionPass {
    static char ID;
    LiveIntervals *LIS;
    MachineRegisterInfo *MRI;


    AdjustLIForBundles() : MachineFunctionPass(ID) {
      initializeAdjustLIForBundlesPass(*PassRegistry::getPassRegistry());
      initializeLiveIntervalsPass(*PassRegistry::getPassRegistry());
    }

    void getAnalysisUsage(AnalysisUsage &AU) const {
      // Only update live interval analysis.
      AU.setPreservesAll();
      MachineFunctionPass::getAnalysisUsage(AU);
      //AU.addRequiredID(PHIEliminationID);
      AU.addRequired<LiveIntervals>();
    }

    inst_it handleCtrlBundle(inst_it Begin);

    void moveDefUseIntoBundle(MachineInstr *I, SlotIndex BundleStartIdx,
                              bool setInsideBundle = true) {
      assert(BundleStartIdx.isRegister() && "Expect register slot!");
      // We are dealing with registers, get the register slot.
      SlotIndex CurIdx = LIS->getInstructionIndex(I).getRegSlot();
      typedef MachineInstr::mop_iterator op_it;
      for (op_it OI = I->operands_begin(), OE = I->operands_end(); OI != OE; ++OI){
        MachineOperand &MO = *OI;

        if (!MO.isReg() || MO.getReg() == 0) continue;

        unsigned RegNo = MO.getReg();
        if (MO.isDef()) {
          // Move the define slot of the register to BundleStartIdx.
          LiveInterval &LI = LIS->getInterval(RegNo);

          VNInfo *VN = LI.getVNInfoAt(CurIdx);
          assert(VN && "Cannot find VN at current slot!");
          LiveRange LR(BundleStartIdx, CurIdx, VN);
          LI.addRange(LR);
        } else if (MO.isKill()) {
          // Only need to trim the live range if the register killed here.
          LiveInterval &LI = LIS->getInterval(RegNo);
          VNInfo *VN = LI.getVNInfoBefore(CurIdx);
          assert(VN && "Cannot find VN at current slot!");
          LiveRange LR(BundleStartIdx, CurIdx, VN);
          LI.removeRange(LR, true);
        }      
      }

      if (setInsideBundle) I->setIsInsideBundle();
      LIS->RemoveMachineInstrFromMaps(I);
    }

    inst_it handleDataPathBundle(inst_it Begin);
    RegSet *extendWireUserLITransitively(MachineInstr *MI,
                                         WireDepMapTy &WireUses);

    // Extend the live interval of UseReg by DefReg.
    void extendLI(unsigned DefReg, unsigned UseReg, SlotIndex DefIdx){
      const LiveInterval &DefLI = LIS->getInterval(DefReg);
      LiveInterval &UseLI = LIS->getInterval(UseReg);
      extendLI(DefLI, UseLI, DefIdx);
    }

    void extendLI(const LiveInterval &DefLI, LiveInterval &UseLI,
                  SlotIndex DefIdx) {
      if (DefLI.empty()) {
      //  assert(DefIdx.isRegister() && "Expect register slot!");
      //  LiveRange *StartRange = UseLI.getLiveRangeBefore(DefIdx);
      //  assert(StartRange && "Definition of DefReg cannot reach UseReg?");
      //  VNInfo *VN = UseLI.extendInBlock(StartRange->start, DefIdx);
      //  assert(VN && "Cannot extend live interval!");
      //  (void) VN;
        return;
      }

      
      DEBUG(dbgs() << "\nReg " << UseLI << " used by " << DefLI << '\n');
      // FIXME: Too dirty!
      while (UseLI.getNumValNums() > 1) {
        UseLI.MergeValueNumberInto(UseLI.getValNumInfo(1),
                                   UseLI.getValNumInfo(0));
        //UseLI.removeValNo(UseLI.getValNumInfo(1));
        UseLI.RenumberValues(*LIS);
      }

      UseLI.MergeRangesInAsValue(DefLI, UseLI.getValNumInfo(0));
      DEBUG(dbgs().indent(2) << "extended to " << UseLI << '\n');
    }

    void coalesceAndEliminateCopy(MachineInstr *MI, SlotIndex DefSlot,
                                  bool Swap = false) {
      MachineOperand &SrcMO = MI->getOperand(1);
      LiveInterval *SrcLI = &LIS->getInterval(SrcMO.getReg()),
                   *DstLI = &LIS->getInterval(MI->getOperand(0).getReg());
      assert(SrcMO.isKill() && "Expect source register be killed by copy!");
      // No need to coalesce the idnetical copies.
      if (SrcLI != DstLI) {
        // We need to swap the SrcLI and DstLI if DstLI contains PHIDef/PHIKill.
        if (Swap) std::swap(SrcLI, DstLI);

        // Extend SrcLI by adding all live range of DstLI.
        extendLI(*DstLI, *SrcLI, DefSlot);
        // Replace DstLI by SrcLI.
        MRI->replaceRegWith(DstLI->reg, SrcLI->reg);
      }

      LIS->RemoveMachineInstrFromMaps(MI);
      MI->eraseFromParent();
    }

    void extendAllWireUserLITransitively(MachineFunction &MF);

    void buildBundleAndAdjustLIs(MachineBasicBlock &MBB) {
      inst_it I = MBB.instr_begin(), E = MBB.instr_end();
      MachineInstr *LastCtrlStart = 0;

      while (I != E) {
        MachineInstr *MI = I;
        unsigned Opcode = MI->getOpcode();

        if (Opcode == VTM::EndState) {
          ++I;
          continue;
        }

        if (I->isCopy()) {
          MachineInstr *MI = I;
          ++I;
          SlotIndex LastBundleSlot = LIS->getInstructionIndex(LastCtrlStart);
          LastBundleSlot = LastBundleSlot.getRegSlot();
          moveDefUseIntoBundle(MI, LastBundleSlot, false);
          // Swap the extending the LI of Destinated register of the copy.
          coalesceAndEliminateCopy(MI, LastBundleSlot, true);
          continue;
        }

        LastCtrlStart = I;
        I = handleCtrlBundle(I);
        if (I->getOpcode() != VTM::Datapath) {
          // Reach the end of the basic block
          continue;
        }

        I = handleDataPathBundle(I);
      }
    }

    bool runOnMachineFunction(MachineFunction &MF) {
      LIS = &getAnalysis<LiveIntervals>();
      MRI = &MF.getRegInfo();

      for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
        buildBundleAndAdjustLIs(*I);

      extendAllWireUserLITransitively(MF);
      DEBUG(LIS->dump());
      return true;
    }
  };
}

inst_it AdjustLIForBundles::handleCtrlBundle(inst_it I) {
  assert(I->getOpcode() == VTM::CtrlStart && "Unexpected instruction!");
  MachineInstr *BundleStart = I;
  // We are dealing with registers, get the register slot.
  SlotIndex BundleStartIdx = LIS->getInstructionIndex(BundleStart).getRegSlot();
  bool IsCopy = false;

  while ((++I)->getOpcode() != VTM::CtrlEnd) {
    IsCopy = (I->getOpcode() == VTM::VOpDefPhi);

    moveDefUseIntoBundle(I, BundleStartIdx, !IsCopy);
    if (IsCopy) {
      LiveInterval &PHILI = LIS->getInterval(I->getOperand(1).getReg());
      typedef LiveInterval::iterator range_it;
      for (range_it LI = PHILI.begin(), LE = PHILI.end(); LI != LE; ++LI) {
        LiveRange LR = *LI;
        // Extend the live range with PHI kill to the end of the block.
        if (LR.valno->hasPHIKill()) {
          MachineBasicBlock *MBB = LIS->getMBBFromIndex(LR.start);
          VNInfo *VN = PHILI.extendInBlock(LR.start, LIS->getMBBEndIdx(MBB));
          assert(VN && "Cannot extend live interval!");
          (void) VN;
        }
      }

      MachineInstr *MI = I;
      // Fall back to previous instruction, we are going to delete MI.
      --I;
      coalesceAndEliminateCopy(MI, BundleStartIdx);
    }
  }

  // Also bundle the CtrlEnd.
  I->setIsInsideBundle();
  LIS->RemoveMachineInstrFromMaps(I);
  return llvm::next(I);
}

inst_it AdjustLIForBundles::handleDataPathBundle(inst_it I) {
  assert(I->getOpcode() == VTM::Datapath && "Unexpected instruction!");

  SlotIndex BundleStartIdx = LIS->getInstructionIndex(I).getRegSlot();
  ++I;

  // No need to bundle the datapath instructions.
  while (VInstrInfo::isDatapath(I->getOpcode()) && !I->isTerminator())
    moveDefUseIntoBundle(I++, BundleStartIdx);

  return I;
}

RegSet *
AdjustLIForBundles::extendWireUserLITransitively(MachineInstr *Inst,
                                                 WireDepMapTy &WireDeps){
  unsigned DefRegNo = 0;
  RegSet *UsedByWire= 0;
  SlotIndex DefSlot = LIS->getInstructionIndex(Inst).getRegSlot();
  bool isPipeStage = Inst->getOpcode() == VTM::VOpPipelineStage;

  typedef MachineInstr::mop_iterator op_it;
  for (op_it OI = Inst->operands_begin(), OE = Inst->operands_end();
       OI != OE; ++OI) {
    MachineOperand &MO = *OI;

    if (!MO.isReg() || MO.getReg() == 0) continue;

    unsigned RegNo = MO.getReg();
    if (MO.isDef()) {
      DefRegNo = RegNo;

      WireMapIt at;
      bool inserted;
      tie(at, inserted) = WireDeps.insert(std::make_pair(DefRegNo, RegSet()));
      UsedByWire = &at->second;
      // Is the wire define already visited out of order?
      if (!inserted) return UsedByWire;
    } else {
      assert(DefRegNo && UsedByWire && "Datapath dose not defines wire?");
      unsigned UseRegNo = RegNo;
      // Make sure we had visit and extend the wire dependences.
      if (MRI->getRegClass(UseRegNo) == VTM::WireRegisterClass) {

        WireMapIt src = WireDeps.find(UseRegNo);
        RegSet *UsedByWireDep = 0;
        if (src == WireDeps.end()) {
          // Dirty Hack: Build the wire map not if we not visited it yet.
          if (MachineInstr *SrcMI = MRI->getVRegDef(UseRegNo)) {
            assert(VInstrInfo::isDatapathBundle(SrcMI) && "Unexpected opcode!");
            UsedByWireDep = extendWireUserLITransitively(SrcMI, WireDeps);
          }
        } else
          UsedByWireDep = &src->second;
        
        // Going to extend the live interval of the register which used by
        // the depending wire.
        typedef RegSet::iterator reg_it;
        for (reg_it RI = UsedByWireDep->begin(), RE = UsedByWireDep->end();
             RI != RE; ++RI) {
          unsigned TransitiveUsedReg = *RI;
          assert(MRI->getRegClass(TransitiveUsedReg) != VTM::WireRegisterClass
                 && "Unexpected wire!");
          extendLI(DefRegNo, TransitiveUsedReg, DefSlot);
          UsedByWire->insert(TransitiveUsedReg);
        }
      } else
        // Don't extend the LI of operand of pipe stage, there are supposed
        // to kill the LI of pipelined FU.
        if (!isPipeStage) {
          // Going to extend the live interval of the register which used by this
          // wire.
          extendLI(DefRegNo, UseRegNo, DefSlot);
          UsedByWire->insert(UseRegNo);
        }
    }
  }

  return UsedByWire;
}

void AdjustLIForBundles::extendAllWireUserLITransitively(MachineFunction &MF) {
  WireDepMapTy WireDeps;

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    bool IsInDataPath = false;
    for (inst_it II = BI->instr_begin(), IE = BI->instr_end(); II != IE; ++II) {
      MachineInstr *MI = II;
      if (MI->getOpcode() == VTM::CtrlStart) IsInDataPath = false;
      else if (MI->getOpcode() == VTM::Datapath) IsInDataPath = true;
      else if (IsInDataPath && MI->isInsideBundle())
        extendWireUserLITransitively(MI, WireDeps);
    }
  }
}

char AdjustLIForBundles::ID = 0;
char &llvm::AdjustLIForBundlesID = AdjustLIForBundles::ID;

INITIALIZE_PASS(AdjustLIForBundles, "adjust-li-for-bundle",
  "Adjust Live Intervals For Bundle", false, false)
