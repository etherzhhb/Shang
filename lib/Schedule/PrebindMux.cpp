//=- PrebindMux.cpp- Allocate Multiplexer for Prebound Function units - C++ -=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Allocate Multiplexer for Prebound Function units.
//
//===----------------------------------------------------------------------===//
#include "vtm/MicroState.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Passes.h"

#include "llvm/Target/TargetMachine.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#define DEBUG_TYPE "vtm-prebind-mux"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
struct PrebindMux : public MachineFunctionPass {
  static char ID;
  unsigned MaxMuxSize;
  enum {
    NO_MUX_NUM = 0,
    FIRST_MUXC_NUM = 1
  };

  typedef DenseMap<ucOperand, unsigned, ucOperandValueTrait> OpSet;

  // The (FUID, InPortNum) pair.
  typedef std::pair<unsigned, unsigned> FUPortTy;

  typedef DenseMap<FUPortTy, OpSet> PortFanInMapTy;

  typedef DenseMap<FUPortTy, unsigned> PortBitwidthMapTy;

  typedef DenseMap<unsigned, unsigned> MuxSizeMapTy;

  typedef DenseSet<unsigned> RegSet;

  PortFanInMapTy FanInInfo;
  PortBitwidthMapTy PortBitwidthInfo;
  RegSet MuxRegs;
  MuxSizeMapTy MuxSizeInfo;
  unsigned MuxCounter;
  PrebindMux()
    : MachineFunctionPass(ID), MaxMuxSize(0), MuxCounter(FIRST_MUXC_NUM) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    // This pass do not change the CFG.
    AU.setPreservesCFG();
  }

  bool doInitialization(Module &) {
    // Compute the proper mux size that fix within 1 cycle.
    while (MaxMuxSize < VFUs::MaxAllowedMuxSize)
      if (VFUs::getMuxLatency(++MaxMuxSize) > 0.5)
        break;

    return false;
  }

  bool runOnMachineFunction(MachineFunction &MF);

  void releaseMemory() {
    MuxSizeInfo.clear();
    FanInInfo.clear();
    PortBitwidthInfo.clear();
    MuxRegs.clear();
    MuxCounter = FIRST_MUXC_NUM;
  }

  void collectFanIns(MachineFunction &MF);
  void allocateBalanceMux();
  void insertDistrubedMuxOp(MachineFunction &MF);

  const char *getPassName() const { return "Bind Mux Ports before Scheduling";}
};
}

char PrebindMux::ID = 0;

Pass *llvm::createPrebindMuxPass() {
  return new PrebindMux();
}

bool PrebindMux::runOnMachineFunction(MachineFunction &MF) {
  // Prevent we build multiplexer for NO_PHYREG.
  MuxRegs.insert(0);

  for (;;) {
    collectFanIns(MF);

    // Nothing to do.
    if (FanInInfo.empty()) break;

    // TODO: Assign priority to the fan-ins, and build un-balance mux.
    allocateBalanceMux();

    insertDistrubedMuxOp(MF);

    FanInInfo.clear();
  }

  return true;
}

void PrebindMux::collectFanIns(MachineFunction &MF){
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock *MBB = BI;

    for (MachineBasicBlock::iterator II = MBB->begin(), IE = MBB->end();
         II != IE; ++II) {
      MachineInstr *Inst = II;
      
      // DisableFUs do not make sense.
      if (Inst->getOpcode() == VTM::VOpDisableFU) continue;

      FuncUnitId Id = VInstrInfo::getPreboundFUId(Inst);
      if (Id.isTrivial()) continue;

      const MCInstrDesc &TID = Inst->getDesc();

      for (unsigned i = 0, e = Inst->getNumOperands(); i != e; ++i) {
        ucOperand &MO = cast<ucOperand>(Inst->getOperand(i));

        // Only handle fan-ins
        if (MO.isReg() && (MO.isDef() || MuxRegs.count(MO.getReg())))
          continue;

        // Predicate not handle by mux.
        if (i < TID.NumOperands && TID.OpInfo[i].isPredicate()) continue;

        // Remember the fan-in for the port.
        FanInInfo[std::make_pair(Id.getData(), i)].FindAndConstruct(MO);
      }
    }
  }

  // Remove the ports with mux that can fit into a single cycle.
  for (PortFanInMapTy::iterator I = FanInInfo.begin(), E = FanInInfo.end();
       I != E; /*++I*/) {
    PortFanInMapTy::iterator at = I;
    ++I;

    if (at->second.size() <= MaxMuxSize)
      FanInInfo.erase(at);
  }
}

void PrebindMux::allocateBalanceMux() {
  // For each port.
  for (PortFanInMapTy::iterator I = FanInInfo.begin(), E = FanInInfo.end();
       I != E; ++I) {
    OpSet &FIs = I->second;
    DEBUG(FuncUnitId(I->first.first).print(dbgs());
          dbgs() << "@" << I->first.second << ": #Fan-in "
                 << FIs.size() << '\n';);

    unsigned FICounter = 0;
    unsigned NumMuxFanins = (FIs.size() + MaxMuxSize) / MaxMuxSize;
    assert(NumMuxFanins > 1 && "Small fan-in set not eliminated?");
    for (OpSet::iterator OI = FIs.begin(), OE = FIs.end(); OI != OE; ++OI) {
      // Allocate and assign the mux number.
      OI->second = MuxCounter;
      // Increase the allocted mux size.
      ++MuxSizeInfo[MuxCounter];
      // Remember the port bitwidth, there maybe fanins with different width.
      PortBitwidthInfo[I->first] = std::max(PortBitwidthInfo[I->first],
                                            OI->first.getBitWidth());
      // If the fanin number exceed the maximum mux size, allocate a new mux.
      if (++FICounter >= NumMuxFanins) {
        ++MuxCounter;
        FICounter = 0;
      }
    }
    // Allocate a new mux for other ports.
    ++MuxCounter;
  }
}

void PrebindMux::insertDistrubedMuxOp(MachineFunction &MF) {
  const TargetInstrInfo *TII = MF.getTarget().getInstrInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock *MBB = BI;

    for (MachineBasicBlock::iterator II = MBB->begin(), IE = MBB->end();
         II != IE; ++II) {
      MachineInstr *Inst = II;

      FuncUnitId Id = VInstrInfo::getPreboundFUId(Inst);
      if (Id.isTrivial()) continue;

      const MCInstrDesc &TID = Inst->getDesc();
      MachineOperand *PredMO = VInstrInfo::getPredOperand(Inst);

      for (unsigned i = 0, e = Inst->getNumOperands(); i != e; ++i) {
        ucOperand &MO = cast<ucOperand>(Inst->getOperand(i));

        // Only handle fan-ins
        if (MO.isReg() && MO.isDef()) continue;

        // Predicate not handle by mux.
        if (i < TID.NumOperands && TID.OpInfo[i].isPredicate()) continue;

        // Not a valid fan-in.
        if (MO.isReg() && MO.getReg() == 0) continue;

        FUPortTy Port = std::make_pair(Id.getData(), i);

        PortFanInMapTy::iterator MuxNumAt = FanInInfo.find(Port);

        // No mux allocated for this fan-in.
        if (MuxNumAt == FanInInfo.end()) continue;
        unsigned MuxNum = MuxNumAt->second.lookup(MO);
        // No mux allocated for this fan-in.
        if (MuxNum == NO_MUX_NUM) continue;

        unsigned BitWidth = PortBitwidthInfo.lookup(Port);
        unsigned MuxSize = MuxSizeInfo.lookup(MuxNum);
        assert(BitWidth && MuxSize && "Bad FU Port bitwidth or Mux size!");

        unsigned NewRegNum = MRI.createVirtualRegister(VTM::RMUXRegisterClass);
        // Remember the register and do not build multiplexer for it again.
        MuxRegs.insert(NewRegNum);
        DEBUG(dbgs() << "Allocated mux register: "
                     << TargetRegisterInfo::virtReg2Index(NewRegNum) << '\n');

        BuildMI(*MBB, Inst, Inst->getDebugLoc(), TII->get((VTM::VOpDstMux)))
          .addOperand(ucOperand::CreateReg(NewRegNum, BitWidth, true))
          .addOperand(MO) // Source value.
          .addOperand(ucOperand::CreateImm(MuxNum, 64)) // MuxNumber.
          .addOperand(ucOperand::CreateImm(MuxSize, 64)) // Mux size.
          .addOperand(*PredMO) // Predicate
          .addOperand(ucOperand::CreateTrace(MBB)); // Trace number.

        // Read the value from mux instead.
        MO.ChangeToRegister(NewRegNum, false);
      }
    }
  }
}
