//=- PrebindMuxBase.cpp- Allocate Multiplexer for Prebound Function units - C++ -=//
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
#include "MuxPrebinding.h"

#include "vtm/VerilogBackendMCTargetDesc.h"

#include "llvm/Target/TargetMachine.h"

using namespace llvm;

char PrebindMuxBase::ID = 0;

Pass *llvm::createBasicPrebindMuxPass() {
  return new PrebindMuxBase();
}

const char *PrebindMuxBase::getPassName() const { return "Bind Mux Ports before Scheduling";}

void PrebindMuxBase::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  //To get each BB's Frequency
  AU.addRequired<MachineBlockFrequencyInfo>();
  // This pass do not change the CFG.
  AU.setPreservesCFG();
}

bool PrebindMuxBase::doInitialization(Module &) {
  // Compute the proper mux size that fix within 1 cycle.
  while (MaxMuxSize < VFUs::MaxAllowedMuxSize)
    if (VFUs::getMuxLatency(++MaxMuxSize) > 0.5)
      break;
  return false;
}

void PrebindMuxBase::releaseMemory() {
    MuxSizeInfo.clear();
    FanInInfo.clear();
    PortBitwidthInfo.clear();
    MuxRegs.clear();
    MuxCounter = FIRST_MUXC_NUM;
  }

bool PrebindMuxBase::runOnMachineFunction(MachineFunction &MF) {
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

void PrebindMuxBase::collectFanIns(MachineFunction &MF){
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end();BI != BE;++BI) {
    MachineBasicBlock *MBB = BI;

    for (MachineBasicBlock::iterator II = MBB->begin(), IE = MBB->end();
      II != IE; ++II) {
        MachineInstr *Inst = II;

        FuncUnitId Id = VInstrInfo::getPreboundFUId(Inst);
        if (Id.isTrivial()) continue;

        const MCInstrDesc &TID = Inst->getDesc();

        for (unsigned i = 0, e = Inst->getNumOperands(); i != e; ++i) {
          MachineOperand &MO = Inst->getOperand(i);

          // Only handle fan-ins
          if (MO.isReg() && (MO.isDef() || MuxRegs.count(MO.getReg())))
            continue;

          // Predicate not handle by mux.
          if (i < TID.NumOperands && TID.OpInfo[i].isPredicate()) continue;
          std::pair<unsigned,unsigned> InstInfoPair = std::make_pair(Id.getData(), i);
          // Remember the fan-in for the port.
          FanInInfo[std::make_pair(Id.getData(), i)].FindAndConstruct(MO);
          getFreq(TmpFanInInfo,MBB,InstInfoPair,MO);
        }
     }
  }
  rmPortInTmpFanIn(TmpFanInInfo,MaxMuxSize);
  // Remove the ports with mux that can fit into a single cycle.
  for (PortFanInMapTy::iterator I = FanInInfo.begin(), E = FanInInfo.end();
    I != E; /*++I*/) {
      PortFanInMapTy::iterator at = I;
      ++I;

      if (at->second.size() <= MaxMuxSize)
        FanInInfo.erase(at);
  }
}

void PrebindMuxBase::allocateBalanceMux() {
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
                                              VInstrInfo::getBitWidth(OI->first));
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

void PrebindMuxBase::insertDistrubedMuxOp(MachineFunction &MF) {
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
        MachineOperand &MO = Inst->getOperand(i);

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
          .addOperand(VInstrInfo::CreateReg(NewRegNum, BitWidth, true))
          .addOperand(MO) // Source value.
          .addOperand(VInstrInfo::CreateImm(MuxNum, 64)) // MuxNumber.
          .addOperand(VInstrInfo::CreateImm(MuxSize, 64)) // Mux size.
          .addOperand(*PredMO) // Predicate
          .addOperand(*VInstrInfo::getTraceOperand(Inst)); // Trace number.

        // Read the value from mux instead.
        MO.ChangeToRegister(NewRegNum, false);
      }
    }
  }
}
