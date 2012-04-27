//=- PrebindMuxSame.h- Allocate Multiplexer for Prebound Function units - C++ -=//
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
#ifndef PRE_BIND_MUX_H
#define PRE_BIND_MUX_H

#include "vtm/VInstrInfo.h"
#include "vtm/Passes.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/Support/Debug.h"

namespace llvm{

class PrebindMuxBase : public MachineFunctionPass {
public:
  static char ID;
  unsigned MaxMuxSize;
  enum {
    NO_MUX_NUM = 0,
    FIRST_MUXC_NUM = 1
  };

  typedef DenseMap<MachineOperand, unsigned, VMachineOperandValueTrait> OpSet;

  // The (FUID, InPortNum) pair, identify the input port of a pre-bound FU.
  typedef std::pair<unsigned, unsigned> FUPortTy;
  // The remember the fan-in operands set for each input port.
  typedef DenseMap<FUPortTy, OpSet> PortFanInMapTy;

  // The bitwidth information for each input port.
  typedef DenseMap<FUPortTy, unsigned> PortBitwidthMapTy;

  // Remember the fan-in size for each multiplexer.
  typedef DenseMap<unsigned, unsigned> MuxSizeMapTy;

  typedef DenseSet<unsigned> RegSet;

  PortFanInMapTy FanInInfo,TmpFanInInfo;
  PortBitwidthMapTy PortBitwidthInfo;
  RegSet MuxRegs;
  MuxSizeMapTy MuxSizeInfo;
  unsigned MuxCounter;

  PrebindMuxBase() : MachineFunctionPass(ID), MaxMuxSize(0), MuxCounter(1) {}
  PrebindMuxBase(char &ID) : MachineFunctionPass(ID), MaxMuxSize(0), MuxCounter(1) {}
  void getAnalysisUsage(AnalysisUsage &AU) const;
  void releaseMemory();

  bool doInitialization(Module &);

  virtual void getFreq(PortFanInMapTy &TmpFanInInfo,MachineBasicBlock *MBB,
                       std::pair<unsigned, unsigned> InstInfoPair,MachineOperand &MO){}
  virtual void rmPortInTmpFanIn(PortFanInMapTy &TmpFanInInfo,unsigned MaxMuxSize){}

  void collectFanIns(MachineFunction &MF);

  virtual void allocateBalanceMux();

  void insertDistrubedMuxOp(MachineFunction &MF);

  bool runOnMachineFunction(MachineFunction &MF);

  const char *getPassName() const ;
};

} // end namespace

#endif
