//===------------- HWAtom.h - Translate LLVM IR to HWAtom  -------*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
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
// This file also define the HWAtomInfo class, which construction the HWAtom
// represent from LLVM IR.
//
//===----------------------------------------------------------------------===//
//

#ifndef VBE_HARDWARE_ATOMINFO_H
#define VBE_HARDWARE_ATOMINFO_H

#include "llvm/Support/MathExtras.h"
#include "llvm/Target/TargetData.h"

#include "llvm/CodeGen/MachineFunctionPass.h"

#include "esly/HWAtom.h"

namespace llvm {
class MemDepInfo;

class MachineLoopInfo;
class LiveVariables;

/// @brief Hardware atom construction pass
///
class HWAtomInfo : public MachineFunctionPass {
  // Allocator
  BumpPtrAllocator HWAtomAllocator;

  HWAtom *getExitRoot(const MachineInstr *MI);

  HWAOpFU *createOpFU(const MachineInstr *MI, unsigned ID = 0);

  FSMState *createState(const MachineBasicBlock *MBB);

  typedef DenseMap<const MachineInstr*, HWAtom*> AtomMapType;
  AtomMapType InstToHWAtoms;

  typedef DenseMap<const MachineBasicBlock*, FSMState*> StateMapType;
  StateMapType MachBBToStates;

  HWValDep *getValDepEdge(HWAtom *Src, bool isSigned = false,
                          enum HWValDep::ValDepTypes T = HWValDep::Normal) {
    return new (HWAtomAllocator) HWValDep(Src, isSigned, T);
  }

  HWCtrlDep *getCtrlDepEdge(HWAtom *Src) {
    return new (HWAtomAllocator) HWCtrlDep(Src);
  }

  HWMemDep *getMemDepEdge(HWAtom *Src, bool isBackEdge,
                          enum HWMemDep::MemDepTypes DepType,
                          unsigned Diff);

  void addOperandDeps(const MachineInstr *MI, SmallVectorImpl<HWEdge*> &Deps);

  // TODO: other export edge.
  void addPhiExportEdges(BasicBlock &BB, SmallVectorImpl<HWEdge*> &Deps);

  void clear();

  void addMemDepEdges(std::vector<HWAOpFU*> &MemOps, BasicBlock &BB);

  bool haveSelfLoop(const MachineBasicBlock *MBB);
  HWADelay *addLoopPredBackEdge(const MachineBasicBlock *MBB);

  // The loop Info
  MachineLoopInfo *LI;
  LiveVariables *LiveVars;
  const VSubtarget *VTarget;
  MachineRegisterInfo *MRI;
  // Total states
  unsigned short totalCycle;
  unsigned short InstIdx;
  // Analysis for dependence analyzes.
  MemDepInfo *MDA;
public:
  /// @name FunctionPass interface
  //{
  static char ID;
  HWAtomInfo() : MachineFunctionPass(ID), LI(0), LiveVars(0), VTarget(0), MRI(0),
    totalCycle(1), InstIdx(0), MDA(0) {}

  HWAtom *buildAtom(const MachineInstr *MI);

  bool runOnMachineFunction(MachineFunction &MF);
  void releaseMemory();
  void getAnalysisUsage(AnalysisUsage &AU) const;
  void print(raw_ostream &O, const Module *M) const;
  //}

  HWAtom *getAtomFor(const MachineInstr *MI) const {
    AtomMapType::const_iterator At = InstToHWAtoms.find(MI);

    if(At != InstToHWAtoms.end())
      return  At->second;

    return 0;
  }

  FSMState *getStateFor(const MachineBasicBlock *MBB) const {
    StateMapType::const_iterator At = MachBBToStates.find(MBB);
    assert(At != MachBBToStates.end() && "State can not be found!");
    return  At->second;    
  }

  HWADelay *getDelay(HWAtom *Src, unsigned Delay);

  unsigned getTotalCycle() const {
    return totalCycle;
  }

  void setTotalCycle(unsigned Cyc) {
    totalCycle = Cyc;
  }

  unsigned getTotalCycleBitWidth() const {
    return Log2_32_Ceil(totalCycle);
  }

  void incTotalCycle() { ++totalCycle; }
};
} // end namespace
#endif
