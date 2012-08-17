//====- DetialLatencyInfo.h - Perpare for RTL code generation -*- C++ -*-===//
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
// Compute the detail ctrlop to ctrlop latency (in cycle ratio) information.
//
//===----------------------------------------------------------------------===//
#include "VInstrInfo.h"
#include "Passes.h"
#include <float.h>
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/PassSupport.h"
#include "llvm/Support/CommandLine.h"

namespace llvm{

struct InstPtrTy : public PointerUnion<MachineInstr*, MachineBasicBlock*> {
  typedef PointerUnion<MachineInstr*, MachineBasicBlock*> Base;

  bool isMI() const { return is<MachineInstr*>(); }
  bool isMBB() const { return is<MachineBasicBlock*>(); }

  MachineInstr* dyn_cast_mi() const {
    return dyn_cast<MachineInstr*>();
  }

  MachineBasicBlock* dyn_cast_mbb() const {
    return dyn_cast<MachineBasicBlock*>();
  }

  MachineInstr* get_mi() const {
    return get<MachineInstr*>();
  }

  MachineBasicBlock* get_mbb() const {
    return get<MachineBasicBlock*>();
  }

  operator MachineInstr*() const {
    return dyn_cast_mi();
  }

  MachineInstr* operator ->() const {
    return get_mi();
  }

  operator MachineBasicBlock*() const {
    return dyn_cast_mbb();
  }

  MachineBasicBlock *getParent() const {
    return isMBB() ? get_mbb() : get_mi()->getParent();
  }

  bool operator< (InstPtrTy RHS) const {
    return getOpaqueValue() < RHS.getOpaqueValue();
  }

  // Dirty Hack: Accept constant pointer and perform const_cast. We should get
  // rid of this.
  /*implicit*/ InstPtrTy(const MachineInstr *MI)
    : Base(const_cast<MachineInstr*>(MI)) {
      assert(MI && "Unexpected null pointer!");
  }

  /*implicit*/ InstPtrTy(const MachineBasicBlock *MBB)
    : Base(const_cast<MachineBasicBlock*>(MBB)) {
      assert(MBB && "Unexpected null pointer!");
  }

  InstPtrTy() : Base(static_cast<MachineInstr*>(0)) {}
};

class DetialLatencyInfo : public MachineFunctionPass {
public:
  static char ID;
  // The latency of MSB and LSB from a particular operation to the current
  // operation.
  typedef std::map<InstPtrTy, std::pair<float, float> > DepLatInfoTy;
  static float getMaxLatency(DepLatInfoTy::value_type v) {
    return std::max(v.second.first, v.second.second);
  }

  static float getMinLatency(DepLatInfoTy::value_type v) {
    return std::min(v.second.first, v.second.second);
  }

  const static float DeltaLatency;

  MachineRegisterInfo *MRI;

private:
  // Cache the computational delay for every instruction.
  typedef std::map<const MachineInstr*, float> CachedLatMapTy;
  CachedLatMapTy CachedLatencies;
  float computeLatencyFor(const MachineInstr *MI);
  CachedLatMapTy::mapped_type
  getCachedLatencyResult(const MachineInstr *MI) const {
    CachedLatMapTy::const_iterator at = CachedLatencies.find(MI);
    assert(at != CachedLatencies.end() && "Latency not calculated!");
    return at->second;
  }

  static bool propagateFromLSB2MSB(unsigned Opcode);

  // The latency from all register source through the datapath to a given
  // wire/register define by a datapath/control op
  typedef std::map<const MachineInstr*, DepLatInfoTy> LatencyMapTy;

  LatencyMapTy LatencyMap;
  // Add the latency information from SrcMI to CurLatInfo.
  template<bool IsCtrlDep>
  bool buildDepLatInfo(const MachineInstr *SrcMI, DepLatInfoTy &CurLatInfo,
                       unsigned OperandWidth, float OperandDelay,
                       unsigned DstOpcode = VTM::INSTRUCTION_LIST_END);

  template<bool IsCtrlDep>
  DepLatInfoTy::mapped_type getLatencyToDst(const MachineInstr *SrcMI,
                                            unsigned DstOpcode,
                                            unsigned UB, unsigned LB);
protected:
  const DepLatInfoTy &addInstrInternal(const MachineInstr *MI,
                                       DepLatInfoTy &CurLatInfo);

public:
  DetialLatencyInfo();

  // Get the source register and the corresponding latency to DstMI
  const DepLatInfoTy *getDepLatInfo(const MachineInstr *DstMI) const {
    LatencyMapTy::const_iterator at = LatencyMap.find(DstMI);
    return at == LatencyMap.end() ? 0 : &at->second;
  }

  // Get the latencies from the control-path dependences to the copy operation
  // which copy MI's result to register.
  void buildLatenciesToCopy(const MachineInstr *MI, DepLatInfoTy &Info) {
    buildDepLatInfo<false>(MI, Info, 0, 0.0, VTM::VOpReadFU);
  }

  typedef const std::set<const MachineInstr*> MISetTy;
  // All operation must finish before the BB exit, this function build the
  // information about the latency from instruction to the BB exit.
  void buildExitMIInfo(const MachineInstr *ExitMI, DepLatInfoTy &Info,
                       MISetTy &MIsToWait, MISetTy &MIsToRead);

  void addDummyLatencyEntry(const MachineInstr *MI, float l = 0.0f) {
    CachedLatencies.insert(std::make_pair(MI, l));
  }

  void clearCachedLatencies() { CachedLatencies.clear(); }

  void reset() {
    LatencyMap.clear();
    clearCachedLatencies();
  }

  float getMaxLatency(const MachineInstr *MI) const {
    return getCachedLatencyResult(MI);
  }

  unsigned getStepsToFinish(const MachineInstr *MI) const {
    return ceil(getMaxLatency(MI));
  }

  // Return the edge latency between SrcInstr and DstInstr considering chaining
  // effect.
  float getChainingLatency(const MachineInstr *SrcInstr,
                           const MachineInstr *DstInstr) const;

  static unsigned getStepsFromEntry(const MachineInstr *DstInstr) {
    // Any Instruction can schedule to the same slot with the BBEntry.
    return 0;
  }

  template<bool IsValDep>
  unsigned getCtrlStepBetween(const MachineInstr *SrcInstr,
                              const MachineInstr *DstInstr) {
    if (!SrcInstr) return getStepsFromEntry(DstInstr);

    if (IsValDep) return ceil(getChainingLatency(SrcInstr, DstInstr));

    return getStepsToFinish(SrcInstr);
  }

  void getAnalysisUsage(AnalysisUsage &AU) const;

  bool runOnMachineFunction(MachineFunction &MF);

  void releaseMemory() { reset(); }
};

}
