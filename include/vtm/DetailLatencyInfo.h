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
  // Compute the detail ctrlop to ctrlop latency (in cycle ratio) infromation of
  // a given MBB.
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
  static float getLatency(DepLatInfoTy::value_type v) {
    return std::max(v.second.first, v.second.second);
  }

  const static float DeltaLatency;

  MachineRegisterInfo *MRI;

private:
  const bool WaitAllOps;
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
  bool buildDepLatInfo(const MachineInstr *SrcMI, const MachineInstr *DstMI,
    DepLatInfoTy &CurLatInfo, unsigned OperandWidth,
    float OperandDelay);
  // Also remember the operations that do not use by any others operations in
  // the same bb.
  std::set<const MachineInstr*> MIsToWait, MIsToRead;

protected:
  const DepLatInfoTy &addInstrInternal(const MachineInstr *MI,
    bool IgnorePHISrc);

public:
  DetialLatencyInfo();
  // Add the a machine instruction and compute the corresponding latency
  // information, return true if the MI is a control operation, false otherwise.
  const DepLatInfoTy &addInstr(const MachineInstr *MI) {
    return addInstrInternal(MI, false);
  }

  // Build the back-edge latency information of PHIs.
  const DepLatInfoTy &buildPHIBELatInfo(const MachineInstr *MI) {
    assert(MI->isPHI() && "Expect PHI!");
    // Simply add the back-edge dependence information of the which not
    // available at the first scan, but already available now.
    // Because PHINodes not depends on PHINodes in the same BB, we should ignore
    // PHINodes if it appear as an operand.
    return addInstrInternal(MI, true);
  }

  // Get the source register and the corresponding latency to DstMI
  const DepLatInfoTy *getDepLatInfo(const MachineInstr *DstMI) const {
    LatencyMapTy::const_iterator at = LatencyMap.find(DstMI);
    return at == LatencyMap.end() ? 0 : &at->second;
  }

  // All operation must finish before the BB exit, this function build the
  // information about the latency from instruction to the BB exit.
  void buildExitMIInfo(const MachineInstr *ExitMI, DepLatInfoTy &Info);

  // Erase the instructions from exit set.
  void eraseFromWaitSet(const MachineInstr *MI);

  void addDummyLatencyEntry(const MachineInstr *MI, float l = 0.0f) {
    CachedLatencies.insert(std::make_pair(MI, l));
  }

  void resetExitSet() {
    MIsToWait.clear();
    MIsToRead.clear();
  }

  void clearCachedLatencies() { CachedLatencies.clear(); }

  void reset() {
    LatencyMap.clear();
    clearCachedLatencies();
    resetExitSet();
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

  static unsigned getStepsFromEntry(const MachineInstr *DstInstr);

  template<bool IsValDep>
  unsigned getCtrlStepBetween(const MachineInstr *SrcInstr,
                              const MachineInstr *DstInstr) {
    if (!SrcInstr) return getStepsFromEntry(DstInstr);

    if (IsValDep) return ceil(getChainingLatency(SrcInstr, DstInstr));

    return getStepsToFinish(SrcInstr);
  }

  void getAnalysisUsage(AnalysisUsage &AU) const;

  bool runOnMachineFunction(MachineFunction &MF) {
    MRI = &MF.getRegInfo();
    return true;
  }

  void releaseMemory() { reset(); }
};

}
