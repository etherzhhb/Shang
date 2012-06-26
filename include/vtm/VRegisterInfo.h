//===------ VRegisterInfo.h - VTM Register Information -----------*- C++ -*-===//
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
// This file contains the VTM implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#ifndef VINREGISTERINFO_H
#define VINREGISTERINFO_H

#include "llvm/Target/TargetRegisterInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

#define  GET_REGINFO_HEADER
#include "VerilogBackendGenRegisterInfo.inc"

#include <map>
#include <set>

namespace llvm {
class TargetInstrInfo;
class Type;
class TargetData;
class TargetLowering;
class MachineRegisterInfo;
class VSubtarget;
class VInstrInfo;

class VRegisterInfo : public VTMGenRegisterInfo {
  static const unsigned MaxPhyRegs = 4096;
public:
  struct PhyRegInfo {
    union {
      struct {
        unsigned RegClassId : 4;
        unsigned AliasSetId : 14;
        unsigned UB         : 7;
        unsigned LB         : 7;
      } SData;
      unsigned IData;
    } UData;

    PhyRegInfo(unsigned ClassId, unsigned AliasSet, unsigned U, unsigned L) {
      UData.SData.RegClassId = ClassId;
      assert(UData.SData.RegClassId == ClassId && "ClassId overflow!");
      UData.SData.AliasSetId = AliasSet;
      assert(UData.SData.AliasSetId == AliasSet && "AliasSet overflow!");
      UData.SData.UB = U;
      assert(UData.SData.UB == U && "U overflow!");
      UData.SData.LB = L;
      assert(UData.SData.LB == L && "L overflow!");
    }

    /*implicit*/ PhyRegInfo(unsigned D) { UData.IData = D; }
    operator unsigned() const {return UData.IData; }

    unsigned getRegClass() const { return UData.SData.RegClassId; }
    unsigned getUB() const { return UData.SData.UB; }
    unsigned getLB() const { return UData.SData.LB; }
    unsigned getBitWidth() const { return getUB() - getLB(); }
    unsigned getAliasSetId() const { return UData.SData.AliasSetId; }
    unsigned getParentRegister() const { return UData.SData.AliasSetId; }
    bool isTopLevelReg(unsigned RegNum) const {
      return RegNum == getParentRegister();
    }
  };
private:
  typedef std::vector<unsigned> PhyRegVec;
  typedef std::set<unsigned> PhyRegSet;
  typedef std::map<unsigned, PhyRegSet> PhyRegAlaisMap;
  PhyRegVec PhyRegs;
  PhyRegAlaisMap PhyRegAliasInfo;

public:
  VRegisterInfo();

  virtual const TargetRegisterClass *
  getPointerRegClass(const MachineFunction &MF, unsigned Kind=0) const;

  // Functions that should be generated by table gen.
  virtual int getDwarfRegNumFull(unsigned RegNum, unsigned Flavour) const;
  virtual bool needsStackRealignment(const MachineFunction &) const;
  unsigned getSubReg(unsigned RegNo, unsigned Index) const;
  unsigned getSubRegIndex(unsigned RegNo, unsigned SubRegNo) const {
    assert(0 && "Unexpected calling getSubRegIndex!");
    return 0;
  }
  unsigned composeSubRegIndices(unsigned IdxA, unsigned IdxB) const {
    assert(0 && "Unexpected calling composeSubRegIndices!");
    return IdxB;
  }
  
  virtual const TargetRegisterClass *
    getMatchingSuperRegClass(const TargetRegisterClass *A,
                             const TargetRegisterClass *B, unsigned Idx) const {
    assert(0 && "Unexpected calling getMatchingSuperRegClass!");
    return 0;
  }

  virtual const TargetRegisterClass *
    getSubClassWithSubReg(const TargetRegisterClass *RC, unsigned Idx) const{
    assert(0 && "Unexpected calling getSubClassWithSubReg!");
    return 0;
  }

  /// Code Generation virtual methods...
  const uint16_t *getCalleeSavedRegs(const MachineFunction *MF = 0) const;
  virtual bool requiresRegisterScavenging(const MachineFunction &MF) const {
    return true;
  }

  BitVector getReservedRegs(const MachineFunction &MF) const;

  const TargetRegisterClass *getPointerRegClass(unsigned Kind = 0) const;

  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, RegScavenger *RS = NULL) const;

  void emitPrologue(MachineFunction &MF) const;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const;

  unsigned getFrameRegister(const MachineFunction &MF) const;

  // Exception handling queries.
  unsigned getEHExceptionRegister() const;
  unsigned getEHHandlerRegister() const;

  int getDwarfRegNum(unsigned RegNum, bool isEH) const;

  static bool IsWire(unsigned RegNo, const MachineRegisterInfo *MRI);

  // Physics register allocate information.
  void resetPhyRegAllocation();

  unsigned allocatePhyReg(unsigned RegClassID, unsigned Width);
  unsigned getSubRegOf(unsigned Parent, unsigned UB, unsigned LB);
  unsigned allocateFN(unsigned FNClassID, unsigned Width = 0);

  PhyRegInfo getPhyRegInfo(unsigned RegNum) const;

  unsigned num_phyreg() const { return PhyRegs.size(); }

  static const TargetRegisterClass *getRepRegisterClass(unsigned OpCode);
};

} // end namespace llvm

#endif
