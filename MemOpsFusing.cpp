//===-- MemOpsFusing - Fuse MemOps to Boost Speed Performance  --*- C++ -*-===//
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
// This file implement the MemOpsFusing Pass, which fully utilize the bandwidth
// of memory bus to boost the speed performance of the design.
//
//===----------------------------------------------------------------------===//
#include "vtm/Passes.h"
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Utilities.h"

#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/Statistic.h"
#define DEBUG_TYPE "vtm-memop-fusing"
#include "llvm/Support/Debug.h"

using namespace llvm;

STATISTIC(MemOpFused, "Number of memory operations fused");
namespace {
struct MemOpsFusing : public MachineFunctionPass {
  static char ID;
  ScalarEvolution *SE;
  AliasAnalysis *AA;
  MachineRegisterInfo *MRI;
  const TargetInstrInfo *TII;

  MemOpsFusing() : MachineFunctionPass(ID), SE(0), AA(0), MRI(0), TII(0) {}

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<ScalarEvolution>();
    AU.addPreserved<ScalarEvolution>();
    AU.addRequired<AliasAnalysis>();
    AU.addPreserved<AliasAnalysis>();
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF) {
    bool changed = false;
    SE = &getAnalysis<ScalarEvolution>();
    AA = &getAnalysis<AliasAnalysis>();
    MRI = &MF.getRegInfo();
    TII = MF.getTarget().getInstrInfo();

    for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
      // We may scan the same BB more than once to fuse all compatible pairs.
      while (runOnMachineBasicBlock(*I))
        changed = true;

    MF.verify(this, "After Memory operations fusion.");

    return changed;
  }

  bool runOnMachineBasicBlock(MachineBasicBlock &MBB);
  bool tryToFuseMemOp(MachineBasicBlock::instr_iterator I,
                      MachineBasicBlock &MBB);

  typedef SmallPtrSet<MachineInstr*, 8> InstSetTy;
  bool canBeFused(MachineInstr *LHS, MachineInstr *RHS);
  bool hasMemDependency(MachineInstr *DstMI, MachineInstr *SrcMI);
  bool trackUsesOfSrc(InstSetTy &UseSet, MachineInstr *Src, MachineInstr *Dst);

  bool isIdenticalMemTrans(MachineInstr *LHS, MachineInstr *RHS) {
    if (LHS->getOpcode() != VTM::VOpMemTrans) return false;
    if (RHS->getOpcode() != VTM::VOpMemTrans) return false;

    for (unsigned i = 1, e = VInstrInfo::mayStore(LHS) ? 5 : 4; i < e; ++i)
      if (!LHS->getOperand(i).isIdenticalTo(RHS->getOperand(i)))
        return false;

    return true;
  }

  void moveUsesAfter(MachineInstr *MergeFrom, MachineInstr *MergeTo,
                     InstSetTy &UseSet);
  void fuseMachineInstr(MachineInstr *From, MachineInstr *To);
};
}

char MemOpsFusing::ID = 0;

bool MemOpsFusing::runOnMachineBasicBlock(MachineBasicBlock &MBB) {
  bool Changed = false;

  // 1. Collect the fusing candidates.
  typedef MachineBasicBlock::instr_iterator instr_it;
  for (instr_it I = MBB.instr_begin(), E = MBB.instr_end(); I != E; /*++I*/) {
    MachineInstr *MI = I++;
    // Only fuse the accesses via memory bus
    if (MI->getOpcode() != VTM::VOpMemTrans)
      continue;

    Changed |= tryToFuseMemOp(MI, MBB);
  }

  return Changed;
}

bool MemOpsFusing::tryToFuseMemOp(MachineBasicBlock::instr_iterator It,
                                  MachineBasicBlock &MBB) {
  MachineInstr *MI = It;
  InstSetTy UseSet;

  typedef MachineBasicBlock::instr_iterator instr_it;
  for (instr_it I = llvm::next(It), E = MBB.instr_end(); I != E; ++I) {
    MachineInstr *LaterMI = I;
    // If there is a dependence between MI and LaterMI, then we cannot fuse
    // them together.
    if (trackUsesOfSrc(UseSet, MI, LaterMI)) continue;

    // Only fuse VOpMemTranses together.
    if (LaterMI->getOpcode() != VTM::VOpMemTrans) continue;    

    // Fuse the two instruction together if they can be fused.
    if (canBeFused(MI, LaterMI)) {
      // Move all users of MI after LaterMI, then merge MI into LaterMI.
      moveUsesAfter(MI, LaterMI, UseSet);
      fuseMachineInstr(MI, LaterMI);
      MI->eraseFromParent();
      return true;
    }
  }

  return false;
}

bool MemOpsFusing::hasMemDependency(MachineInstr *DstMI, MachineInstr *SrcMI) {
  unsigned Opcode = DstMI->getOpcode();
  bool ForceDstDep =  Opcode == VTM::VOpInternalCall;
  bool IsDstMemTrans = Opcode == VTM::VOpMemTrans;
  IsDstMemTrans |= Opcode == VTM::VOpBRAMTrans;

  if (!IsDstMemTrans && !ForceDstDep) return false;

  bool DstMayStore = VInstrInfo::mayStore(DstMI);
  MachineMemOperand *DstMO = IsDstMemTrans ? *DstMI->memoperands_begin() : 0;
  // Do not add loop to dependent graph.
  if (SrcMI == DstMI) return false;

  unsigned SrcOpcode = SrcMI->getOpcode();
  if (SrcOpcode != VTM::VOpMemTrans && SrcOpcode != VTM::VOpBRAMTrans
      && SrcOpcode != VTM::VOpInternalCall)
    return false;

  if (VInstrInfo::isPredicateMutex(SrcMI, DstMI)) return false;

  // Handle force dependency.
  if (ForceDstDep || SrcOpcode == VTM::VOpInternalCall)
    return true;

  bool SrcMayStore = VInstrInfo::mayStore(SrcMI);

  // Ignore RAR dependency.
  if (!SrcMayStore && ! DstMayStore) return false;

  MachineMemOperand *SrcMO = *SrcMI->memoperands_begin();

  // Is DstMI depends on SrcMI?
  return isMachineMemOperandAlias(SrcMO, DstMO, AA, SE);
}

bool MemOpsFusing::trackUsesOfSrc(InstSetTy &UseSet, MachineInstr *Src,
                                  MachineInstr *Dst) {
  assert(!Src->isPHI() && !Dst->isPHI() && "Unexpected PHI!");

  if (UseSet.count(Dst))
    return true;

  // Check the memory dependence, note that we can move the identical accesses
  // across each other.
  if (hasMemDependency(Dst, Src) && !isIdenticalMemTrans(Dst, Src)) {
    // There is a memory dependency between Src and Dst.
    UseSet.insert(Dst);
    return true;
  }

  typedef InstSetTy::iterator iterator;
  for (iterator I = UseSet.begin(), E = UseSet.end(); I != E; ++I) {
    MachineInstr *SrcUser = *I;
    if (hasMemDependency(Dst, SrcUser)) {
      // There is a memory dependency between SrcUser and Dst, so we cannot
      // move SrcUser after Dst, and we need to model this as a use
      // relationship.
      UseSet.insert(Dst);
      return true;
    }      
  }

  // Iterate from use to define.
  for (unsigned i = 0, e = Dst->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = Dst->getOperand(i);

    // Only care about a use register.
    if (!MO.isReg() || MO.isDef() || MO.getReg() == 0)
      continue;

    unsigned SrcReg = MO.getReg();
    MachineInstr *DefMI = MRI->getVRegDef(SrcReg);
    if (DefMI->getParent() != Src->getParent() || DefMI == Dst || DefMI->isPHI())
      continue;

    if (DefMI == Src || UseSet.count(DefMI)) {
      // Src is (indirectly) used by Dst.
      UseSet.insert(Dst);
      return true;
    }
  }

  return false;
}

bool MemOpsFusing::canBeFused(MachineInstr *LHS, MachineInstr *RHS) {
  MachineMemOperand *LHSAddr = *LHS->memoperands_begin(),
                    *RHSAddr = *RHS->memoperands_begin();

  // Only fuse the memory accesses with the same access type.
  if (LHSAddr->isStore() != RHSAddr->isStore()) return false;

  const SCEVConstant *DeltaSCEV
    = dyn_cast<SCEVConstant>(getAddressDeltaSCEV(RHSAddr, LHSAddr, SE));

  // Cannot fuse two memory access with unknown distance.
  if (!DeltaSCEV) return false;

  int64_t Delta = DeltaSCEV->getValue()->getSExtValue();
  // Make sure LHS is in the lower address.
  if (Delta < 0) {
    Delta = -Delta;
    std::swap(LHS, RHS);
    std::swap(LHSAddr, RHSAddr);
  }

  uint64_t FusedWidth = std::max(LHSAddr->getSize(), Delta + RHSAddr->getSize());
  // Do not generate unaligned memory access.
  if (FusedWidth > LHSAddr->getAlignment()) return false;

  if (LHSAddr->isStore()) {
    // Cannot store with irregular byteenable at the moment.
    if (!isPowerOf2_64(FusedWidth)) return false;

    // For the stores, we must make sure the higher address is just next to
    // the lower address.
    if (Delta && uint64_t(Delta) != LHSAddr->getSize()) return false;

    // LHS and RHS have the same address.
    if (Delta == 0 &&
      // Need to check if the two access are writing the same data, and writing
      // the same size.
        (!LHS->getOperand(2).isIdenticalTo(RHS->getOperand(2))
          || LHSAddr->getSize() != RHSAddr->getSize()))
        return false;
  }

  uint64_t BusWidth = getFUDesc<VFUMemBus>()->getDataWidth() / 8;

  // Don't exceed the width of data port of MemBus.
  if (FusedWidth > BusWidth) return false;

  return true;
}

void MemOpsFusing::moveUsesAfter(MachineInstr *MergeFrom, MachineInstr *MergeTo,
                                 InstSetTy &UseSet) {
  DEBUG(dbgs() << "Going to merge:\n" << *MergeFrom << "into\n" << *MergeTo);
  MachineBasicBlock *MBB = MergeFrom->getParent();

  typedef MachineBasicBlock::instr_iterator instr_it;
  instr_it L = MergeFrom, InsertPos = MergeTo;

  // Skip MergeFromMI.
  ++L;
  // Insert moved instruction after MergeToMI.
  ++InsertPos;

  while (L != instr_it(MergeTo)) {
    assert(L != MBB->instr_end() && "Iterator pass the end of the list!");
    MachineInstr *MIToMove = L++;

    if (!UseSet.count(MIToMove)) continue;

    MIToMove->removeFromParent();
    MBB->insert(InsertPos, MIToMove);
  }
}

void MemOpsFusing::fuseMachineInstr(MachineInstr *From, MachineInstr *To) {
  assert(From->getOpcode() == VTM::VOpMemTrans
         && To->getOpcode() == VTM::VOpMemTrans
         && "Unexpected type of MachineInstr to merge!!");

  // Get the new address, i.e. the lower address which has a bigger
  // alignment.
  unsigned LowerReg = From->getOperand(0).getReg();
  MachineOperand LowerAddr = From->getOperand(1);
  MachineOperand LowerData = From->getOperand(2);
  MachineMemOperand *LowerMemOp = *From->memoperands_begin();
  unsigned LowerByteEn = getBitSlice64(From->getOperand(4).getImm(), 8);
  LowerAddr.clearParent();
  LowerData.clearParent();
  unsigned HigherReg = To->getOperand(0).getReg();
  MachineOperand HigherAddr = To->getOperand(1);
  MachineOperand HigherData = To->getOperand(2);
  MachineMemOperand *HigherMemOp = *To->memoperands_begin();
  unsigned HigherByteEn = getBitSlice64(To->getOperand(4).getImm(), 8);
  HigherAddr.clearParent();
  HigherData.clearParent();
  int64_t Delta = getAddressDelta(HigherMemOp, LowerMemOp, SE);
  // Make sure lower address is actually lower.
  if (Delta < 0) {
    Delta = - Delta;
    std::swap(LowerAddr, HigherAddr);
    std::swap(LowerData, HigherData);
    std::swap(LowerMemOp, HigherMemOp);
    std::swap(LowerReg, HigherReg);
    std::swap(LowerByteEn, HigherByteEn);
  }

  int64_t NewSize = std::max(LowerMemOp->getSize(),
                             Delta + HigherMemOp->getSize());
  NewSize = NextPowerOf2(NewSize - 1);
  MachineBasicBlock *CurMBB = To->getParent();
  MachineFunction *MF = CurMBB->getParent();
  // Get the new address from the lower address.
  MachineMemOperand **NewMO = MF->allocateMemRefsArray(1);
  NewMO[0] = MF->getMachineMemOperand(LowerMemOp, 0, NewSize);
  To->setMemRefs(NewMO, NewMO + 1);

  // Get the Byte enable.
  unsigned ByteEn = getByteEnable(NewSize);
  assert((((LowerByteEn | (HigherByteEn << Delta)) == ByteEn)
         || !NewMO[0]->isStore()) && "New Access writing extra bytes!");
  (void) LowerByteEn;
  (void) HigherByteEn;

  assert((!NewMO[0]->isStore()
          || From->getOperand(3).isIdenticalTo(To->getOperand(3)))
         && "Cannot mixing load and store!");

  // Merge the predicate and get the trace operand.
  MachineOperand *FromPred = VInstrInfo::getPredOperand(From),
                 *ToPred = VInstrInfo::getPredOperand(To);

  // Build the new data to store by concatenating them together.
  if (NewMO[0]->isStore()) {
    unsigned NewReg = MRI->createVirtualRegister(&VTM::DRRegClass);
    unsigned NewDataSizeInBit = VInstrInfo::getBitWidth(LowerData)
                                + VInstrInfo::getBitWidth(HigherData);
    BuildMI(*CurMBB, To, DebugLoc(), VInstrInfo::getDesc(VTM::VOpBitCat))
      .addOperand(VInstrInfo::CreateReg(NewReg, NewDataSizeInBit, true))
      .addOperand(HigherData).addOperand(LowerData)
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());

    LowerData = VInstrInfo::CreateReg(NewReg, NewDataSizeInBit);
  }

  unsigned NewPred =
    VInstrInfo::MergePred(*FromPred, *ToPred, *To->getParent(),
                          To, MRI, TII, VTM::VOpOr).getReg();


  if (!FromPred[1].isIdenticalTo(ToPred[1]))
    VInstrInfo::ResetTrace(To);

  MachineOperand TraceOperand = *VInstrInfo::getTraceOperand(To);
  TraceOperand.clearParent();

  // Refresh the machine operand.
  To->RemoveOperand(6);
  To->RemoveOperand(5);
  To->RemoveOperand(4);
  To->RemoveOperand(3);
  To->RemoveOperand(2);
  To->RemoveOperand(1);

  // Add the new address operand.
  To->addOperand(LowerAddr);
  // Add the data to store.
  To->addOperand(LowerData);
  To->addOperand(VInstrInfo::CreateImm(NewMO[0]->isStore(), 1));
  To->addOperand(VInstrInfo::CreateImm(ByteEn, 8));
  To->addOperand(VInstrInfo::CreatePredicate(NewPred));
  To->addOperand(TraceOperand);

  // Update the result registers.
  if (NewMO[0]->isLoad()) {
    const TargetRegisterClass *RC = MRI->getRegClass(LowerReg);
    // Get the new higher data from the higher part of the result of the fused
    // memory access.
    unsigned NewHigherReg = MRI->createVirtualRegister(RC);
    unsigned HigherRegSizeInBits = HigherMemOp->getSize() * 8;
    unsigned DataSizeInBit = getFUDesc<VFUMemBus>()->getDataWidth() ;
    unsigned LB = Delta * 8;
    unsigned UB = LB + HigherRegSizeInBits;
    // Insert the bitslice after the load.
    MachineBasicBlock::instr_iterator IP = To;
    ++IP;
    BuildMI(*CurMBB, IP, DebugLoc(), VInstrInfo::getDesc(VTM::VOpBitSlice))
      .addOperand(VInstrInfo::CreateReg(NewHigherReg, HigherRegSizeInBits, true))
      .addOperand(VInstrInfo::CreateReg(LowerReg, DataSizeInBit))
      .addOperand(VInstrInfo::CreateImm(UB, 8))
      .addOperand(VInstrInfo::CreateImm(LB, 8))
      .addOperand(VInstrInfo::CreatePredicate())
      .addOperand(VInstrInfo::CreateTrace());

    // Update the result register.
    MRI->replaceRegWith(HigherReg, NewHigherReg);
    To->getOperand(0).ChangeToRegister(LowerReg, true);
  }
  
  ++MemOpFused;
}

Pass *llvm::createMemOpsFusingPass() {
  return new MemOpsFusing();
}
