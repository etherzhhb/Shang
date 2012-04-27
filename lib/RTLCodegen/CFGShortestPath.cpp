// CFGShortestPath.cpp --- find shortest paths in a weighted graph -- C++ --==//
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
// This pass is a graph analysis algorithm for finding shortest paths in a
// weighted graph using Floyd¨CWarshall algorithm.
//
//===----------------------------------------------------------------------===//
#include "CFGShortestPath.h"

#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#define DEBUG_TYPE "FindShortestPath"
#include "llvm/Support/Debug.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include <vector>
#include <map>
using namespace llvm;

bool CFGShortestPath::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  FInfo = MF->getInfo<VFInfo>();
  mapStartSlotToMBBNum();
  InitPath();
  computeShortestPathFloyd();
  return false;
}

// Get the Key to PathVector according to the source and destination index
unsigned CFGShortestPath::getKey(unsigned Def, unsigned Use) {
  unsigned MBBNum = MF->getNumBlockIDs();
  unsigned returnKey = Def * MBBNum + Use;
  return returnKey;
}

// Get distance between the source MBB and the destination MBB.
unsigned &CFGShortestPath::getDistance(unsigned DefIdx, unsigned UseIdx) {
  unsigned Index = getKey(DefIdx, UseIdx);
  return PathVector[Index];
}

// Get distance between Two Slots.
int CFGShortestPath::getSlotDistance(VASTSlot *DefSlot, VASTSlot *UseSlot) {
  int SlotDistance = -1;

  int DefSlotIdx = DefSlot->SlotNum;
  int UseSlotIdx = UseSlot->SlotNum;

  // Trivial case: entry slot has a self loop.
  if (DefSlotIdx == UseSlotIdx && DefSlotIdx == 0) return 1;

  assert(UseSlotIdx != 0 && "Unexpected value used in slot0!");

  int DefSlotStartIdx = DefSlot->ParentIdx;
  unsigned DefMBBNum = getMBBNum(DefSlotStartIdx);

  int UseSlotStartIdx = UseSlot->ParentIdx;
  unsigned UseMBBNum = getMBBNum(UseSlotStartIdx);

  int MBBDistance = getDistance(DefMBBNum, UseMBBNum);

  // When the slots are in the same MBB and the SrcSlotIdx > DstSlotIdx, return
  // compute the distance considering the IISlot.
  if (DefMBBNum != UseMBBNum || DefSlotIdx >= UseSlotIdx) {
    // If the MBBDistance is infinite, then the srcMBB can not reach the dstMBB,
    // Return -1 which is the largest number in unsigned type.
    if(MBBDistance == Infinite) return -1;

    SlotDistance = MBBDistance - (DefSlotIdx - DefSlotStartIdx)
                               + (UseSlotIdx - UseSlotStartIdx);
  } else
    // When the slots are in the same MBB and the SrcSlotIdx < DstSlotIdx, return
    // DstSlotIdx - SrcSlotIdx.
    SlotDistance = UseSlotIdx - DefSlotIdx;

  assert((DefMBBNum != UseMBBNum || SlotDistance <= MBBDistance)
         && "Chain's distance bigger than II!");

  return SlotDistance;
}

// Map the StartSlot to MBBNum.
void CFGShortestPath::mapStartSlotToMBBNum() {
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    unsigned StartSlot = FInfo->getStartSlotFor(MBB);
    unsigned MBBNum = MBB->getNumber();
    StartSlotToMBBNumMap[StartSlot] = MBBNum;
  }
}

// Get MBB number.
unsigned CFGShortestPath::getMBBNum(unsigned SlotStartIdx) const {
  // The start slot of the module.
  if (SlotStartIdx == 0) return 0;

  Slot2BBMapTy::const_iterator at = StartSlotToMBBNumMap.find(SlotStartIdx);
  assert(at != StartSlotToMBBNumMap.end() && "Bad start slot!");
  return at->second;
}

// Initial the Path between two related Machine Basic Block.
void CFGShortestPath::InitPath() {
  unsigned MBBNum = MF->getNumBlockIDs();

  // assign infinite number to the PathVector.
  unsigned PathVectorSize = MBBNum * MBBNum;
  PathVector.assign(PathVectorSize, Infinite);

  // Initial the PathVector with the Path.
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock *DefMBB = I;
    unsigned TolalSlot = FInfo->getTotalSlotFor(DefMBB);

    // Initialize the distance between the direct connected MBBs.
    for (MachineBasicBlock::succ_iterator I = DefMBB->succ_begin(),
         E = DefMBB->succ_end(); I != E; ++I) {
      MachineBasicBlock *UseMBB = *I;
      unsigned &Distance = getDistance(DefMBB, UseMBB);
      // Initialize the distance of self-loop edge to the total delay of the BB,
      // we will fix it later.
      Distance = TolalSlot;
    }
  }
}

// Use the Floyd Algorithm to find the shortest Path between two Machine Basic
// Block.
void CFGShortestPath::computeShortestPathFloyd() {
  unsigned MBBNum = MF->getNumBlockIDs();

  // We use the Floyd algorithm to get the shortest path of two different MBBs.
  for (unsigned Thru = 0; Thru < MBBNum; ++Thru) {
    for (unsigned Def = 0; Def < MBBNum; ++Def) {
      for (unsigned Use = 0; Use < MBBNum; ++Use) {
        unsigned &Src2Dst = getDistance(Def, Use);
        unsigned Src2Thru2Dst = getDistance(Def, Thru) +  getDistance(Thru, Use);
        Src2Dst = std::min(Src2Dst, Src2Thru2Dst);
      }
    }
  }

  // if the DefMBB == UseMBB, get the IISlot and assign it to the PathVector.
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock *DefMBB = I;
    unsigned IISlot = FInfo->getIIFor(DefMBB);
    for (MachineBasicBlock::succ_iterator I = DefMBB->succ_begin(),
         E = DefMBB->succ_end(); I != E; ++I) {
      MachineBasicBlock *UseMBB = *I;
      if (DefMBB != UseMBB) continue;
      getDistance(DefMBB, UseMBB) = IISlot;
    }
  }
}

char CFGShortestPath::ID = 0;
const int CFGShortestPath::Infinite = 100000;

INITIALIZE_PASS_BEGIN(CFGShortestPath, "CFGShortestPath",
                      "CFGShortestPath", false, false)
INITIALIZE_PASS_END(CFGShortestPath, "CFGShortestPath",
                    "CFGShortestPath", false, false)
Pass *llvm::createCFGShortestPathPass() {
  return new CFGShortestPath();
}
