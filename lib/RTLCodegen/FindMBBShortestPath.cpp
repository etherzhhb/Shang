// FindMBBShortestPath.cpp - find shortest paths in a weighted graph - C++ -==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass is a graph analysis algorithm for finding shortest paths in a 
// weighted graph using Floyd¨CWarshall algorithm.
//
//===----------------------------------------------------------------------===//
#include "vtm/FindMBBShortestPath.h"

#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#define DEBUG_TYPE "FindShortestPath"
#include "llvm/Support/Debug.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include <vector>
#include <map>
using namespace llvm;

bool FindShortestPath::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  FInfo = MF->getInfo<VFInfo>();
  mapStartSlotToMBBNum();
  InitPath();
  Floyd();
  return false;
}

// Get the Key to PathVector according to the source and destination index
unsigned FindShortestPath::getKey(unsigned src, unsigned dst) {
  unsigned MBBNum = MF->getNumBlockIDs();
  unsigned returnKey = src * MBBNum + dst;
  return returnKey;
}

// Get distance between the source MBB and the destination MBB.
unsigned &FindShortestPath::getDistance(unsigned Src, unsigned Dst) {
  unsigned Index = getKey(Src, Dst);
  return PathVector[Index];
}

// Get distance between Two Slots.
int FindShortestPath::getSlotDistance(VASTSlot *SrcSlot,
                                           VASTSlot *DstSlot) {
  signed SlotDistance;

  unsigned SrcSlotStartIdx = SrcSlot->getParentIdx();
  unsigned SrcSlotIdx = SrcSlot->getSlotNum();
  unsigned SrcMBBNum = getMBBNum(SrcSlotStartIdx);

  unsigned DstSlotStartIdx = DstSlot->getParentIdx();
  unsigned DstSlotIdx = DstSlot->getSlotNum();
  unsigned DstMBBNum = getMBBNum(DstSlotStartIdx);

  unsigned MBBDistance = getDistance(SrcMBBNum, DstMBBNum);

  // If the MBBDistance is infinite, then the srcMBB can not reach the dstMBB,
  // Return -1 which is the largest number in unsigned type.
  if(MBBDistance == infinite){
    return -1;
  }

  // when the slots are in different MBB. return the SlotDistance.
  // SlotDistance = MBBDistance - (SrcSlotIdx - SrcSlotStartIdx)
  //                            + (DstSlotIdx - DstSlotStartIdx)
  if (SrcMBBNum != DstMBBNum) {
    MBBDistance -= (SrcSlotIdx - SrcSlotStartIdx);
    assert(MBBDistance > 0 && "MBBDistance <= 0 !!!");

    SlotDistance = MBBDistance + (DstSlotIdx - DstSlotStartIdx);
    assert(SlotDistance > 0 && "SlotDistance <= 0 !!!");

    return SlotDistance;
  }

  // when the slots are in the same MBB and the SrcSlotIdx > DstSlotIdx, return
  // IISlot.
  if (SrcSlotIdx > DstSlotIdx) {
    SlotDistance = MBBDistance;
    return (unsigned)SlotDistance;
  }

  // when the slots are in the same MBB and the SrcSlotIdx <= DstSlotIdx, return
  // DstSlotIdx - SrcSlotIdx.
  SlotDistance = DstSlotIdx - SrcSlotIdx;
  assert(SlotDistance >= 0 && "SlotDistance < 0!");

  return (unsigned)SlotDistance;
}

// map the StartSlot to MBBNum.
void FindShortestPath::mapStartSlotToMBBNum() {
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    unsigned StartSlot = FInfo->getStartSlotFor(MBB);
    unsigned MBBNum = MBB->getNumber();
    StartSlotToMBBNumMap[StartSlot] = MBBNum;
  }
}

// Get MBB number.
unsigned FindShortestPath::getMBBNum(unsigned SlotStartIdx) {
  return StartSlotToMBBNumMap[SlotStartIdx];
}

// Initial the Path between two related Machine Basic Block.
void FindShortestPath::InitPath() {
  unsigned MBBNum = MF->getNumBlockIDs();

  // assign infinite number to the PathVector.
  unsigned PathVectorSize = MBBNum * MBBNum;
  PathVector.assign(PathVectorSize, infinite);

  // Initial the PathVector with the Path.
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock *SrcMBB = I;
    unsigned TolalSlot = FInfo->getTotalSlotFor(SrcMBB);
    unsigned IISlot = FInfo->getIISlotFor(SrcMBB);

    // assign 0 to the same MBB.
    getDistance(SrcMBB, SrcMBB) = 0;

    for (MachineBasicBlock::succ_iterator I = SrcMBB->succ_begin(), 
         E = SrcMBB->succ_end(); I != E; ++I) {
      MachineBasicBlock *DstMBB = *I;
      if (SrcMBB == DstMBB) continue;
      getDistance(SrcMBB, DstMBB) = TolalSlot;
    }
  }
}

// Use the Floyd Algorithm to find the shortest Path between two Machine Basic
// Block.
void FindShortestPath::Floyd() {
  unsigned MBBNum = MF->getNumBlockIDs();

  // We use the Floyd algorithm to get the shortest path of two different MBBs.
  for (unsigned Mid = 0; Mid < MBBNum; ++Mid) {
    for (unsigned Src = 0; Src < MBBNum; ++Src) {
      for (unsigned Dst = 0; Dst < MBBNum; ++Dst) {
        unsigned Src2DstIndex = getKey(Src, Dst);
        unsigned Src2MidIndex = getKey(Src, Mid);
        unsigned Mid2DstIndex = getKey(Mid, Dst);
        if (PathVector[Src2MidIndex] + PathVector[Mid2DstIndex]
            < PathVector[Src2DstIndex])
          PathVector[Src2DstIndex]
            = PathVector[Src2MidIndex] + PathVector[Mid2DstIndex];
      }
    }
  }
  // if the SrcMBB == DstMBB, get the IISlot and assign it to the PathVector.
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock *SrcMBB = I;
    unsigned IISlot = FInfo->getIISlotFor(SrcMBB);
    for (MachineBasicBlock::succ_iterator I = SrcMBB->succ_begin(),
         E = SrcMBB->succ_end(); I != E; ++I) {
      MachineBasicBlock *DstMBB = *I;
      if (SrcMBB != DstMBB) continue;
      getDistance(SrcMBB, DstMBB) = IISlot;
    }
  }
}

char FindShortestPath::ID = 0;
const unsigned FindShortestPath::infinite = 100000;

INITIALIZE_PASS_BEGIN(FindShortestPath, "FindShortestPath",
                      "FindShortestPath", false, false)
INITIALIZE_PASS_END(FindShortestPath, "FindShortestPath",
                    "FindShortestPath", false, false)
Pass *llvm::createFindShortestPathPass() {
  return new FindShortestPath();
};