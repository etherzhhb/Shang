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
#include "FindMBBShortestPath.h"

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
unsigned FindShortestPath::getKey(unsigned Def, unsigned Use) {
  unsigned MBBNum = MF->getNumBlockIDs();
  unsigned returnKey = Def * MBBNum + Use;
  return returnKey;
}

// Get distance between the source MBB and the destination MBB.
unsigned &FindShortestPath::getDistance(unsigned DefIdx, unsigned UseIdx) {
  unsigned Index = getKey(DefIdx, UseIdx);
  return PathVector[Index];
}

// Get distance between Two Slots.
int FindShortestPath::getSlotDistance(VASTSlot *DefSlot, VASTSlot *UseSlot) {
  int SlotDistance = -1;

  unsigned DefSlotStartIdx = DefSlot->getParentIdx();
  unsigned DefSlotIdx = DefSlot->getSlotNum();
  unsigned DefMBBNum = getMBBNum(DefSlotStartIdx);

  unsigned UseSlotStartIdx = UseSlot->getParentIdx();
  unsigned UseSlotIdx = UseSlot->getSlotNum();
  unsigned UseMBBNum = getMBBNum(UseSlotStartIdx);

  int MBBDistance = getDistance(DefMBBNum, UseMBBNum);
  // When the slots are in different MBB. return the SlotDistance.
  // SlotDistance = MBBDistance - (SrcSlotIdx - SrcSlotStartIdx)
  //                            + (DstSlotIdx - DstSlotStartIdx)
  // When the slots are in the same MBB and the SrcSlotIdx > DstSlotIdx, return
  // compute the distance considering the IISlot.
  if (DefMBBNum != UseMBBNum || DefSlotIdx > UseSlotIdx) {
    // If the MBBDistance is infinite, then the srcMBB can not reach the dstMBB,
    // Return -1 which is the largest number in unsigned type.
    if(MBBDistance == Infinite) return -1;

    SlotDistance = MBBDistance - (DefSlotIdx - DefSlotStartIdx)
                               + (UseSlotIdx - UseSlotStartIdx);
    // SlotDistance < 0 means UseSlot is unreachable from DefSlot.
    // assert((SlotDistance >= 0 || DefMBBNum == UseMBBNum)
    //       && "SlotDistance < 0 !!!");
    return SlotDistance;
  }

  // When the slots are in the same MBB and the SrcSlotIdx <= DstSlotIdx, return
  // DstSlotIdx - SrcSlotIdx.
  SlotDistance = UseSlotIdx - DefSlotIdx;
  // SlotDistance < 0 means UseSlot is unreachable from DefSlot.
  // assert(SlotDistance >= 0 && "SlotDistance < 0!");

  return SlotDistance;
}

// Map the StartSlot to MBBNum.
void FindShortestPath::mapStartSlotToMBBNum() {
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    unsigned StartSlot = FInfo->getStartSlotFor(MBB);
    unsigned MBBNum = MBB->getNumber();
    StartSlotToMBBNumMap[StartSlot] = MBBNum;
  }
}

// Get MBB number.
unsigned FindShortestPath::getMBBNum(unsigned SlotStartIdx) const {
  // The start slot of the module.
  if (SlotStartIdx == 0) return 0;
  
  Slot2BBMapTy::const_iterator at = StartSlotToMBBNumMap.find(SlotStartIdx);
  assert(at != StartSlotToMBBNumMap.end() && "Bad start slot!");
  return at->second;
}

// Initial the Path between two related Machine Basic Block.
void FindShortestPath::InitPath() {
  unsigned MBBNum = MF->getNumBlockIDs();

  // assign infinite number to the PathVector.
  unsigned PathVectorSize = MBBNum * MBBNum;
  PathVector.assign(PathVectorSize, Infinite);
}

// Use the Floyd Algorithm to find the shortest Path between two Machine Basic
// Block.
void FindShortestPath::Floyd() {
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
    unsigned IISlot = FInfo->getIISlotFor(DefMBB);
    for (MachineBasicBlock::succ_iterator I = DefMBB->succ_begin(),
         E = DefMBB->succ_end(); I != E; ++I) {
      MachineBasicBlock *UseMBB = *I;
      if (DefMBB != UseMBB) continue;
      getDistance(DefMBB, UseMBB) = IISlot;
    }
  }
}

char FindShortestPath::ID = 0;
const unsigned FindShortestPath::Infinite = 100000;

INITIALIZE_PASS_BEGIN(FindShortestPath, "FindShortestPath",
                      "FindShortestPath", false, false)
INITIALIZE_PASS_END(FindShortestPath, "FindShortestPath",
                    "FindShortestPath", false, false)
Pass *llvm::createFindShortestPathPass() {
  return new FindShortestPath();
}
