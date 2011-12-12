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
using namespace llvm;

bool FindShortestPath::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  FInfo = MF->getInfo<VFInfo>();
  InitPath();
  Floyd();
  return false;
}

// Get the Key to PathVector according to the source and destination index
unsigned FindShortestPath::getKey(unsigned src, unsigned dst) {
  unsigned MBBNum = MF->getNumBlockIDs();
  unsigned returnKey = src*MBBNum + dst;
  return returnKey;
}

// Get distance between the source MBB and the destination MBB.
unsigned FindShortestPath::getDistance(MachineBasicBlock *srcMBB,
                                       MachineBasicBlock *dstMBB) {
  unsigned Src = srcMBB->getNumber();
  unsigned Dst = dstMBB->getNumber();
  unsigned Index = getKey(Src, Dst);
  return PathVector[Index];
}

// Initial the Path between two related Machine Basic Block.
void FindShortestPath::InitPath() {
  // Initial the pathVector with infinite.
  const unsigned infinite = -1;
  unsigned MBBNum = MF->getNumBlockIDs();
  unsigned PathVectorSize = MBBNum * MBBNum;
  PathVector.assign(PathVectorSize, infinite);
  
  // Initial the PathVector with the Path.
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock *SrcMBB = I;
    unsigned TolalSlot = FInfo->getTotalSlotFor(SrcMBB);
    unsigned IISlot = FInfo->getIISlotFor(SrcMBB);
    for (MachineBasicBlock::succ_iterator I = SrcMBB->succ_begin(), 
         E = SrcMBB->succ_end(); I != E; ++I) {
      MachineBasicBlock *DstMBB = *I;
      unsigned Src = SrcMBB->getNumber();
      unsigned Dst = DstMBB->getNumber();
      unsigned Index = getKey(Src, Dst);
      if (Src == Dst) {
        PathVector[Index] = IISlot;
        continue;
      }
      PathVector[Index] = TolalSlot;
    }
  }
}

// Use the Floyd Algorithm to find the shortest Path between two Machine Basic
// Block.
void FindShortestPath::Floyd() {
  unsigned MBBNum = MF->getNumBlockIDs();
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
  
}

char FindShortestPath::ID = 0;
INITIALIZE_PASS_BEGIN(FindShortestPath, "FindShortestPath",
                      "FindShortestPath", false, false)
INITIALIZE_PASS_END(FindShortestPath, "FindShortestPath",
                    "FindShortestPath", false, false)
Pass *llvm::createFindShortestPathPass() {
  return new FindShortestPath();
};