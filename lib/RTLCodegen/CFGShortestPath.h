// CFGShortestPath.h --- find shortest paths in a weighted graph --- C++ ---==//
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
#ifndef CFGSHORTESTPATH_H
#define CFGSHORTESTPATH_H
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/VerilogAST.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include <vector>
#include <map>

namespace llvm {
  class VASTSlot;
  class CFGShortestPath : public MachineFunctionPass{

    MachineFunction *MF;
    VFInfo *FInfo;

    // Define a Vector to record the path.
    std::vector<unsigned> PathVector;

    // Map StartSlot to MBB Number
    typedef std::map<unsigned, unsigned> Slot2BBMapTy;
    Slot2BBMapTy StartSlotToMBBNumMap;
    // Get the Key to PathVector according to the source and destination index
    unsigned getKey(unsigned Def, unsigned Use);

    // map the StartSlot to MBBNum.
    void mapStartSlotToMBBNum();

    // Get MBB number.
    unsigned getMBBNum(unsigned SlotStartIdx) const;

    // Initial the Path between two related Machine Basic Block.
    void InitPath();

    // Use the Floyd Algorithm to find the shortest Path between two Machine Basic
    // Block.
    void computeShortestPathFloyd();

  public:

    static char ID;

    // Initial the pathVector with infinite.
    const static int Infinite;

    // Get distance between the source MBB and the destination MBB.
    unsigned &getDistance(unsigned DefIdx, unsigned UseIdx);
    unsigned &getDistance(MachineBasicBlock *DefMBB, MachineBasicBlock *UseMBB){
      return getDistance(DefMBB->getNumber(), UseMBB->getNumber());
    }

    // Get distance between Two Slots.
    int getSlotDistance(VASTSlot *DefSlot, VASTSlot *UseSlot);

    bool runOnMachineFunction(MachineFunction &MF);
    void releaseMemory() {
      PathVector.clear();
      StartSlotToMBBNumMap.clear();
    }

    CFGShortestPath() : MachineFunctionPass(ID) {
      initializeCFGShortestPathPass(*PassRegistry::getPassRegistry());
    }
  };
} // end anonymous.
#endif
