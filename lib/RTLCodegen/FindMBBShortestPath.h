// FindMBBShortestPath.h -- find shortest paths in a weighted graph -- C++ -==//
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
#ifndef FINDMBBSHORTESTPATH_H
#define FINDMBBSHORTESTPATH_H
#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "vtm/VerilogAST.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include <vector>
#include <map>

namespace llvm {
  class VASTSlot;
  class FindShortestPath : public MachineFunctionPass{

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
    void Floyd();

  public:

    static char ID;

    // Initial the pathVector with infinite.
    const static unsigned Infinite;

    // Get distance between the source MBB and the destination MBB.
    unsigned &getDistance(unsigned DefIdx, unsigned UseIdx);
    unsigned &getDistance(MachineBasicBlock *DefMBB, MachineBasicBlock *UseMBB){
      return getDistance(DefMBB->getNumber(), UseMBB->getNumber());
    }

    // Get distance between Two Slots.
    int getSlotDistance(VASTSlot *DefSlot, VASTSlot *UseSlot);

    bool runOnMachineFunction(MachineFunction &MF);

    FindShortestPath() : MachineFunctionPass(ID) {
      initializeFindShortestPathPass(*PassRegistry::getPassRegistry());
    }
  };
} // end anonymous.
#endif
