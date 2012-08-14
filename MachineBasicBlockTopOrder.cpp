//===------------------------IRAnalysis.cpp--------------------------------===//
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
//  Place the BBs in topological order, this can benefit some
//  of the later algorithms.
//
//===----------------------------------------------------------------------===//
#include "llvm/Pass.h"

#include "vtm/Passes.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"

using namespace llvm;

namespace llvm {
  class MachineBasicBlockTopOrder : public MachineFunctionPass {
public:
  static char ID;

  MachineBasicBlockTopOrder() : MachineFunctionPass(ID) {};

  bool runOnMachineFunction(MachineFunction &MF);
};
}

char MachineBasicBlockTopOrder::ID = 0;
char &llvm::MachineBasicBlockTopOrderID= MachineBasicBlockTopOrder::ID;

bool MachineBasicBlockTopOrder::runOnMachineFunction(MachineFunction &MF){
  // Place the BBs in topological order this can benefit some of the later
  // algorithms.
  typedef po_iterator<MachineBasicBlock*> po_it;
  for (po_it I = po_begin(&MF.front()), E = po_end(&MF.front()); I != E; ++I) {
    MachineBasicBlock *MBB = *I;
    MF.splice(MF.begin(), MBB);
  }

  // Reset the MBB numbering.
  MF.RenumberBlocks();
  return false;
}
