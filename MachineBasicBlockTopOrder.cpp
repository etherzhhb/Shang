//===------------------------IRAnalysis.cpp--------------------------------===//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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

  MachineBasicBlockTopOrder() : MachineFunctionPass(ID) {
    initializeMachineBasicBlockTopOrderPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF);

  void getAnalysisUsage(AnalysisUsage &AU) const {
    // FIXME: This pass break MachineBlockPlacementPass.
    AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
};
}

INITIALIZE_PASS(MachineBasicBlockTopOrder, "machine-basicblock-top-order",
               "Place the Machine BasicBlocks in topological order",
               false, true)

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
