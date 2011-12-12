#include "vtm/Passes.h"
#include "vtm/VFInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include <vector>
using namespace llvm;

namespace{
class FindShortestPath : public MachineFunctionPass{
  
  MachineFunction *MF;
  VFInfo *FInfo;

  // Define a Vector to record the path.
  std::vector<unsigned> PathVector;

  // Get the Key to PathVector according to the source and destination index
  unsigned getKey(unsigned src, unsigned dst);

  // Initial the Path between two related Machine Basic Block.
  void InitPath();

  // Use the Floyd Algorithm to find the shortest Path between two Machine Basic
  // Block.
  void Floyd();

public:

  static char ID;
  // Get distance between the source MBB and the destination MBB.
  unsigned getDistance(MachineBasicBlock *srcMBB, MachineBasicBlock *dstMBB);

  bool runOnMachineFunction(MachineFunction &MF);

  FindShortestPath() : MachineFunctionPass(ID) {
    initializeFindShortestPathPass(*PassRegistry::getPassRegistry());
  }
};
} // end anonymous.