//====---- VTMFrameInfo.h - Define TargetFrameInfo for VTM --*- C++ -*----====//
//
//                      The Shang HLS frameowrk                               //
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#ifndef VTM_FRAMEINFO_H
#define VTM_FRAMEINFO_H
#include "vtm/VerilogBackendMCTargetDesc.h"
#include "llvm/Target/TargetFrameLowering.h"

namespace llvm {
class VSubtarget;
class AllocaInst;

class VFrameInfo : public TargetFrameLowering {
protected:

public:
  VFrameInfo() : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, 2, -2) {
  }

  /// emitProlog/emitEpilog - These methods insert prolog and epilog code into
  /// the function.
  void emitPrologue(MachineFunction &MF) const;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const;

  bool hasFP(const MachineFunction &MF) const { return false; }
};

} // End llvm namespace

#endif
