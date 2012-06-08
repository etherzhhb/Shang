//====---- VTMFrameInfo.h - Define TargetFrameInfo for VTM --*- C++ -*----====//
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
