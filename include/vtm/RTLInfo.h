//====--------- RTLInfo.h - RTL module of synthesised moudle ----*- C++ -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file declares the pass that build the Register Transfer Level
// represented by verilog abstract syntax tree for the synthesised module.
//
// TODO: Seperate the building and writing logic.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_RTL_INFO_H
#define VTM_RTL_INFO_H
#include "vtm/VerilogAST.h"
#include "vtm/MicroState.h"
#include "vtm/VFuncInfo.h"
#include "vtm/VTargetMachine.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"

#include "llvm/ADT/StringExtras.h"

#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/ToolOutputFile.h"

namespace llvm {
class VFuncInfo;
class BitLevelInfo;
class ucState;
class ucOp;

class MachineOperand;
class MachineBasicBlock;
class MachineRegisterInfo;
class TargetRegisterClass;


class RTLInfo : public MachineFunctionPass {
  tool_output_file *FOut;
  formatted_raw_ostream Out;
  VTargetMachine &VTM;

  MachineFunction *MF;
  VFuncInfo *FuncInfo;
  MachineRegisterInfo *MRI;
  BitLevelInfo *BLI;
  VASTModule *VM;
  Mangler *Mang;

  unsigned TotalFSMStatesBit, CurFSMStateNum;

  SmallVector<const Argument*, 8> Arguments; 

  void emitFunctionSignature();
  void emitCommonPort();

  /// emitAllocatedFUs - Set up a vector for allocated resources, and
  /// emit the ports and register, wire and datapath for them.
  void emitAllocatedFUs();

  void emitBasicBlock(MachineBasicBlock &MBB);
  void emitAllRegister();
  void emitRegClassRegs(const TargetRegisterClass *RC, unsigned BitWidth);

  void clear();

  inline std::string getStateName(MachineBasicBlock *MBB) {
    SmallVector<char, 16> Name;
    // Use mangler to handle escaped characters.
    Mang->getNameWithPrefix(Name, MBB->getName().str() + "BB"
      + itostr(MBB->getNumber()));
    return std::string(Name.data(), Name.size());
  }

  inline std::string getucStateEnableName(MachineBasicBlock *MBB) {
    std::string StateName = getStateName(MBB);

    StateName = "cur_" + StateName;

    return StateName + "_enable";
  }

  inline std::string getucStateEnable(ucState &State) {
    return getucStateEnable(State->getParent(), State.getSlot());
  }
  inline std::string getucStateEnable(MachineBasicBlock *MBB, unsigned Slot) {
    std::string StateName = getucStateEnableName(MBB);
    raw_string_ostream ss(StateName);
    // Ignore the laster slot, we do nothing at that slot.
    if (FuncInfo->getTotalSlotFor(MBB) - 1 > 1)
      ss << "[" << (Slot - FuncInfo->getStartSlotFor(MBB)) << "]";

    return ss.str();
  }

  void emitNextFSMState(raw_ostream &ss, MachineBasicBlock *MBB);

  void createucStateEnable(MachineBasicBlock *MBB);
  void emitNextMicroState(raw_ostream &ss, MachineBasicBlock *MBB,
    const std::string &NewState);

  // Emit the operations in the first micro state in the FSM state when we are
  // jumping to it.
  void emitFirstCtrlState(MachineBasicBlock *MBB);

  void emitDatapath(ucState &State);

  void emitUnaryOp(ucOp &BinOp, const std::string &Operator);
  void emitBinaryOp(ucOp &BinOp, const std::string &Operator);

  void emitOpAdd(ucOp &OpAdd);

  void emitOpBitCat(ucOp &OpBitCat);
  void emitOpBitSlice(ucOp &OpBitSlice);
  void emitOpBitRepeat(ucOp &OpBitRepeat);

  void emitOperand(raw_ostream &OS, MachineOperand &Operand,
                   bool PrintBitRange = true);
  unsigned getOperandWitdh(MachineOperand &Operand);

  void emitCtrlOp(ucState &State);
  void emitOpArg(ucOp &VOpArg);
  void emitOpRetVal(ucOp &OpRetVal);
  void emitOpRet(ucOp &OpRet);
  void emitOpLatchVal(ucOp &OpLatchVal);
  void emitOpMemAccess(ucOp &OpMemAccess);
  void emitOpToState(ucOp &OpToState);

  void emitFUCtrl(unsigned Slot);

public:
  /// @name FunctionPass interface
  //{
  static char ID;
  RTLInfo(VTargetMachine &TM);
  RTLInfo();

  ~RTLInfo();

  VASTModule *getRTLModule() const { return VM; }

  bool doInitialization(Module &M) {
    MachineModuleInfo *MMI = getAnalysisIfAvailable<MachineModuleInfo>();
    assert(MMI && "MachineModuleInfo will always available"
                  " in a machine function pass!");
    Mang = new Mangler(MMI->getContext(), *VTM.getTargetData());

    FOut = VTM.getOutFile("v");
    Out.setStream(FOut->os());
    return false;
  }

  bool doFinalization(Module &M) {
    Out.flush();
    FOut->keep();
    delete Mang;
    return false;
  }

  bool runOnMachineFunction(MachineFunction &MF);
  void releaseMemory() { clear(); }
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};
}
#endif
