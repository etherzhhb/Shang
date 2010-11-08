//===----------- RTLWriter.cpp - HWAtom to RTL verilog  ---------*- C++ -*-===//
//
//                            The Verilog Backend
//
// Copyright: 2010 by Hongbin Zheng. all rights reserved.
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
// This file implement the RTLWriter pass, which write out HWAtom into RTL
// verilog form.
//
//===----------------------------------------------------------------------===//

#include "HWAtomPasses.h"
#include "MicroState.h"
#include "VerilogAST.h"
#include "VTargetMachine.h"
#include "VTMFunctionInfo.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/CommandLine.h"

#include "llvm/Support/Debug.h"

using namespace llvm;
namespace {
class RTLWriter : public MachineFunctionPass {
  raw_ostream &Out;
  VLang *vlang;
  VTMFunctionInfo *FuncInfo;
  VASTModule *VM;
  MachineFunction *MF;

  unsigned TotalFSMStatesBit, CurFSMStateNum;
  
  SmallVector<const Argument*, 8> Arguments;

  void emitFunctionSignature();
  void emitCommonPort();
  void emitBasicBlock(MachineBasicBlock &MBB);

  void clear();
  
  inline static std::string getStateNameForMachineBB(MachineBasicBlock *MBB) {
    return MBB->getName().str() + "BB" + itostr(MBB->getNumber());
  }

  inline static std::string getMircoStateEnableName(MachineBasicBlock *MBB) {
    std::string StateName = getStateNameForMachineBB(MBB);

    StateName = "cur_" + StateName;

    return StateName + "_enable";
  }

  inline std::string getMircoStateEnable(MachineInstr &MI) {
    assert(MI.getOpcode() == VTM::VOpBundle && "Bad micro state!");
    return getMircoStateEnable(MI.getParent(), MI.getOperand(0).getImm());
  }
  inline std::string getMircoStateEnable(MachineBasicBlock *MBB, unsigned Slot) {
    std::string StateName = getMircoStateEnableName(MBB);
    raw_string_ostream ss(StateName);
    if (FuncInfo->getTotalSlotFor(MBB) > 1)
      ss << "[" << (Slot - FuncInfo->getStartSlotFor(MBB)) << "]";

    return ss.str();
  }

  void emitNextFSMState(raw_ostream &ss, MachineBasicBlock *MBB);

  void createMircoStateEnable(MachineBasicBlock *MBB);
  void emitNextMicroState(raw_ostream &ss, MachineBasicBlock *MBB,
    const std::string &NewState);

  // Emit the operations in the first micro state in the FSM state when we are
  // jumping to it.
  void emitFirstCtrlState(MachineBasicBlock *MBB);

  void emitCtrlState(MachineInstr &MI);
  void emitDatapath(MachineInstr &MI);

  void emitOperand(raw_ostream &OS, MachineOperand &Operand, unsigned BitWidth = 0);

  // Emit mircro operation in microstate
  void emitCtrlOp(ucOp &MOp);

  void emitOpArg(ucOp &VOpArg);

public:
  /// @name FunctionPass interface
  //{
  static char ID;
  explicit RTLWriter(raw_ostream &O = nulls()) : MachineFunctionPass(ID),
    Out(O), vlang(0), VM(0), TotalFSMStatesBit(0), CurFSMStateNum(0) {}
  ~RTLWriter();

  VASTModule *getVerilogModule() const { return VM; }

  bool runOnMachineFunction(MachineFunction &MF);
  void releaseMemory() { clear(); }
  void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};
}

//===----------------------------------------------------------------------===//


static cl::opt<std::string>
DesignName("vbe-design-name", cl::desc("Design Name."), cl::init(""));

using namespace llvm;

char RTLWriter::ID = 0;

bool RTLWriter::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  FuncInfo = MF->getInfo<VTMFunctionInfo>();
  vlang = &getAnalysis<VLang>();

  if (DesignName.empty())
    DesignName = MF->getFunction()->getNameStr();

  VM = new VASTModule(DesignName);
  emitFunctionSignature();

  // Emit control register and idle state
  unsigned totalFSMStates = MF->size() + 1;
  TotalFSMStatesBit = Log2_32_Ceil(totalFSMStates);

  VM->addRegister("NextFSMState", TotalFSMStatesBit);
  
  // Idle state
  vlang->param(VM->getStateDeclBuffer(), "state_idle", TotalFSMStatesBit, 0);
  vlang->matchCase(VM->getControlBlockBuffer(6), "state_idle");
  // Idle state is always ready.
  VM->getControlBlockBuffer(8) << "fin <= 1'h0;\n";
  vlang->ifBegin(VM->getControlBlockBuffer(8), "start");
  // The module is busy now
  MachineBasicBlock *EntryBB =  GraphTraits<MachineFunction*>::getEntryNode(MF);
  emitNextFSMState(VM->getControlBlockBuffer(), EntryBB);
  emitNextMicroState(VM->getControlBlockBuffer(10), EntryBB, "1'b1");
  //
  vlang->ifElse(VM->getControlBlockBuffer(8));
  VM->getControlBlockBuffer(10) << "NextFSMState <= state_idle;\n";
  vlang->end(VM->getControlBlockBuffer(8));
  vlang->end(VM->getControlBlockBuffer(6));

  // Emit basicblocks
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock &BB = *I;
    emitBasicBlock(BB);
  }


  // Write buffers to output
  VM->printModuleDecl(Out);
  Out << "\n\n";
  // States
  vlang->comment(Out.indent(2)) << "States\n";
  Out << VM->getStateDeclStr();
  Out << "\n\n";
  // Reg and wire
  vlang->comment(Out.indent(2)) << "Reg and wire decl\n";
  VM->printSignalDecl(Out, 2);
  Out << "\n\n";

  // Datapath
  vlang->comment(Out.indent(2)) << "Datapath\n";
  Out << VM->getDataPathStr();

  Out << "\n\n";
  vlang->comment(Out.indent(2)) << "Always Block\n";
  vlang->alwaysBegin(Out, 2);
  VM->printRegisterReset(Out, 6);
  vlang->ifElse(Out.indent(4));

  vlang->comment(Out.indent(6)) << "SeqCompute:\n";
  Out << VM->getSeqComputeStr();

  vlang->comment(Out.indent(6)) << "FSM\n";
  vlang->switchCase(Out.indent(6), "NextFSMState");
  Out << VM->getControlBlockStr();
  // Case default.
  Out.indent(6) << "default:  NextFSMState <= state_idle;\n";
  vlang->endSwitch(Out.indent(6));
  vlang->alwaysEnd(Out, 2);

  vlang->endModule(Out);

  return false;
}

void RTLWriter::clear() {
  Arguments.clear();

  delete VM;
}

void RTLWriter::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.addRequired<VLang>();
  AU.setPreservesAll();
}

void RTLWriter::print(raw_ostream &O, const Module *M) const {

}

void RTLWriter::emitFunctionSignature() {
  const Function *F = MF->getFunction();
  const TargetData *TD = MF->getTarget().getTargetData();

  for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
      I != E; ++I) {
    const Argument *Arg = I;
    std::string Name = Arg->getNameStr();
    unsigned BitWidth = TD->getTypeSizeInBits(Arg->getType());
    // Add port declaration.
    VM->addInputPort(Name, BitWidth);
    Arguments.push_back(Arg);
  }

  const Type *RetTy = F->getReturnType();
  if (!RetTy->isVoidTy()) {
    assert(RetTy->isIntegerTy() && "Only support return integer now!");
    VM->addOutputPort("return_value", TD->getTypeSizeInBits(RetTy));
  }

  emitCommonPort();
}

void RTLWriter::emitBasicBlock(MachineBasicBlock &MBB) {
  std::string StateName = getStateNameForMachineBB(&MBB);
  unsigned totalSlot = FuncInfo->getTotalSlotFor(&MBB);

  //unsigned StartSlot = State->getSlot(), EndSlot = State->getEndSlot();
  vlang->comment(VM->getStateDeclBuffer()) << "State for " << StateName << '\n';
  vlang->param(VM->getStateDeclBuffer(), StateName, TotalFSMStatesBit,
               ++CurFSMStateNum);

  // State information.
  vlang->comment(VM->getControlBlockBuffer(6)) << StateName 
    << " Total Slot: " << totalSlot
    << " II: " << 0 <<  '\n';
  // Mirco state enable.
  createMircoStateEnable(&MBB);

  // Case begin
  vlang->matchCase(VM->getControlBlockBuffer(6), StateName);

  MachineBasicBlock::iterator I = MBB.getFirstNonPHI(), E = prior(MBB.end());
  while (I != E) {
    MachineInstr &CurInstr = *I;
    // Emit the datepath of current state.
    emitDatapath(CurInstr);
    
    // Emit next ucOp.
    MachineInstr &NextInstr = *(++I);
    vlang->ifBegin(VM->getControlBlockBuffer(8),
                   getMircoStateEnable(CurInstr));
    emitCtrlState(NextInstr);
    vlang->end(VM->getControlBlockBuffer(8));
  }
  
  // Emit the control logic of last state.
  // Note: The last state do not expect to have datapath.
  MachineInstr &LastInstr = *I;
  vlang->ifBegin(VM->getControlBlockBuffer(8),
                 getMircoStateEnable(LastInstr));
  emitCtrlState(LastInstr);
  vlang->end(VM->getControlBlockBuffer(8));
  
  //for (unsigned i = StartSlot, e = EndSlot + 1; i != e; ++i) {
  //  vlang->ifBegin(VM->getControlBlockBuffer(8), getMircoStateEnable(State, i, true));
  //  // Emit all atoms at cycle i

  //  vlang->comment(VM->getDataPathBuffer(2)) << "at cycle: " << i << '\n';
  //  for (cycle_iterator CI = Atoms.lower_bound(i), CE = Atoms.upper_bound(i);
  //      CI != CE; ++CI)
  //    emitAtom(CI->second);
  //  vlang->end(VM->getControlBlockBuffer(8));
  //}// end for

  //// Emit Self Loop logic.
  //if (State->haveSelfLoop()) {
  //  vlang->comment(VM->getControlBlockBuffer(8)) << "For self loop:\n";
  //  std::string SelfLoopEnable = computeSelfLoopEnable(State);
  //  emitNextMicroState(VM->getControlBlockBuffer(8), BB, SelfLoopEnable);
  //} else
    emitNextMicroState(VM->getControlBlockBuffer(8), &MBB, "1'b0");
  // Case end
  vlang->end(VM->getControlBlockBuffer(6));
}

void RTLWriter::emitCommonPort() {
  VM->addInputPort("clk", 1);
  VM->addInputPort("rstN", 1);
  VM->addInputPort("start", 1);
  VM->addOutputPort("fin", 1);
}

RTLWriter::~RTLWriter() {}


//===----------------------------------------------------------------------===//
// Emit hardware atoms


//===----------------------------------------------------------------------===//
void RTLWriter::createMircoStateEnable(MachineBasicBlock *MBB)  {
  std::string StateName = getStateNameForMachineBB(MBB);
  // We do not need the last state.
  unsigned totalSlot = FuncInfo->getTotalSlotFor(MBB) - 1;

  // current state
  VM->addRegister("cur_" + StateName + "_enable", totalSlot);
}

void RTLWriter::emitNextMicroState(raw_ostream &ss, MachineBasicBlock *MBB,
                                   const std::string &NewState) {
  // We do not need the last state.
  unsigned totalSlot = FuncInfo->getTotalSlotFor(MBB) - 1;
  std::string StateName = getMircoStateEnableName(MBB);
  ss << StateName << " <= ";
  if (totalSlot > 1) {
    ss << "{ " << StateName << "[";
    if (totalSlot > 2)  ss << (totalSlot - 2) << ": ";
    ss << "0], ";
  }

  ss << NewState;

  if (totalSlot > 1)
    ss << " }";

  ss << ";\n";
}

void RTLWriter::emitCtrlState(MachineInstr &MI){
  for (ucOpIterator I = ucOpIterator::begin(MI),
      E = ucOpIterator::end(MI); I != E; ++I) {
    ucOp Op = *I;

    if (!Op.haveDataPath())
      emitCtrlOp(Op);
  }
}

void RTLWriter::emitFirstCtrlState(MachineBasicBlock *MBB) {
  // TODO: Emit PHINodes if necessary.
  MachineInstr &FirstInstr = MBB->front();
  
  emitCtrlState(FirstInstr);
}

void RTLWriter::emitCtrlOp(ucOp &MOp) {
  switch (MOp.getOpCode()) {
  case VTM::VOpArgi8:
  case VTM::VOpArgi16:
  case VTM::VOpArgi32:
  case VTM::VOpArgi64: emitOpArg(MOp); return;
  }
}

void RTLWriter::emitOpArg(ucOp &VOpArg) {
  // Assign input port to some register.
  raw_ostream &OS = VM->getControlBlockBuffer(10);
  emitOperand(OS, VOpArg.getOperand(0));
  // FIXME: Use getInputPort instead;
  OS << " <= " << VM->getInputPort(VOpArg.getOperand(1).getImm()).getName()
     << ";\n";
}

void RTLWriter::emitOperand(raw_ostream &OS, MachineOperand &Operand,
                            unsigned BitWidth) {
  switch (Operand.getType()) {
  case MachineOperand::MO_Register:
    OS << "reg" << Operand.getReg();
    return;
  }

}

void RTLWriter::emitNextFSMState(raw_ostream &ss, MachineBasicBlock *MBB) {
  // Emit the first micro state of the target state.
  emitFirstCtrlState(MBB);

  ss.indent(10) << "NextFSMState <= " << getStateNameForMachineBB(MBB) << ";\n";
}

void RTLWriter::emitDatapath(MachineInstr &MI) {

}

static RegisterPass<RTLWriter> X("vbe-rtl", "vbe - Write HWAtom as RTL Verilog");

Pass *llvm::createRTLWriterPass(raw_ostream &O) {
  return new RTLWriter(O);
}
