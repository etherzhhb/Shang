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
#include "llvm/CodeGen/MachineRegisterInfo.h"
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
  void emitAllRegister();
  void emitRegClassRegs(const TargetRegisterClass *RC, unsigned BitWidth);

  void clear();
  
  inline static std::string getStateNameForMachineBB(MachineBasicBlock *MBB) {
    return MBB->getName().str() + "BB" + itostr(MBB->getNumber());
  }

  inline static std::string getucStateEnableName(MachineBasicBlock *MBB) {
    std::string StateName = getStateNameForMachineBB(MBB);

    StateName = "cur_" + StateName;

    return StateName + "_enable";
  }

  inline std::string getucStateEnable(ucState &State) {
    return getucStateEnable(State->getParent(), State.getSlot());
  }
  inline std::string getucStateEnable(MachineBasicBlock *MBB, unsigned Slot) {
    std::string StateName = getucStateEnableName(MBB);
    raw_string_ostream ss(StateName);
    if (FuncInfo->getTotalSlotFor(MBB) > 1)
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
  void emitOpAdd(ucOp &OpAdd);

  void emitOperand(raw_ostream &OS, MachineOperand &Operand, unsigned BitWidth = 0);

  void emitCtrlOp(ucState &State);
  void emitOpArg(ucOp &VOpArg);
  void emitOpRetVal(ucOp &OpRetVal);
  void emitOpRet(ucOp &OpRet);

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

  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock &BB = *I;
    emitBasicBlock(BB);
  }

  emitAllRegister();


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
  createucStateEnable(&MBB);

  // Case begin
  vlang->matchCase(VM->getControlBlockBuffer(6), StateName);

  MachineBasicBlock::iterator I = MBB.getFirstNonPHI(), E = prior(MBB.end());
  while (I != E) {
    ucState CurState(*I);
    // Emit the datepath of current state.
    emitDatapath(CurState);
    
    // Emit next ucOp.
    ucState NextState(*(++I));
    vlang->ifBegin(VM->getControlBlockBuffer(8), getucStateEnable(CurState));
    emitCtrlOp(NextState);
    vlang->end(VM->getControlBlockBuffer(8));
  }
  
  //for (unsigned i = StartSlot, e = EndSlot + 1; i != e; ++i) {
  //  vlang->ifBegin(VM->getControlBlockBuffer(8), getucStateEnable(State, i, true));
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
void RTLWriter::createucStateEnable(MachineBasicBlock *MBB)  {
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
  std::string StateName = getucStateEnableName(MBB);
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

void RTLWriter::emitCtrlOp(ucState &State) {
  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;

    if (Op.haveDataPath())
      continue;

    // Emit the operations.
    switch (Op.getOpCode()) {
    case VTM::VOpArgi8: case VTM::VOpArgi16: case VTM::VOpArgi32:
    case VTM::VOpArgi64:
      emitOpArg(Op);
      break;
    case VTM::VOpRetVali8: case VTM::VOpRetVali16: case VTM::VOpRetVali32:
    case VTM::VOpRetVali64:
      emitOpRetVal(Op);
      break;
    case VTM::VOpRet:
      emitOpRet(Op);
      break;
    default:
      assert(0 && "Unexpect opcode!");
      break;
    }
  }
}

void RTLWriter::emitFirstCtrlState(MachineBasicBlock *MBB) {
  // TODO: Emit PHINodes if necessary.
  ucState FirstState(MBB->front());
  
  emitCtrlOp(FirstState);
}

void RTLWriter::emitOpArg(ucOp &OpArg) {
  // Assign input port to some register.
  raw_ostream &OS = VM->getControlBlockBuffer(10);
  emitOperand(OS, OpArg.getOperand(0));
  // FIXME: Use getInputPort instead;
  OS << " <= " << VM->getInputPort(OpArg.getOperand(1).getImm()).getName()
     << ";\n";
}

void RTLWriter::emitOpRet(ucOp &OpArg) {
  raw_ostream &OS = VM->getControlBlockBuffer(10);
  OS << "fin <= 1'h0;\n";
  OS.indent(10) << "NextFSMState <= state_idle;\n";
}

void RTLWriter::emitOpRetVal(ucOp &OpRetVal) {
  raw_ostream &OS = VM->getControlBlockBuffer(10);
  unsigned retChannel = OpRetVal.getOperand(1).getImm();
  assert(retChannel == 0 && "Only support Channel 0!");
  OS << "return_value <= ";
  emitOperand(OS, OpRetVal.getOperand(0));
  OS << ";\n";
}

void RTLWriter::emitOperand(raw_ostream &OS, MachineOperand &Operand,
                            unsigned BitWidth) {
  switch (Operand.getType()) {
  case MachineOperand::MO_Register:
    OS << "reg" << Operand.getReg();
    return;
  case MachineOperand::MO_Metadata: {
    BundleToken MetaOp(Operand.getMetadata());
    assert((MetaOp.isDefWire() || MetaOp.isReadWire()) && "Bad operand!");
    
    std::string WireName = MetaOp.getWireName();
    OS << WireName;

    // Emit the wire here, because it only define once, wires will never
    // be emitted more than once.
    if (MetaOp.isDefWire())
      VM->addWire(WireName, MetaOp.getBitWidth());
    
    return;
  }
  }

}

void RTLWriter::emitNextFSMState(raw_ostream &ss, MachineBasicBlock *MBB) {
  // Emit the first micro state of the target state.
  emitFirstCtrlState(MBB);

  ss.indent(10) << "NextFSMState <= " << getStateNameForMachineBB(MBB) << ";\n";
}

void RTLWriter::emitDatapath(ucState &State) {
  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;

    if (!Op.haveDataPath())
      continue;
    
    switch (Op.getOpCode()) {
    case VTM::VOpADD:
      emitOpAdd(Op);
      break;
    }
  } 
}

void RTLWriter::emitOpAdd(ucOp &OpAdd) {
  raw_ostream &OS = VM->getDataPathBuffer(2);

  OS << "assign ";
  emitOperand(OS, OpAdd.getOperand(0));
  OS << " = ";
  emitOperand(OS, OpAdd.getOperand(1));
  OS << " + ";
  emitOperand(OS, OpAdd.getOperand(2));
  OS << ";\n";
}

void RTLWriter::emitAllRegister() {
  emitRegClassRegs(VTM::DR1RegisterClass, 1);
  emitRegClassRegs(VTM::DR8RegisterClass, 8);
  emitRegClassRegs(VTM::DR16RegisterClass, 16);
  emitRegClassRegs(VTM::DR32RegisterClass, 32);
  emitRegClassRegs(VTM::DR64RegisterClass, 64);
}

void RTLWriter::emitRegClassRegs(const TargetRegisterClass *RC,
                                 unsigned BitWidth) {
  MachineRegisterInfo &MRI = MF->getRegInfo();
  const std::vector<unsigned> &Regs = MRI.getRegClassVirtRegs(RC);
  for (std::vector<unsigned>::const_iterator I = Regs.begin(), E = Regs.end();
      I != E; ++I)
    VM->addRegister("reg"+utostr(*I), BitWidth);
}

static RegisterPass<RTLWriter> X("vbe-rtl", "vbe - Write HWAtom as RTL Verilog");

Pass *llvm::createRTLWriterPass(raw_ostream &O) {
  return new RTLWriter(O);
}
