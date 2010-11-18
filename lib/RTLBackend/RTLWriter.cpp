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

#include "vtm/Passes.h"
#include "vtm/MicroState.h"
#include "vtm/VerilogAST.h"
#include "vtm/VTargetMachine.h"
#include "vtm/VFuncInfo.h"
#include "vtm/BitLevelInfo.h"

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
  MachineFunction *MF;
  VTargetMachine &VTM;
  VFuncInfo *FuncInfo;
  MachineRegisterInfo *MRI;
  BitLevelInfo *BLI;
  VASTModule *VM;

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
  void emitBinOp(ucOp &BinOp, const std::string &Operator);

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
  void emitOpWriteReg(ucOp &OpWriteReg);
  void emitOpMemAccess(ucOp &OpMemAccess);

  void emitFUCtrl(unsigned Slot);

public:
  /// @name FunctionPass interface
  //{
  static char ID;
  RTLWriter(VTargetMachine &TM, raw_ostream &O)
    : MachineFunctionPass(ID), VTM(TM), Out(O), VM(0),
    TotalFSMStatesBit(0), CurFSMStateNum(0) {}
  
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
  FuncInfo = MF->getInfo<VFuncInfo>();
  MRI = &MF->getRegInfo();
  BLI = &getAnalysis<BitLevelInfo>();

  if (DesignName.empty())
    DesignName = MF->getFunction()->getNameStr();

  VM = new VASTModule(DesignName);
  emitFunctionSignature();

  // Emit control register and idle state
  unsigned totalFSMStates = MF->size() + 1;
  TotalFSMStatesBit = Log2_32_Ceil(totalFSMStates);

  VM->addRegister("NextFSMState", TotalFSMStatesBit);
  
  // Idle state
  verilogParam(VM->getStateDeclBuffer(), "state_idle", TotalFSMStatesBit, 0);
  verilogMatchCase(VM->getControlBlockBuffer(6), "state_idle");
  // Idle state is always ready.
  verilogIfBegin(VM->getControlBlockBuffer(8), "start");
  // The module is busy now
  MachineBasicBlock *EntryBB =  GraphTraits<MachineFunction*>::getEntryNode(MF);
  emitNextFSMState(VM->getControlBlockBuffer(), EntryBB);
  emitNextMicroState(VM->getControlBlockBuffer(10), EntryBB, "1'b1");
  //
  verilogIfElse(VM->getControlBlockBuffer(8));
  VM->getControlBlockBuffer(10) << "NextFSMState <= state_idle;\n";
  // Emit function unit control at idle state, simply disable all
  // function units.
  emitFUCtrl(0);
  verilogEnd(VM->getControlBlockBuffer(8));
  verilogEnd(VM->getControlBlockBuffer(6));
  
  emitAllRegister();
  emitAllocatedFUs();

  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock &BB = *I;
    emitBasicBlock(BB);
  }



  // Write buffers to output
  VM->printModuleDecl(Out);
  Out << "\n\n";
  // States
  verilogCommentBegin(Out.indent(2)) << "States\n";
  Out << VM->getStateDeclStr();
  Out << "\n\n";
  // Reg and wire
  verilogCommentBegin(Out.indent(2)) << "Reg and wire decl\n";
  VM->printSignalDecl(Out, 2);
  Out << "\n\n";

  // Datapath
  verilogCommentBegin(Out.indent(2)) << "Datapath\n";
  Out << VM->getDataPathStr();

  Out << "\n\n";
  verilogCommentBegin(Out.indent(2)) << "Always Block\n";
  verilogAlwaysBegin(Out, 2);
  VM->printRegisterReset(Out, 6);
  verilogIfElse(Out.indent(4));

  verilogCommentBegin(Out.indent(6)) << "SeqCompute:\n";
  Out << VM->getSeqComputeStr();

  verilogCommentBegin(Out.indent(6)) << "FSM\n";
  verilogSwitchCase(Out.indent(6), "NextFSMState");
  Out << VM->getControlBlockStr();
  // Case default.
  Out.indent(6) << "default:  NextFSMState <= state_idle;\n";
  verilogEndSwitch(Out.indent(6));
  verilogAlwaysEnd(Out, 2);

  verilogEndModule(Out);

  return false;
}

void RTLWriter::clear() {
  Arguments.clear();

  delete VM;
}

void RTLWriter::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.addRequired<BitLevelInfo>();
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
  verilogCommentBegin(VM->getStateDeclBuffer()) << "State for " << StateName << '\n';
  verilogParam(VM->getStateDeclBuffer(), StateName, TotalFSMStatesBit,
               ++CurFSMStateNum);

  // State information.
  verilogCommentBegin(VM->getControlBlockBuffer(6)) << StateName 
    << " Total Slot: " << totalSlot
    << " II: " << 0 <<  '\n';
  // Mirco state enable.
  createucStateEnable(&MBB);

  // Case begin
  verilogMatchCase(VM->getControlBlockBuffer(6), StateName);

  MachineBasicBlock::iterator I = MBB.getFirstNonPHI(), E = prior(MBB.end());
  while (I != E) {
    ucState CurState(*I);
    // Emit the datepath of current state.
    emitDatapath(CurState);
    
    // Emit next ucOp.
    ucState NextState(*(++I));
    verilogIfBegin(VM->getControlBlockBuffer(8), getucStateEnable(CurState));
    emitCtrlOp(NextState);
    verilogEnd(VM->getControlBlockBuffer(8));
  }
  
  //for (unsigned i = StartSlot, e = EndSlot + 1; i != e; ++i) {
  //  verilogIfBegin(VM->getControlBlockBuffer(8), getucStateEnable(State, i, true));
  //  // Emit all atoms at cycle i

  //  verilogCommentBegin(VM->getDataPathBuffer(2)) << "at cycle: " << i << '\n';
  //  for (cycle_iterator CI = Atoms.lower_bound(i), CE = Atoms.upper_bound(i);
  //      CI != CE; ++CI)
  //    emitAtom(CI->second);
  //  verilogEnd(VM->getControlBlockBuffer(8));
  //}// end for

  //// Emit Self Loop logic.
  //if (State->haveSelfLoop()) {
  //  verilogCommentBegin(VM->getControlBlockBuffer(8)) << "For self loop:\n";
  //  std::string SelfLoopEnable = computeSelfLoopEnable(State);
  //  emitNextMicroState(VM->getControlBlockBuffer(8), BB, SelfLoopEnable);
  //} else
    emitNextMicroState(VM->getControlBlockBuffer(8), &MBB, "1'b0");
  // Case end
  verilogEnd(VM->getControlBlockBuffer(6));
}

void RTLWriter::emitCommonPort() {
  VM->addInputPort("clk", 1);
  VM->addInputPort("rstN", 1);
  VM->addInputPort("start", 1);
  VM->addOutputPort("fin", 1);
}

void RTLWriter::emitAllocatedFUs() {
  // Dirty Hack: only Memory bus supported at this moment.
  typedef VFuncInfo::const_id_iterator id_iterator;

  VFUMemBus *MemBus = VTM.getFUDesc<VFUMemBus>();

  for (id_iterator I = FuncInfo->id_begin(VFUs::MemoryBus),
       E = FuncInfo->id_end(VFUs::MemoryBus); I != E; ++I) {
    // FIXME: In fact, *I return the FUId instead of FUNum. 
    unsigned FUNum = (*I).getFUNum();
    // Address port.
    VM->addOutputPort(VFUMemBus::getAddrBusName(FUNum),
                      MemBus->getAddrWidth());

    // Data ports.
    VM->addInputPort(VFUMemBus::getInDataBusName(FUNum),
                     MemBus->getDataWidth());
    VM->addOutputPort(VFUMemBus::getOutDataBusName(FUNum),
                      MemBus->getDataWidth());

    // Control ports.
    VM->addOutputPort(VFUMemBus::getEnableName(FUNum), 1);
    VM->addOutputPort(VFUMemBus::getWriteEnableName(FUNum), 1);
  }
  
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
      I != E; ++I) {
    unsigned RegNum = *I;

    // Do not emit dead registers.
    if (MRI.use_empty(RegNum)) continue;
    
    VM->addRegister("reg" + utostr(RegNum), BitWidth);
  }
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

  if (totalSlot > 1)
    ss << "{ " << StateName << verilogBitRange(totalSlot - 1) << ", ";

  ss << NewState;

  if (totalSlot > 1)
    ss << " }";

  ss << ";\n";
}

void RTLWriter::emitFUCtrl(unsigned Slot) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  // The FSM operation.
  if (FuncInfo->isFUActiveAt(VFUs::FSMFinish, Slot))
    OS.indent(10) << "fin <= 1'b1;\n";
  else
    OS.indent(10) << "fin <= 1'b0;\n";
  
  // Membus control operation.
  for (VFuncInfo::const_id_iterator I = FuncInfo->id_begin(VFUs::MemoryBus),
       E = FuncInfo->id_end(VFUs::MemoryBus); I != E; ++I) {
    FuncUnitId Id = *I;

    OS.indent(10) << VFUMemBus::getEnableName(Id.getFUNum());
    // Enable the membus if it is activated.
    if (FuncInfo->isFUActiveAt(Id, Slot)) OS << " <= 1'b1;\n";
    else                                  OS << " <= 1'b0;\n";
  }
  
}

void RTLWriter::emitCtrlOp(ucState &State) {
  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;

    if (Op.haveDataPath())
      continue;

    // Emit the operations.
    switch (Op.getOpCode()) {
    case VTM::VOpArg:       emitOpArg(Op);        break;
    case VTM::VOpRetVal:    emitOpRetVal(Op);     break;
    case VTM::VOpRet:       emitOpRet(Op);        break;
    case VTM::VOpWriteReg:  emitOpWriteReg(Op);   break;
    case VTM::VOpMemAccess: emitOpMemAccess(Op);  break;
    default:  assert(0 && "Unexpect opcode!");    break;
    }
  }

  emitFUCtrl(State.getSlot());
}

void RTLWriter::emitFirstCtrlState(MachineBasicBlock *MBB) {
  // TODO: Emit PHINodes if necessary.
  ucState FirstState(MBB->front());
  
  emitCtrlOp(FirstState);
}

void RTLWriter::emitOpWriteReg(ucOp &OpWriteReg) {
  raw_ostream &OS = VM->getControlBlockBuffer(10);
  MachineOperand &MO = OpWriteReg.getOperand(0);
  emitOperand(OS, MO);
  MachineBasicBlock *MBB = MO.getParent()->getParent();
  std::string BBName = getStateNameForMachineBB(MBB);
  OS << " <= " << OpWriteReg.getOpCodeMD().getWireName(BBName) << ";\n";
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

void RTLWriter::emitOpMemAccess(ucOp &OpMemAccess) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  unsigned FUNum = OpMemAccess.getFUNum();

  // Assign store data.
  OS.indent(10) << VFUMemBus::getOutDataBusName(FUNum) << " <= ";
  emitOperand(OS, OpMemAccess.getOperand(1));
  OS << ";\n";
  // Emit Address.
  OS.indent(10) << VFUMemBus::getAddrBusName(FUNum) << " <= ";
  emitOperand(OS, OpMemAccess.getOperand(2));
  OS << ";\n";
  // And write enable, write is enable if the operation is NOT load.
  OS.indent(10) << VFUMemBus::getWriteEnableName(FUNum) << " <= ~";
  emitOperand(OS, OpMemAccess.getOperand(3));
  OS << ";\n";
}

unsigned RTLWriter::getOperandWitdh(MachineOperand &Operand) {
  switch (Operand.getType()) {
  case MachineOperand::MO_Register: {
    const TargetRegisterClass *RC = MRI->getRegClass(Operand.getReg());
    return RC->vt_begin()->getSizeInBits();
  }
  case MachineOperand::MO_Metadata: {
    MetaToken MetaOp(Operand.getMetadata());
    return MetaOp.getBitWidth(); 
  }
  }
  assert(0 && "Unknown bitwidth!");
  return 0;
}

void RTLWriter::emitOperand(raw_ostream &OS, MachineOperand &Operand,
                            bool PrintBitRange) {
  switch (Operand.getType()) {
  case MachineOperand::MO_Register: {
    OS << "reg" << Operand.getReg();

    if (PrintBitRange)
      OS << verilogBitRange(BLI->getBitWidth(Operand));

    return;
  }
  case MachineOperand::MO_Metadata: {
    MetaToken MetaOp(Operand.getMetadata());
    assert((MetaOp.isDefWire() || MetaOp.isReadWire()) && "Bad operand!");

    MachineBasicBlock *MBB = Operand.getParent()->getParent();
    std::string WireName = MetaOp.getWireName(getStateNameForMachineBB(MBB));
    OS << WireName;

    // Emit the wire here, because it only define once, wires will never
    // be emitted more than once.
    if (MetaOp.isDefWire())
      VM->addWire(WireName, MetaOp.getBitWidth());
    
    return;
  }
  case MachineOperand::MO_Immediate:
    OS << verilogConstToStr(Operand.getImm(), BLI->getBitWidth(Operand), false);
    return;
  case MachineOperand::MO_ExternalSymbol:
    OS << Operand.getSymbolName();
    return;
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
    case VTM::VOpAdd:       emitOpAdd(Op);          break;
    case VTM::VOpXor:       emitBinOp(Op, "^");     break;
    case VTM::VOpSHL:       emitBinOp(Op, "<<");    break;
    case VTM::VOpBitCat:    emitOpBitCat(Op);       break;
    case VTM::VOpBitSlice:  emitOpBitSlice(Op);     break;
    case VTM::VOpBitRepeat: emitOpBitRepeat(Op);    break;
    }
  } 
}

void RTLWriter::emitBinOp(ucOp &BinOp, const std::string &Operator) {
  raw_ostream &OS = VM->getDataPathBuffer(2);
  OS << "assign ";
  emitOperand(OS, BinOp.getOperand(0));
  OS << " = ";
  emitOperand(OS, BinOp.getOperand(1));
  OS << ' ' << Operator << ' ';
  emitOperand(OS, BinOp.getOperand(2));
  OS << ";\n";
}

void RTLWriter::emitOpAdd(ucOp &OpAdd) {
  raw_ostream &OS = VM->getDataPathBuffer(2);

  OS << "assign {";
  // Carry out.
  emitOperand(OS, OpAdd.getOperand(1));
  OS << ", ";
  // Sum.
  emitOperand(OS, OpAdd.getOperand(0));
  OS << "} = ";
  // Operands.
  emitOperand(OS, OpAdd.getOperand(2));
  OS << " + ";
  emitOperand(OS, OpAdd.getOperand(3));
  OS << " + ";
  // Carry in.
  emitOperand(OS, OpAdd.getOperand(4));
  OS << ";\n";
}

void RTLWriter::emitOpBitSlice(ucOp &OpBitSlice) {
  raw_ostream &OS = VM->getDataPathBuffer(2);
  // Get the range of the bit slice, Note that the
  // bit at upper bound is excluded in VOpBitSlice,
  // now we are going to get the included upper bound.
  unsigned UB = OpBitSlice.getOperand(2).getImm(),
           LB = OpBitSlice.getOperand(3).getImm();

  OS << "assign ";
  emitOperand(OS, OpBitSlice.getOperand(0));
  OS << " = ";
  emitOperand(OS, OpBitSlice.getOperand(1), false);
  OS << verilogBitRange(UB, LB) << ";\n";
}

void RTLWriter::emitOpBitCat(ucOp &OpBitCat) {
  raw_ostream &OS = VM->getDataPathBuffer(2);
  OS << "assign ";
  emitOperand(OS, OpBitCat.getOperand(0));
  OS << " = {";
  
  // Skip the dst operand.

  ucOp::op_iterator I = OpBitCat.op_begin() + 1;
  emitOperand(OS, *I);
  while (++I != OpBitCat.op_end()) {
    OS << ',';
    emitOperand(OS, *I);
  }

  OS << "};\n";
}

void RTLWriter::emitOpBitRepeat(ucOp &OpBitRepeat) {
  raw_ostream &OS = VM->getDataPathBuffer(2);
  OS << "assign ";
  emitOperand(OS, OpBitRepeat.getOperand(0));
  OS << " = {";

  unsigned Times = OpBitRepeat.getOperand(2).getImm();
  OS << Times << '{';
  emitOperand(OS, OpBitRepeat.getOperand(1));
  OS << "}};\n";
}

Pass *llvm::createRTLWriterPass(VTargetMachine &TM, raw_ostream &O) {
  return new RTLWriter(TM, O);
}
