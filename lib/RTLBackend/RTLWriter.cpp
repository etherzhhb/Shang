//===- Writer.cpp - VTM machine instructions to RTL verilog  ----*- C++ -*-===//
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
// This file implement the RTLWriter pass, which write VTM machine instructions
// in form of RTL verilog code.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/RTLInfo.h"
#include "vtm/BitLevelInfo.h"
#include "vtm/FileInfo.h"

#include "llvm/Type.h"

#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetMachine.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SmallString.h"

#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"

#include "llvm/Support/Debug.h"

using namespace llvm;

//===----------------------------------------------------------------------===//
char RTLInfo::ID = 0;

Pass *llvm::createRTLInfoPass(VTargetMachine &TM) {
  return new RTLInfo(TM);
}

INITIALIZE_PASS_BEGIN(RTLInfo, "vtm-rtl-info",
                      "Build RTL Verilog module for synthesised function.",
                      false, true)
INITIALIZE_PASS_DEPENDENCY(BitLevelInfo);
INITIALIZE_PASS_END(RTLInfo, "vtm-rtl-info",
                    "Build RTL Verilog module for synthesised function.",
                    false, true)

RTLInfo::RTLInfo() : MachineFunctionPass(ID), Out() ,
  VTM((VTargetMachine&)TheVBackendTarget) {
  initializeRTLInfoPass(*PassRegistry::getPassRegistry());
}
RTLInfo::RTLInfo(VTargetMachine &TM) 
  : MachineFunctionPass(ID), Out(), VTM(TM), VM(0), Mang(0), 
    TotalFSMStatesBit(0), CurFSMStateNum(0) {
  initializeRTLInfoPass(*PassRegistry::getPassRegistry());
}

bool RTLInfo::doInitialization(Module &M) {
  MachineModuleInfo *MMI = getAnalysisIfAvailable<MachineModuleInfo>();
  assert(MMI && "MachineModuleInfo will always available"
    " in a machine function pass!");
  Mang = new Mangler(MMI->getContext(), *VTM.getTargetData());

  FOut = vtmfiles().getRTLOut();
  Out.setStream(FOut->os());
  return false;
}

bool RTLInfo::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  FuncInfo = MF->getInfo<VFuncInfo>();
  MRI = &MF->getRegInfo();
  BLI = &getAnalysis<BitLevelInfo>();

  // FIXME: Demangle the c++ name.
  // Dirty Hack: Force the module have the name of the hw subsystem.
  VM = new VASTModule(vtmfiles().getSystemName());
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

  // Dirty Hack: Disable the "Warning-WIDTH" from verilator.
  // FIXME: We should only generate this when we are going to simulate the
  // module with verilator.
  Out << "/* verilator lint_off WIDTH */\n";

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
  Out.flush();

  return false;
}

void RTLInfo::clear() {
  Arguments.clear();

  delete VM;
}

void RTLInfo::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.addRequired<BitLevelInfo>();
  AU.setPreservesAll();
}

void RTLInfo::print(raw_ostream &O, const Module *M) const {

}

void RTLInfo::emitFunctionSignature() {
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

void RTLInfo::emitBasicBlock(MachineBasicBlock &MBB) {
  std::string StateName = getStateName(&MBB);
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

  MachineBasicBlock::iterator I = MBB.getFirstNonPHI(),
                              E = MBB.getFirstTerminator();
  // FIXME: Refactor the loop.
  do {
    ++I;
    ucState CurDatapath = *I;
    // Emit the datepath of current state.
    emitDatapath(CurDatapath);

    // Emit next ucOp.
    ucState NextControl = *++I;
    verilogIfBegin(VM->getControlBlockBuffer(8), getucStateEnable(CurDatapath));
    emitCtrlOp(NextControl);
    verilogEnd(VM->getControlBlockBuffer(8));
  } while(I != E);

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

void RTLInfo::emitCommonPort() {
  VM->addInputPort("clk", 1, VASTModule::Clk);
  VM->addInputPort("rstN", 1, VASTModule::RST);
  VM->addInputPort("start", 1, VASTModule::Start);
  VM->addOutputPort("fin", 1, VASTModule::Finish);
}

void RTLInfo::emitAllocatedFUs() {
  // Dirty Hack: only Memory bus supported at this moment.
  typedef VFuncInfo::const_id_iterator id_iterator;

  VFUMemBus *MemBus = vtmfus().getFUDesc<VFUMemBus>();

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

void RTLInfo::emitAllRegister() {
  emitRegClassRegs(VTM::DR1RegisterClass, 1);
  emitRegClassRegs(VTM::DR8RegisterClass, 8);
  emitRegClassRegs(VTM::DR16RegisterClass, 16);
  emitRegClassRegs(VTM::DR32RegisterClass, 32);
  emitRegClassRegs(VTM::DR64RegisterClass, 64);
}

void RTLInfo::emitRegClassRegs(const TargetRegisterClass *RC,
                                 unsigned BitWidth) {
  MachineRegisterInfo &MRI = MF->getRegInfo();
  for (TargetRegisterClass::iterator I = RC->begin(), E = RC->end();
      I != E; ++I) {
    unsigned RegNum = *I;

    if (MRI.isPhysRegUsed(RegNum))
      VM->addRegister("reg" + utostr(RegNum), BitWidth);
  }
}

RTLInfo::~RTLInfo() {}

//===----------------------------------------------------------------------===//
void RTLInfo::createucStateEnable(MachineBasicBlock *MBB)  {
  std::string StateName = getStateName(MBB);
  // We do not need the last state.
  unsigned totalSlot = FuncInfo->getTotalSlotFor(MBB) - 1;

  // current state
  VM->addRegister("cur_" + StateName + "_enable", totalSlot);
}

void RTLInfo::emitNextMicroState(raw_ostream &ss, MachineBasicBlock *MBB,
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

void RTLInfo::emitFUCtrl(unsigned Slot) {
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

void RTLInfo::emitCtrlOp(ucState &State) {
  assert((State->getOpcode() == VTM::Control
          // ToDo: sepreate terminator from control.
          || State->getOpcode() == VTM::Terminator)
        && "Bad ucState!");
  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    // Emit the operations.
    switch (Op.getOpCode()) {
    case VTM::VOpArg:       emitOpArg(Op);        break;
    case VTM::VOpRetVal:    emitOpRetVal(Op);     break;
    case VTM::VOpRet:       emitOpRet(Op);        break;
    case VTM::COPY:         emitOpLatchVal(Op);   break;
    case VTM::VOpMemAccess: emitOpMemAccess(Op);  break;
    case VTM::VOpToState:   emitOpToState(Op);    break;
    default:  assert(0 && "Unexpect opcode!");    break;
    }
  }

  if (!State->getDesc().isTerminator())  emitFUCtrl(State.getSlot());
}

void RTLInfo::emitFirstCtrlState(MachineBasicBlock *MBB) {
  // TODO: Emit PHINodes if necessary.
  ucState FirstState = *MBB->getFirstNonPHI();
  emitCtrlOp(FirstState);
}

void RTLInfo::emitOpLatchVal(ucOp &OpLatchVal) {
  raw_ostream &OS = VM->getControlBlockBuffer(10);
  MachineOperand &MO = OpLatchVal.getOperand(0);
  emitOperand(OS, MO);
  MachineBasicBlock *MBB = MO.getParent()->getParent();
  std::string BBName = getStateName(MBB);
  OS << " <= " << OpLatchVal.getSrcWireName(BBName) << ";\n";
}

void RTLInfo::emitOpArg(ucOp &OpArg) {
  // Assign input port to some register.
  raw_ostream &OS = VM->getControlBlockBuffer(10);
  emitOperand(OS, OpArg.getOperand(0));
  // FIXME: Use getInputPort instead;
  OS << " <= " << VM->getCommonInPort(OpArg.getOperand(1).getImm()).getName()
     << ";\n";
}

void RTLInfo::emitOpRet(ucOp &OpArg) {
  raw_ostream &OS = VM->getControlBlockBuffer(10);
  OS << "NextFSMState <= state_idle;\n";
  //
  OS.indent(10) << "fin <= 1'b1;\n";
}

void RTLInfo::emitOpToState(ucOp &OpToState) {
  raw_ostream &OS = VM->getControlBlockBuffer();

  // Get the condition.
  std::string s;
  raw_string_ostream ss(s);
  emitOperand(ss, OpToState.getOperand(0));
  verilogIfBegin(OS.indent(10), ss.str());
  OS.indent(12) << "NextFSMState <= ";
  emitOperand(OS, OpToState.getOperand(1));
  OS << ";\n";
  emitFirstCtrlState(OpToState.getOperand(1).getMBB());
  verilogEnd(VM->getControlBlockBuffer(10));
}

void RTLInfo::emitOpRetVal(ucOp &OpRetVal) {
  raw_ostream &OS = VM->getControlBlockBuffer(10);
  unsigned retChannel = OpRetVal.getOperand(1).getImm();
  assert(retChannel == 0 && "Only support Channel 0!");
  OS << "return_value <= ";
  emitOperand(OS, OpRetVal.getOperand(0));
  OS << ";\n";
}

void RTLInfo::emitOpMemAccess(ucOp &OpMemAccess) {
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

unsigned RTLInfo::getOperandWitdh(MachineOperand &Operand) {
  switch (Operand.getType()) {
  case MachineOperand::MO_Register: {
    const TargetRegisterClass *RC = MRI->getRegClass(Operand.getReg());
    return RC->vt_begin()->getSizeInBits();
  }
  case MachineOperand::MO_Metadata: {
    MetaToken MetaOp(Operand.getMetadata());
    return MetaOp.getBitWidth(); 
  }
  default:
    assert(0 && "Unknown bitwidth!");
  }
  return 0;
}

void RTLInfo::emitOperand(raw_ostream &OS, MachineOperand &Operand,
                            bool PrintBitRange) {
  switch (Operand.getType()) {
  case MachineOperand::MO_Register: {
    OS << "reg" << Operand.getReg();

    unsigned BitWidth = BLI->getBitWidth(Operand);
    // Do not print out reg[0] if register has only one bit.
    if (PrintBitRange && BitWidth != 1) {
      OS << verilogBitRange(BitWidth);
    }

    return;
  }
  case MachineOperand::MO_Metadata: {
    MetaToken MetaOp(Operand.getMetadata());
    assert((MetaOp.isDefWire() || MetaOp.isReadWire()) && "Bad operand!");

    MachineBasicBlock *MBB = Operand.getParent()->getParent();
    std::string WireName = MetaOp.getWireName(getStateName(MBB));
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
  case MachineOperand::MO_MachineBasicBlock:
    OS << getStateName(Operand.getMBB());
    return;
  default:
    assert(0 && "Unknown Operand type!");
  }
}

void RTLInfo::emitNextFSMState(raw_ostream &ss, MachineBasicBlock *MBB) {
  // Emit the first micro state of the target state.
  emitFirstCtrlState(MBB);

  ss.indent(10) << "NextFSMState <= " << getStateName(MBB) << ";\n";
}

void RTLInfo::emitDatapath(ucState &State) {
  assert(State->getOpcode() == VTM::Datapath && "Bad ucState!");
  verilogCommentBegin(VM->getDataPathBuffer(2)) << "Issue datapath for "
    "operations at slot " << State.getSlot() << '\n';

  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    switch (Op.getOpCode()) {
    default:  assert(0 && "Unexpect opcode!");      break;
    case VTM::VOpAdd:       emitOpAdd(Op);          break;

    case VTM::VOpXor:       emitBinaryOp(Op, "^");  break;
    case VTM::VOpAnd:       emitBinaryOp(Op, "&");  break;
    case VTM::VOpOr:        emitBinaryOp(Op, "|");  break;

    case VTM::VOpSHL:       emitBinaryOp(Op, "<<"); break;
    // FIXME: Add signed modifier to the first operand.
    case VTM::VOpSRA:       emitBinaryOp(Op, ">>>");break;

    case VTM::VOpNot:       emitUnaryOp(Op, "~");   break;

    case VTM::VOpROr:       emitUnaryOp(Op, "~");   break;
    case VTM::VOpRAnd:      emitUnaryOp(Op, "&");   break;
    case VTM::VOpRXor:      emitUnaryOp(Op, "^");   break;

    case VTM::VOpBitCat:    emitOpBitCat(Op);       break;
    case VTM::VOpBitSlice:  emitOpBitSlice(Op);     break;
    case VTM::VOpBitRepeat: emitOpBitRepeat(Op);    break;
    }
  } 
}
void RTLInfo::emitUnaryOp(ucOp &BinOp, const std::string &Operator) {
  raw_ostream &OS = VM->getDataPathBuffer(2);
  OS << "assign ";
  emitOperand(OS, BinOp.getOperand(0));
  OS << " = " << Operator << ' ';
  emitOperand(OS, BinOp.getOperand(1));
  OS << ";\n";
}

void RTLInfo::emitBinaryOp(ucOp &BinOp, const std::string &Operator) {
  raw_ostream &OS = VM->getDataPathBuffer(2);
  OS << "assign ";
  emitOperand(OS, BinOp.getOperand(0));
  OS << " = ";
  emitOperand(OS, BinOp.getOperand(1));
  OS << ' ' << Operator << ' ';
  emitOperand(OS, BinOp.getOperand(2));
  OS << ";\n";
}

void RTLInfo::emitOpAdd(ucOp &OpAdd) {
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

void RTLInfo::emitOpBitSlice(ucOp &OpBitSlice) {
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

void RTLInfo::emitOpBitCat(ucOp &OpBitCat) {
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

void RTLInfo::emitOpBitRepeat(ucOp &OpBitRepeat) {
  raw_ostream &OS = VM->getDataPathBuffer(2);
  OS << "assign ";
  emitOperand(OS, OpBitRepeat.getOperand(0));
  OS << " = {";

  unsigned Times = OpBitRepeat.getOperand(2).getImm();
  OS << Times << '{';
  emitOperand(OS, OpBitRepeat.getOperand(1));
  OS << "}};\n";
}
