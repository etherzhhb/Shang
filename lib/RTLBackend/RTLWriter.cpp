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

Pass *llvm::createRTLInfoPass() {
  return new RTLInfo();
}

INITIALIZE_PASS_BEGIN(RTLInfo, "vtm-rtl-info",
                      "Build RTL Verilog module for synthesised function.",
                      false, true)
INITIALIZE_PASS_DEPENDENCY(BitLevelInfo);
INITIALIZE_PASS_END(RTLInfo, "vtm-rtl-info",
                    "Build RTL Verilog module for synthesised function.",
                    false, true)

RTLInfo::RTLInfo() : MachineFunctionPass(ID), Out() {
  initializeRTLInfoPass(*PassRegistry::getPassRegistry());
}

bool RTLInfo::doInitialization(Module &M) {
  MachineModuleInfo *MMI = getAnalysisIfAvailable<MachineModuleInfo>();
  TargetData *TD = getAnalysisIfAvailable<TargetData>();

  assert(MMI && TD && "MachineModuleInfo and TargetData will always available"
                      " in a machine function pass!");
  Mang = new Mangler(MMI->getContext(), *TD);

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
  VM->getControlBlockBuffer().match_case("state_idle");
  // Idle state is always ready.
  VM->getControlBlockBuffer().if_begin("start");
  // The module is busy now
  MachineBasicBlock *EntryBB =  GraphTraits<MachineFunction*>::getEntryNode(MF);
  emitNextFSMState(VM->getControlBlockBuffer(), EntryBB);
  //
  VM->getControlBlockBuffer().else_begin();
  VM->getControlBlockBuffer() << "NextFSMState <= state_idle;\n";
  // Emit function unit control at idle state, simply disable all
  // function units.
  emitFUCtrl(0);
  VM->getControlBlockBuffer().exit_block();
  VM->getControlBlockBuffer().exit_block();
  
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
  Out.module_begin();
  Out << "\n\n";
  // States
  Out << "// States\n";
  Out << VM->getStateDeclStr();
  Out << "\n\n";
  // Reg and wire
  Out << "// Reg and wire decl\n";
  VM->printSignalDecl(Out);
  Out << "\n\n";

  // Datapath
  Out << "// Datapath\n";
  Out << VM->getDataPathStr();

  Out << "\n\n";
  Out << "// Always Block\n";
  Out.always_ff_begin();
  VM->printRegisterReset(Out);
  Out.else_begin();

  Out << "// FSM\n";
  Out.switch_begin("NextFSMState");
  Out << VM->getControlBlockStr();
  // Case default.
  Out << "default:  NextFSMState <= state_idle;\n";
  Out.switch_end();
  Out.always_ff_end();

  Out.module_end();
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
  VM->getStateDeclBuffer() << "// State for " << StateName << '\n';
  verilogParam(VM->getStateDeclBuffer(), StateName, TotalFSMStatesBit,
               ++CurFSMStateNum);

  // State information.
  VM->getControlBlockBuffer() << "// " << StateName 
                              << " Total Slot: " << totalSlot
                              << " II: " << 0 <<  '\n';
  // Mirco state enable.
  createucStateEnable(&MBB);

  // Case begin
  VM->getControlBlockBuffer().match_case(StateName);

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
    VM->getControlBlockBuffer().if_begin(getucStateEnable(CurDatapath));
    emitCtrlOp(NextControl);
    VM->getControlBlockBuffer().exit_block();
  } while(I != E);

  //// Emit Self Loop logic.
  //if (State->haveSelfLoop()) {
  //  VM->getControlBlockBuffer() << "// For self loop:\n";
  //  std::string SelfLoopEnable = computeSelfLoopEnable(State);
  //  emitNextMicroState(VM->getControlBlockBuffer(), BB, SelfLoopEnable);
  //} else
    emitNextMicroState(VM->getControlBlockBuffer(), &MBB, "1'b0");
  // Case end
  VM->getControlBlockBuffer().exit_block();
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
    FuncUnitId ID = *I;
    unsigned FUNum = ID.getFUNum();

    VM->setFUPortBegin(ID);
    // Control ports.
    VM->addOutputPort(VFUMemBus::getEnableName(FUNum), 1);
    VM->addOutputPort(VFUMemBus::getWriteEnableName(FUNum), 1);

    // Address port.
    VM->addOutputPort(VFUMemBus::getAddrBusName(FUNum),
                      MemBus->getAddrWidth());

    // Data ports.
    VM->addInputPort(VFUMemBus::getInDataBusName(FUNum),
                     MemBus->getDataWidth());
    VM->addOutputPort(VFUMemBus::getOutDataBusName(FUNum),
                      MemBus->getDataWidth());
    // Data size, in bytes.
    VM->addOutputPort(VFUMemBus::getDataSizeName(FUNum),
                      Log2_32_Ceil(MemBus->getDataWidth() / 8));
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
    OS << "fin <= 1'b1;\n";
  else
    OS << "fin <= 1'b0;\n";
  
  // Membus control operation.
  for (VFuncInfo::const_id_iterator I = FuncInfo->id_begin(VFUs::MemoryBus),
       E = FuncInfo->id_end(VFUs::MemoryBus); I != E; ++I) {
    FuncUnitId Id = *I;

    OS << VFUMemBus::getEnableName(Id.getFUNum());
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
  bool IsRet = false;
  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    // Emit the operations.
    switch (Op.getOpCode()) {
    case VTM::VOpArg:       emitOpArg(Op);                break;
    case VTM::VOpRetVal:    emitOpRetVal(Op);             break;
    case VTM::VOpRet:       emitOpRet(Op); IsRet = true;  break;
    case VTM::COPY:         emitOpLatchVal(Op);           break;
    case VTM::VOpMemAccess: emitOpMemAccess(Op);          break;
    case VTM::VOpToState:   emitOpToState(Op);            break;
    case VTM::IMPLICIT_DEF: emitImplicitDef(Op);          break;
    default:  assert(0 && "Unexpect opcode!");            break;
    }
  }

  // Do not emit function unit when we are transferring to a new state, because
  // the first micro operation of the new state will be emitted immediately.
  // But if the terminator state is a return, we need to emit control logic
  // because there are no succeeding state.
  if (!State->getDesc().isTerminator() || IsRet)
    emitFUCtrl(State.getSlot());
}

void RTLInfo::emitFirstCtrlState(MachineBasicBlock *MBB) {
  // TODO: Emit PHINodes if necessary.
  ucState FirstState = *MBB->getFirstNonPHI();
  assert(FuncInfo->getStartSlotFor(MBB) == FirstState.getSlot());
  emitCtrlOp(FirstState);
}

void RTLInfo::emitImplicitDef(ucOp &ImpDef) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  OS << "/*IMPLICIT_DEF " << ImpDef << "*/\n";
}

void RTLInfo::emitOpLatchVal(ucOp &OpLatchVal) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  MachineOperand &MO = OpLatchVal.getOperand(0);
  emitOperand(OS, MO);
  MachineBasicBlock *MBB = MO.getParent()->getParent();
  std::string BBName = getStateName(MBB);
  OS << " <= " << OpLatchVal.getSrcWireName(BBName) << ";\n";
}

void RTLInfo::emitOpArg(ucOp &OpArg) {
  // Assign input port to some register.
  raw_ostream &OS = VM->getControlBlockBuffer();
  emitOperand(OS, OpArg.getOperand(0));
  // FIXME: Use getInputPort instead;
  OS << " <= " << VM->getCommonPort(OpArg.getOperand(1).getImm()).getName()
     << ";\n";
}

void RTLInfo::emitOpRet(ucOp &OpArg) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  OS << "NextFSMState <= state_idle;\n";
}

void RTLInfo::emitOpToState(ucOp &OpToState) {
  vlang_raw_ostream &OS = VM->getControlBlockBuffer();

  // Get the condition.
  std::string s;
  raw_string_ostream ss(s);
  emitOperand(ss, OpToState.getOperand(0));
  OS.if_begin(ss.str());
  MachineBasicBlock *TargetBB = OpToState.getOperand(1).getMBB();
  emitNextFSMState(OS, TargetBB);
  VM->getControlBlockBuffer().exit_block();
}

void RTLInfo::emitOpRetVal(ucOp &OpRetVal) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  unsigned retChannel = OpRetVal.getOperand(1).getImm();
  assert(retChannel == 0 && "Only support Channel 0!");
  OS << "return_value <= ";
  emitOperand(OS, OpRetVal.getOperand(0));
  OS << ";\n";
}

void RTLInfo::emitOpMemAccess(ucOp &OpMemAccess) {
  unsigned FUNum = OpMemAccess.getFUNum();
  raw_ostream &DPS = VM->getDataPathBuffer();
  DPS << "// Dirty Hack: Emit the wire define by this operation\n"
         "// some register copying operation may need this wire.\n";
  DPS << "assign ";
  emitOperand(DPS, OpMemAccess.getOperand(0));
  DPS << " = " << VFUMemBus::getInDataBusName(FUNum) << ";\n";

  // Emit the control logic.
  raw_ostream &OS = VM->getControlBlockBuffer();
  // Assign store data.
  OS << VFUMemBus::getOutDataBusName(FUNum) << " <= ";
  emitOperand(OS, OpMemAccess.getOperand(1));
  OS << ";\n";
  // Emit Address.
  OS << VFUMemBus::getAddrBusName(FUNum) << " <= ";
  emitOperand(OS, OpMemAccess.getOperand(2));
  OS << ";\n";
  // And write enable, write is enable if the operation is NOT load.
  OS << VFUMemBus::getWriteEnableName(FUNum) << " <= ~";
  emitOperand(OS, OpMemAccess.getOperand(3));
  OS << ";\n";
  // The byte enable.
  // FIXME: This is the data size instead of the byte enable.
  OS << VFUMemBus::getDataSizeName(FUNum) << " <= ";
  emitOperand(OS, OpMemAccess.getOperand(4));
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

  ss << "NextFSMState <= " << getStateName(MBB) << ";\n";
  emitNextMicroState(VM->getControlBlockBuffer(), MBB, "1'b1");
}

void RTLInfo::emitDatapath(ucState &State) {
  assert(State->getOpcode() == VTM::Datapath && "Bad ucState!");
  VM->getDataPathBuffer() << "// Issue datapath for "
    "operations at slot " << State.getSlot() << '\n';

  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    switch (Op.getOpCode()) {
    case VTM::VOpBitSlice:  emitOpBitSlice(Op);     break;
    case VTM::VOpBitCat:    emitOpBitCat(Op);       break;
    case VTM::VOpBitRepeat: emitOpBitRepeat(Op);    break;

    case VTM::VOpSetRI:     emitOpSetRI(Op);        break;
    case VTM::VOpAdd:       emitOpAdd(Op);          break;

    case VTM::VOpXor:       emitBinaryOp(Op, "^");  break;
    case VTM::VOpAnd:       emitBinaryOp(Op, "&");  break;
    case VTM::VOpOr:        emitBinaryOp(Op, "|");  break;

    case VTM::VOpSHL:       emitBinaryOp(Op, "<<"); break;
    // FIXME: Add signed modifier to the first operand.
    case VTM::VOpSRA:       emitBinaryOp(Op, ">>>");break;

    case VTM::VOpNot:       emitUnaryOp(Op, "~");   break;

    case VTM::VOpROr:       emitUnaryOp(Op, "|");   break;
    case VTM::VOpRAnd:      emitUnaryOp(Op, "&");   break;
    case VTM::VOpRXor:      emitUnaryOp(Op, "^");   break;

    default:  assert(0 && "Unexpected opcode!");    break;
    }
  } 
}

void RTLInfo::emitOpSetRI(ucOp &OpSetRI) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, OpSetRI.getOperand(0));
  OS << " = ";
  emitOperand(OS, OpSetRI.getOperand(1));
  OS << ";\n";
}

void RTLInfo::emitUnaryOp(ucOp &UnaOp, const std::string &Operator) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, UnaOp.getOperand(0));
  OS << " = " << Operator << ' ';
  emitOperand(OS, UnaOp.getOperand(1));
  OS << ";\n";
}

void RTLInfo::emitBinaryOp(ucOp &BinOp, const std::string &Operator) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, BinOp.getOperand(0));
  OS << " = ";
  emitOperand(OS, BinOp.getOperand(1));
  OS << ' ' << Operator << ' ';
  emitOperand(OS, BinOp.getOperand(2));
  OS << ";\n";
}

void RTLInfo::emitOpAdd(ucOp &OpAdd) {
  raw_ostream &OS = VM->getDataPathBuffer();

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
  raw_ostream &OS = VM->getDataPathBuffer();
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
  raw_ostream &OS = VM->getDataPathBuffer();
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
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, OpBitRepeat.getOperand(0));
  OS << " = {";

  unsigned Times = OpBitRepeat.getOperand(2).getImm();
  OS << Times << '{';
  emitOperand(OS, OpBitRepeat.getOperand(1));
  OS << "}};\n";
}
