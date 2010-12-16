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
#include "vtm/VerilogAST.h"
#include "vtm/MicroState.h"
#include "vtm/VFuncInfo.h"
#include "vtm/VTargetMachine.h"
#include "vtm/LangSteam.h"
#include "vtm/BitLevelInfo.h"
#include "vtm/FileInfo.h"

#include "llvm/Type.h"

#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetMachine.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SmallString.h"

#include "llvm/ADT/StringExtras.h"

#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/ToolOutputFile.h"

#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"

#include "llvm/Support/Debug.h"

using namespace llvm;
namespace {
class RTLWriter : public MachineFunctionPass {
  tool_output_file *FOut;
  vlang_raw_ostream Out;

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

  void emitOpSetRI(ucOp &OpSetRI);

  void emitUnaryOp(ucOp &UnOp, const std::string &Operator);
  void emitBinaryOp(ucOp &BinOp, const std::string &Operator);

  void emitOpAdd(ucOp &OpAdd);

  void emitOpBitCat(ucOp &OpBitCat);
  void emitOpBitSlice(ucOp &OpBitSlice);
  void emitOpBitRepeat(ucOp &OpBitRepeat);

  void emitImplicitDef(ucOp &ImpDef);

  void emitOperand(raw_ostream &OS, MachineOperand &Operand,
                   bool PrintBitRange = true);
  unsigned getOperandWitdh(MachineOperand &Operand);

  // Return true if the control operation loop back to the entry of the state
  // this can detect the loop boundary of the pipelined loop.
  bool emitCtrlOp(ucState &State, std::string &LoopPred);

  void emitOpArg(ucOp &VOpArg);
  void emitOpRetVal(ucOp &OpRetVal);
  void emitOpRet(ucOp &OpRet);
  void emitOpLatchVal(ucOp &OpLatchVal);
  void emitOpMemAccess(ucOp &OpMemAccess);

  void emitFUCtrl(unsigned Slot);

  void emitTestBench();

public:
  /// @name FunctionPass interface
  //{
  static char ID;
  RTLWriter();

  ~RTLWriter();

  bool doInitialization(Module &M);

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

//===----------------------------------------------------------------------===//
char RTLWriter::ID = 0;

Pass *llvm::createRTLWriterPass() {
  return new RTLWriter();
}

INITIALIZE_PASS_BEGIN(RTLWriter, "vtm-rtl-info",
                      "Build RTL Verilog module for synthesised function.",
                      false, true)
INITIALIZE_PASS_DEPENDENCY(BitLevelInfo);
INITIALIZE_PASS_END(RTLWriter, "vtm-rtl-info",
                    "Build RTL Verilog module for synthesised function.",
                    false, true)

RTLWriter::RTLWriter() : MachineFunctionPass(ID), Out() {
  initializeRTLWriterPass(*PassRegistry::getPassRegistry());
}

bool RTLWriter::doInitialization(Module &M) {
  MachineModuleInfo *MMI = getAnalysisIfAvailable<MachineModuleInfo>();
  TargetData *TD = getAnalysisIfAvailable<TargetData>();

  assert(MMI && TD && "MachineModuleInfo and TargetData will always available"
                      " in a machine function pass!");
  Mang = new Mangler(MMI->getContext(), *TD);

  FOut = vtmfiles().getRTLOut();
  Out.setStream(FOut->os());
  return false;
}

bool RTLWriter::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  FuncInfo = MF->getInfo<VFuncInfo>();
  MRI = &MF->getRegInfo();
  BLI = &getAnalysis<BitLevelInfo>();

  // Reset the current fsm state number.
  CurFSMStateNum = 0;

  // FIXME: Demangle the c++ name.
  // Dirty Hack: Force the module have the name of the hw subsystem.
  VM = FuncInfo->createRtlMod(vtmfiles().getSystemName());
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

  // TODO: Emit testbench only user wants to.
  DEBUG_WITH_TYPE("tb-for-rtl", emitTestBench());

  Out.flush();

  return false;
}

void RTLWriter::clear() {
  Arguments.clear();

  VM = 0;
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
  std::string StateName = getStateName(&MBB);
  unsigned totalSlot = FuncInfo->getTotalSlotFor(&MBB);
  unsigned IISlot = FuncInfo->getIISlotFor(&MBB);
  
  std::string NewMicroStatePred = "1'b0";

  vlang_raw_ostream &CtrlS = VM->getControlBlockBuffer();

  //unsigned StartSlot = State->getSlot(), EndSlot = State->getEndSlot();
  VM->getStateDeclBuffer() << "// State for " << StateName << '\n';
  verilogParam(VM->getStateDeclBuffer(), StateName, TotalFSMStatesBit,
               ++CurFSMStateNum);

  // State information.
  CtrlS << "// " << StateName << " Total Slot: " << totalSlot
                 << " IISlot: " << IISlot <<  '\n';
  // Mirco state enable.
  createucStateEnable(&MBB);

  // Case begin
  CtrlS.match_case(StateName);

  MachineBasicBlock::iterator I = MBB.getFirstNonPHI(),
                              E = MBB.getFirstTerminator();
  // FIXME: Refactor the loop.
  do {
    ucState CurDatapath = *++I;
    // Emit the datepath of current state.
    emitDatapath(CurDatapath);

    // Emit next ucOp.
    ucState NextControl = *++I;
    std::string StateEnable = getucStateEnable(CurDatapath);
    CtrlS.if_begin(StateEnable);
    if (emitCtrlOp(NextControl, NewMicroStatePred)) {
      // The predicate is only valid at the correct slot.
      NewMicroStatePred += " & " + StateEnable;
      // TODO: Tell the writer the loop boundary reached and do not emit
      // function unit control logic anymore.
    }
    CtrlS.exit_block();
  } while(I != E);

  emitNextMicroState(CtrlS, &MBB, NewMicroStatePred);

  // Case end
  CtrlS.exit_block();
}

void RTLWriter::emitCommonPort() {
  VM->addInputPort("clk", 1, VASTModule::Clk);
  VM->addInputPort("rstN", 1, VASTModule::RST);
  VM->addInputPort("start", 1, VASTModule::Start);
  VM->addOutputPort("fin", 1, VASTModule::Finish);
}

void RTLWriter::emitAllocatedFUs() {
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
  for (TargetRegisterClass::iterator I = RC->begin(), E = RC->end();
      I != E; ++I) {
    unsigned RegNum = *I;

    if (MRI.isPhysRegUsed(RegNum))
      VM->addRegister("reg" + utostr(RegNum), BitWidth);
  }
}

RTLWriter::~RTLWriter() {}

//===----------------------------------------------------------------------===//
void RTLWriter::createucStateEnable(MachineBasicBlock *MBB)  {
  std::string StateName = getStateName(MBB);
  // We do not need the last state.
  unsigned totalSlot = FuncInfo->getTotalSlotFor(MBB) - 1;

  // current state
  VM->addRegister("cur_" + StateName + "_enable", totalSlot);
}

void RTLWriter::emitNextFSMState(raw_ostream &ss, MachineBasicBlock *MBB) {
  // Emit the first micro state of the target state.
  emitFirstCtrlState(MBB);

  // Only jump to other state if target MBB is not current MBB.
  ss << "NextFSMState <= " << getStateName(MBB) << ";\n";
  emitNextMicroState(VM->getControlBlockBuffer(), MBB, "1'b1");
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

bool RTLWriter::emitCtrlOp(ucState &State, std::string &LoopPred) {
  assert((State->getOpcode() == VTM::Control
          // ToDo: sepreate terminator from control.
          || State->getOpcode() == VTM::Terminator)
        && "Bad ucState!");
  bool IsRet = false;
  bool loopToEntry = false;
  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    // Emit the operations.
    switch (Op.getOpCode()) {
    case VTM::VOpArg:       emitOpArg(Op);                break;
    case VTM::VOpRetVal:    emitOpRetVal(Op);             break;
    case VTM::VOpRet:       emitOpRet(Op); IsRet = true;  break;
    case VTM::COPY:         emitOpLatchVal(Op);           break;
    case VTM::VOpMemAccess: emitOpMemAccess(Op);          break;
    case VTM::IMPLICIT_DEF: emitImplicitDef(Op);          break;
    case VTM::VOpToState:{
      MachineBasicBlock *TargetBB = Op.getOperand(1).getMBB();
      vlang_raw_ostream &CtrlS = VM->getControlBlockBuffer();
      std::string Pred;
      raw_string_ostream ss(Pred);
      emitOperand(ss, Op.getOperand(0));
      ss.flush();
      CtrlS.if_begin(Pred);

      if (TargetBB == State->getParent()) { // Self loop detected.
        CtrlS << "// Issue first state.\n";
        emitFirstCtrlState(TargetBB);
        // Set up the self loop predicate.
        LoopPred = Pred;
        loopToEntry = true;
      } else // Transfer to other state.
        emitNextFSMState(CtrlS, TargetBB);

      CtrlS.exit_block();
      break;
    }
    default:  assert(0 && "Unexpect opcode!");            break;
    }
  }

  // Do not emit function unit when we are transferring to a new state, because
  // the first micro operation of the new state will be emitted immediately.
  // But if the terminator state is a return, we need to emit control logic
  // because there are no succeeding state.
  if (!State->getDesc().isTerminator() || IsRet)
    emitFUCtrl(State.getSlot());

  return loopToEntry;
}

void RTLWriter::emitFirstCtrlState(MachineBasicBlock *MBB) {
  // TODO: Emit PHINodes if necessary.
  ucState FirstState = *MBB->getFirstNonPHI();
  assert(FuncInfo->getStartSlotFor(MBB) == FirstState.getSlot());
  std::string dummy;
  emitCtrlOp(FirstState, dummy);
  assert(dummy.empty() && "Can not loop back at first state!");
}

void RTLWriter::emitImplicitDef(ucOp &ImpDef) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  OS << "/*IMPLICIT_DEF " << ImpDef << "*/\n";
}

void RTLWriter::emitOpLatchVal(ucOp &OpLatchVal) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  MachineOperand &MO = OpLatchVal.getOperand(0);
  emitOperand(OS, MO);
  MachineBasicBlock *MBB = MO.getParent()->getParent();
  std::string BBName = getStateName(MBB);
  OS << " <= " << OpLatchVal.getSrcWireName(BBName) << ";\n";
}

void RTLWriter::emitOpArg(ucOp &OpArg) {
  // Assign input port to some register.
  raw_ostream &OS = VM->getControlBlockBuffer();
  emitOperand(OS, OpArg.getOperand(0));
  // FIXME: Use getInputPort instead;
  OS << " <= " << VM->getCommonPort(OpArg.getOperand(1).getImm()).getName()
     << ";\n";
}

void RTLWriter::emitOpRet(ucOp &OpArg) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  OS << "NextFSMState <= state_idle;\n";
}

void RTLWriter::emitOpRetVal(ucOp &OpRetVal) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  unsigned retChannel = OpRetVal.getOperand(1).getImm();
  assert(retChannel == 0 && "Only support Channel 0!");
  OS << "return_value <= ";
  emitOperand(OS, OpRetVal.getOperand(0));
  OS << ";\n";
}

void RTLWriter::emitOpMemAccess(ucOp &OpMemAccess) {
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
  default:
    assert(0 && "Unknown bitwidth!");
  }
  return 0;
}

void RTLWriter::emitOperand(raw_ostream &OS, MachineOperand &Operand,
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

void RTLWriter::emitDatapath(ucState &State) {
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

void RTLWriter::emitOpSetRI(ucOp &OpSetRI) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, OpSetRI.getOperand(0));
  OS << " = ";
  emitOperand(OS, OpSetRI.getOperand(1));
  OS << ";\n";
}

void RTLWriter::emitUnaryOp(ucOp &UnaOp, const std::string &Operator) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, UnaOp.getOperand(0));
  OS << " = " << Operator << ' ';
  emitOperand(OS, UnaOp.getOperand(1));
  OS << ";\n";
}

void RTLWriter::emitBinaryOp(ucOp &BinOp, const std::string &Operator) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, BinOp.getOperand(0));
  OS << " = ";
  emitOperand(OS, BinOp.getOperand(1));
  OS << ' ' << Operator << ' ';
  emitOperand(OS, BinOp.getOperand(2));
  OS << ";\n";
}

void RTLWriter::emitOpAdd(ucOp &OpAdd) {
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

void RTLWriter::emitOpBitSlice(ucOp &OpBitSlice) {
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

void RTLWriter::emitOpBitCat(ucOp &OpBitCat) {
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

void RTLWriter::emitOpBitRepeat(ucOp &OpBitRepeat) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, OpBitRepeat.getOperand(0));
  OS << " = {";

  unsigned Times = OpBitRepeat.getOperand(2).getImm();
  OS << Times << '{';
  emitOperand(OS, OpBitRepeat.getOperand(1));
  OS << "}};\n";
}

void RTLWriter::emitTestBench() {
  Out << "\n\n";
  Out << "module tb_" << VM->getName() << ";\n\n";
  Out.module_begin();
  // Emit signal to drive the ports of DUT.
  Out << "// DUT port driver.\n";
  for (VASTModule::port_iterator I = VM->ports_begin(), E = VM->ports_end();
       I != E; ++I) {
    VASTPort *port = *I;
    port->printExternalDriver(Out);
    Out << '\n';
  }

  // Create the clock logic.
  Out << "\n\n";
  Out << "always ";
  Out.enter_block("// Clock\n");
  Out << "#5ns clk = ~clk;\n";
  Out.exit_block();
  Out << "\n\n";

  // And the initialize block.
  Out << "initial ";
  Out.enter_block();
  Out << "integer starttime = 0;\n";
  Out << "#6ns rstN = 1'b1;\n";
  Out << "#5ns;\n";
  Out << "forever ";
  Out.enter_block();

  for (VASTModule::port_iterator I = VM->common_ports_begin(),
       E = VM->ports_end(); I != E; ++I) {
    VASTPort *port = *I;
    if (!port->isInput())
      continue;

    const std::string &Name = port->getName();
    Out << Name << " = $random();\n";
  }

  Out << "@(negedge clk)";
  Out.enter_block();
  Out << "start = 1'b1;\n";
  Out << "starttime = $time;\n";
  Out.exit_block();

  Out <<  "@(negedge clk) start <= 1'b0;\n";

  Out <<  "while (!fin)";
  Out.enter_block();
  Out <<  "#1ns;\n";
  Out <<  "if ($time > 100000) $finish();\n";
  Out.exit_block();

  Out << "$display(\"total time: %t\\n\", $time - starttime);\n";
  Out << "$finish();\n";

  Out.exit_block("// end forever\n");
  Out.exit_block("// end initialize block\n");
  Out << "\n\n";
  
  // Design instance.
  Out << "// Design instance\n";
  Out  << VM->getName() << " dut_" << VM->getName() << "(\n";
  Out.indent(4) << "." << VM->getPortName(0) << '('
                << VM->getPortName(0) << ')';
  for (unsigned I = 1, E = VM->getNumPorts(); I != E; ++I) {
    Out << ",\n";
    Out.indent(4) << '.' << VM->getPortName(I)
                  << '(' << VM->getPortName(I) << ')';
  }
  Out << ");\n\n";
  
  Out.module_end();
}
