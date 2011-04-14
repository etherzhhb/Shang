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
// This file implement the RTLCodegen pass, which write VTM machine instructions
// in form of RTL verilog code.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/VerilogAST.h"
#include "vtm/MicroState.h"
#include "vtm/VFuncInfo.h"
#include "vtm/LangSteam.h"
#include "vtm/BitLevelInfo.h"
#include "vtm/FileInfo.h"
#include "vtm/VRegisterInfo.h"

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
class RTLCodegen : public MachineFunctionPass {
  tool_output_file *FOut;
  vlang_raw_ostream Out;

  MachineFunction *MF;
  VFuncInfo *FuncInfo;
  MachineRegisterInfo *MRI;
  BitLevelInfo *BLI;
  VASTModule *VM;
  Mangler *Mang;

  unsigned TotalFSMStatesBit, CurFSMStateNum, SignedWireNum;

  SmallVector<const Argument*, 8> Arguments;

  // Mapping success fsm state to their predicate in current state.
  typedef std::map<MachineBasicBlock*, std::string> PredMapTy;

  // If the FSM ready to move to next state?
  std::string ReadyPred;
  void addReadyPred(std::string &Pred) {
    ReadyPred += " & (" + Pred + ")";
  }

  void emitFunctionSignature();
  void emitCommonPort();

  /// emitAllocatedFUs - Set up a vector for allocated resources, and
  /// emit the ports and register, wire and datapath for them.
  void emitAllocatedFUs();

  void emitIdleState();

  void emitBasicBlock(MachineBasicBlock &MBB);

  void emitAllRegister();

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

  void emitUnaryOp(ucOp &UnOp, const std::string &Operator);
  void emitBinaryOp(ucOp &BinOp, const std::string &Operator);

  // Emit a signal with "signed" modifier and return the name of signed signal.
  std::string emitSignedOperand(MachineOperand &Op);

  void emitOpSRA(ucOp &OpSRA);

  void emitOpAdd(ucOp &OpAdd);
  void emitOpMult(ucOp &OpMult);

  void emitOpBitCat(ucOp &OpBitCat);
  void emitOpBitSlice(ucOp &OpBitSlice);
  void emitOpBitRepeat(ucOp &OpBitRepeat);

  void emitImplicitDef(ucOp &ImpDef);

  void emitOperand(raw_ostream &OS, MachineOperand &Operand,
                   /*FIXME: Get the value from the max word length*/
                   unsigned UB = 64, unsigned LB = 0);

  // Return true if the control operation contains a return operation.
  bool emitCtrlOp(ucState &State, PredMapTy &PredMap);

  void emitOpArg(ucOp &VOpArg);
  void emitOpRetVal(ucOp &OpRetVal);
  void emitOpRet(ucOp &OpRet);
  void emitOpCopy(ucOp &OpCopy);
  void emitOpMemTrans(ucOp &OpMemAccess);

  // Return the FSM ready predicate.
  // The FSM only move to next micro-state if the predicate become true.
  void emitFUCtrlForState(vlang_raw_ostream &CtrlS,
                          MachineBasicBlock *CurBB,
                          const PredMapTy &NextStatePred);

  void emitTestBench();

public:
  /// @name FunctionPass interface
  //{
  static char ID;
  RTLCodegen();

  ~RTLCodegen();

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
char RTLCodegen::ID = 0;

Pass *llvm::createRTLCodegenPass() {
  return new RTLCodegen();
}

INITIALIZE_PASS_BEGIN(RTLCodegen, "vtm-rtl-info",
                      "Build RTL Verilog module for synthesised function.",
                      false, true)
INITIALIZE_PASS_DEPENDENCY(BitLevelInfo);
INITIALIZE_PASS_END(RTLCodegen, "vtm-rtl-info",
                    "Build RTL Verilog module for synthesised function.",
                    false, true)

RTLCodegen::RTLCodegen() : MachineFunctionPass(ID), Out() {
  initializeRTLCodegenPass(*PassRegistry::getPassRegistry());
}

bool RTLCodegen::doInitialization(Module &M) {
  MachineModuleInfo *MMI = getAnalysisIfAvailable<MachineModuleInfo>();
  TargetData *TD = getAnalysisIfAvailable<TargetData>();

  assert(MMI && TD && "MachineModuleInfo and TargetData will always available"
                      " in a machine function pass!");
  Mang = new Mangler(MMI->getContext(), *TD);

  FOut = vtmfiles().getRTLOut();
  Out.setStream(FOut->os());
  return false;
}

bool RTLCodegen::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  FuncInfo = MF->getInfo<VFuncInfo>();
  MRI = &MF->getRegInfo();
  BLI = &getAnalysis<BitLevelInfo>();

  DEBUG(dbgs() << "Function for RTL Codegen:\n";
        printVMF(dbgs(), F);
  ); 

  SignedWireNum = 0;
  // Reset the current fsm state number.
  CurFSMStateNum = 0;
  // The FSM is always ready by default.
  ReadyPred = "1'b1";

  // FIXME: Demangle the c++ name.
  // Dirty Hack: Force the module have the name of the hw subsystem.
  VM = FuncInfo->createRtlMod(vtmfiles().getSystemName());
  emitFunctionSignature();

  // Emit control register and idle state
  unsigned totalFSMStates = MF->size() + 1;
  TotalFSMStatesBit = Log2_32_Ceil(totalFSMStates);

  VM->addRegister("NextFSMState", TotalFSMStatesBit);

  emitIdleState();

  
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
  Out.else_begin().if_begin(ReadyPred, "// if all resources ready?\n");

  Out << "// FSM\n";
  Out.switch_begin("NextFSMState");
  Out << VM->getControlBlockStr();
  // Case default.
  Out << "default:  NextFSMState <= state_idle;\n";
  Out.switch_end();
  Out.exit_block("// end ready\n");
  Out.always_ff_end();

  Out.module_end();

  // TODO: Emit testbench only user wants to.
  DEBUG_WITH_TYPE("tb-for-rtl", emitTestBench());

  Out.flush();

  return false;
}

void RTLCodegen::clear() {
  Arguments.clear();

  VM = 0;
}

void RTLCodegen::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.addRequired<BitLevelInfo>();
  AU.setPreservesAll();
}

void RTLCodegen::print(raw_ostream &O, const Module *M) const {

}

void RTLCodegen::emitFunctionSignature() {
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

void RTLCodegen::emitIdleState() {
  vlang_raw_ostream &CtrlS = VM->getControlBlockBuffer();

  // Idle state
  verilogParam(VM->getStateDeclBuffer(), "state_idle", TotalFSMStatesBit, 0);
  CtrlS.match_case("state_idle");
  // Idle state is always ready.
  CtrlS.if_begin("start");
  // The module is busy now
  MachineBasicBlock *EntryBB =  GraphTraits<MachineFunction*>::getEntryNode(MF);
  emitNextFSMState(CtrlS, EntryBB);
  //
  CtrlS.else_begin();
  CtrlS << "NextFSMState <= state_idle;\n";
  // End if-else
  CtrlS.exit_block();
  // Emit function unit control.
  PredMapTy IdleNextStatePred;
  IdleNextStatePred.insert(std::make_pair(EntryBB, "start"));
  emitFUCtrlForState(CtrlS, 0, IdleNextStatePred);
  // End case.
  CtrlS.exit_block();
}

void RTLCodegen::emitBasicBlock(MachineBasicBlock &MBB) {
  std::string StateName = getStateName(&MBB);
  unsigned startSlot = FuncInfo->getStartSlotFor(&MBB);
  unsigned totalSlot = FuncInfo->getTotalSlotFor(&MBB);
  unsigned IISlot = FuncInfo->getIISlotFor(&MBB);
  PredMapTy NextStatePred;

  vlang_raw_ostream &CtrlS = VM->getControlBlockBuffer();

  //unsigned StartSlot = State->getSlot(), EndSlot = State->getEndSlot();
  VM->getStateDeclBuffer() << "// State for " << StateName << '\n';
  verilogParam(VM->getStateDeclBuffer(), StateName, TotalFSMStatesBit,
               ++CurFSMStateNum);

  // State information.
  CtrlS << "// " << StateName << " Total Slot: " << totalSlot
                 << " II: " << (IISlot - startSlot) <<  '\n';
  // Mirco state enable.
  createucStateEnable(&MBB);

  // Case begin
  CtrlS.match_case(StateName);

  MachineBasicBlock::iterator I = ++MBB.getFirstNonPHI(),
                              E = MBB.getFirstTerminator();
  // FIXME: Refactor the loop.
  do {
    ucState CurDatapath = *I;
    // Emit the datepath of current state.
    emitDatapath(CurDatapath);

    // Emit next ucOp.
    ucState NextControl = *++I;
    CtrlS << "// Slot " << NextControl.getSlot() << '\n';
    emitCtrlOp(NextControl, NextStatePred);
  } while(++I != E);

  CtrlS << "// Next micro state.\n";
  PredMapTy::iterator at = NextStatePred.find(&MBB);
  if (at != NextStatePred.end())
    emitNextMicroState(CtrlS, &MBB, at->second);
  else
    emitNextMicroState(CtrlS, &MBB, "1'b0");

  emitFUCtrlForState(CtrlS, &MBB, NextStatePred);

  // Case end
  CtrlS.exit_block();
}

void RTLCodegen::emitCommonPort() {
  VM->addInputPort("clk", 1, VASTModule::Clk);
  VM->addInputPort("rstN", 1, VASTModule::RST);
  VM->addInputPort("start", 1, VASTModule::Start);
  VM->addOutputPort("fin", 1, VASTModule::Finish);
}

void RTLCodegen::emitAllocatedFUs() {
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
    // Byte enable.
    VM->addOutputPort(VFUMemBus::getByteEnableName(FUNum),
                      MemBus->getDataWidth() / 8);

    // Bus ready.
    VM->addInputPort(VFUMemBus::getReadyName(FUNum), 1);
  }
  
}

void RTLCodegen::emitAllRegister() {
  // We organize the registers in a module like 64 bits width ram, and we treat
  // the register number as the address of the register in the 'ram'. The
  // address of the register is always aligned with the register's size in byte,
  // that is, the register number of a 16 bits register will always divisible by
  // 2. So we can address the register as if it is in the ram. for example, if
  // we have a 32 bit register whose number is 20, that means it located at the
  // high part of the 2th 64 bits physics register.

  // Emit the register with max word length.
  for (VFuncInfo::phyreg_iterator I = FuncInfo->phyreg_begin(8),
       E = FuncInfo->phyreg_end(8); I < E; ++I)
    VM->addRegister("reg" + utostr(*I), 64);
}

RTLCodegen::~RTLCodegen() {}

//===----------------------------------------------------------------------===//
void RTLCodegen::createucStateEnable(MachineBasicBlock *MBB)  {
  std::string StateName = getStateName(MBB);
  // We do not need the last state.
  unsigned totalSlot = FuncInfo->getTotalSlotFor(MBB);

  // current state
  VM->addRegister("cur_" + StateName + "_enable", totalSlot);
}

void RTLCodegen::emitNextFSMState(raw_ostream &ss, MachineBasicBlock *MBB) {
  // Emit the first micro state of the target state.
  emitFirstCtrlState(MBB);

  // Only jump to other state if target MBB is not current MBB.
  ss << "NextFSMState <= " << getStateName(MBB) << ";\n";
  emitNextMicroState(VM->getControlBlockBuffer(), MBB, "1'b1");
}

void RTLCodegen::emitNextMicroState(raw_ostream &ss, MachineBasicBlock *MBB,
                                   const std::string &NewState) {
  // We do not need the last state.
  unsigned totalSlot = FuncInfo->getTotalSlotFor(MBB);
  std::string StateName = getucStateEnableName(MBB);
  ss << StateName << " <= ";

  if (totalSlot > 1)
    ss << "{ " << StateName << verilogBitRange(totalSlot - 1) << ", ";

  ss << NewState;

  if (totalSlot > 1)
    ss << " }";

  ss << ";\n";
}

void RTLCodegen::emitFUCtrlForState(vlang_raw_ostream &CtrlS,
                                   MachineBasicBlock *CurBB,
                                   const PredMapTy &NextStatePred) {
  unsigned startSlot = 0, totalSlot = 0;
  // Get the slot information for no-idle state.
  if (CurBB) {
    startSlot = FuncInfo->getStartSlotFor(CurBB);
    totalSlot = FuncInfo->getTotalSlotFor(CurBB);
  }

  unsigned endSlot = startSlot + totalSlot;
  unsigned MemBusLatency = vtmfus().getFUDesc<VFUMemBus>()->getLatency();

  // Emit function unit control.
  // Membus control operation.
  for (VFuncInfo::const_id_iterator I = FuncInfo->id_begin(VFUs::MemoryBus),
      E = FuncInfo->id_end(VFUs::MemoryBus); I != E; ++I) {
    FuncUnitId Id = *I;
    CtrlS << "// " << Id << " control for next micro state.\n";
    CtrlS << VFUMemBus::getEnableName(Id.getFUNum()) << " <= 1'b0";
    // Resource control operation when in the current state.
    for (unsigned i = startSlot + 1, e = endSlot; i < e; ++i) {
      if (FuncInfo->isFUActiveAt(Id, i))
        CtrlS << " | " << getucStateEnable(CurBB, i - 1);
    }

    // Resource control operation when we are transferring fsm state.
    for (PredMapTy::const_iterator NI = NextStatePred.begin(),
         NE = NextStatePred.end(); NI != NE; ++NI) {
      MachineBasicBlock *NextBB = NI->first;
      unsigned FirstSlot = FuncInfo->getStartSlotFor(NextBB);
      if (FuncInfo->isFUActiveAt(Id, FirstSlot))
        CtrlS << " | (" << getucStateEnable(NextBB, FirstSlot)
              << " & " << NI->second << ") ";
    }

    // Build the ready predicate for waiting membus ready.
    // We expect all operation will finish before the FSM jump to another state,
    // so we do not need to worry about if we need to wait the memory operation
    // issused from the previous state.
    for (unsigned i = startSlot + MemBusLatency, e = endSlot + 1; i != e; ++i)
      if (FuncInfo->isFUActiveAt(Id, i - MemBusLatency)) {
        std::string Pred = "~" +getucStateEnable(CurBB, i - 1)
                            + "|" + VFUMemBus::getReadyName(Id.getFUNum());
        addReadyPred(Pred);
      }

    CtrlS << ";\n";
  }

  // Control the finish port
  CtrlS << "// Finish port control\n";
  CtrlS << "fin <= 1'b0";
  unsigned LastSlot = startSlot + totalSlot;
  if (FuncInfo->isFUActiveAt(VFUs::FSMFinish, LastSlot))
    CtrlS << " | " << getucStateEnable(CurBB, LastSlot - 1);

  CtrlS << ";\n";
}

bool RTLCodegen::emitCtrlOp(ucState &State, PredMapTy &PredMap) {
  assert(State->getOpcode() == VTM::Control && "Bad ucState!");
  bool IsRet = false;
  MachineBasicBlock *CurBB = State->getParent();
  unsigned startSlot = FuncInfo->getStartSlotFor(CurBB),
           totalSlot = FuncInfo->getTotalSlotFor(CurBB),
           IISlot    = FuncInfo->getIISlotFor(CurBB);
  unsigned endSlot = startSlot + totalSlot,
           II = IISlot - startSlot;

  vlang_raw_ostream &CtrlS = VM->getControlBlockBuffer();
  std::string GenSlotPred = "1'b0";

  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    unsigned Slot = Op->getPredSlot();
    bool isFirstSlot = (Slot == startSlot);
    // Emit the control operation at the rising edge of the clock.
    std::string SlotPred;
    if (Slot == MetaToken::GeneralSlot) {
      if (GenSlotPred.length() == 4) { // Build the general slot if necessary.
        for (unsigned i = State.getSlot(), e = endSlot; i <= e; i += II)
          GenSlotPred += " | " + getucStateEnable(CurBB, i - 1);
        GenSlotPred += " /*General Slot*/";
      }
      SlotPred = GenSlotPred;
    } else
      SlotPred = getucStateEnable(CurBB, Slot - 1);

    // Special case for state transferring operation.
    if (Op.getOpCode() == VTM::VOpToState) {
      assert(!isFirstSlot && "Can not transfer to other state at first slot!");
      MachineBasicBlock *TargetBB = Op.getOperand(1).getMBB();
      raw_string_ostream ss(SlotPred);
      ss << " & ";
      emitOperand(ss, Op.getOperand(0));
      ss.flush();
      // Emit control operation for next state.
      CtrlS.if_begin(SlotPred);
      if (TargetBB == CurBB) { // Self loop detected.
        CtrlS << "// Loop back to entry.\n";
        emitFirstCtrlState(TargetBB);
      } else // Transfer to other state.
        emitNextFSMState(CtrlS, TargetBB);

      CtrlS.exit_block();
      PredMap.insert(std::make_pair(TargetBB, SlotPred));
      continue;
    }

    // Predicate the control logic by slot predicate.
    // First slot is always predicated by state transferring condition,
    // so we do not need to predicate it again.
    if (!isFirstSlot)
      CtrlS.if_begin(SlotPred);

    // Emit the operations.
    switch (Op.getOpCode()) {
    case VTM::VOpArg:       emitOpArg(Op);                break;
    case VTM::VOpRetVal:    emitOpRetVal(Op);             break;
    case VTM::VOpRet:       emitOpRet(Op); IsRet = true;  break;
    case VTM::VOpMemTrans:  emitOpMemTrans(Op);           break;
    case VTM::IMPLICIT_DEF: emitImplicitDef(Op);          break;
    case VTM::VOpSetRI:
    case VTM::COPY:         emitOpCopy(Op);               break;
    default:  assert(0 && "Unexpected opcode!");          break;
    }

    if(!isFirstSlot)  CtrlS.exit_block();
  }
  return IsRet;
}

void RTLCodegen::emitFirstCtrlState(MachineBasicBlock *MBB) {
  // TODO: Emit PHINodes if necessary.
  ucState FirstState = *MBB->getFirstNonPHI();
  assert(FuncInfo->getStartSlotFor(MBB) == FirstState.getSlot());
  PredMapTy dummy;
  emitCtrlOp(FirstState, dummy);
  assert(dummy.empty() && "Can not loop back at first state!");
}

void RTLCodegen::emitImplicitDef(ucOp &ImpDef) {
  DEBUG(VM->getControlBlockBuffer() << "/*IMPLICIT_DEF " << ImpDef << "*/\n");
}

void RTLCodegen::emitOpCopy(ucOp &OpCopy) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  emitOperand(OS, OpCopy.getOperand(0));
  OS << " <= ";
  emitOperand(OS, OpCopy.getOperand(1));
  OS << ";\n";
}

void RTLCodegen::emitOpArg(ucOp &OpArg) {
  // Assign input port to some register.
  raw_ostream &OS = VM->getControlBlockBuffer();
  emitOperand(OS, OpArg.getOperand(0));
  // FIXME: Use getInputPort instead;
  OS << " <= " << VM->getCommonPort(OpArg.getOperand(1).getImm()).getName()
     << ";\n";
}

void RTLCodegen::emitOpRet(ucOp &OpArg) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  OS << "NextFSMState <= state_idle;\n";
}

void RTLCodegen::emitOpRetVal(ucOp &OpRetVal) {
  raw_ostream &OS = VM->getControlBlockBuffer();
  unsigned retChannel = OpRetVal.getOperand(1).getImm();
  assert(retChannel == 0 && "Only support Channel 0!");
  OS << "return_value <= ";
  emitOperand(OS, OpRetVal.getOperand(0));
  OS << ";\n";
}

void RTLCodegen::emitOpMemTrans(ucOp &OpMemAccess) {
  unsigned FUNum = OpMemAccess->getFUId().getFUNum();
  raw_ostream &DPS = VM->getDataPathBuffer();
  DPS << "// Dirty Hack: Emit the wire define by this operation\n"
         "// some register copying operation may need this wire.\n";
  DPS << "assign ";
  emitOperand(DPS, OpMemAccess.getOperand(0));
  DPS << " = " << VFUMemBus::getInDataBusName(FUNum) << ";\n";

  // Emit the control logic.
  raw_ostream &OS = VM->getControlBlockBuffer();
  // Emit Address.
  OS << VFUMemBus::getAddrBusName(FUNum) << " <= ";
  emitOperand(OS, OpMemAccess.getOperand(1));
  OS << ";\n";
  // Assign store data.
  OS << VFUMemBus::getOutDataBusName(FUNum) << " <= ";
  emitOperand(OS, OpMemAccess.getOperand(2));
  OS << ";\n";
  // And write enable.
  OS << VFUMemBus::getWriteEnableName(FUNum) << " <= ";
  emitOperand(OS, OpMemAccess.getOperand(3));
  OS << ";\n";
  // The byte enable.
  // FIXME: This is the data size instead of the byte enable.
  OS << VFUMemBus::getByteEnableName(FUNum) << " <= ";
  emitOperand(OS, OpMemAccess.getOperand(4));
  OS << ";\n";
}

void RTLCodegen::emitOperand(raw_ostream &OS, MachineOperand &Operand,
                             unsigned UB, unsigned LB) {
  switch (Operand.getType()) {
  case MachineOperand::MO_Register: {
    unsigned Reg = Operand.getReg();
    // Get the one of the 64 bit registers.
    OS << "/*reg" << Reg <<"*/ reg" << (Reg & ~0x7);
    // Select the sub register
    unsigned Offset = (Reg & 0x7) * 8;
    UB = std::min(BLI->getBitWidth(Operand), UB);
    OS << verilogBitRange(UB + Offset, LB + Offset);

    return;
  }
  case MachineOperand::MO_Metadata: {
    MetaToken MetaOp(Operand.getMetadata());
    assert((MetaOp.isDefWire() || MetaOp.isReadWire()) && "Bad operand!");

    MachineBasicBlock *MBB = Operand.getParent()->getParent();
    std::string WireName = MetaOp.getWireName(getStateName(MBB));
    OS << WireName << verilogBitRange(MetaOp.getBitWidth(), 0, false);

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

void RTLCodegen::emitDatapath(ucState &State) {
  assert(State->getOpcode() == VTM::Datapath && "Bad ucState!");
  VM->getDataPathBuffer() << "// Issue datapath for "
    "operations at slot " << State.getSlot() << '\n';

  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    switch (Op.getOpCode()) {
    case VTM::VOpBitSlice:  emitOpBitSlice(Op);     break;
    case VTM::VOpBitCat:    emitOpBitCat(Op);       break;
    case VTM::VOpBitRepeat: emitOpBitRepeat(Op);    break;

    case VTM::VOpAdd:       emitOpAdd(Op);          break;
    case VTM::VOpMult:      emitOpMult(Op);         break;

    case VTM::VOpXor:       emitBinaryOp(Op, "^");  break;
    case VTM::VOpAnd:       emitBinaryOp(Op, "&");  break;
    case VTM::VOpOr:        emitBinaryOp(Op, "|");  break;

    case VTM::VOpSHL:       emitBinaryOp(Op, "<<"); break;
    case VTM::VOpSRL:       emitBinaryOp(Op, ">>"); break;
    case VTM::VOpSRA:       emitOpSRA(Op);break;

    case VTM::VOpNot:       emitUnaryOp(Op, "~");   break;

    case VTM::VOpROr:       emitUnaryOp(Op, "|");   break;
    case VTM::VOpRAnd:      emitUnaryOp(Op, "&");   break;
    case VTM::VOpRXor:      emitUnaryOp(Op, "^");   break;

    default:  assert(0 && "Unexpected opcode!");    break;
    }
  } 
}

void RTLCodegen::emitUnaryOp(ucOp &UnaOp, const std::string &Operator) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, UnaOp.getOperand(0));
  OS << " = " << Operator << ' ';
  emitOperand(OS, UnaOp.getOperand(1));
  OS << ";\n";
}

void RTLCodegen::emitBinaryOp(ucOp &BinOp, const std::string &Operator) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, BinOp.getOperand(0));
  OS << " = ";
  emitOperand(OS, BinOp.getOperand(1));
  OS << ' ' << Operator << ' ';
  emitOperand(OS, BinOp.getOperand(2));
  OS << ";\n";
}

std::string RTLCodegen::emitSignedOperand(MachineOperand &Op) {
  unsigned BitWidth = 0;
  switch (Op.getType()) {
  case MachineOperand::MO_Immediate:
  case MachineOperand::MO_Register:
    BitWidth = BLI->getBitWidth(Op);
    break;
  case MachineOperand::MO_Metadata: {
    MetaToken MDOp(Op.getMetadata());
    BitWidth = MDOp.getBitWidth();
    break;
  }
  default:
    assert(0 && "Can not compute bitwidth!");
    break;
  }
  raw_ostream &OS = VM->getDataPathBuffer();
  std::string WireName = "SignedWire" + utostr(SignedWireNum);
  OS << "wire signed" << verilogBitRange(BitWidth) << ' ' << WireName << " = ";
  emitOperand(OS, Op);
  OS << ";\n";

  ++SignedWireNum;
  return WireName;
}

void RTLCodegen::emitOpSRA(ucOp &OpSRA) {
  std::string Op0 = emitSignedOperand(OpSRA.getOperand(1));

  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, OpSRA.getOperand(0));
  OS << " = " << Op0 << " >>> ";
  emitOperand(OS, OpSRA.getOperand(2));
  OS << ";\n";
}

void RTLCodegen::emitOpAdd(ucOp &OpAdd) {
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

void RTLCodegen::emitOpMult(ucOp &OpMult) {
  raw_ostream &OS = VM->getDataPathBuffer();

  OS << "assign ";
  emitOperand(OS, OpMult.getOperand(0));
  OS << " = ";

  emitOperand(OS, OpMult.getOperand(1));
  OS << " * ";
  emitOperand(OS, OpMult.getOperand(2));
  OS << ";\n";
}

void RTLCodegen::emitOpBitSlice(ucOp &OpBitSlice) {
  raw_ostream &OS = VM->getDataPathBuffer();
  // Get the range of the bit slice, Note that the
  // bit at upper bound is excluded in VOpBitSlice,
  // now we are going to get the included upper bound.
  unsigned UB = OpBitSlice.getOperand(2).getImm(),
           LB = OpBitSlice.getOperand(3).getImm();

  OS << "assign ";
  emitOperand(OS, OpBitSlice.getOperand(0));
  OS << " = ";
  emitOperand(OS, OpBitSlice.getOperand(1), UB, LB);
  OS << ";\n";
}

void RTLCodegen::emitOpBitCat(ucOp &OpBitCat) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, OpBitCat.getOperand(0));
  OS << " = {";
  // BitCat is a binary instruction now.
  emitOperand(OS, OpBitCat.getOperand(1));
  OS << ',';
  emitOperand(OS, OpBitCat.getOperand(2));
  OS << "};\n";
}

void RTLCodegen::emitOpBitRepeat(ucOp &OpBitRepeat) {
  raw_ostream &OS = VM->getDataPathBuffer();
  OS << "assign ";
  emitOperand(OS, OpBitRepeat.getOperand(0));
  OS << " = {";

  unsigned Times = OpBitRepeat.getOperand(2).getImm();
  OS << Times << '{';
  emitOperand(OS, OpBitRepeat.getOperand(1));
  OS << "}};\n";
}

void RTLCodegen::emitTestBench() {
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
