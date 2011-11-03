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
#include "vtm/VFInfo.h"
#include "vtm/LangSteam.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/VInstrInfo.h"
#include "vtm/Utilities.h"

#include "llvm/Constants.h"
#include "llvm/GlobalVariable.h"
#include "llvm/Type.h"
#include "llvm/Module.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SourceMgr.h"
#define DEBUG_TYPE "vtm-rtl-codegen"
#include "llvm/Support/Debug.h"

using namespace llvm;
STATISTIC(TotalRegisterBits,
          "Number of total register bits in synthesised modules");
namespace {
class RTLCodegen : public MachineFunctionPass {
  vlang_raw_ostream Out;

  const Module *M;
  MachineFunction *MF;
  TargetData *TD;
  VFInfo *FInfo;
  MachineRegisterInfo *MRI;
  VASTModule *VM;
  Mangler *Mang;

  unsigned TotalFSMStatesBit, CurFSMStateNum, SignedWireNum;

  // Mapping success fsm state to their predicate in current state.
  typedef std::map<MachineBasicBlock*, VASTRegister::AssignCndTy> PredMapTy;

  void emitFunctionSignature(const Function *F);
  void emitCommonPort(unsigned FNNum);

  struct MuxBuilder {
    std::string MuxLogic;
    vlang_raw_ostream MuxLogicS;
    std::string MuxWiresDecl;
    vlang_raw_ostream MuxWiresDeclS;

    MuxBuilder()
      : MuxLogic("//Mux\n"),
        MuxLogicS(*new raw_string_ostream(MuxLogic), true, 2),
        MuxWiresDecl("// Mux\n"),
        MuxWiresDeclS(*new raw_string_ostream(MuxWiresDecl), true, 2) {
      MuxLogicS << "always @(*)";
      MuxLogicS.enter_block(" // begin mux logic\n");
      MuxLogicS << VASTModule::ParallelCaseAttr << ' ';
      MuxLogicS.switch_begin("1'b1");
    }

    void addOutput(const std::string &OutputName, unsigned Bitwidth) {
      MuxWiresDeclS << "reg ";
      if (Bitwidth > 1) MuxWiresDeclS << verilogBitRange(Bitwidth, 0, false);
      MuxWiresDeclS  << OutputName << "_mux_wire = "
                     << Bitwidth << "'b0;\n";
      MuxWiresDeclS << "assign " << OutputName << " = " << OutputName
                    << "_mux_wire;\n";
    }

    void assignValueInCase(const std::string &Dst, const std::string &Src) {
      MuxLogicS << Dst << "_mux_wire = " << Src << ";\n";
    }

    void writeToStream(raw_ostream &S) {
      MuxWiresDeclS.flush();
      S << MuxWiresDecl << "\n";

      MuxLogicS.exit_block();
      MuxLogicS.switch_end();
      MuxLogicS.exit_block(" // end mux logic\n\n\n\n");
      MuxLogicS.flush();
      S << MuxLogic;
    }
  };

  struct MemBusBuilder {
    VASTModule *VM;
    VFUMemBus *Bus;
    unsigned BusNum;
    std::string EnableLogic;
    vlang_raw_ostream EnableLogicS;
    MuxBuilder BusMux;

    void createOutputPort(const std::string &PortName, unsigned BitWidth,
                          bool isEn = false) {
      // We need to create multiplexer to allow current module and its submodules
      // share the bus.
      std::string PortReg = PortName + "_r";
      VM->getOrCreateSymbol(PortReg, VM->addRegister(PortReg, BitWidth));
      VM->addOutputPort(PortName, BitWidth, VASTModule::Others, false);
      if (isEn) {
        EnableLogicS << "assign " << PortName << " = " << PortReg;
        // The top level module control the memory bus by default.
        BusMux.MuxLogicS.match_case("default");
      } else {
        BusMux.addOutput(PortName, BitWidth);
        BusMux.assignValueInCase(PortName, PortReg);
      }
    }

    std::string addSubModulePort(const std::string &PortName, unsigned BitWidth,
                                 const std::string &SubModuleName,
                                 bool isOut = true, bool isEn = false){
      std::string ConnectedWire = PortName;
      if (isOut) { // Create extra wire for bus mux.
        ConnectedWire = SubModuleName + "_" + ConnectedWire;
        VM->addWire(ConnectedWire, BitWidth);
        if (isEn) {
          EnableLogicS << " | " << ConnectedWire ;
          BusMux.MuxLogicS.exit_block();
          BusMux.MuxLogicS.match_case(ConnectedWire);
        } else
          BusMux.assignValueInCase(PortName, ConnectedWire);
      }

      return "." + PortName + "(" +  ConnectedWire + "),\n\t";
    }

    void addSubModule(const std::string &SubModuleName, raw_ostream &S) {
      S << addSubModulePort(VFUMemBus::getEnableName(BusNum), 1, SubModuleName,
                            true, true);
      S << addSubModulePort(VFUMemBus::getCmdName(BusNum), VFUMemBus::CMDWidth,
                            SubModuleName);
      S << addSubModulePort(VFUMemBus::getAddrBusName(BusNum),
                            Bus->getAddrWidth(), SubModuleName);
      S << addSubModulePort(VFUMemBus::getInDataBusName(BusNum),
                            Bus->getDataWidth(), SubModuleName, false);
      S << addSubModulePort(VFUMemBus::getOutDataBusName(BusNum),
                            Bus->getDataWidth(), SubModuleName);
      S << addSubModulePort(VFUMemBus::getByteEnableName(BusNum),
                            Bus->getDataWidth()/8, SubModuleName);
      S << addSubModulePort(VFUMemBus::getReadyName(BusNum), 1, SubModuleName,
                            false);
    }

    MemBusBuilder(VASTModule *M, unsigned N)
      : VM(M), Bus(getFUDesc<VFUMemBus>()), BusNum(N),
      EnableLogic("  // Membus enables\n"),
      EnableLogicS(*new raw_string_ostream(EnableLogic), true, 2) {
      // Build the ports for current module.
      FuncUnitId ID(VFUs::MemoryBus, BusNum);
      // We need to create multiplexer to allow current module and its submodules
      // share the memory bus.

      VM->setFUPortBegin(ID);
      // Control ports.
      createOutputPort(VFUMemBus::getEnableName(BusNum), 1, true);
      createOutputPort(VFUMemBus::getCmdName(BusNum), VFUMemBus::CMDWidth);

      // Address port.
      createOutputPort(VFUMemBus::getAddrBusName(BusNum), Bus->getAddrWidth());
      // Data ports.
      VM->addInputPort(VFUMemBus::getInDataBusName(BusNum), Bus->getDataWidth());
      createOutputPort(VFUMemBus::getOutDataBusName(BusNum), Bus->getDataWidth());
      // Byte enable.
      createOutputPort(VFUMemBus::getByteEnableName(BusNum), Bus->getDataWidth() / 8);
      // Bus ready.
      VM->addInputPort(VFUMemBus::getReadyName(BusNum), 1);
    }

    void writeToStream(raw_ostream &S) {
      EnableLogicS.flush();
      S << EnableLogic << ";\n";
      BusMux.writeToStream(S);
    }
  };

  /// emitAllocatedFUs - Set up a vector for allocated resources, and
  /// emit the ports and register, wire and datapath for them.
  void emitAllocatedFUs();

  void emitIdleState();
  // We need create the slots for the empty slot.
  void emitSlotsForEmptyState(unsigned Slot, unsigned EndSlot, unsigned II) {
    // Dirty Hack: When we need is the slot before current slot, we issue the
    // control operation of current slot there.
    --Slot;

    while (Slot < EndSlot) {
      (void) VM->getSlot(Slot);
      Slot += II;
    }
  }

  void emitBasicBlock(MachineBasicBlock &MBB);

  void emitAllSignals();
  void emitSignals(const TargetRegisterClass *RC, bool isRegister);
  VASTValue *emitFUAdd(unsigned FUNum, unsigned BitWidth);
  VASTValue *emitFUMult(unsigned FUNum, unsigned BitWidth);
  VASTValue *emitFUShift(unsigned FUNum, unsigned BitWidth,
                         const char *Operator, bool isSAR);

  void clear();

  inline std::string getucStateEnable(ucState &State) {
    return getucStateEnable(State.getSlot());
  }

  inline std::string getucStateEnable(unsigned Slot) {
    return "Slot" + utostr_32(Slot) + "Active";
  }

  // Emit the operations in the first micro state in the FSM state when we are
  // jumping to it.
  void emitFirstCtrlState(MachineBasicBlock *DstBB, VASTSlot *Slot,
                          SmallVectorImpl<VASTCnd> &Cnds);

  void emitDatapath(ucState &State);

  void emitUnaryOp(ucOp &UnOp, const std::string &Operator);
  void emitBinaryOp(ucOp &BinOp, const std::string &Operator);

  void emitOpSel(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);

  void emitOpAdd(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitBinaryFUOp(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);

  void emitOpBitCat(ucOp &OpBitCat);
  void emitOpBitSlice(ucOp &OpBitSlice);
  void emitOpBitRepeat(ucOp &OpBitRepeat);

  void emitImplicitDef(ucOp &ImpDef);

  void emitCtrlOp(ucState &State, PredMapTy &PredMap);

  void printPredicate(ucOperand &Pred, raw_ostream &SS);
  // Create a condition from a predicate operand.
  VASTCnd createCondition(ucOperand &Op);

  VASTRValue getSignal(ucOperand &Op);
  void printOperand(ucOperand &Op, raw_ostream &OS, bool printBitwidth = true);
  void printAsOperand(ucOperand &Op, VASTWire &Wire,
                      unsigned UB = 64, unsigned LB = 0);

  void emitOpInternalCall(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpReadReturn(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpUnreachable(ucOp &OpUr, VASTSlot *CurSlot);
  void emitOpRetVal(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpRet(ucOp &OpRet, VASTSlot *CurSlot);
  void emitOpCopy(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpReadFU(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpMemTrans(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpBRam(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);

  std::string getSubModulePortName(unsigned FNNum,
                                   const std::string PortName) const {
    return "SubMod" + utostr(FNNum) + "_" + PortName;
  }

public:
  /// @name FunctionPass interface
  //{
  static char ID;
  RTLCodegen(raw_ostream &O);
  RTLCodegen() : MachineFunctionPass(ID) {
    assert( 0 && "Cannot construct the class without the raw_stream!");
  }

  ~RTLCodegen();

  bool doInitialization(Module &M);

  bool doFinalization(Module &M) {
    delete Mang;
    return false;
  }

  bool runOnMachineFunction(MachineFunction &MF);

  void releaseMemory() { clear(); }
  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};

}

//===----------------------------------------------------------------------===//
char RTLCodegen::ID = 0;

Pass *llvm::createRTLCodegenPass(raw_ostream &O) {
  return new RTLCodegen(O);
}

INITIALIZE_PASS_BEGIN(RTLCodegen, "vtm-rtl-info",
                      "Build RTL Verilog module for synthesised function.",
                      false, true)
INITIALIZE_PASS_END(RTLCodegen, "vtm-rtl-info",
                    "Build RTL Verilog module for synthesised function.",
                    false, true)

RTLCodegen::RTLCodegen(raw_ostream &O) : MachineFunctionPass(ID), Out(O) {
  initializeRTLCodegenPass(*PassRegistry::getPassRegistry());
}

bool RTLCodegen::doInitialization(Module &Mod) {
  MachineModuleInfo *MMI = getAnalysisIfAvailable<MachineModuleInfo>();
  TD = getAnalysisIfAvailable<TargetData>();

  assert(MMI && TD && "MachineModuleInfo and TargetData will always available"
                      " in a machine function pass!");
  Mang = new Mangler(MMI->getContext(), *TD);
  M = &Mod;

  SMDiagnostic Err;
  const char *GlobalScriptPath[] = { "Misc", "RTLGlobalScript" };
  std::string GlobalScript = getStrValueFromEngine(GlobalScriptPath);
  if (!runScriptOnGlobalVariables(Mod, TD, GlobalScript, Err))
    report_fatal_error("RTLCodegen: Cannot run globalvariable script:\n"
                        + Err.getMessage());

  const char *GlobalCodePath[] = { "RTLGlobalCode" };
  std::string GlobalCode = getStrValueFromEngine(GlobalCodePath);
  Out << GlobalCode << '\n';

  return false;
}

bool RTLCodegen::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  FInfo = MF->getInfo<VFInfo>();
  MRI = &MF->getRegInfo();

  DEBUG(
    Out << "`ifdef wtf_is_this\n" << "Function for RTL Codegen:\n";
    printVMF(Out, F);
    Out << "`endif\n";
  );

  SignedWireNum = 0;
  // Reset the current fsm state number.
  CurFSMStateNum = 0;

  // FIXME: Demangle the c++ name.
  // Dirty Hack: Force the module have the name of the hw subsystem.
  VM = FInfo->getRtlMod();
  emitFunctionSignature(F.getFunction());

  // Emit all function units then emit all register/wires because function units
  // may alias with registers.
  emitAllocatedFUs();
  emitAllSignals();

  // States of the control flow.
  emitIdleState();
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I) {
    MachineBasicBlock &BB = *I;
    emitBasicBlock(BB);
  }

  // Write buffers to output
  VM->printModuleDecl(Out);
  Out.module_begin();
  Out << "\n\n";
  // Reg and wire
  Out << "// Reg and wire decl\n";
  VM->printSignalDecl(Out);
  Out << "\n\n";
  // Slot active signals.
  VM->printSlotActives(Out);
  // Datapath
  Out << "// Datapath\n";
  Out << VM->getDataPathStr();
  VM->printDatapath(Out);

  Out << "\n\n";
  Out << "// Always Block\n";
  Out.always_ff_begin();

  VM->printRegisterReset(Out);
  Out.else_begin();
  VM->printRegisterAssign(Out);
  Out << VM->getControlBlockStr();
  VM->printSlotCtrls(Out);

  Out.always_ff_end();

  Out.module_end();
  Out.flush();

  return false;
}

void RTLCodegen::clear() {
  VM = 0;
}

void RTLCodegen::print(raw_ostream &O, const Module *M) const {

}

void RTLCodegen::emitFunctionSignature(const Function *F) {
  raw_ostream &S = VM->getDataPathBuffer();
  unsigned FNNum = FInfo->getCalleeFNNum(F->getName());
  for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
      I != E; ++I) {
    const Argument *Arg = I;
    std::string Name = Arg->getNameStr();
    unsigned BitWidth = TD->getTypeSizeInBits(Arg->getType());
    // Add port declaration.
    if (FNNum == 0)
      VM->addInputPort(Name, BitWidth, VASTModule::ArgPort);
    else {
      std::string RegName = getSubModulePortName(FNNum, Name);
      VM->getOrCreateSymbol(RegName, VM->addRegister(RegName, BitWidth));
      S << "." << Name << '(' << RegName << "),\n\t";
    }
  }

  const Type *RetTy = F->getReturnType();
  if (!RetTy->isVoidTy()) {
    assert(RetTy->isIntegerTy() && "Only support return integer now!");
    unsigned BitWidth = TD->getTypeSizeInBits(RetTy);
    if (FNNum == 0)
      VM->addOutputPort("return_value", BitWidth, VASTModule::RetPort);
    else {
      std::string WireName = getSubModulePortName(FNNum, "return_value");
      VM->getOrCreateSymbol(WireName, VM->addWire(WireName, BitWidth));
      S << ".return_value(" << WireName << "),\n\t";
    }
  }

  emitCommonPort(FNNum);
}

void RTLCodegen::emitIdleState() {
  vlang_raw_ostream &CtrlS = VM->getControlBlockBuffer();

  CtrlS.if_begin(getucStateEnable(0), "// Idle state\n");
  //CtrlS.match_case("state_idle");
  // Idle state is always ready.
  CtrlS.if_begin("start");
  // The module is busy now
  MachineBasicBlock *EntryBB =  GraphTraits<MachineFunction*>::getEntryNode(MF);
  VASTSlot *IdleSlot = VM->getSlot(0);
  VASTValue *StartPort = VM->getPort(VASTModule::Start).get();
  IdleSlot->addNextSlot(FInfo->getStartSlotFor(EntryBB),
                        StartPort);
  IdleSlot->addNextSlot(0, VASTCnd(StartPort, true));

  // Always Disable the finish signal.
  IdleSlot->addDisable(VM->getPort(VASTModule::Finish));
  SmallVector<VASTCnd, 1> Cnds(1, StartPort);
  emitFirstCtrlState(EntryBB, IdleSlot, Cnds);
  // End if-else
  CtrlS.exit_block();
  // End idle state.
  CtrlS.exit_block("// End idle state\n");
}

void RTLCodegen::emitBasicBlock(MachineBasicBlock &MBB) {
  unsigned totalSlot = FInfo->getTotalSlotFor(&MBB);
  unsigned II = FInfo->getIIFor(&MBB);
  unsigned EndSlot = FInfo->getEndSlotFor(&MBB);
  PredMapTy NextStatePred;

  vlang_raw_ostream &CtrlS = VM->getControlBlockBuffer();

  // State information.
  CtrlS << "// BB#" << MBB.getNumber() << " Total Slot: " << totalSlot
        << " II: " << II;
  if (II != totalSlot) CtrlS << " pipelined";
  CtrlS << '\n';

  MachineBasicBlock::iterator I = MBB.getFirstNonPHI(),
                              E = MBB.getFirstTerminator();
  //ucState FstCtrl(I);
  //if (FstCtrl.empty())
  //  emitSlotsForEmptyState(FstCtrl.getSlot(), EndSlot, II);

  // FIXME: Refactor the loop.
  while(++I != E) {
    ucState CurDatapath = *I;
    // Emit the datepath of current state.
    emitDatapath(CurDatapath);

    // Emit next ucOp.
    ucState NextControl = *++I;
    CtrlS << "// Slot " << NextControl.getSlot() << '\n';
    if (NextControl.empty())
      emitSlotsForEmptyState(NextControl.getSlot(), EndSlot, II);

    emitCtrlOp(NextControl, NextStatePred);
  };
}

void RTLCodegen::emitCommonPort(unsigned FNNum) {
  if (FNNum == 0) { // If F is current function.
    VM->addInputPort("clk", 1, VASTModule::Clk);
    VM->addInputPort("rstN", 1, VASTModule::RST);
    VM->addInputPort("start", 1, VASTModule::Start);
    VM->addOutputPort("fin", 1, VASTModule::Finish);
  } else { // It is a callee function, emit the signal for the sub module.
    std::string StartPortName = getSubModulePortName(FNNum, "start");
    std::string FinPortName = getSubModulePortName(FNNum, "fin");
    VM->addRegister(StartPortName, 1);
    VM->addWire(FinPortName, 1);
    // Connect to the ports
    raw_ostream &S = VM->getDataPathBuffer();
    S << ".clk(clk),\n\t.rstN(rstN),\n\t";
    S << ".start(" << StartPortName << "),\n\t";
    S << ".fin(" <<FinPortName << ")";
  }
}

void RTLCodegen::emitAllocatedFUs() {
  //for (id_iterator I = FInfo->id_begin(VFUs::MemoryBus),
  //     E = FInfo->id_end(VFUs::MemoryBus); I != E; ++I) {
  // FIXME: In fact, *I return the FUId instead of FUNum.
  // DIRTYHACK: Every module use memory bus 0, connect the bus.
  MemBusBuilder MBBuilder(VM, 0);
  //}

  raw_ostream &S = VM->getDataPathBuffer();

  // Generate code for allocated bram.
  VFUBRam *BlockRam = getFUDesc<VFUBRam>();
  for (VFInfo::const_bram_iterator I = FInfo->bram_begin(), E = FInfo->bram_end();
       I != E; ++I) {
    const VFInfo::BRamInfo &Info = I->second;
    unsigned BramNum = Info.PhyRegNum;
    const Value* Initializer = Info.Initializer; 
    unsigned NumElem = Info.NumElem;
    unsigned DataWidth = Info.ElemSizeInBytes * 8;
    std::string Filename;

    //Print the Constant into a .txt file as the initializer to bram
    Filename = BlockRam->generateInitFile(DataWidth, Initializer, NumElem);

    S << BlockRam->generateCode(VM->getPortName(VASTModule::Clk), BramNum,
                                DataWidth, Log2_32_Ceil(NumElem), Filename)
      << '\n';
  }

  // Generate the code for sub modules/external modules
  typedef VFInfo::const_fn_iterator fn_iterator;
  for (fn_iterator I = FInfo->fn_begin(), E = FInfo->fn_end(); I != E; ++I) {
    if (const Function *Callee = M->getFunction(I->getKey())) {
      if (!Callee->isDeclaration()) {
        S << getSynSetting(Callee->getName())->getModName() << ' '
          << getSubModulePortName(I->second, "_inst")
          << "(\n\t";
        MBBuilder.addSubModule(getSubModulePortName(I->second, "_inst"), S);
        emitFunctionSignature(Callee);
        S << ");\n";
        continue;
      }
    }

    unsigned FNNum = I->second;
    std::string Ports[5] = {
      VM->getPortName(VASTModule::Clk),
      VM->getPortName(VASTModule::RST),
      getSubModulePortName(FNNum, "start"),
      getSubModulePortName(FNNum, "fin"),
      getSubModulePortName(FNNum, "return_value")
    };
    // Add the finsh signal to the signal list.
    VM->addRegister(Ports[2], 1);

    // Else ask the constraint about how to instantiates this submodule.
    S << "// External module: " << I->getKey() << '\n';
    S << VFUs::instantiatesModule(I->getKey(), FNNum, Ports);
  }

  // Write the memory bus mux.
  MBBuilder.writeToStream(S);
}

VASTValue *RTLCodegen::emitFUAdd(unsigned FUNum, unsigned BitWidth) {
  // Write the datapath for function unit.
  std::string ResultName = "addsub" + utostr_32(FUNum);
  VASTWire *Result = VM->addWire(ResultName, BitWidth);
  VASTWire::builder_stream &DPS = Result->openCodeBuffer();
  unsigned OperandWidth = BitWidth - 1;

  DPS << "assign "<< ResultName << " = ";
  std::string OpName = ResultName + "_a";
  Result->addOperand(VM->addRegister(OpName, OperandWidth));
  DPS << OpName << " + ";
  OpName = ResultName + "_b";
  Result->addOperand(VM->addRegister(OpName, OperandWidth));
  DPS << OpName << " + ";
  OpName = ResultName + "_c";
  Result->addOperand(VM->addRegister(OpName, 1));
  DPS << OpName << ";\n";
  Result->closeCodeBuffer();
  return Result;
}

VASTValue *RTLCodegen::emitFUMult(unsigned FUNum, unsigned BitWidth) {
  std::string ResultName = "mult" + utostr_32(FUNum);
  VASTWire *Result = VM->addWire(ResultName, BitWidth);
  VASTWire::builder_stream &DPS = Result->openCodeBuffer();
  DPS << "assign "<< ResultName << " = ";
  std::string OpName = ResultName + "_a";
  Result->addOperand(VM->addRegister(OpName, BitWidth));
  DPS << OpName << " * ";
  OpName = ResultName + "_b";
  Result->addOperand(VM->addRegister(OpName, BitWidth));
  DPS << OpName << ";\n";
  Result->closeCodeBuffer();
  return Result;
}

VASTValue *RTLCodegen::emitFUShift(unsigned FUNum, unsigned BitWidth,
                                   const char *Operator, bool isSAR) {
  std::string ResultName = "shift" + utostr_32(FUNum);
  VASTWire *Result = VM->addWire(ResultName, BitWidth);
  VASTWire::builder_stream &DPS = Result->openCodeBuffer();
  unsigned RHSWidth = Log2_32_Ceil(BitWidth);

  std::string OpName = ResultName + "_a";
  Result->addOperand(VM->addRegister(OpName, BitWidth));
  if (isSAR) {
    // Convert the operand to signed wire if necessary.
    DPS << "wire signed" << verilogBitRange(BitWidth) << ' '
        << OpName << "_signed = " << OpName << ";\n";
    OpName += "_signed";
  }

  DPS << "assign "<< ResultName << " = " << OpName << Operator;
  OpName = ResultName + "_b";
  Result->addOperand(VM->addRegister(OpName, RHSWidth));
  DPS << OpName << ";\n";
  Result->closeCodeBuffer();
  return Result;
}

void RTLCodegen::emitAllSignals() {
  for (unsigned i = 0, e = FInfo->num_phyreg(); i != e; ++i) {
    unsigned RegNum = i + 1;
    VFInfo::PhyRegInfo Info = FInfo->getPhyRegInfo(RegNum);
    if (!Info.isTopLevelReg(RegNum)) {
      unsigned Parent = Info.getParentRegister();
      VASTRValue V = VASTRValue(VM->lookupSignal(Parent), Info.getUB(), Info.getLB());
      VM->indexVASTValue(RegNum, V);
      continue;
    }

    switch (Info.getRegClass()) {
    case VTM::DRRegClassID:
      VM->addRegister(RegNum, Info.getBitWidth());
      break;
    case VTM::RADDRegClassID:
      VM->indexVASTValue(RegNum, emitFUAdd(RegNum, Info.getBitWidth()));
      break;
    case VTM::RMULRegClassID:
      VM->indexVASTValue(RegNum, emitFUMult(RegNum, Info.getBitWidth()));
      break;
    case VTM::RASRRegClassID: {
      VASTValue *V = emitFUShift(RegNum, Info.getBitWidth(), ">>>", true);
      VM->indexVASTValue(RegNum, V);
      break;
    }
    case VTM::RLSRRegClassID:{
      VASTValue *V = emitFUShift(RegNum, Info.getBitWidth(), ">>", false);
      VM->indexVASTValue(RegNum, V);
      break;
    }
    case VTM::RSHLRegClassID:{
      VASTValue *V = emitFUShift(RegNum, Info.getBitWidth(), "<<", false);
      VM->indexVASTValue(RegNum, V);
      break;
    }
    case VTM::RBRMRegClassID: {
      VASTValue *V = VM->getOrCreateSymbol(VFUBRam::getInDataBusName(RegNum));
      VM->indexVASTValue(RegNum, V);
      break;
    }
    case VTM::RINFRegClassID: {
      // The offset of data input port is 3
      unsigned DataInIdx = VM->getFUPortOf(FuncUnitId(VFUs::MemoryBus, 0)) + 3;
      VM->indexVASTValue(RegNum, VASTRValue(VM->getPort(DataInIdx)));
      break;
    }
    }
  }

  emitSignals(VTM::WireRegisterClass, false);
}

void RTLCodegen::emitSignals(const TargetRegisterClass *RC, bool isRegister) {
  // And Emit the wires defined in this module.
  const std::vector<unsigned>& Wires = MRI->getRegClassVirtRegs(RC);

  for (std::vector<unsigned>::const_iterator I = Wires.begin(), E = Wires.end();
       I != E; ++I) {
    unsigned SignalNum = *I;
    MachineRegisterInfo::def_iterator DI = MRI->def_begin(SignalNum);
    if (DI == MRI->def_end()) continue;

    const ucOperand &Op = cast<ucOperand>(DI.getOperand());
    unsigned Bitwidth = Op.getBitWidth();
    if (!isRegister)
      VM->addWire(SignalNum, Bitwidth);
    else {
      VM->addRegister(SignalNum, Bitwidth);
      TotalRegisterBits += Bitwidth;
    }
  }
}


RTLCodegen::~RTLCodegen() {}

//===----------------------------------------------------------------------===//
void RTLCodegen::emitCtrlOp(ucState &State, PredMapTy &PredMap) {
  assert(State->getOpcode() == VTM::Control && "Bad ucState!");
  MachineBasicBlock *CurBB = State->getParent();
  unsigned startSlot = FInfo->getStartSlotFor(CurBB);
  unsigned IISlot = FInfo->getIISlotFor(CurBB);
  unsigned EndSlot = FInfo->getEndSlotFor(CurBB);
  unsigned II = IISlot - startSlot;
  SmallVector<VASTCnd, 4> Cnds;

  vlang_raw_ostream &CtrlS = VM->getControlBlockBuffer();

  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;

    unsigned SlotNum = Op->getPredSlot();
    VASTSlot *CurSlot = VM->getSlot(SlotNum - 1);

    assert(SlotNum != startSlot && "Unexpected first slot!");
    // Skip the marker.
    if (Op->getOpcode() == VTM::ImpUse) continue;
    
    Cnds.clear();
    // Emit the control operation at the rising edge of the clock.
    std::string SlotPred = "(";
    raw_string_ostream SlotPredSS(SlotPred);

    SlotPredSS << getucStateEnable(SlotNum - 1) << ')';
    // Emit the predicate operand.
    SlotPredSS << " & ";
    printPredicate(Op.getPredicate(), SlotPredSS);
    Cnds.push_back(createCondition(Op.getPredicate()));

    // Special case for state transferring operation.
    if (VInstrInfo::isBrCndLike(Op->getOpcode())) {
      SlotPredSS << " & ";
      ucOperand &CndOp = Op.getOperand(0);
      printPredicate(CndOp, SlotPredSS);
      Cnds.push_back(createCondition(CndOp));

      MachineBasicBlock *TargetBB = Op.getOperand(1).getMBB();
      unsigned TargetSlotNum = FInfo->getStartSlotFor(TargetBB);
      assert(Op.getPredicate().getReg() == 0 && "Cannot handle predicated BrCnd");
      VASTCnd Cnd = createCondition(CndOp);
      CurSlot->addNextSlot(TargetSlotNum, Cnd);

      // Emit control operation for next state.
      SlotPredSS.flush();
      CtrlS.if_begin(SlotPred);
      if (TargetBB == CurBB && IISlot < EndSlot)
        // The loop op of pipelined loop enable next slot explicitly.
        CurSlot->addNextSlot(CurSlot->getSlotNum() + 1);

      // Emit the first micro state of the target state.
      emitFirstCtrlState(TargetBB, CurSlot, Cnds);

      CtrlS.exit_block();
      VASTRegister::AssignCndTy JumpCnd
        = std::make_pair(CurSlot, VM->allocateAndCndVec(Cnds));
      PredMap.insert(std::make_pair(TargetBB, JumpCnd));
      continue;
    }

    // Loop back PHI node moving only active when current slot and the same
    // slot at previous (.i.e Slot - II) are both enable. Which means we are
    // looping back.
    if (Op->getOpcode() == VTM::VOpMvPhi) {
      MachineBasicBlock *TargetBB = Op.getOperand(2).getMBB();
      unsigned CndSlot = SlotNum - II;
      SlotPredSS << " & (";
      if (TargetBB == CurBB && CndSlot > startSlot) {
        SlotPredSS << getucStateEnable(CndSlot - 1);
        Cnds.push_back(VM->getOrCreateSymbol(getucStateEnable(CndSlot - 1)));
      } else {
        assert(PredMap.count(TargetBB) && "Loop back predicate not found!");
        VASTRegister::AssignCndTy PredCnd = PredMap.find(TargetBB)->second;
        VASTRegister::printCondition(SlotPredSS, PredCnd);
        std::string SlotActive = getucStateEnable(PredCnd.first->getSlotNum());
        Cnds.push_back(VM->getOrCreateSymbol(SlotActive));
        Cnds.insert(Cnds.end(), PredCnd.second.begin(), PredCnd.second.end());
      }

      SlotPredSS << ')';
    }

    SlotPredSS.flush();
    CtrlS.if_begin(SlotPred);

    // Emit the operations.
    switch (Op->getOpcode()) {
    case VTM::VOpMove_ri:
    case VTM::VOpMove_rw:
    case VTM::VOpMove_rr:
    case VTM::VOpMvPhi:
    case VTM::VOpMvPipe:
    case VTM::COPY:             emitOpCopy(Op, CurSlot, Cnds);break;
    case VTM::VOpAdd:           emitOpAdd(Op, CurSlot, Cnds); break;
    case VTM::VOpMult:
    case VTM::VOpSHL:
    case VTM::VOpSRL:
    case VTM::VOpSRA:           emitBinaryFUOp(Op, CurSlot, Cnds); break;
    case VTM::VOpReadFU:        emitOpReadFU(Op, CurSlot, Cnds);break;
    case VTM::VOpInternalCall:  emitOpInternalCall(Op, CurSlot, Cnds);break;
    case VTM::VOpRetVal:        emitOpRetVal(Op, CurSlot, Cnds);break;
    case VTM::VOpRet:           emitOpRet(Op, CurSlot);       break;
    case VTM::VOpCmdSeq:
    case VTM::VOpMemTrans:      emitOpMemTrans(Op, CurSlot, Cnds);break;
    case VTM::VOpBRam:          emitOpBRam(Op, CurSlot, Cnds);    break;
    case VTM::IMPLICIT_DEF:     emitImplicitDef(Op);          break;
    case VTM::VOpSel:           emitOpSel(Op, CurSlot, Cnds); break;
    case VTM::VOpReadReturn:    emitOpReadReturn(Op, CurSlot, Cnds);break;
    case VTM::VOpUnreachable:   emitOpUnreachable(Op, CurSlot);break;
    default:  assert(0 && "Unexpected opcode!");              break;
    }

    CtrlS.exit_block();
  }

  // There will be alias slot if the BB is pipelined.
  if (startSlot + II < EndSlot) {
    unsigned stateSlot = State.getSlot() - 1;
    for (unsigned slot = stateSlot; slot < EndSlot; slot += II)
      VM->getSlot(slot)->setAliasSlots(stateSlot, EndSlot, II);
  }
}

void RTLCodegen::emitFirstCtrlState(MachineBasicBlock *DstBB, VASTSlot *Slot,
                                    SmallVectorImpl<VASTCnd> &Cnds) {
  // TODO: Emit PHINodes if necessary.
  ucState FirstState = *DstBB->getFirstNonPHI();
  assert(FInfo->getStartSlotFor(DstBB) == FirstState.getSlot()
         && "Broken Slot!");

  for (ucState::iterator I = FirstState.begin(), E = FirstState.end();
       I != E; ++I) {
    ucOp Op = *I;

    switch (Op->getOpcode()) {
    case VTM::VOpMove_ri:
    case VTM::VOpMove_rw:
    case VTM::VOpMove_rr:
    case VTM::VOpMvPhi:
    case VTM::COPY:             emitOpCopy(Op, Slot, Cnds);   break;
    case VTM::VOpDefPhi:                                      break;
    case VTM::ImpUse:           /*Not need to handle*/        break;
    case VTM::VOpSel:           emitOpSel(Op, Slot, Cnds);    break;
    case VTM::VOpRetVal:        emitOpRetVal(Op, Slot, Cnds); break;
    case VTM::IMPLICIT_DEF:     emitImplicitDef(Op);          break;
    default:  assert(0 && "Unexpected opcode!");              break;
    }
  }
}

void RTLCodegen::emitOpUnreachable(ucOp &OpUr, VASTSlot *CurSlot) {
  raw_ostream &CtrlS = VM->getControlBlockBuffer();
  CtrlS << "$display(\"BAD BAD BAD BAD! Run to unreachable\");\n";
  CtrlS << "$finish();\n";
  CurSlot->addNextSlot(0);
}

void RTLCodegen::emitOpAdd(ucOp &Op, VASTSlot *Slot,
                           SmallVectorImpl<VASTCnd> &Cnds) {
  VASTWire *Result = cast<VASTWire>(getSignal(Op.getOperand(0)));
  VASTRegister *R = cast<VASTRegister>(Result->getOperand(0));
  VM->addAssignment(R, getSignal(Op.getOperand(2)), Slot, Cnds);
  R = cast<VASTRegister>(Result->getOperand(1));
  VM->addAssignment(R, getSignal(Op.getOperand(3)), Slot, Cnds);
  R = cast<VASTRegister>(Result->getOperand(2));
  VM->addAssignment(R, getSignal(Op.getOperand(4)), Slot, Cnds);
}

void RTLCodegen::emitBinaryFUOp(ucOp &Op, VASTSlot *Slot,
                                SmallVectorImpl<VASTCnd> &Cnds) {
  VASTWire *Result = cast<VASTWire>(getSignal(Op.getOperand(0)));
  VASTRegister *R = cast<VASTRegister>(Result->getOperand(0));
  VM->addAssignment(R, getSignal(Op.getOperand(1)), Slot, Cnds);
  R = cast<VASTRegister>(Result->getOperand(1));
  VM->addAssignment(R, getSignal(Op.getOperand(2)), Slot, Cnds);
}

void RTLCodegen::emitImplicitDef(ucOp &ImpDef) {
  //VASTDatapath *data = VM->createDatapath();
  //VASTDatapath::builder_stream &OS = data->getCodeBuffer();
  //OS << "// IMPLICIT_DEF " << ImpDef.getOperand(0) << "\n";
}

void RTLCodegen::emitOpSel(ucOp &Op, VASTSlot *Slot,
                           SmallVectorImpl<VASTCnd> &Cnds) {
  VASTRegister *R = cast<VASTRegister>(getSignal(Op.getOperand(0)));
  std::string SelWireName = R->getName() + "_Sel_" + Slot->getName();
  // Dirty Hack: Create a wire for select result.
  VASTWire *SelWire = VM->addWire(SelWireName, R->getBitWidth());
  raw_ostream &OS = SelWire->openCodeBuffer();
  OS << "assign " << SelWireName << " = ";
  if (Op.getOperand(1).isPredicateInverted())
    OS << "~";
  printAsOperand(Op.getOperand(1), *SelWire, 1);
  OS << " ? ";
  printAsOperand(Op.getOperand(2), *SelWire);
  OS << " : ";
  printAsOperand(Op.getOperand(3), *SelWire);
  OS << ";\n";
  SelWire->closeCodeBuffer();
  VM->addAssignment(R, SelWire, Slot, Cnds);
}

void RTLCodegen::emitOpCopy(ucOp &Op, VASTSlot *Slot,
                            SmallVectorImpl<VASTCnd> &Cnds) {
  ucOperand &Dst = Op.getOperand(0), &Src = Op.getOperand(1);
  // Ignore the identical copy.
  if (Src.isReg() && Dst.getReg() == Src.getReg()) return;

  VASTRegister *R = cast<VASTRegister>(getSignal(Dst));
  VM->addAssignment(R, getSignal(Src), Slot, Cnds);
}

void RTLCodegen::emitOpReadFU(ucOp &Op, VASTSlot *CurSlot,
                              SmallVectorImpl<VASTCnd> &Cnds) {
  FuncUnitId Id = Op->getFUId();
  VASTValue *ReadyPort = 0;

  switch (Id.getFUType()) {
  case VFUs::MemoryBus:
    ReadyPort = VM->getOrCreateSymbol(VFUMemBus::getReadyName(Id.getFUNum()));
    break;
  case VFUs::CalleeFN:
    ReadyPort = VM->getOrCreateSymbol(getSubModulePortName(Id.getFUNum(), "fin"));
    break;
  default:
    break;
  }

  if (ReadyPort)
    CurSlot->addReady(ReadyPort, createCondition(Op.getPredicate()));

  // The dst operand of ReadFU change to immediate if it is dead.
  if (Op.getOperand(0).isReg()) emitOpCopy(Op, CurSlot, Cnds);
}

void RTLCodegen::emitOpReadReturn(ucOp &Op, VASTSlot *Slot,
                                  SmallVectorImpl<VASTCnd> &Cnds) {
  VASTRegister *R = cast<VASTRegister>(getSignal(Op.getOperand(0)));
  unsigned FNNum = Op.getOperand(1).getTargetFlags();
  std::string PortName =
    getSubModulePortName(FNNum, Op.getOperand(1).getSymbolName());
  VM->addAssignment(R, VM->getOrCreateSymbol(PortName), Slot, Cnds);
}

void RTLCodegen::emitOpInternalCall(ucOp &Op, VASTSlot *Slot,
                                    SmallVectorImpl<VASTCnd> &Cnds) {
  // Assign input port to some register.
  const char *CalleeName = Op.getOperand(1).getSymbolName();
  // The FNNum is encoded into the target flags field of the MachineOperand.
  unsigned FNNum = Op.getOperand(1).getTargetFlags();

  VASTCnd Pred = createCondition(Op.getPredicate());
  std::string StartPortName = getSubModulePortName(FNNum, "start");
  VASTValue *StartSignal = VM->getOrCreateSymbol(StartPortName);
  Slot->addEnable(StartSignal, Pred);
  VASTSlot *NextSlot = VM->getSlot(Slot->getSlotNum() + 1);
  NextSlot->addDisable(StartSignal, Pred);

  const Function *FN = M->getFunction(CalleeName);
  if (FN && !FN->isDeclaration()) {
    Function::const_arg_iterator ArgIt = FN->arg_begin();
    for (unsigned i = 0, e = FN->arg_size(); i != e; ++i) {
      VASTRegister *R =
        VM->getSymbol<VASTRegister>(getSubModulePortName(FNNum, ArgIt->getName()));
      VM->addAssignment(R, getSignal(Op.getOperand(2 + i)), Slot, Cnds);
      ++ArgIt;
    }
    return;
  }

  // Else we had to write the control code to the control block.
  vlang_raw_ostream &OS = VM->getControlBlockBuffer();
  OS << "// Calling function: " << CalleeName << ";\n";
  std::string PredStr;
  raw_string_ostream SS(PredStr);
  VASTRegister::printCondition(SS, Slot, Cnds);
  SS.flush();

  OS.if_begin(PredStr);
  if (FN /*&& FN->isDeclaration()*/) {
    // Dirty Hack.
    // TODO: Extract these to some special instruction?
    OS << "$c(\"" << FN->getName() << "(\",";
    for (unsigned i = 2, e = Op.getNumOperands(); i != e; ++i) {
      ucOperand &Operand = Op.getOperand(i);
      if (Operand.isReg() && (Operand.getReg() == 0 || Operand.isImplicit()))
        continue;

      if (i != 2) OS << ",\",\", ";

      if (Operand.isGlobal()) // It is the format string?
        if (const GlobalVariable *Str = cast<GlobalVariable>(Operand.getGlobal())){
          if (Str->hasInitializer()) {
            const Constant *Initialer = Str->getInitializer();
            if (const ConstantArray *Fmt = dyn_cast<ConstantArray>(Initialer)){
              if (Fmt->isCString()) {
                std::string FmtStr;
                raw_string_ostream SS(FmtStr);
                SS << '"';
                PrintEscapedString(Fmt->getAsString(), SS);
                SS << '"';
                SS.flush();
                OS << '"';
                PrintEscapedString(FmtStr, OS);
                OS << '"';
                continue;
              }
            }
          }
        }
      printOperand(Operand, OS);
    }

    OS << ", \");\""; // Enclose the c function call.
    OS << ");\n";
  } else {
    // Else ask the constraint about how to handle this call.
    SmallVector<std::string, 8> InPorts;
    std::string s;
    raw_string_ostream SS(s);
    for (unsigned i = 2, e = Op.getNumOperands(); i != e; ++i) {
      printOperand(Op.getOperand(i), SS);
      SS.flush();
      InPorts.push_back(SS.str());
      s.clear();
    }

    std::string Name = CalleeName;
    OS << VFUs::startModule(Name, FInfo->getCalleeFNNum(CalleeName), InPorts);
  }
  OS.exit_block();
}

void RTLCodegen::emitOpRet(ucOp &OpArg, VASTSlot *CurSlot) {
  // Go back to the idle slot.
  CurSlot->addNextSlot(0);
  CurSlot->addEnable(VM->getPort(VASTModule::Finish));
}

void RTLCodegen::emitOpRetVal(ucOp &Op, VASTSlot *Slot,
                              SmallVectorImpl<VASTCnd> &Cnds) {
  VASTRegister &RetReg = cast<VASTRegister>(*VM->getRetPort());
  unsigned retChannel = Op.getOperand(1).getImm();
  assert(retChannel == 0 && "Only support Channel 0!");
  VM->addAssignment(&RetReg, getSignal(Op.getOperand(0)), Slot, Cnds);
}

void RTLCodegen::emitOpMemTrans(ucOp &Op, VASTSlot *Slot,
                                SmallVectorImpl<VASTCnd> &Cnds) {
  unsigned FUNum = Op->getFUId().getFUNum();

  // Emit the control logic.
  raw_ostream &OS = VM->getControlBlockBuffer();
  // Emit Address.
  std::string RegName = VFUMemBus::getAddrBusName(FUNum) + "_r";
  VASTRegister *R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getSignal(Op.getOperand(1)), Slot, Cnds);
  // Assign store data.
  RegName = VFUMemBus::getOutDataBusName(FUNum) + "_r";
  R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getSignal(Op.getOperand(2)), Slot, Cnds);
  // And write enable.
  RegName = VFUMemBus::getCmdName(FUNum) + "_r";
  R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getSignal(Op.getOperand(3)), Slot, Cnds);
  // The byte enable.
  RegName = VFUMemBus::getByteEnableName(FUNum) + "_r";
  R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getSignal(Op.getOperand(4)), Slot, Cnds);

  // Remember we enabled the memory bus at this slot.
  std::string EnableName = VFUMemBus::getEnableName(FUNum) + "_r";
  VASTValue *MemEn = VM->getOrCreateSymbol(EnableName);
  VASTCnd Pred = createCondition(Op.getPredicate());
  Slot->addEnable(MemEn, Pred);

  // Disable the memory at next slot.
  // TODO: Assert the control flow is linear.
  VASTSlot *NextSlot = VM->getSlot(Slot->getSlotNum() + 1);
  NextSlot->addDisable(MemEn, Pred);
}

void RTLCodegen::emitOpBRam(ucOp &Op, VASTSlot *Slot,
                            SmallVectorImpl<VASTCnd> &Cnds) {
  unsigned FUNum = Op.getOperand(0).getReg();

  // Emit the control logic.
  vlang_raw_ostream &OS = VM->getControlBlockBuffer();
  std::string PredStr;
  raw_string_ostream PredSS(PredStr);
  VASTRegister::printCondition(PredSS, Slot, Cnds);
  OS.if_begin(PredStr);
  // Emit Address.
  OS << VFUBRam::getAddrBusName(FUNum) << " <= (";
  printOperand(Op.getOperand(1), OS);
  unsigned SizeInBits
    = FInfo->getBRamInfo(Op->getFUId().getFUNum()).ElemSizeInBytes;
  OS << " >> " << Log2_32_Ceil(SizeInBits) << ");\n";
  // Assign store data.
  OS << VFUBRam::getOutDataBusName(FUNum) << " <= ";
  printOperand(Op.getOperand(2), OS);
  OS << ";\n";
  // And write enable.
  OS << VFUBRam::getWriteEnableName(FUNum) << " <= ";
  printOperand(Op.getOperand(3), OS);
  OS << ";\n";
  OS.exit_block();
  // The byte enable.
  // OS << VFUMemBus::getByteEnableName(FUNum) << " <= ";
  // OpBRam.getOperand(4).print(OS);
  // OS << ";\n";

  // Remember we enabled the memory bus at this slot.
  std::string EnableName = VFUBRam::getEnableName(FUNum);
  VASTValue *MemEn = VM->getOrCreateSymbol(EnableName);
  VASTCnd Pred = createCondition(Op.getPredicate());
  Slot->addEnable(MemEn, Pred);

  // Disable the memory at next slot.
  // TODO: Assert the control flow is linear.
  VASTSlot *NextSlot = VM->getSlot(Slot->getSlotNum() + 1);
  NextSlot->addDisable(MemEn, Pred);
}

void RTLCodegen::emitDatapath(ucState &State) {
  assert(State->getOpcode() == VTM::Datapath && "Bad ucState!");

  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    switch (Op->getOpcode()) {
    case VTM::VOpBitSlice:  emitOpBitSlice(Op);     break;
    case VTM::VOpBitCat:    emitOpBitCat(Op);       break;
    case VTM::VOpBitRepeat: emitOpBitRepeat(Op);    break;

    case VTM::ImpUse:       /*Not need to handle*/  break;

    case VTM::VOpXor:       emitBinaryOp(Op, "^");  break;
    case VTM::VOpAnd:       emitBinaryOp(Op, "&");  break;
    case VTM::VOpOr:        emitBinaryOp(Op, "|");  break;

    case VTM::VOpNot:       emitUnaryOp(Op, "~");   break;

    case VTM::VOpROr:       emitUnaryOp(Op, "|");   break;
    case VTM::VOpRAnd:      emitUnaryOp(Op, "&");   break;
    case VTM::VOpRXor:      emitUnaryOp(Op, "^");   break;

    default:  assert(0 && "Unexpected opcode!");    break;
    }
  }
}

void RTLCodegen::emitUnaryOp(ucOp &UnaOp, const std::string &Operator) {
  VASTWire &V = *cast<VASTWire>(getSignal(UnaOp.getOperand(0)));
  VASTWire::builder_stream &OS = V.openCodeBuffer();
  OS << "assign ";
  printOperand(UnaOp.getOperand(0), OS);
  OS << " = " << Operator << ' ';
  printAsOperand(UnaOp.getOperand(1), V);
  OS << ";\n";
  V.closeCodeBuffer();
}

void RTLCodegen::emitBinaryOp(ucOp &BinOp, const std::string &Operator) {
  VASTWire &V = *cast<VASTWire>(getSignal(BinOp.getOperand(0)));
  VASTWire::builder_stream &OS = V.openCodeBuffer();
  OS << "assign ";
  printOperand(BinOp.getOperand(0), OS);
  OS << " = ";
  printAsOperand(BinOp.getOperand(1), V);
  OS << ' ' << Operator << ' ';
  printAsOperand(BinOp.getOperand(2), V);
  OS << ";\n";
  V.closeCodeBuffer();
}

void RTLCodegen::emitOpBitSlice(ucOp &OpBitSlice) {
  VASTWire &V = *cast<VASTWire>(getSignal(OpBitSlice.getOperand(0)));
  VASTWire::builder_stream &OS = V.openCodeBuffer();
  // Get the range of the bit slice, Note that the
  // bit at upper bound is excluded in VOpBitSlice,
  // now we are going to get the included upper bound.
  unsigned UB = OpBitSlice.getOperand(2).getImm(),
           LB = OpBitSlice.getOperand(3).getImm();

  OS << "assign ";
  printOperand(OpBitSlice.getOperand(0), OS);
  OS << " = ";
  printAsOperand(OpBitSlice.getOperand(1), V, UB, LB);
  OS << ";\n";
  V.closeCodeBuffer();
}

void RTLCodegen::emitOpBitCat(ucOp &OpBitCat) {
  VASTWire &V = *cast<VASTWire>(getSignal(OpBitCat.getOperand(0)));
  VASTWire::builder_stream &OS = V.openCodeBuffer();
  OS << "assign ";
  printOperand(OpBitCat.getOperand(0), OS);
  OS << " = {";
  // BitCat is a binary instruction now.
  printAsOperand(OpBitCat.getOperand(1), V);
  OS << ',';
  printAsOperand(OpBitCat.getOperand(2), V);
  OS << "};\n";
  V.closeCodeBuffer();
}

void RTLCodegen::emitOpBitRepeat(ucOp &OpBitRepeat) {
  VASTWire &V = *cast<VASTWire>(getSignal(OpBitRepeat.getOperand(0)));
  VASTWire::builder_stream &OS = V.openCodeBuffer();
  OS << "assign ";
  printOperand(OpBitRepeat.getOperand(0), OS);
  OS << " = {";

  unsigned Times = OpBitRepeat.getOperand(2).getImm();
  OS << Times << '{';
  printAsOperand(OpBitRepeat.getOperand(1), V);
  OS << "}};\n";
  V.closeCodeBuffer();
}

VASTCnd RTLCodegen::createCondition(ucOperand &Op) {
  VASTRValue V = VM->lookupSignal(Op.getReg());

  return VASTCnd(V, Op.isPredicateInverted());
}

VASTRValue RTLCodegen::getSignal(ucOperand &Op) {
  if (Op.isReg()) {
    VASTRValue V = VM->lookupSignal(Op.getReg());
    assert (V != 0 && "Cannot find this Value in vector!");
    return V;
  }

  // Otherwise simply create a symbol.
  std::string Name;
  raw_string_ostream SS(Name);
  Op.print(SS);
  SS.flush();

  return VM->getOrCreateSymbol(Name);
}

void RTLCodegen::printOperand(ucOperand &Op, raw_ostream &OS, bool printRange) {
  if(Op.isReg()){
    VASTRValue V = getSignal(Op);
    OS << V->getName();

    if(printRange && V->getBitWidth() != 0)
      OS << verilogBitRange(V.UB, V.LB, V->getBitWidth() !=1);

    return;
  }

  Op.print(OS);
}

void RTLCodegen::printAsOperand(ucOperand &Op, VASTWire &Wire,
                                unsigned UB, unsigned LB) {
  if(Op.isReg()){
    VASTRValue V = getSignal(Op);
    Wire.getCodeBuffer() << V->getName();
    UB = std::min(unsigned(V.UB), UB);
    LB = std::max(unsigned(V.LB), LB);
    if(UB - LB != 0)
      Wire.getCodeBuffer() << verilogBitRange(UB, LB, V->getBitWidth() != 1);
    Wire.addOperand(VASTRValue(V, UB, LB));
    return;
  }

  assert(UB == 64 && LB == 0 && "Get bitslice on bad operand type?");
  Op.print(Wire.getCodeBuffer());
}

void RTLCodegen::printPredicate(ucOperand &Pred, raw_ostream &SS) {
  if (Pred.getReg()) {
    SS << '(';
    if (Pred.isPredicateInverted()) SS << '~';
    printOperand(Pred, SS, true);
    SS << ')';
  } else
    SS << "1'b1";
}
