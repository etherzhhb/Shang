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
  VRegisterInfo *TRI;
  VFInfo *FInfo;
  MachineRegisterInfo *MRI;
  VASTModule *VM;
  Mangler *Mang;

  unsigned TotalFSMStatesBit, CurFSMStateNum, SignedWireNum;

  // Mapping success fsm state to their predicate in current state.
  typedef std::map<MachineBasicBlock*, VASTRegister::AssignCndTy> PredMapTy;

  void emitFunctionSignature(const Function *F);
  void emitCommonPort(unsigned FNNum);

  struct MemBusBuilder {
    VASTModule *VM;
    VFUMemBus *Bus;
    unsigned BusNum;
    VASTWire *MembusEn, *MembusCmd, *MemBusAddr, *MemBusOutData, *MemBusByteEn;

    VASTWire *createOutputPort(const std::string &PortName, unsigned BitWidth,
                               VASTRegister *&LocalEn) {
      // We need to create multiplexer to allow current module and its submodules
      // share the bus.
      std::string PortReg = PortName + "_r";
      VASTRegister *LocalReg = VM->addRegister(PortReg, BitWidth);
      VASTPort *P = VM->addOutputPort(PortName, BitWidth, VASTModule::Others,
                                      false);
      VASTWire *OutputWire = cast<VASTWire>(P->get());
      // Are we creating the enable port?
      if (LocalEn == 0) {
        // Or all enables together to generate the enable output
        OutputWire->setOpcode(VASTWire::dpOr);
        // Add the local enable.
        OutputWire->addOperand(LocalReg);
        LocalEn = LocalReg;
      } else{
        OutputWire->setOpcode(VASTWire::dpMux);
        // Select the local signal if local enable is true.
        OutputWire->addOperand(LocalEn);
        OutputWire->addOperand(LocalReg);
      }

      return OutputWire;
    }

    void addSubModuleOutPort(raw_ostream &S, VASTWire *OutputWire,
                             unsigned BitWidth, const std::string &SubModuleName,
                             VASTWire *&SubModEn) {
      std::string ConnectedWireName = SubModuleName + "_"
                                      + std::string(OutputWire->getName());

      VASTWire *SubModWire = VM->addWire(ConnectedWireName, BitWidth);

      // Are we creating the enable signal from sub module?
      if (SubModEn == 0) {
        OutputWire->addOperand(SubModWire);
        SubModEn = SubModWire;
      } else {
        // Select the signal from submodule if sub module enable is true.
        OutputWire->addOperand(SubModEn);
        OutputWire->addOperand(SubModWire);
      }

      // Write the connection.
      // The corresponding port name of submodule should be the same as current
      // output port name.
      S << '.' << OutputWire->getName() << '(' << ConnectedWireName << "),\n\t";
    }

    void addSubModuleInPort(raw_ostream &S, const std::string &PortName) {
      // Simply connect the input port to the corresponding port of submodule,
      // which suppose to have the same name.
      S << '.' << PortName << '(' <<  PortName << "),\n\t";
    }

    void addSubModule(const std::string &SubModuleName, raw_ostream &S) {
      VASTWire *SubModEn = 0;
      addSubModuleOutPort(S, MembusEn, 1, SubModuleName, SubModEn);
      // Output ports.
      addSubModuleOutPort(S, MembusCmd, VFUMemBus::CMDWidth, SubModuleName,
                          SubModEn);
      addSubModuleOutPort(S, MemBusAddr, Bus->getAddrWidth(), SubModuleName,
                          SubModEn);
      addSubModuleOutPort(S, MemBusOutData, Bus->getDataWidth(), SubModuleName,
                          SubModEn);
      addSubModuleOutPort(S, MemBusByteEn, Bus->getDataWidth()/8, SubModuleName,
                          SubModEn);

      // Input ports.
      addSubModuleInPort(S, VFUMemBus::getInDataBusName(BusNum));
      addSubModuleInPort(S, VFUMemBus::getReadyName(BusNum));
    }

    MemBusBuilder(VASTModule *M, unsigned N)
      : VM(M), Bus(getFUDesc<VFUMemBus>()), BusNum(N) {
      // Build the ports for current module.
      FuncUnitId ID(VFUs::MemoryBus, BusNum);
      // We need to create multiplexer to allow current module and its submodules
      // share the memory bus.
      VM->setFUPortBegin(ID);
      // The enable signal for local memory bus.
      VASTRegister *LocalEn = 0;
      // Control ports.
      MembusEn =
        createOutputPort(VFUMemBus::getEnableName(BusNum), 1, LocalEn);
      MembusCmd =
        createOutputPort(VFUMemBus::getCmdName(BusNum), VFUMemBus::CMDWidth,
                         LocalEn);

      // Address port.
      MemBusAddr =
        createOutputPort(VFUMemBus::getAddrBusName(BusNum), Bus->getAddrWidth(),
                         LocalEn);
      // Data ports.
      VM->addInputPort(VFUMemBus::getInDataBusName(BusNum), Bus->getDataWidth());
      MemBusOutData =
        createOutputPort(VFUMemBus::getOutDataBusName(BusNum),
                         Bus->getDataWidth(), LocalEn);
      // Byte enable.
      MemBusByteEn =
        createOutputPort(VFUMemBus::getByteEnableName(BusNum),
                         Bus->getDataWidth() / 8, LocalEn);
      // Bus ready.
      VM->addInputPort(VFUMemBus::getReadyName(BusNum), 1);
    }
  };

  /// emitAllocatedFUs - Set up a vector for allocated resources, and
  /// emit the ports and register, wire and datapath for them.
  void emitAllocatedFUs();

  void emitIdleState();

  void emitBasicBlock(MachineBasicBlock &MBB);

  void emitAllSignals();
  void emitSignals(const TargetRegisterClass *RC, bool isRegister);
  VASTValue *emitFUAdd(unsigned FUNum, unsigned BitWidth);
  VASTValue *emitFUMult(unsigned FUNum, unsigned BitWidth, bool HasHi);
  VASTValue *emitFUShift(unsigned FUNum, unsigned BitWidth,
                         VASTWire::Opcode Opc);
  VASTValue *emitFUCmp(unsigned FUNum, unsigned BitWidth, bool isSigned);

  void clear();

  // Emit the operations in the first micro state in the FSM state when we are
  // jumping to it.
  void emitFirstCtrlState(MachineBasicBlock *DstBB, VASTSlot *Slot,
                          SmallVectorImpl<VASTCnd> &Cnds);

  void emitDatapath(ucState &State);

  void emitUnaryOp(ucOp &UnOp, VASTWire::Opcode Opc);
  void emitBinaryOp(ucOp &BinOp, VASTWire::Opcode Opc);

  void emitOpSel(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpCase(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);

  void emitOpAdd(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitBinaryFUOp(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);

  void emitOpBitSlice(ucOp &OpBitSlice);

  void emitImplicitDef(ucOp &ImpDef);

  void emitCtrlOp(ucState &State, PredMapTy &PredMap,
                  unsigned II, bool Pipelined);

  // Create a condition from a predicate operand.
  VASTCnd createCondition(ucOperand &Op);

  VASTUse getAsOperand(ucOperand &Op);
  void printOperand(ucOperand &Op, raw_ostream &OS);

  void emitOpInternalCall(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpReadReturn(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpUnreachable(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpRetVal(ucOp &Op, VASTSlot *Slot, SmallVectorImpl<VASTCnd> &Cnds);
  void emitOpRet(ucOp &OpRet, VASTSlot *CurSlot, SmallVectorImpl<VASTCnd> &Cnds);
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

  TargetRegisterInfo *RegInfo
    = const_cast<TargetRegisterInfo*>(MF->getTarget().getRegisterInfo());
  TRI = reinterpret_cast<VRegisterInfo*>(RegInfo);

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

  // Building the Slot active signals.
  // FIXME: It is in fact simply printing the logic out.
  VM->buildSlotLogic();

  // TODO: Optimize the RTL net list.
  // FIXME: Do these in seperate passes.

  // Write buffers to output
  VM->printModuleDecl(Out);
  Out.module_begin();
  Out << "\n\n";
  // Reg and wire
  Out << "// Reg and wire decl\n";
  VM->printSignalDecl(Out);
  Out << "\n\n";
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
      VM->addRegister(RegName, BitWidth);
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
      VM->indexVASTValue(FNNum + VFUs::RetPortOffset,
                         VM->addWire(WireName, BitWidth));
      S << ".return_value(" << WireName << "),\n\t";
    }
  }

  emitCommonPort(FNNum);
}

void RTLCodegen::emitIdleState() {
  // The module is busy now
  MachineBasicBlock *EntryBB =  GraphTraits<MachineFunction*>::getEntryNode(MF);
  VASTSlot *IdleSlot = VM->getOrCreateSlot(0, 0);
  VASTValue *StartPort = VM->getPort(VASTModule::Start).get();
  IdleSlot->addNextSlot(FInfo->getStartSlotFor(EntryBB),
                        StartPort);
  IdleSlot->addNextSlot(0, VASTCnd(StartPort, true));

  // Always Disable the finish signal.
  IdleSlot->addDisable(VM->getPort(VASTModule::Finish));
  SmallVector<VASTCnd, 1> Cnds(1, StartPort);
  emitFirstCtrlState(EntryBB, IdleSlot, Cnds);
}

void RTLCodegen::emitBasicBlock(MachineBasicBlock &MBB) {
  unsigned startSlot = FInfo->getStartSlotFor(&MBB);
  unsigned IISlot = FInfo->getIISlotFor(&MBB);
  unsigned II = IISlot - startSlot;
  unsigned EndSlot = FInfo->getEndSlotFor(&MBB);
  PredMapTy NextStatePred;
  MachineBasicBlock::iterator I = MBB.getFirstNonPHI(),
                              E = MBB.getFirstTerminator();
  //ucState FstCtrl(I);
  //if (FstCtrl.empty())
  //  emitSlotsForEmptyState(FstCtrl.getOrCreateSlot(), EndSlot, II);

  // FIXME: Refactor the loop.
  while(++I != E) {
    ucState CurDatapath = *I;
    // Emit the datepath of current state.
    emitDatapath(CurDatapath);

    // Emit next ucOp.
    ucState NextControl = *++I;
    // We are assign the register at the previous slot of this slot, so the
    // datapath op with same slot can read the register schedule to this slot.
    unsigned stateSlot = NextControl.getSlot() - 1;
    // Create the slots.
    VM->getOrCreateSlot(stateSlot, startSlot);
    // There will be alias slot if the BB is pipelined.
    if (startSlot + II < EndSlot) {
      for (unsigned slot = stateSlot; slot < EndSlot; slot += II)
        VM->getOrCreateSlot(slot, startSlot)->setAliasSlots(stateSlot, EndSlot, II);
    }

    if (NextControl.empty()) continue;

    emitCtrlOp(NextControl, NextStatePred, II, IISlot < EndSlot);
  }
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
    // Create the enable signal for bram.
    VM->getOrCreateSymbol(VFUBRam::getEnableName(BramNum), 1);
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
    // Add the start/finsh signal and return_value to the signal list.
    VM->addRegister(Ports[2], 1);
    VM->getOrCreateSymbol(Ports[3], 1);
    unsigned RetPortIdx = FNNum + VFUs::RetPortOffset;
    // Had we allocate the return port?
    VRegisterInfo::PhyRegInfo Info = TRI->getPhyRegInfo(RetPortIdx);
    if (Info.getParentRegister() == FNNum) {
      VASTValue *PortName = VM->getOrCreateSymbol(Ports[4], Info.getBitWidth());
      VM->indexVASTValue(RetPortIdx, PortName);
    }

    // Else ask the constraint about how to instantiates this submodule.
    S << "// External module: " << I->getKey() << '\n';
    S << VFUs::instantiatesModule(I->getKey(), FNNum, Ports);
  }
}

VASTValue *RTLCodegen::emitFUAdd(unsigned FUNum, unsigned BitWidth) {
  // Write the datapath for function unit.
  std::string ResultName = "addsub" + utostr_32(FUNum);
  VASTWire *Result = VM->addWire(ResultName, BitWidth);
  Result->setOpcode(VASTWire::dpAdd);

  unsigned OperandWidth = BitWidth - 1;
  std::string OpName = ResultName + "_a";
  Result->addOperand(VM->addRegister(OpName, OperandWidth));
  OpName = ResultName + "_b";
  Result->addOperand(VM->addRegister(OpName, OperandWidth));
  OpName = ResultName + "_c";
  Result->addOperand(VM->addRegister(OpName, 1));

  return Result;
}

VASTValue *RTLCodegen::emitFUMult(unsigned FUNum, unsigned BitWidth, bool HasHi){
  std::string ResultName = "mult" + utostr_32(FUNum);
  VASTWire *Result = VM->addWire(ResultName, BitWidth);
  Result->setOpcode(VASTWire::dpMul);

  // No need to include the high part is included in the operand register.
  if (HasHi) BitWidth /= 2;

  std::string OpName = ResultName + "_a";
  Result->addOperand(VM->addRegister(OpName, BitWidth));
  OpName = ResultName + "_b";
  Result->addOperand(VM->addRegister(OpName, BitWidth));

  return Result;
}

VASTValue *RTLCodegen::emitFUShift(unsigned FUNum, unsigned BitWidth,
                                   VASTWire::Opcode Opc) {
  std::string ResultName = "shift" + utostr_32(FUNum);
  VASTWire *Result = VM->addWire(ResultName, BitWidth);
  Result->setOpcode(Opc);


  std::string OpName = ResultName + "_a";
  Result->addOperand(VM->addRegister(OpName, BitWidth));
  OpName = ResultName + "_b";
  Result->addOperand(VM->addRegister(OpName, Log2_32_Ceil(BitWidth)));

  return Result;
}

VASTValue *RTLCodegen::emitFUCmp(unsigned FUNum, unsigned BitWidth,
                                 bool isSigned) {
  std::string ResultName = "cmp" + utostr_32(FUNum);
  if (isSigned)  ResultName = "s" + ResultName;
  else           ResultName = "u" + ResultName;

  // Comparer have 4 output port.
  VASTWire *Result = VM->addWire(ResultName, 5);
  Result->setOpcode(isSigned ? VASTWire::dpSCmp : VASTWire::dpUCmp);

  std::string OpName = ResultName + "_a";
  Result->addOperand(VM->addRegister(OpName, BitWidth));
  OpName = ResultName + "_b";
  Result->addOperand(VM->addRegister(OpName, BitWidth));

  return Result;
}

void RTLCodegen::emitAllSignals() {
  for (unsigned i = 0, e = TRI->num_phyreg(); i != e; ++i) {
    unsigned RegNum = i + 1;
    VRegisterInfo::PhyRegInfo Info = TRI->getPhyRegInfo(RegNum);
    if (!Info.isTopLevelReg(RegNum)
        // Sub-register for RCFNRegClass already handled in
        // emitFunctionSignature called by emitAllocatedFUs;
        && Info.getRegClass() != VTM::RCFNRegClassID) {
      unsigned Parent = Info.getParentRegister();
      VASTUse V = VASTUse(VM->lookupSignal(Parent).get(),
                          Info.getUB(), Info.getLB());
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
    case VTM::RUCMPRegClassID:
      VM->indexVASTValue(RegNum, emitFUCmp(RegNum, Info.getBitWidth(), false));
      break;
    case VTM::RSCMPRegClassID:
      VM->indexVASTValue(RegNum, emitFUCmp(RegNum, Info.getBitWidth(), true));
      break;
    case VTM::RMULRegClassID:
      VM->indexVASTValue(RegNum, emitFUMult(RegNum, Info.getBitWidth(), false));
      break;
    case VTM::RMULLHRegClassID:
      VM->indexVASTValue(RegNum, emitFUMult(RegNum, Info.getBitWidth(), true));
      break;
    case VTM::RASRRegClassID: {
      VASTValue *V = emitFUShift(RegNum, Info.getBitWidth(), VASTWire::dpSRA);
      VM->indexVASTValue(RegNum, V);
      break;
    }
    case VTM::RLSRRegClassID:{
      VASTValue *V = emitFUShift(RegNum, Info.getBitWidth(), VASTWire::dpSRL);
      VM->indexVASTValue(RegNum, V);
      break;
    }
    case VTM::RSHLRegClassID:{
      VASTValue *V = emitFUShift(RegNum, Info.getBitWidth(), VASTWire::dpShl);
      VM->indexVASTValue(RegNum, V);
      break;
    }
    case VTM::RBRMRegClassID: {
      VASTValue *V = VM->getOrCreateSymbol(VFUBRam::getInDataBusName(RegNum),
                                           Info.getBitWidth());
      VM->indexVASTValue(RegNum, V);
      break;
    }
    case VTM::RINFRegClassID: {
      // The offset of data input port is 3
      unsigned DataInIdx = VM->getFUPortOf(FuncUnitId(VFUs::MemoryBus, 0)) + 3;
      VM->indexVASTValue(RegNum, VASTUse(VM->getPort(DataInIdx)));
      break;
    }
    case VTM::RCFNRegClassID: /*Nothing to do*/ break;
    default: llvm_unreachable("Unexpected register class!"); break;
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
void RTLCodegen::emitCtrlOp(ucState &State, PredMapTy &PredMap,
                            unsigned II, bool Pipelined){
  assert(State->getOpcode() == VTM::Control && "Bad ucState!");
  MachineBasicBlock *CurBB = State->getParent();
  SmallVector<VASTCnd, 4> Cnds;

  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    unsigned SlotNum = Op->getPredSlot();
    VASTSlot *CurSlot = VM->getSlot(SlotNum - 1);

    assert(SlotNum != CurSlot->getParentIdx() && "Unexpected first slot!");
    // Skip the marker.
    if (Op->getOpcode() == VTM::ImpUse) continue;

    Cnds.clear();
    Cnds.push_back(createCondition(Op.getPredicate()));

    // Special case for state transferring operation.
    if (VInstrInfo::isBrCndLike(Op->getOpcode())) {
      ucOperand &CndOp = Op.getOperand(0);
      Cnds.push_back(createCondition(CndOp));

      MachineBasicBlock *TargetBB = Op.getOperand(1).getMBB();
      unsigned TargetSlotNum = FInfo->getStartSlotFor(TargetBB);
      assert(Op.getPredicate().getReg() == 0 && "Cannot handle predicated BrCnd");
      VASTCnd Cnd = createCondition(CndOp);
      CurSlot->addNextSlot(TargetSlotNum, Cnd);

      // Emit control operation for next state.
      if (TargetBB == CurBB && Pipelined)
        // The loop op of pipelined loop enable next slot explicitly.
        CurSlot->addNextSlot(CurSlot->getSlotNum() + 1);

      // Emit the first micro state of the target state.
      emitFirstCtrlState(TargetBB, CurSlot, Cnds);

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
      if (TargetBB == CurBB && CndSlot > CurSlot->getParentIdx()) {
        Cnds.push_back(VM->getSlot(CndSlot - 1)->getActive());
      } else {
        assert(PredMap.count(TargetBB) && "Loop back predicate not found!");
        VASTRegister::AssignCndTy PredCnd = PredMap.find(TargetBB)->second;
        // Do we need extra predicate slot?
        if (PredCnd.first != CurSlot)
          Cnds.push_back(PredCnd.first->getActive());

        Cnds.insert(Cnds.end(), PredCnd.second.begin(), PredCnd.second.end());
      }
    }

    // Emit the operations.
    switch (Op->getOpcode()) {
    case VTM::VOpMove_ri:
    case VTM::VOpMove_rw:
    case VTM::VOpMove_rr:
    case VTM::VOpMvPhi:
    case VTM::VOpMvPipe:
    case VTM::COPY:             emitOpCopy(Op, CurSlot, Cnds);break;
    case VTM::VOpAdd:           emitOpAdd(Op, CurSlot, Cnds); break;
    case VTM::VOpICmp:
    case VTM::VOpMultLoHi:
    case VTM::VOpMult:
    case VTM::VOpSHL:
    case VTM::VOpSRL:
    case VTM::VOpSRA:           emitBinaryFUOp(Op, CurSlot, Cnds); break;
    case VTM::VOpReadFU:        emitOpReadFU(Op, CurSlot, Cnds);break;
    case VTM::VOpInternalCall:  emitOpInternalCall(Op, CurSlot, Cnds);break;
    case VTM::VOpRetVal:        emitOpRetVal(Op, CurSlot, Cnds);  break;
    case VTM::VOpRet:           emitOpRet(Op, CurSlot, Cnds);     break;
    case VTM::VOpCmdSeq:
    case VTM::VOpMemTrans:      emitOpMemTrans(Op, CurSlot, Cnds);break;
    case VTM::VOpBRam:          emitOpBRam(Op, CurSlot, Cnds);    break;
    case VTM::IMPLICIT_DEF:     emitImplicitDef(Op);          break;
    case VTM::VOpSel:           emitOpSel(Op, CurSlot, Cnds); break;
    case VTM::VOpCase:          emitOpCase(Op, CurSlot, Cnds); break;
    case VTM::VOpReadReturn:    emitOpReadReturn(Op, CurSlot, Cnds);break;
    case VTM::VOpUnreachable:   emitOpUnreachable(Op, CurSlot, Cnds);break;
    default:  assert(0 && "Unexpected opcode!");              break;
    }
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
    case VTM::VOpCase:          emitOpCase(Op, Slot, Cnds);   break;
    case VTM::VOpSel:           emitOpSel(Op, Slot, Cnds);    break;
    case VTM::VOpRetVal:        emitOpRetVal(Op, Slot, Cnds); break;
    case VTM::IMPLICIT_DEF:     emitImplicitDef(Op);          break;
    default:  assert(0 && "Unexpected opcode!");              break;
    }
  }
}

void RTLCodegen::emitOpUnreachable(ucOp &Op, VASTSlot *Slot,
                                   SmallVectorImpl<VASTCnd> &Cnds) {
  vlang_raw_ostream &OS = VM->getControlBlockBuffer();
  std::string PredStr;
  raw_string_ostream SS(PredStr);
  VASTRegister::printCondition(SS, Slot, Cnds);
  SS.flush();
  OS.if_begin(PredStr);
  OS << "$display(\"BAD BAD BAD BAD! Run to unreachable\");\n";
  OS << "$finish();\n";
  OS.exit_block();

  Slot->addNextSlot(0);
}

void RTLCodegen::emitOpAdd(ucOp &Op, VASTSlot *Slot,
                           SmallVectorImpl<VASTCnd> &Cnds) {
  VASTWire *Result = cast<VASTWire>(getAsOperand(Op.getOperand(0)));
  VASTRegister *R = cast<VASTRegister>(Result->getOperand(0));
  VM->addAssignment(R, getAsOperand(Op.getOperand(1)), Slot, Cnds);
  R = cast<VASTRegister>(Result->getOperand(1));
  VM->addAssignment(R, getAsOperand(Op.getOperand(2)), Slot, Cnds);
  R = cast<VASTRegister>(Result->getOperand(2));
  VM->addAssignment(R, getAsOperand(Op.getOperand(3)), Slot, Cnds);
}

void RTLCodegen::emitBinaryFUOp(ucOp &Op, VASTSlot *Slot,
                                SmallVectorImpl<VASTCnd> &Cnds) {
  VASTWire *Result = cast<VASTWire>(getAsOperand(Op.getOperand(0)));
  VASTRegister *R = cast<VASTRegister>(Result->getOperand(0));
  VM->addAssignment(R, getAsOperand(Op.getOperand(1)), Slot, Cnds);
  R = cast<VASTRegister>(Result->getOperand(1));
  VM->addAssignment(R, getAsOperand(Op.getOperand(2)), Slot, Cnds);
}

void RTLCodegen::emitImplicitDef(ucOp &ImpDef) {
  //VASTDatapath *data = VM->createDatapath();
  //VASTDatapath::builder_stream &OS = data->getCodeBuffer();
  //OS << "// IMPLICIT_DEF " << ImpDef.getOperand(0) << "\n";
}

void RTLCodegen::emitOpSel(ucOp &Op, VASTSlot *Slot,
                           SmallVectorImpl<VASTCnd> &Cnds) {
  VASTRegister *R = cast<VASTRegister>(getAsOperand(Op.getOperand(0)));
  // Assign the value for condition true.
  VASTCnd Cnd = createCondition(Op.getOperand(1));
  Cnds.push_back(Cnd);
  VM->addAssignment(R, getAsOperand(Op.getOperand(2)), Slot, Cnds);
  // Assign the value for condition false.
  Cnds.back() = Cnd.invert();
  VM->addAssignment(R, getAsOperand(Op.getOperand(3)), Slot, Cnds);
  Cnds.pop_back();
}

void RTLCodegen::emitOpCase(ucOp &Op, VASTSlot *Slot,
                            SmallVectorImpl<VASTCnd> &Cnds) {
  // Check if we got any case hitted
  vlang_raw_ostream &OS = VM->getControlBlockBuffer();
  OS.if_();
  VASTRegister::printCondition(OS, Slot, Cnds);
  OS._then();
  OS.switch_begin("1'b1");

  VASTRegister *R = cast<VASTRegister>(getAsOperand(Op.getOperand(0)));
  for (unsigned i = 1, e = Op.getNumOperands(); i < e; i +=2) {
    Cnds.push_back(createCondition(Op.getOperand(i)));
    VM->addAssignment(R, getAsOperand(Op.getOperand(i + 1)), Slot, Cnds);
    // Do nothing if any case hit.
    Cnds.back().print(OS);
    OS << ":/*Case hit, do nothing*/;\n";

    Cnds.pop_back();
  }

  // Report an error if no case hitted.
  OS << "default: begin $display(\"Case miss in VOpCase at "
     << Slot->getName() << " assigning: " << R->getName() << " in "
     << VM->getName() << "\"); $finish(); end\n";
  OS.switch_end();
  OS.exit_block() << "\n";
}

void RTLCodegen::emitOpCopy(ucOp &Op, VASTSlot *Slot,
                            SmallVectorImpl<VASTCnd> &Cnds) {
  ucOperand &Dst = Op.getOperand(0), &Src = Op.getOperand(1);
  // Ignore the identical copy.
  if (Src.isReg() && Dst.getReg() == Src.getReg()) return;

  VASTRegister *R = cast<VASTRegister>(getAsOperand(Dst));
  VM->addAssignment(R, getAsOperand(Src), Slot, Cnds);
}

void RTLCodegen::emitOpReadFU(ucOp &Op, VASTSlot *CurSlot,
                              SmallVectorImpl<VASTCnd> &Cnds) {
  FuncUnitId Id = Op->getFUId();
  VASTValue *ReadyPort = 0;

  switch (Id.getFUType()) {
  case VFUs::MemoryBus:
    ReadyPort = VM->getSymbol(VFUMemBus::getReadyName(Id.getFUNum()));
    break;
  case VFUs::CalleeFN: {
    unsigned FNNum = Op.getOperand(1).getReg();
    ReadyPort = VM->getSymbol(getSubModulePortName(FNNum, "fin"));
    break;
  }
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
  VASTRegister *R = cast<VASTRegister>(getAsOperand(Op.getOperand(0)));
  VM->addAssignment(R, getAsOperand(Op.getOperand(1)), Slot, Cnds);
}

void RTLCodegen::emitOpInternalCall(ucOp &Op, VASTSlot *Slot,
                                    SmallVectorImpl<VASTCnd> &Cnds) {
  // Assign input port to some register.
  const char *CalleeName = Op.getOperand(1).getSymbolName();
  unsigned FNNum = FInfo->getCalleeFNNum(CalleeName);

  VASTCnd Pred = createCondition(Op.getPredicate());
  std::string StartPortName = getSubModulePortName(FNNum, "start");
  VASTValue *StartSignal = VM->getSymbol(StartPortName);
  Slot->addEnable(StartSignal, Pred);
  VASTSlot *NextSlot = VM->getOrCreateNextSlot(Slot);
  NextSlot->addDisable(StartSignal, Pred);

  const Function *FN = M->getFunction(CalleeName);
  if (FN && !FN->isDeclaration()) {
    Function::const_arg_iterator ArgIt = FN->arg_begin();
    for (unsigned i = 0, e = FN->arg_size(); i != e; ++i) {
      VASTRegister *R =
        VM->getSymbol<VASTRegister>(getSubModulePortName(FNNum, ArgIt->getName()));
      VM->addAssignment(R, getAsOperand(Op.getOperand(2 + i)), Slot, Cnds);
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

void RTLCodegen::emitOpRet(ucOp &Op, VASTSlot *CurSlot,
                           SmallVectorImpl<VASTCnd> &Cnds) {
  // Go back to the idle slot.
  VASTCnd Pred = createCondition(Op.getPredicate());
  CurSlot->addNextSlot(0, Pred);
  CurSlot->addEnable(VM->getPort(VASTModule::Finish), Pred);
}

void RTLCodegen::emitOpRetVal(ucOp &Op, VASTSlot *Slot,
                              SmallVectorImpl<VASTCnd> &Cnds) {
  VASTRegister &RetReg = cast<VASTRegister>(*VM->getRetPort());
  unsigned retChannel = Op.getOperand(1).getImm();
  assert(retChannel == 0 && "Only support Channel 0!");
  VM->addAssignment(&RetReg, getAsOperand(Op.getOperand(0)), Slot, Cnds);
}

void RTLCodegen::emitOpMemTrans(ucOp &Op, VASTSlot *Slot,
                                SmallVectorImpl<VASTCnd> &Cnds) {
  unsigned FUNum = Op->getFUId().getFUNum();

  // Emit Address.
  std::string RegName = VFUMemBus::getAddrBusName(FUNum) + "_r";
  VASTRegister *R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getAsOperand(Op.getOperand(1)), Slot, Cnds);
  // Assign store data.
  RegName = VFUMemBus::getOutDataBusName(FUNum) + "_r";
  R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getAsOperand(Op.getOperand(2)), Slot, Cnds);
  // And write enable.
  RegName = VFUMemBus::getCmdName(FUNum) + "_r";
  R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getAsOperand(Op.getOperand(3)), Slot, Cnds);
  // The byte enable.
  RegName = VFUMemBus::getByteEnableName(FUNum) + "_r";
  R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getAsOperand(Op.getOperand(4)), Slot, Cnds);

  // Remember we enabled the memory bus at this slot.
  std::string EnableName = VFUMemBus::getEnableName(FUNum) + "_r";
  VASTValue *MemEn = VM->getSymbol(EnableName);
  VASTCnd Pred = createCondition(Op.getPredicate());
  Slot->addEnable(MemEn, Pred);

  // Disable the memory at next slot.
  // TODO: Assert the control flow is linear.
  VASTSlot *NextSlot = VM->getOrCreateNextSlot(Slot);
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
  PredSS.flush();

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
  VASTValue *MemEn = VM->getSymbol(EnableName);
  VASTCnd Pred = createCondition(Op.getPredicate());
  Slot->addEnable(MemEn, Pred);

  // Disable the memory at next slot.
  // TODO: Assert the control flow is linear.
  VASTSlot *NextSlot = VM->getOrCreateNextSlot(Slot);
  NextSlot->addDisable(MemEn, Pred);
}

void RTLCodegen::emitDatapath(ucState &State) {
  assert(State->getOpcode() == VTM::Datapath && "Bad ucState!");

  for (ucState::iterator I = State.begin(), E = State.end(); I != E; ++I) {
    ucOp Op = *I;
    switch (Op->getOpcode()) {
    case VTM::VOpBitSlice:  emitOpBitSlice(Op);                       break;
    case VTM::VOpBitCat:    emitBinaryOp(Op, VASTWire::dpBitCat);     break;
    case VTM::VOpBitRepeat: emitBinaryOp(Op, VASTWire::dpBitRepeat);  break;

    case VTM::ImpUse:       /*Not need to handle*/  break;

    case VTM::VOpXor:       emitBinaryOp(Op, VASTWire::dpXor);    break;
    case VTM::VOpAnd:       emitBinaryOp(Op, VASTWire::dpAnd);    break;
    case VTM::VOpOr:        emitBinaryOp(Op, VASTWire::dpOr);     break;

    case VTM::VOpNot:       emitUnaryOp(Op, VASTWire::dpNot);     break;
    case VTM::VOpROr:       emitUnaryOp(Op, VASTWire::dpROr);     break;
    case VTM::VOpRAnd:      emitUnaryOp(Op, VASTWire::dpRAnd);    break;
    case VTM::VOpRXor:      emitUnaryOp(Op, VASTWire::dpRXor);    break;

    default:  assert(0 && "Unexpected opcode!");    break;
    }
  }
}

void RTLCodegen::emitUnaryOp(ucOp &UnaOp, VASTWire::Opcode Opc) {
  VASTWire &V = *cast<VASTWire>(getAsOperand(UnaOp.getOperand(0)));
  if (!V.isUnknownOp()) return;

  V.setOpcode(Opc);
  V.addOperand(getAsOperand(UnaOp.getOperand(1)));
}

void RTLCodegen::emitBinaryOp(ucOp &BinOp, VASTWire::Opcode Opc) {
  VASTWire &V = *cast<VASTWire>(getAsOperand(BinOp.getOperand(0)));
  if (!V.isUnknownOp()) return;

  V.setOpcode(Opc);
  V.addOperand(getAsOperand(BinOp.getOperand(1)));
  V.addOperand(getAsOperand(BinOp.getOperand(2)));
}

void RTLCodegen::emitOpBitSlice(ucOp &OpBitSlice) {
  VASTWire &V = *cast<VASTWire>(getAsOperand(OpBitSlice.getOperand(0)));
  if (!V.isUnknownOp()) return;

  V.setOpcode(VASTWire::dpAssign);
  // Get the range of the bit slice, Note that the
  // bit at upper bound is excluded in VOpBitSlice
  unsigned UB = OpBitSlice.getOperand(2).getImm(),
           LB = OpBitSlice.getOperand(3).getImm();

  VASTUse RHS = getAsOperand(OpBitSlice.getOperand(1));
  assert(RHS.UB != 0 && "Cannot get bitslice without bitwidth information!");
  // Adjust ub and lb.
  LB += RHS.LB;
  UB += RHS.LB;
  assert(UB <= RHS.UB && "Bitslice out of range!");
  V.addOperand(VASTUse(RHS.get(), UB, LB));
}

VASTCnd RTLCodegen::createCondition(ucOperand &Op) {
  // Is there an always true predicate?
  if (VInstrInfo::isAlwaysTruePred(Op)) return VASTCnd(true);

  // Otherwise it must be some signal.
  return VASTCnd(VM->lookupSignal(Op.getReg()), Op.isPredicateInverted());
}

VASTUse RTLCodegen::getAsOperand(ucOperand &Op) {
  switch (Op.getType()) {
  case MachineOperand::MO_Register: {
    VASTUse V = VM->lookupSignal(Op.getReg());
    assert (V.get() && "Cannot find this Value in vector!");
    return V;
  }
  case MachineOperand::MO_Immediate:
    return VASTUse(Op.getImm(), Op.getBitWidth());
  case MachineOperand::MO_ExternalSymbol:
    return VASTUse(Op.getSymbolName(), Op.getBitWidth());
  default:
    break;
  }

  // DirtyHack: simply create a symbol.
  std::string Name;
  raw_string_ostream SS(Name);
  Op.print(SS);
  SS.flush();

  return VM->getOrCreateSymbol(Name, 0);
}

void RTLCodegen::printOperand(ucOperand &Op, raw_ostream &OS) {
  if(Op.isReg()){
    getAsOperand(Op).print(OS);
    return;
  }

  Op.print(OS);
}
