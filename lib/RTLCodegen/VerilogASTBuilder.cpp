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
// This file implement the VerilogASTBuilder pass, which write VTM machine instructions
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
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetData.h"
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
class VerilogASTBuilder : public MachineFunctionPass {
  const Module *M;
  MachineFunction *MF;
  TargetData *TD;
  VRegisterInfo *TRI;
  VFInfo *FInfo;
  MachineRegisterInfo *MRI;
  VASTModule *VM;

  // Mapping success fsm state to their predicate in current state.
  typedef std::map<MachineBasicBlock*, VASTWire*> PredMapTy;

  void emitFunctionSignature(const Function *F);
  void emitCommonPort(unsigned FNNum);

  struct MemBusBuilder {
    VASTModule *VM;
    VFUMemBus *Bus;
    unsigned BusNum;
    VASTWire *MembusEn, *MembusCmd, *MemBusAddr, *MemBusOutData, *MemBusByteEn;
    // Helper class to build the expression.
    VASTWireBuilder EnExpr, CmdExpr, AddrExpr, OutDataExpr, BeExpr;

    VASTWire *createOutputPort(const std::string &PortName, unsigned BitWidth,
                               VASTRegister *&LocalEn, VASTWireBuilder &Expr) {
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
        Expr.init(VASTWire::dpOr, OutputWire->getBitWidth());
        // Add the local enable.
        Expr.addOperand(LocalReg);
        LocalEn = LocalReg;
      } else{
        Expr.init(VASTWire::dpMux, OutputWire->getBitWidth());
        // Select the local signal if local enable is true.
        Expr.addOperand(LocalEn);
        Expr.addOperand(LocalReg);
      }

      return OutputWire;
    }

    void addSubModuleOutPort(raw_ostream &S, VASTWire *OutputWire,
                             unsigned BitWidth, const std::string &SubModuleName,
                             VASTWire *&SubModEn, VASTWireBuilder &Expr) {
      std::string ConnectedWireName = SubModuleName + "_"
                                      + std::string(OutputWire->getName());

      VASTWire *SubModWire = VM->addWire(ConnectedWireName, BitWidth);

      // Are we creating the enable signal from sub module?
      if (SubModEn == 0) {
        Expr.addOperand(SubModWire);
        SubModEn = SubModWire;
      } else {
        // Select the signal from submodule if sub module enable is true.
        Expr.addOperand(SubModEn);
        Expr.addOperand(SubModWire);
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
      addSubModuleOutPort(S, MembusEn, 1, SubModuleName, SubModEn, EnExpr);
      // Output ports.
      addSubModuleOutPort(S, MembusCmd, VFUMemBus::CMDWidth, SubModuleName,
                          SubModEn, CmdExpr);
      addSubModuleOutPort(S, MemBusAddr, Bus->getAddrWidth(), SubModuleName,
                          SubModEn, AddrExpr);
      addSubModuleOutPort(S, MemBusOutData, Bus->getDataWidth(), SubModuleName,
                          SubModEn, OutDataExpr);
      addSubModuleOutPort(S, MemBusByteEn, Bus->getDataWidth()/8, SubModuleName,
                          SubModEn, BeExpr);

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
        createOutputPort(VFUMemBus::getEnableName(BusNum), 1, LocalEn, EnExpr);
      MembusCmd =
        createOutputPort(VFUMemBus::getCmdName(BusNum), VFUMemBus::CMDWidth,
                         LocalEn, CmdExpr);

      // Address port.
      MemBusAddr =
        createOutputPort(VFUMemBus::getAddrBusName(BusNum), Bus->getAddrWidth(),
                         LocalEn, AddrExpr);
      // Data ports.
      VM->addInputPort(VFUMemBus::getInDataBusName(BusNum), Bus->getDataWidth());
      MemBusOutData =
        createOutputPort(VFUMemBus::getOutDataBusName(BusNum),
                         Bus->getDataWidth(), LocalEn, OutDataExpr);
      // Byte enable.
      MemBusByteEn =
        createOutputPort(VFUMemBus::getByteEnableName(BusNum),
                         Bus->getDataWidth() / 8, LocalEn, BeExpr);
      // Bus ready.
      VM->addInputPort(VFUMemBus::getReadyName(BusNum), 1);
    }

    ~MemBusBuilder() {
      VM->buildExpr(EnExpr, MembusEn);
      VM->buildExpr(CmdExpr, MembusCmd);
      VM->buildExpr(AddrExpr, MemBusAddr);
      VM->buildExpr(OutDataExpr, MemBusOutData);
      VM->buildExpr(BeExpr, MemBusByteEn);
    }
  };

  /// emitAllocatedFUs - Set up a vector for allocated resources, and
  /// emit the ports and register, wire and datapath for them.
  void emitAllocatedFUs();

  void emitIdleState();

  void emitBasicBlock(MachineBasicBlock &MBB);

  void emitAllSignals();
  bool emitVReg(unsigned RegNum, const TargetRegisterClass *RC, bool isReg);
  VASTValue *emitFUAdd(unsigned FUNum, unsigned BitWidth);
  VASTValue *emitFUMult(unsigned FUNum, unsigned BitWidth, bool HasHi);
  VASTValue *emitFUShift(unsigned FUNum, unsigned BitWidth,
                         VASTWire::Opcode Opc);
  VASTValue *emitFUCmp(unsigned FUNum, unsigned BitWidth, bool isSigned);

  // Emit the operations in the first micro state in the FSM state when we are
  // jumping to it.
  void emitFirstCtrlBundle(MachineBasicBlock *DstBB, VASTSlot *Slot,
                          SmallVectorImpl<VASTUse> &Cnds);

  MachineBasicBlock::instr_iterator emitDatapath(MachineInstr *Bundle);

  void emitUnaryOp(MachineInstr *MI, VASTWire::Opcode Opc);
  void emitBinaryOp(MachineInstr *MI, VASTWire::Opcode Opc);
  void emitOpLut(MachineInstr *MI);

  void emitChainedOpAdd(MachineInstr *MI);
  void emitChainedOpICmp(MachineInstr *MI);

  void emitOpSel(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);
  void emitOpCase(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);

  void emitOpAdd(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);
  void emitBinaryFUOp(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);

  void emitOpBitSlice(MachineInstr *MI);

  void emitImplicitDef(MachineInstr *MI);

  MachineBasicBlock::instr_iterator
  emitCtrlOp(MachineInstr *Bundle, PredMapTy &PredMap, unsigned II, bool Pipelined);

  // Create a condition from a predicate operand.
  VASTUse createCnd(ucOperand &Op);
  VASTUse createCnd(MachineOperand &Op) {
    return createCnd(cast<ucOperand>(Op));
  }
  
  void getPredValAtNextSlot(MachineInstr *MI, VASTUse &Pred);

  VASTUse getAsOperand(ucOperand &Op);
  VASTUse getAsOperand(MachineOperand &Op) {
    return getAsOperand(cast<ucOperand>(Op));
  }

  template <class Ty>
  Ty *getAsLValue(ucOperand &Op) {
    assert(Op.isReg() && "Bad MO type for LValue!");

    VASTUse U = VM->lookupSignal(Op.getReg());
    if (VASTValue *V = U.getOrNull())
      return dyn_cast<Ty>(V);

    return 0;
  }
  template <class Ty>
  Ty *getAsLValue(MachineOperand &Op) {
    return getAsLValue<Ty>(cast<ucOperand>(Op));
  }

  void printOperand(ucOperand &Op, raw_ostream &OS);
  void printOperand(MachineOperand &Op, raw_ostream &OS) {
    printOperand(cast<ucOperand>(Op), OS);
  }

  void emitOpInternalCall(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);
  void emitOpReadReturn(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);
  void emitOpUnreachable(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);
  void emitOpRetVal(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);
  void emitOpRet(MachineInstr *MIRet, VASTSlot *CurSlot, SmallVectorImpl<VASTUse> &Cnds);
  void emitOpCopy(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);
  void emitOpReadFU(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);
  void emitOpMemTrans(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);
  void emitOpBRam(MachineInstr *MI, VASTSlot *Slot, SmallVectorImpl<VASTUse> &Cnds);

  std::string getSubModulePortName(unsigned FNNum,
                                   const std::string PortName) const {
    return "SubMod" + utostr(FNNum) + "_" + PortName;
  }

public:
  /// @name FunctionPass interface
  //{
  static char ID;

  VerilogASTBuilder() : MachineFunctionPass(ID) {
    initializeVerilogASTBuilderPass(*PassRegistry::getPassRegistry());
  }

  ~VerilogASTBuilder();

  bool doInitialization(Module &M);

  bool runOnMachineFunction(MachineFunction &MF);

  virtual void print(raw_ostream &O, const Module *M) const;
  //}
};

}

//===----------------------------------------------------------------------===//
char VerilogASTBuilder::ID = 0;

Pass *llvm::createVerilogASTBuilderPass() {
  return new VerilogASTBuilder();
}

INITIALIZE_PASS_BEGIN(VerilogASTBuilder, "vtm-rtl-info-VerilogASTBuilder",
                      "Build RTL Verilog module for synthesised function.",
                      false, true)
INITIALIZE_PASS_END(VerilogASTBuilder, "vtm-rtl-info-VerilogASTBuilder",
                    "Build RTL Verilog module for synthesised function.",
                    false, true)

bool VerilogASTBuilder::doInitialization(Module &Mod) {
  M = &Mod;
  return false;
}

bool VerilogASTBuilder::runOnMachineFunction(MachineFunction &F) {
  MF = &F;
  TD = getAnalysisIfAvailable<TargetData>();
  FInfo = MF->getInfo<VFInfo>();
  MRI = &MF->getRegInfo();

  TargetRegisterInfo *RegInfo
    = const_cast<TargetRegisterInfo*>(MF->getTarget().getRegisterInfo());
  TRI = reinterpret_cast<VRegisterInfo*>(RegInfo);

  // Reset the current fsm state number.

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
  for (MachineFunction::iterator I = MF->begin(), E = MF->end(); I != E; ++I)
    emitBasicBlock(*I);

  // Building the Slot active signals.
  VM->buildSlotLogic();

  // TODO: Optimize the RTL net list.
  // FIXME: Do these in separate passes.
  VM->eliminateConstRegisters();
  return false;
}

void VerilogASTBuilder::print(raw_ostream &O, const Module *M) const {

}

void VerilogASTBuilder::emitFunctionSignature(const Function *F) {
  raw_ostream &S = VM->getDataPathBuffer();
  unsigned FNNum = FInfo->getCalleeFNNum(F->getName());
  for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
      I != E; ++I) {
    const Argument *Arg = I;
    std::string Name = Arg->getName();
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

  Type *RetTy = F->getReturnType();
  if (!RetTy->isVoidTy()) {
    assert(RetTy->isIntegerTy() && "Only support return integer now!");
    unsigned BitWidth = TD->getTypeSizeInBits(RetTy);
    if (FNNum == 0)
      VM->addOutputPort("return_value", BitWidth, VASTModule::RetPort);
    else {
      std::string WireName = getSubModulePortName(FNNum, "return_value");
      VM->indexVASTValue(FNNum, VM->addWire(WireName, BitWidth));
      S << ".return_value(" << WireName << "),\n\t";
    }
  }

  emitCommonPort(FNNum);
}

void VerilogASTBuilder::emitIdleState() {
  // The module is busy now
  MachineBasicBlock *EntryBB =  GraphTraits<MachineFunction*>::getEntryNode(MF);
  VASTSlot *IdleSlot = VM->getOrCreateSlot(0, 0);
  VASTValue *StartPort = VM->getPort(VASTModule::Start).get();
  unsigned EntryStartSlot = FInfo->getStartSlotFor(EntryBB);
  IdleSlot->addNextSlot(VM->getOrCreateSlot(EntryStartSlot, EntryStartSlot),
                        StartPort);
  IdleSlot->addNextSlot(IdleSlot, VM->buildNotExpr(StartPort));

  // Always Disable the finish signal.
  IdleSlot->addDisable(VM->getPort(VASTModule::Finish));
  SmallVector<VASTUse, 1> Cnds(1, StartPort);
  emitFirstCtrlBundle(EntryBB, IdleSlot, Cnds);
}

void VerilogASTBuilder::emitBasicBlock(MachineBasicBlock &MBB) {
  unsigned startSlot = FInfo->getStartSlotFor(&MBB);
  unsigned IISlot = FInfo->getIISlotFor(&MBB);
  unsigned II = IISlot - startSlot;
  unsigned EndSlot = FInfo->getEndSlotFor(&MBB);
  PredMapTy NextStatePred;
  typedef MachineBasicBlock::instr_iterator instr_it;
  instr_it I = &*llvm::next(MachineBasicBlock::iterator(MBB.getFirstNonPHI()));

  //ucState FstCtrl(I);
  //if (FstCtrl.empty())
  //  emitSlotsForEmptyState(FstCtrl.getOrCreateSlot(), EndSlot, II);

  // FIXME: Refactor the loop.
  while(!I->isTerminator()) {
    // Emit the datepath of current state.
    I = emitDatapath(I);

    // Emit next ucOp.
    // We are assign the register at the previous slot of this slot, so the
    // datapath op with same slot can read the register schedule to this slot.
    unsigned stateSlot = getBundleSlot(I) - 1;
    // Create the slots.
    VM->getOrCreateSlot(stateSlot, startSlot);
    // There will be alias slot if the BB is pipelined.
    if (startSlot + II < EndSlot) {
      for (unsigned slot = stateSlot; slot < EndSlot; slot += II)
        VM->getOrCreateSlot(slot, startSlot)->setAliasSlots(stateSlot, EndSlot, II);
    }

    I = emitCtrlOp(I, NextStatePred, II, IISlot < EndSlot);
  }
}

void VerilogASTBuilder::emitCommonPort(unsigned FNNum) {
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

void VerilogASTBuilder::emitAllocatedFUs() {
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
    VM->addRegister(VFUBRam::getEnableName(BramNum), 1);
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
          << I->getKey() << "_inst" << "(\n\t";
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
    // Else ask the constraint about how to instantiates this submodule.
    S << "// External module: " << I->getKey() << '\n';
    S << VFUs::instantiatesModule(I->getKey(), FNNum, Ports);

    // Add the start/finsh signal and return_value to the signal list.
    VM->addRegister(Ports[2], 1);
    VM->getOrCreateSymbol(Ports[3], 1);
    unsigned RetPortIdx = FNNum;
    // Dose the submodule have a return port?
    VRegisterInfo::PhyRegInfo Info = TRI->getPhyRegInfo(RetPortIdx);
    if (Info.getBitWidth()) {
      SmallVector<VFUs::ModOpInfo, 4> OpInfo;
      unsigned Latency = VFUs::getModuleOperands(I->getKey(), FNNum, OpInfo);

      if (Latency == 0) {
        VASTValue *PortName = VM->getOrCreateSymbol(Ports[4], Info.getBitWidth());
        VM->indexVASTValue(RetPortIdx, PortName);
        continue;
      }

      VASTWire *ResultWire = VM->addWire(Ports[4], Info.getBitWidth());
      VM->indexVASTValue(RetPortIdx, ResultWire);

      SmallVector<VASTUse, 4> Ops;
      for (unsigned i = 0, e = OpInfo.size(); i < e; ++i)
        Ops.push_back(VM->addRegister(OpInfo[i].first, OpInfo[i].second));

      VM->buildExpr(VASTWire::dpVarLatBB, Ops, Info.getBitWidth(), ResultWire);
      ResultWire->setLatency(Latency);
      // Remember the latency information.
      VM->addBBLatInfo(FNNum, ResultWire);
    }
  }
}

VASTValue *VerilogASTBuilder::emitFUAdd(unsigned FUNum, unsigned BitWidth) {
  // Write the datapath for function unit.
  std::string ResultName = "addsub" + utostr_32(FUNum) + "o";
  VASTWire *Result = VM->addWire(ResultName, BitWidth);
  unsigned OperandWidth = BitWidth - 1;

  VM->buildExpr(VASTWire::dpAdd,
                VM->addRegister(ResultName + "_a", OperandWidth),
                VM->addRegister(ResultName + "_b", OperandWidth),
                VM->addRegister(ResultName + "_c", 1), BitWidth,
                Result);
  return Result;
}

VASTValue *VerilogASTBuilder::emitFUMult(unsigned FUNum, unsigned BitWidth, bool HasHi){
  std::string ResultName = "mult" + utostr_32(FUNum) + "o";
  VASTWire *Result = VM->addWire(ResultName, BitWidth);

  // No need to include the high part is included in the operand register.
  unsigned OperandWidth = BitWidth;
  if (HasHi) OperandWidth /= 2;

  VM->buildExpr(VASTWire::dpMul,
                VM->addRegister(ResultName + "_a", OperandWidth),
                VM->addRegister(ResultName + "_b", OperandWidth),
                BitWidth, Result);

  return Result;
}

VASTValue *VerilogASTBuilder::emitFUShift(unsigned FUNum, unsigned BitWidth,
                                   VASTWire::Opcode Opc) {
  std::string ResultName = "shift" + utostr_32(FUNum) + "o";
  VASTWire *Result = VM->addWire(ResultName, BitWidth);

  VM->buildExpr(Opc, VM->addRegister(ResultName + "_a", BitWidth),
                VM->addRegister(ResultName + "_b", Log2_32_Ceil(BitWidth)),
                BitWidth, Result);
  return Result;
}

VASTValue *VerilogASTBuilder::emitFUCmp(unsigned FUNum, unsigned BitWidth,
                                 bool isSigned) {
  std::string ResultName = "cmp" + utostr_32(FUNum) + "o";
  if (isSigned)  ResultName = "s" + ResultName;
  else           ResultName = "u" + ResultName;

  // Comparer have 4 output port.
  VASTWire *Result = VM->addWire(ResultName, 5);

  VM->buildExpr(isSigned ? VASTWire::dpSCmp : VASTWire::dpUCmp,
                VM->addRegister(ResultName + "_a", BitWidth),
                VM->addRegister(ResultName + "_b", BitWidth), 5,
                Result);
  return Result;
}

void VerilogASTBuilder::emitAllSignals() {
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
      // FIXME: Do not use such magic number!
      // The offset of data input port is 3
      unsigned DataInIdx = VM->getFUPortOf(FuncUnitId(VFUs::MemoryBus, 0)) + 3;
      VM->indexVASTValue(RegNum, VASTUse(VM->getPort(DataInIdx)));
      break;
    }
    case VTM::RCFNRegClassID:
      /*Nothing to do, it is allocated by emitAllocatedFUs*/
      break;
    case VTM::RMUXRegClassID: {
      std::string Name = "dstmux" + utostr_32(RegNum) + "r";
      VM->indexVASTValue(RegNum, VM->addRegister(Name, Info.getBitWidth()));
      break;
    }
    default: llvm_unreachable("Unexpected register class!"); break;
    }
  }

  for (unsigned i = 0, e = MRI->getNumVirtRegs(); i != e; ++i) {
    unsigned VReg = TargetRegisterInfo::index2VirtReg(i);
    emitVReg(VReg, VTM::WireRegisterClass, false);
  }
}

bool VerilogASTBuilder::emitVReg(unsigned RegNum, const TargetRegisterClass *RC,
                                 bool isReg) {
  if (MRI->getRegClass(RegNum) != RC)
    return false;

  MachineRegisterInfo::def_iterator DI = MRI->def_begin(RegNum);
  if (DI == MRI->def_end()) return false;

  const ucOperand &Op = cast<ucOperand>(DI.getOperand());
  unsigned Bitwidth = Op.getBitWidth();
  if (!isReg)
    VM->addWire(RegNum, Bitwidth);
  else {
    VM->addRegister(RegNum, Bitwidth);
    TotalRegisterBits += Bitwidth;
  }

  return true;
}


VerilogASTBuilder::~VerilogASTBuilder() {}

//===----------------------------------------------------------------------===//
MachineBasicBlock::instr_iterator
VerilogASTBuilder::emitCtrlOp(MachineInstr *Bundle, PredMapTy &PredMap,
                              unsigned II, bool Pipelined){
  MachineBasicBlock *CurBB = Bundle->getParent();
  assert(Bundle->getOpcode() == VTM::CtrlStart && "Expect control bundle!");
  SmallVector<VASTUse, 4> Cnds;

  typedef MachineBasicBlock::instr_iterator instr_it;
  instr_it I = Bundle;
  while ((++I)->isInsideBundle()) {
    MachineInstr *MI = I;
    // Skip the marker.
    if (MI->getOpcode() == VTM::CtrlEnd) continue;

    unsigned SlotNum = getInstrSlot(MI);
    VASTSlot *CurSlot = VM->getSlot(SlotNum - 1);

    assert(SlotNum != CurSlot->getParentIdx() && "Unexpected first slot!");

    Cnds.clear();
    Cnds.push_back(createCnd(*VInstrInfo::getPredOperand(MI)));

    // Special case for state transferring operation.
    if (VInstrInfo::isBrCndLike(MI->getOpcode())) {
      ucOperand &CndOp = cast<ucOperand>(MI->getOperand(0));
      Cnds.push_back(createCnd(CndOp));

      MachineBasicBlock *TargetBB = MI->getOperand(1).getMBB();
      unsigned TargetSlotNum = FInfo->getStartSlotFor(TargetBB);
      VASTSlot *TargetSlot = VM->getOrCreateSlot(TargetSlotNum, TargetSlotNum);
      assert(VInstrInfo::getPredOperand(MI)->getReg() == 0 &&
             "Cannot handle predicated BrCnd");
      VASTUse Cnd = createCnd(CndOp);
      CurSlot->addNextSlot(TargetSlot, Cnd);

      // Emit control operation for next state.
      if (TargetBB == CurBB && Pipelined)
        // The loop op of pipelined loop enable next slot explicitly.
        CurSlot->addNextSlot(VM->getOrCreateNextSlot(CurSlot));

      // Emit the first micro state of the target state.
      emitFirstCtrlBundle(TargetBB, CurSlot, Cnds);
      PredMap.insert(std::make_pair(TargetBB, VM->buildAssignCnd(CurSlot, Cnds)));
      continue;
    }

    // Loop back PHI node moving only active when current slot and the same
    // slot at previous (.i.e Slot - II) are both enable. Which means we are
    // looping back, so besides the predicate condition of current slot, the we
    // need to also add the predicate condition for looping back, that means we
    // only assign the loop back PHI when we are looping back.
    if (MI->getOpcode() == VTM::VOpMvPhi) {
      MachineBasicBlock *TargetBB = MI->getOperand(2).getMBB();
      unsigned CndSlot = SlotNum - II;
      if (TargetBB == CurBB && CndSlot > CurSlot->getParentIdx()) {
        Cnds.push_back(VM->getSlot(CndSlot - 1)->getActive());
      } else {
        // Get the loop back condition, merged the loop back condition into
        // current condition.
        //assert(PredMap.count(TargetBB) && "Loop back predicate not found!");
        //VASTWire *PredCnd = PredMap.find(TargetBB)->second;
        // Slot active already of PredCnd included, no need to worry about.
        //Cnds.append(PredCnd->op_begin(), PredCnd->op_end());
      }
    }

    // Emit the operations.
    switch (MI->getOpcode()) {
    case VTM::VOpDstMux:
    case VTM::VOpMove_ri:
    case VTM::VOpMove_rw:
    case VTM::VOpMove_rr:
    case VTM::VOpMvPhi:
    case VTM::VOpMvPipe:
    case VTM::COPY:             emitOpCopy(MI, CurSlot, Cnds);break;
    case VTM::VOpAdd:           emitOpAdd(MI, CurSlot, Cnds); break;
    case VTM::VOpICmp:
    case VTM::VOpMultLoHi:
    case VTM::VOpMult:
    case VTM::VOpSHL:
    case VTM::VOpSRL:
    case VTM::VOpSRA:           emitBinaryFUOp(MI, CurSlot, Cnds); break;
    case VTM::VOpReadFU:        emitOpReadFU(MI, CurSlot, Cnds);break;
    case VTM::VOpInternalCall:  emitOpInternalCall(MI, CurSlot, Cnds);break;
    case VTM::VOpRetVal:        emitOpRetVal(MI, CurSlot, Cnds);  break;
    case VTM::VOpRet_nt:        emitOpRet(MI, CurSlot, Cnds);     break;
    case VTM::VOpCmdSeq:
    case VTM::VOpMemTrans:      emitOpMemTrans(MI, CurSlot, Cnds);break;
    case VTM::VOpBRam:          emitOpBRam(MI, CurSlot, Cnds);    break;
    case VTM::IMPLICIT_DEF:     emitImplicitDef(MI);          break;
    case VTM::VOpSel:           emitOpSel(MI, CurSlot, Cnds); break;
    case VTM::VOpCase:          emitOpCase(MI, CurSlot, Cnds); break;
    case VTM::VOpReadReturn:    emitOpReadReturn(MI, CurSlot, Cnds);break;
    case VTM::VOpUnreachable:   emitOpUnreachable(MI, CurSlot, Cnds);break;
    default:  assert(0 && "Unexpected opcode!");              break;
    }
  }

  return I;
}

void VerilogASTBuilder::emitFirstCtrlBundle(MachineBasicBlock *DstBB,
                                            VASTSlot *Slot,
                                            SmallVectorImpl<VASTUse> &Cnds) {
  // TODO: Emit PHINodes if necessary.
  MachineInstr *FirstBundle = DstBB->instr_begin();
  assert(FInfo->getStartSlotFor(DstBB) == getBundleSlot(FirstBundle)
         && FirstBundle->getOpcode() == VTM::CtrlStart && "Broken Slot!");

  typedef MachineBasicBlock::instr_iterator instr_it;
  instr_it I = FirstBundle;
  while ((++I)->isInsideBundle()) {
    MachineInstr *MI = I;

    switch (I->getOpcode()) {
    case VTM::VOpDstMux:
    case VTM::VOpMove_ri:
    case VTM::VOpMove_rw:
    case VTM::VOpMove_rr:
    case VTM::VOpMvPhi:
    case VTM::COPY:             emitOpCopy(MI, Slot, Cnds);   break;
    case VTM::VOpDefPhi:                                      break;
    case VTM::CtrlEnd:          /*Not need to handle*/        break;
    case VTM::VOpCase:          emitOpCase(MI, Slot, Cnds);   break;
    case VTM::VOpSel:           emitOpSel(MI, Slot, Cnds);    break;
    case VTM::VOpRetVal:        emitOpRetVal(MI, Slot, Cnds); break;
    case VTM::IMPLICIT_DEF:     emitImplicitDef(MI);          break;
    default:  llvm_unreachable("Unexpected opcode!");              break;
    }
  }
}

void VerilogASTBuilder::emitOpUnreachable(MachineInstr *MI, VASTSlot *Slot,
                                          SmallVectorImpl<VASTUse> &Cnds) {
  vlang_raw_ostream &OS = VM->getControlBlockBuffer();
  std::string PredStr;
  raw_string_ostream SS(PredStr);
  VASTRegister::printCondition(SS, Slot, Cnds);
  SS.flush();
  OS.if_begin(PredStr);
  OS << "$display(\"BAD BAD BAD BAD! Run to unreachable\");\n";
  OS << "$finish();\n";
  OS.exit_block();

  Slot->addNextSlot(VM->getOrCreateSlot(0, 0));
}

void VerilogASTBuilder::emitOpAdd(MachineInstr *MI, VASTSlot *Slot,
                                  SmallVectorImpl<VASTUse> &Cnds) {
  VASTWire *Result = getAsLValue<VASTWire>(MI->getOperand(0));
  assert(Result && "FU result port replaced?");
  VASTRegister *R = cast<VASTRegister>(Result->getOperand(0));
  VM->addAssignment(R, getAsOperand(MI->getOperand(1)), Slot, Cnds);
  R = cast<VASTRegister>(Result->getOperand(1));
  VM->addAssignment(R, getAsOperand(MI->getOperand(2)), Slot, Cnds);
  R = cast<VASTRegister>(Result->getOperand(2));
  VM->addAssignment(R, getAsOperand(MI->getOperand(3)), Slot, Cnds);
}

void VerilogASTBuilder::emitChainedOpAdd(MachineInstr *MI) {
  VASTWire *V = getAsLValue<VASTWire>(MI->getOperand(0));
  if (!V || V->hasExpr()) return;
  VM->buildExpr(VASTWire::dpAdd,
                getAsOperand(MI->getOperand(1)),
                getAsOperand(MI->getOperand(2)),
                getAsOperand(MI->getOperand(3)),
                V->getBitWidth(), V);
}

void VerilogASTBuilder::emitChainedOpICmp(MachineInstr *MI) {
  unsigned CndCode = MI->getOperand(3).getImm();
  emitBinaryOp(MI, CndCode == VFUs::CmpSigned ? VASTWire::dpSCmp
                                              : VASTWire::dpUCmp);
}

void VerilogASTBuilder::emitBinaryFUOp(MachineInstr *MI, VASTSlot *Slot,
                                SmallVectorImpl<VASTUse> &Cnds) {
  VASTWire *Result = getAsLValue<VASTWire>(MI->getOperand(0));
  assert(Result && "FU result port replaced?");
  VASTRegister *R = cast<VASTRegister>(Result->getOperand(0));
  VM->addAssignment(R, getAsOperand(MI->getOperand(1)), Slot, Cnds);
  R = cast<VASTRegister>(Result->getOperand(1));
  VM->addAssignment(R, getAsOperand(MI->getOperand(2)), Slot, Cnds);
}

void VerilogASTBuilder::emitImplicitDef(MachineInstr *MI) {
  //VASTDatapath *data = VM->createDatapath();
  //VASTDatapath::builder_stream &OS = data->getCodeBuffer();
  //OS << "// IMPLICIT_DEF " << ImpDef.getOperand(0) << "\n";
}

void VerilogASTBuilder::emitOpSel(MachineInstr *MI, VASTSlot *Slot,
                           SmallVectorImpl<VASTUse> &Cnds) {
  VASTRegister *R = cast<VASTRegister>(getAsOperand(MI->getOperand(0)));
  // Assign the value for condition true.
  VASTUse Cnd = createCnd(MI->getOperand(1));
  Cnds.push_back(Cnd);
  VM->addAssignment(R, getAsOperand(MI->getOperand(2)), Slot, Cnds);
  // Assign the value for condition false.
  Cnds.back() = VM->buildNotExpr(Cnd);
  VM->addAssignment(R, getAsOperand(MI->getOperand(3)), Slot, Cnds);
  Cnds.pop_back();
}

void VerilogASTBuilder::emitOpCase(MachineInstr *MI, VASTSlot *Slot,
                            SmallVectorImpl<VASTUse> &Cnds) {
  // Check if we got any case hitted
  vlang_raw_ostream &OS = VM->getControlBlockBuffer();
  OS.if_();
  VASTRegister::printCondition(OS, Slot, Cnds);
  OS._then();
  OS.switch_begin("1'b1");

  VASTRegister *R = cast<VASTRegister>(getAsOperand(MI->getOperand(0)));
  for (unsigned i = 3, e = MI->getNumOperands(); i < e; i +=2) {
    Cnds.push_back(createCnd(MI->getOperand(i)));
    VM->addAssignment(R, getAsOperand(MI->getOperand(i + 1)), Slot, Cnds);
    // Do nothing if any case hit.
    Cnds.back().print(OS);
    Cnds.back().PinUser();
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

void VerilogASTBuilder::emitOpCopy(MachineInstr *MI, VASTSlot *Slot,
                                   SmallVectorImpl<VASTUse> &Cnds) {
  MachineOperand &Dst = MI->getOperand(0), &Src = MI->getOperand(1);
  // Ignore the identical copy.
  if (Src.isReg() && Dst.getReg() == Src.getReg()) return;

  VASTRegister *R = cast<VASTRegister>(getAsOperand(Dst));
  VM->addAssignment(R, getAsOperand(Src), Slot, Cnds);
}

void VerilogASTBuilder::emitOpReadFU(MachineInstr *MI, VASTSlot *CurSlot,
                              SmallVectorImpl<VASTUse> &Cnds) {
  FuncUnitId Id = VInstrInfo::getPreboundFUId(MI);
  VASTValue *ReadyPort = 0;

  switch (Id.getFUType()) {
  case VFUs::MemoryBus:
    ReadyPort = VM->getSymbol(VFUMemBus::getReadyName(Id.getFUNum()));
    break;
  case VFUs::CalleeFN: {
    // The register representing the function unit is store in the src operand
    // of VOpReadFU.
    unsigned FNNum = MI->getOperand(1).getReg();
    ReadyPort = VM->getSymbol(getSubModulePortName(FNNum, "fin"));
    break;
  }
  default:
    break;
  }

  if (ReadyPort)
    CurSlot->addReady(ReadyPort, createCnd(*VInstrInfo::getPredOperand(MI)));

  // The dst operand of ReadFU change to immediate if it is dead.
  if (MI->getOperand(0).isReg() && MI->getOperand(0).getReg())
    emitOpCopy(MI, CurSlot, Cnds);
}

void VerilogASTBuilder::emitOpReadReturn(MachineInstr *MI, VASTSlot *Slot,
                                         SmallVectorImpl<VASTUse> &Cnds) {
  VASTRegister *R = cast<VASTRegister>(getAsOperand(MI->getOperand(0)));
  VM->addAssignment(R, getAsOperand(MI->getOperand(1)), Slot, Cnds);
}

void VerilogASTBuilder::emitOpInternalCall(MachineInstr *MI, VASTSlot *Slot,
                                    SmallVectorImpl<VASTUse> &Cnds) {
  // Assign input port to some register.
  const char *CalleeName = MI->getOperand(1).getSymbolName();
  unsigned FNNum = FInfo->getCalleeFNNum(CalleeName);

  VASTUse Pred = createCnd(*VInstrInfo::getPredOperand(MI));
  std::string StartPortName = getSubModulePortName(FNNum, "start");
  VASTValue *StartSignal = VM->getSymbol(StartPortName);
  Slot->addEnable(StartSignal, Pred);
  VASTSlot *NextSlot = VM->getOrCreateNextSlot(Slot);
  getPredValAtNextSlot(MI, Pred);
  NextSlot->addDisable(StartSignal, Pred);

  const Function *FN = M->getFunction(CalleeName);
  if (FN && !FN->isDeclaration()) {
    Function::const_arg_iterator ArgIt = FN->arg_begin();
    for (unsigned i = 0, e = FN->arg_size(); i != e; ++i) {
      VASTRegister *R =
        VM->getSymbol<VASTRegister>(getSubModulePortName(FNNum, ArgIt->getName()));
      VM->addAssignment(R, getAsOperand(MI->getOperand(4 + i)), Slot, Cnds);
      ++ArgIt;
    }
    return;
  }

  // Is the function have latency information not captured by schedule?
  if (VASTWire *RetPort = VM->getBBLatInfo(FNNum)) {
    for (unsigned i = 0, e = RetPort->num_operands(); i < e; ++i) {
      VASTRegister *R = cast<VASTRegister>(RetPort->getOperand(i));
      VM->addAssignment(R, getAsOperand(MI->getOperand(4 + i)), Slot, Cnds);
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
    for (unsigned i = 4, e = MI->getNumOperands(); i != e; ++i) {
      MachineOperand &Operand = MI->getOperand(i);
      if (Operand.isReg() && (Operand.getReg() == 0 || Operand.isImplicit()))
        continue;

      if (i != 4) OS << ",\",\", ";

      if (Operand.isGlobal()) // It is the format string?
        if (const GlobalVariable *Str = cast<GlobalVariable>(Operand.getGlobal())){
          if (Str->hasInitializer()) {
            const Constant *Initialer = Str->getInitializer();
            if (const ConstantArray *Fmt = dyn_cast<ConstantArray>(Initialer)){
              StringRef FmtStr;
              if (getConstantStringInfo(Fmt, FmtStr)) {
                std::string FmtStr;
                raw_string_ostream SS(FmtStr);
                SS << '"';
                PrintEscapedString(FmtStr, SS);
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
    for (unsigned i = 4, e = MI->getNumOperands(); i != e; ++i) {
      printOperand(MI->getOperand(i), SS);
      SS.flush();
      InPorts.push_back(SS.str());
      s.clear();
    }

    std::string Name = CalleeName;
    OS << VFUs::startModule(Name, FInfo->getCalleeFNNum(CalleeName), InPorts);
  }
  OS.exit_block();
}

void VerilogASTBuilder::emitOpRet(MachineInstr *MI, VASTSlot *CurSlot,
                           SmallVectorImpl<VASTUse> &Cnds) {
  // Go back to the idle slot.
  VASTUse Pred = createCnd(*VInstrInfo::getPredOperand(MI));
  CurSlot->addNextSlot(VM->getOrCreateSlot(0, 0), Pred);
  CurSlot->addEnable(VM->getPort(VASTModule::Finish), Pred);
}

void VerilogASTBuilder::emitOpRetVal(MachineInstr *MI, VASTSlot *Slot,
                              SmallVectorImpl<VASTUse> &Cnds) {
  VASTRegister &RetReg = cast<VASTRegister>(*VM->getRetPort());
  unsigned retChannel = MI->getOperand(1).getImm();
  assert(retChannel == 0 && "Only support Channel 0!");
  VM->addAssignment(&RetReg, getAsOperand(MI->getOperand(0)), Slot, Cnds);
}

void VerilogASTBuilder::emitOpMemTrans(MachineInstr *MI, VASTSlot *Slot,
                                       SmallVectorImpl<VASTUse> &Cnds) {
  unsigned FUNum = VInstrInfo::getPreboundFUId(MI).getFUNum();

  // Emit Address.
  std::string RegName = VFUMemBus::getAddrBusName(FUNum) + "_r";
  VASTRegister *R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getAsOperand(MI->getOperand(1)), Slot, Cnds);
  // Assign store data.
  RegName = VFUMemBus::getOutDataBusName(FUNum) + "_r";
  R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getAsOperand(MI->getOperand(2)), Slot, Cnds);
  // And write enable.
  RegName = VFUMemBus::getCmdName(FUNum) + "_r";
  R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getAsOperand(MI->getOperand(3)), Slot, Cnds);
  // The byte enable.
  RegName = VFUMemBus::getByteEnableName(FUNum) + "_r";
  R = VM->getSymbol<VASTRegister>(RegName);
  VM->addAssignment(R, getAsOperand(MI->getOperand(4)), Slot, Cnds);

  // Remember we enabled the memory bus at this slot.
  std::string EnableName = VFUMemBus::getEnableName(FUNum) + "_r";
  VASTValue *MemEn = VM->getSymbol(EnableName);
  VASTUse Pred = createCnd(*VInstrInfo::getPredOperand(MI));
  Slot->addEnable(MemEn, Pred);

  // Disable the memory at next slot.
  // TODO: Assert the control flow is linear.
  VASTSlot *NextSlot = VM->getOrCreateNextSlot(Slot);
  getPredValAtNextSlot(MI, Pred);
  NextSlot->addDisable(MemEn, Pred);
}

void VerilogASTBuilder::emitOpBRam(MachineInstr *MI, VASTSlot *Slot,
                                   SmallVectorImpl<VASTUse> &Cnds) {
  unsigned FUNum = MI->getOperand(0).getReg();

  // Emit the control logic.
  vlang_raw_ostream &OS = VM->getControlBlockBuffer();
  std::string PredStr;
  raw_string_ostream PredSS(PredStr);
  VASTRegister::printCondition(PredSS, Slot, Cnds);
  PredSS.flush();

  OS.if_begin(PredStr);
  // Emit Address.
  OS << VFUBRam::getAddrBusName(FUNum) << " <= (";
  printOperand(MI->getOperand(1), OS);
  unsigned SizeInBits = FInfo->getBRamInfo(FUNum).ElemSizeInBytes;
  OS << " >> " << Log2_32_Ceil(SizeInBits) << ");\n";
  // Assign store data.
  OS << VFUBRam::getOutDataBusName(FUNum) << " <= ";
  printOperand(MI->getOperand(2), OS);
  OS << ";\n";
  // And write enable.
  OS << VFUBRam::getWriteEnableName(FUNum) << " <= ";
  printOperand(MI->getOperand(3), OS);
  OS << ";\n";
  OS.exit_block();
  // The byte enable.
  // OS << VFUMemBus::getByteEnableName(FUNum) << " <= ";
  // OpBRam.getOperand(4).print(OS);
  // OS << ";\n";

  // Remember we enabled the memory bus at this slot.
  std::string EnableName = VFUBRam::getEnableName(FUNum);
  VASTValue *MemEn = VM->getSymbol(EnableName);

  VASTUse Pred = createCnd(*VInstrInfo::getPredOperand(MI));
  Slot->addEnable(MemEn, Pred);

  // Disable the memory at next slot.
  // TODO: Assert the control flow is linear.
  VASTSlot *NextSlot = VM->getOrCreateNextSlot(Slot);
  getPredValAtNextSlot(MI, Pred);
  NextSlot->addDisable(MemEn, Pred);
}

MachineBasicBlock::instr_iterator
VerilogASTBuilder::emitDatapath(MachineInstr *Bundle) {
  typedef MachineBasicBlock::instr_iterator instr_it;
  assert(Bundle->getOpcode() == VTM::Datapath
         && "Expect data-path bundle start!");

  instr_it I = Bundle;
   while ((++I)->isInsideBundle()) {
    MachineInstr *MI = I;
    switch (MI->getOpcode()) {
    case VTM::VOpBitSlice:  emitOpBitSlice(MI);                       break;
    case VTM::VOpBitCat:    emitBinaryOp(MI, VASTWire::dpBitCat);     break;
    case VTM::VOpBitRepeat: emitBinaryOp(MI, VASTWire::dpBitRepeat);  break;

    case VTM::ImpUse:       /*Not need to handle*/  break;

    case VTM::VOpAdd_c:     emitChainedOpAdd(MI); break;
    case VTM::VOpICmp_c:    emitChainedOpICmp(MI); break;

    case VTM::VOpSHL_c:     emitBinaryOp(MI, VASTWire::dpShl);  break;
    case VTM::VOpSRA_c:     emitBinaryOp(MI, VASTWire::dpSRA);  break;
    case VTM::VOpSRL_c:     emitBinaryOp(MI, VASTWire::dpSRL);  break;
    case VTM::VOpMultLoHi_c:
    case VTM::VOpMult_c:    emitBinaryOp(MI, VASTWire::dpMul);  break;

    case VTM::VOpLUT:       emitOpLut(MI);                      break;
    case VTM::VOpXor:       emitBinaryOp(MI, VASTWire::dpXor);  break;
    case VTM::VOpAnd:       emitBinaryOp(MI, VASTWire::dpAnd);  break;
    case VTM::VOpOr:        emitBinaryOp(MI, VASTWire::dpOr);   break;

    case VTM::VOpNot:       emitUnaryOp(MI, VASTWire::dpNot);   break;
    case VTM::VOpROr:       emitUnaryOp(MI, VASTWire::dpROr);   break;
    case VTM::VOpRAnd:      emitUnaryOp(MI, VASTWire::dpRAnd);  break;
    case VTM::VOpRXor:      emitUnaryOp(MI, VASTWire::dpRXor);  break;

    default:  assert(0 && "Unexpected opcode!");    break;
    }
  }

  return I;
}

void VerilogASTBuilder::emitUnaryOp(MachineInstr *MI, VASTWire::Opcode Opc) {
  VASTWire *V = getAsLValue<VASTWire>(MI->getOperand(0));
  if (!V || V->hasExpr()) return;
  VM->buildExpr(Opc, getAsOperand(MI->getOperand(1)), V->getBitWidth(), V);
}

void VerilogASTBuilder::emitBinaryOp(MachineInstr *MI, VASTWire::Opcode Opc) {
  VASTWire *V = getAsLValue<VASTWire>(MI->getOperand(0));
  if (!V || V->hasExpr()) return;
  VM->buildExpr(Opc,
                getAsOperand(MI->getOperand(1)),
                getAsOperand(MI->getOperand(2)),
                V->getBitWidth(), V);
}

void VerilogASTBuilder::emitOpLut(MachineInstr *MI) {
  VASTWire *V = getAsLValue<VASTWire>(MI->getOperand(0));
  if (!V || V->hasExpr()) return;

  unsigned SizeInBits = V->getBitWidth();
  std::string NamePrefix = V->getName();
  unsigned NameIdx = 0;

  SmallVector<VASTUse, 8> Operands;
  for (unsigned i = 4, e = MI->getNumOperands(); i < e; ++i)
    Operands.push_back(getAsOperand(MI->getOperand(i)));
  unsigned NumInputs = Operands.size();

  // Interpret the sum of product table.
  const char *p = MI->getOperand(1).getSymbolName();
  SmallVector<VASTUse, 8> ProductOps, SumOps;
  bool isComplement = false;
  
  while (*p) {
    // Interpret the product.
    ProductOps.clear();
    for (unsigned i = 0; i < NumInputs; ++i) {
      char c = *p++;
      switch (c) {
      default: llvm_unreachable("Unexpected SOP char!");
      case '-': /*Dont care*/ break;
      case '1': ProductOps.push_back(Operands[i]); break;
      case '0': {
        std::string InvWireName = NamePrefix + utostr_32(++NameIdx) + "inv";
        VASTUse U = VM->buildExpr(VASTWire::dpNot, Operands[i], SizeInBits,
                                  VM->addWire(InvWireName, SizeInBits));
        ProductOps.push_back(U);
        break;
      }
      }
    }

    // Inputs and outputs are seperated by blank space.
    assert(*p == ' ' && "Expect the blank space!");
    ++p;

    // Create the product.
    std::string ProductWireName = NamePrefix + utostr_32(++NameIdx) + "p";
    VASTUse P = VM->buildExpr(VASTWire::dpAnd, ProductOps, SizeInBits,
                              VM->addWire(ProductWireName, SizeInBits));
    // Add the product to the operand list of the sum.
    SumOps.push_back(P);

    // Is the output inverted?
    char c = *p++;
    assert((c == '0' || c == '1') && "Unexpected SOP char!");
    isComplement = (c == '0');

    // Products are separated by new line.
    assert(*p == '\n' && "Expect the new line!");
    ++p;
  }

  // Or the products together to build the SOP (Sum of Product).
  std::string SumWireName = NamePrefix + utostr_32(++NameIdx) + "s";
  VASTUse SOP = VM->buildExpr(VASTWire::dpOr, SumOps, SizeInBits,
                              VM->addWire(SumWireName, SizeInBits));

  if (isComplement) {
    std::string InvWireName = NamePrefix + utostr_32(++NameIdx) + "inv";
    SOP = VM->buildExpr(VASTWire::dpNot, SOP, SizeInBits,
                        VM->addWire(InvWireName, SizeInBits));
  }

  // Build the sum;
  VM->buildExpr(VASTWire::dpAssign, SOP, SizeInBits, V);
  return;
}

void VerilogASTBuilder::emitOpBitSlice(MachineInstr *MI) {
  VASTWire *V = getAsLValue<VASTWire>(MI->getOperand(0));
  if (!V || V->hasExpr()) return;

  // Get the range of the bit slice, Note that the
  // bit at upper bound is excluded in VOpBitSlice
  unsigned UB = MI->getOperand(2).getImm(),
           LB = MI->getOperand(3).getImm();

  VASTUse RHS = getAsOperand(MI->getOperand(1));
  assert(RHS.UB != 0 && "Cannot get bitslice without bitwidth information!");
  // Already replaced.
  if (RHS.getOrNull() == V) return;

  // Adjust ub and lb.
  LB += RHS.LB;
  UB += RHS.LB;

  assert(UB <= RHS.UB && "Bitslice out of range!");
  VM->buildExpr(VASTWire::dpAssign, VASTUse(RHS.get(), UB, LB),
                V->getBitWidth(), V);
}

VASTUse VerilogASTBuilder::createCnd(ucOperand &Op) {
  // Is there an always true predicate?
  if (VInstrInfo::isAlwaysTruePred(Op)) return VASTUse(true, 1);

  // Otherwise it must be some signal.
  VASTUse C = VM->lookupSignal(Op.getReg());

  if (Op.isPredicateInverted()) C = VM->buildNotExpr(C);

  return C;
}

void VerilogASTBuilder::getPredValAtNextSlot(MachineInstr *MI, VASTUse &Pred) {
  // Get the predicate value at next slot, if the predicate operand is copied to
  // a register, use that register.
  ucOperand &PredCnd = cast<ucOperand>(*VInstrInfo::getPredOperand(MI));

  if (VInstrInfo::isAlwaysTruePred(PredCnd)) return;

  typedef MachineRegisterInfo::use_iterator it;
  for (it I = MRI->use_begin(PredCnd.getReg()); I != MRI->use_end();++I) {
    MachineInstr *UseMI = &*I;
    // We are find the copy operation that copy the current predicate value
    // to a register.
    if (UseMI->getOpcode() != VTM::VOpReadFU)
      continue;

    // Skip the MI predicate by current predicate operand.
    if (UseMI == MI) continue;

    // Be careful of 1 slot pipelined loops.
    if (getInstrSlot(MI) != getInstrSlot(UseMI))
      continue;

    Pred = createCnd(UseMI->getOperand(0));
    // Invert flag is not copied.
    if (PredCnd.isPredicateInverted())
      Pred = VM->buildNotExpr(Pred);

    return;
  }
}

VASTUse VerilogASTBuilder::getAsOperand(ucOperand &Op) {
  switch (Op.getType()) {
  case MachineOperand::MO_Register: {
    if (unsigned Reg = Op.getReg())
      return VM->lookupSignal(Reg);

    return VASTUse();
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

void VerilogASTBuilder::printOperand(ucOperand &Op, raw_ostream &OS) {
  if(Op.isReg()){
    VASTUse U = getAsOperand(Op);
    U.print(OS);
    U.PinUser();
    return;
  }

  Op.print(OS);
}
