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
#include "llvm/DerivedTypes.h"
#include "llvm/Instructions.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/InstIterator.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/Debug.h"

#include "HWAtomInfo.h"
#include "RTLWriter.h"


static cl::opt<std::string>
DesignName("vbe-design-name", cl::desc("Design Name."), cl::init(""));

using namespace esyn;

char RTLWriter::ID = 0;

bool RTLWriter::runOnFunction(Function &F) {
  TD = &getAnalysis<TargetData>();
  vlang = &getAnalysis<VLang>();
  HI = &getAnalysis<HWAtomInfo>();
  RC = &getAnalysis<ResourceConfig>();

  if (DesignName.empty())
    DesignName = F.getNameStr();

  VM = new VModule(DesignName);

  // Emit control register and idle state
  unsigned totalFSMStates = F.size() + 1;
  TotalFSMStatesBit = Log2_32_Ceil(totalFSMStates);

  vlang->declSignal(VM->getSignalDeclBuffer(), "NextFSMState", TotalFSMStatesBit, 0);
  vlang->resetRegister(VM->getResetBlockBuffer(), "NextFSMState", TotalFSMStatesBit, 0);
  
  // Idle state
  vlang->param(VM->getStateDeclBuffer(), "state_idle", TotalFSMStatesBit, 0);
  vlang->matchCase(VM->getControlBlockBuffer(6), "state_idle");
  // Idle state is always ready.
  VM->getControlBlockBuffer(8) << "fin <= 1'h0;\n";
  vlang->ifBegin(VM->getControlBlockBuffer(8), "start");
  // The module is busy now
  emitNextFSMState(VM->getControlBlockBuffer(10), F.getEntryBlock());
  emitNextMicroState(VM->getControlBlockBuffer(10), F.getEntryBlock(), "1'b1");
  
  // Remember the parameters
  emitFunctionSignature(F);

  //
  vlang->ifElse(VM->getControlBlockBuffer(8));
  VM->getControlBlockBuffer(10) << "NextFSMState <= state_idle;\n";
  vlang->end(VM->getControlBlockBuffer(8));
  vlang->end(VM->getControlBlockBuffer(6));

  // Emit basicblocks
  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
    BasicBlock &BB = *I;
    emitBasicBlock(BB);
  }

  // Emit resouces
  emitResources();

  // Emit all Regs
  emitAllRegisters();

  // emit misc
  emitCommonPort();

  // Write buffers to output
  VM->printModuleDecl(Out);
  vlang->endModuleDecl(Out);
  Out << "\n\n";
  // States
  vlang->comment(Out.indent(2)) << "States\n";
  Out << VM->getStateDeclStr();
  Out << "\n\n";
  // Reg and wire
  vlang->comment(Out.indent(2)) << "Reg and wire decl\n";
  Out << VM->getSignalDeclStr();
  Out << "\n\n";

  // Datapath
  vlang->comment(Out.indent(2)) << "Datapath\n";
  Out << VM->getDataPathStr();

  Out << "\n\n";
  vlang->comment(Out.indent(2)) << "Always Block\n";
  vlang->alwaysBegin(Out, 2);
  Out << VM->getResetBlockStr();
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
  delete VM;
  // Clear resource map
  UsedRegs.clear();
  ResourceMap.clear();
}

void RTLWriter::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<VLang>();
  AU.addRequired<TargetData>();
  AU.addRequired<ResourceConfig>();
  AU.setPreservesAll();
}

void RTLWriter::print(raw_ostream &O, const Module *M) const {

}

void RTLWriter::emitFunctionSignature(Function &F) {
  const AttrListPtr &PAL = F.getAttributes();
  unsigned Idx = 1;
  for (Function::arg_iterator I = F.arg_begin(), E = F.arg_end();
      I != E; ++I) {
    Argument *Arg = I;
    HWRegister *ArgReg = HI->lookupRegForValue(Arg);
    unsigned BitWidth = ArgReg->getBitWidth();

    std::string Name = vlang->GetValueName(Arg), 
                RegName = getAsOperand(ArgReg);
    // Add port declaration.
    VM->addInputPort(Name, BitWidth);

    UsedRegs.insert(std::make_pair(ArgReg->getRegNum(), ArgReg));
    // Assign the register
    VM->getControlBlockBuffer(10) << RegName << " <= " << Name << ";\n";
    ++Idx;
  }

  const Type *RetTy = F.getReturnType();
  if (RetTy->isVoidTy()) {
    // Do something?
    //vlang->indent(ModDecl) << "/*return void*/";
  } else {
    assert(RetTy->isIntegerTy() && "Only support return integer now!");
    VM->addOutputPort("return_value", cast<IntegerType>(RetTy)->getBitWidth());
    // reset the register
    vlang->resetRegister(VM->getResetBlockBuffer(), "return_value",
                         cast<IntegerType>(RetTy)->getBitWidth());
  }
}

void RTLWriter::emitBasicBlock(BasicBlock &BB) {
  FSMState *State = HI->getStateFor(BB);
  std::string StateName = vlang->GetValueName(&BB);

  unsigned StartSlot = State->getSlot(), EndSlot = State->getEndSlot();

  vlang->comment(VM->getStateDeclBuffer()) << "State for " << StateName << '\n';
  vlang->param(VM->getStateDeclBuffer(), StateName, TotalFSMStatesBit,
               ++CurFSMStateNum);

  // State information.
  vlang->comment(VM->getControlBlockBuffer(6)) << StateName 
    << " Total Slot: " << State->getTotalSlot()
    << " II: " << State->getII() <<  '\n';
  // Mirco state enable.
  createMircoStateEnable(State);

  FSMState::ScheduleMapType Atoms;
  typedef FSMState::ScheduleMapType::iterator cycle_iterator;

  State->getScheduleMap(Atoms);

  // Case begin
  vlang->matchCase(VM->getControlBlockBuffer(6), StateName);

  for (unsigned i = StartSlot, e = EndSlot + 1; i != e; ++i) {
    vlang->ifBegin(VM->getControlBlockBuffer(8), getMircoStateEnable(State, i, true));
    // Emit all atoms at cycle i

    vlang->comment(VM->getDataPathBuffer(2)) << "at cycle: " << i << '\n';
    for (cycle_iterator CI = Atoms.lower_bound(i), CE = Atoms.upper_bound(i);
        CI != CE; ++CI)
      emitAtom(CI->second);
    vlang->end(VM->getControlBlockBuffer(8));
  }// end for

  // Emit Self Loop logic.
  if (State->haveSelfLoop()) {
    vlang->comment(VM->getControlBlockBuffer(8)) << "For self loop:\n";
    std::string SelfLoopEnable = computeSelfLoopEnable(State);
    emitNextMicroState(VM->getControlBlockBuffer(8), BB, SelfLoopEnable);
  } else
    emitNextMicroState(VM->getControlBlockBuffer(8), BB, "1'b0");
  // Case end
  vlang->end(VM->getControlBlockBuffer(6));
}

void RTLWriter::emitCommonPort() {
  VM->addInputPort("clk", 1);
  VM->addInputPort("rstN", 1);
  VM->addInputPort("start", 1);
  VM->addOutputPort("fin", 1);
  // Reset fin
  vlang->resetRegister(VM->getResetBlockBuffer(), "fin", 1);
}

RTLWriter::~RTLWriter() {}


//===----------------------------------------------------------------------===//
// Emit hardware atoms
std::string RTLWriter::getAsOperand(Value *V, const std::string &postfix) {
  if (Argument *Arg = dyn_cast<Argument>(V))
    return vlang->GetValueName(V) + "_pr";

  return vlang->GetValueName(V) + postfix;
}

std::string RTLWriter::getAsOperand(HWAtom *A) {
  Value *V = &A->getValue();

  switch (A->getHWAtomType()) {
    case atomOpFU: {
      HWAOpFU *OF = cast<HWAOpFU>(A);
      if (OF->isTrivial())
        return getAsOperand(V, "_w");
      else
        return getRegPrefix(OF->getResType()) + utostr(OF->getUnitID());
    }
    case atomWrReg:
      return "/*" + vlang->GetValueName(&A->getValue()) + "*/"
        + getAsOperand(cast<HWAWrReg>(A)->getReg());
    case atomLIReg:
      return "/*" + vlang->GetValueName(&A->getValue()) + "*/"
        + getAsOperand(HI->lookupRegForValue(&A->getValue()));
    default:
      llvm_unreachable("Do not use other atom as operand!");
      return "<Unknown Atom>";
  }
}

std::string RTLWriter::getAsOperand(const HWRegister *R) {
  return getRegPrefix(R->getResType()) + utostr(R->getRegNum());
}

std::string RTLWriter::getAsOperand(HWEdge &E) {
  switch (E.getEdgeType()) {
  case edgeConst:
    return vlang->printConstant(cast<HWConst>(E).getConstant());
  case edgeValDep:
    return getAsOperand(cast<HWValDep>(E).getSrc());
  default:
    llvm_unreachable("Do not use other edge as operand!");
    return "<Unknown edge>";
  }
}

void RTLWriter::emitAtom(HWAtom *A) {
  switch (A->getHWAtomType()) {
      case atomOpFU:
        // Emit the origin IR as comment.
        vlang->comment(VM->getControlBlockBuffer(10)) << "Emit: "
          << A->getValue() << '\n';
        emitOpFU(cast<HWAOpFU>(A));
        break;
      case atomWrReg:
        emitWrReg(cast<HWAWrReg>(A));
        break;
      case atomLIReg:
        vlang->comment(VM->getControlBlockBuffer(10)) << "Import:"
          << A->getValue() << '\n';
        emitLIReg(cast<HWALIReg>(A));
        break;
      case atomDelay:
        vlang->comment(VM->getControlBlockBuffer(10));
        A->print(VM->getControlBlockBuffer());
        VM->getControlBlockBuffer() << '\n';
        break;
      // Do nothing.
      case atomVRoot:
        break;
      default:
        llvm_unreachable("Unknow Atom!");
        break;
  }
}

void RTLWriter::emitWrReg(HWAWrReg *DR) {
  const HWRegister *R = DR->getReg();
  // Function unit register will emit with function unit.
  if (R->isFuReg())
    return;

  UsedRegs.insert(std::make_pair(R->getRegNum(), R));
  HWEdge &E = DR->getDep(0);
  if (!isa<HWConst>(E))
    vlang->comment(VM->getControlBlockBuffer(10)) << "Read:"
    << DR->getValue() << '\n';

  std::string Name = getAsOperand(DR);
  VM->getControlBlockBuffer(10) << Name << " <= " << getAsOperand(E) << ";\n";
}

void RTLWriter::emitLIReg(HWALIReg *LIR) {
  if (!LIR->isPHINode())
    return;

  const HWRegister *R = HI->lookupRegForValue(&LIR->getValue());
  UsedRegs.insert(std::make_pair(R->getRegNum(), R));
}

void RTLWriter::emitAllRegisters() {
  for (std::map<unsigned, const HWRegister*>::iterator I = UsedRegs.begin(),
      E = UsedRegs.end(); I != E; ++I) {
    const HWRegister *R = I->second;
    unsigned BitWidth = R->getBitWidth();
    std::string Name = getAsOperand(R);

    vlang->declSignal(VM->getSignalDeclBuffer(), Name, BitWidth, 0);
    vlang->resetRegister(VM->getResetBlockBuffer(), Name, BitWidth, 0);
  } 
}

void RTLWriter::emitOpFU(HWAOpFU *OF) {
  if (OF->isBinded()) { 
    // Remember this atom
    assert(!OF->isTrivial() &&"Unexcept resource type!");
    ResourceMap[OF->getFUnit()].push_back(OF);
  } else {
    Instruction &Inst = cast<Instruction>(OF->getValue());
    assert(!isa<PHINode>(Inst) && "PHINode is not PostBind atom!");
    std::string Name = getAsOperand(OF);
    // Do not decl signal for void type
    // And do not emit data path for phi node.
    if (!Inst.getType()->isVoidTy()) {
      // Declare the signal
      vlang->declSignal(VM->getSignalDeclBuffer(), Name, OF->getBitWidth(), 0, false);
      vlang->comment(VM->getDataPathBuffer(2)) << OF->getValue() << '\n';
      // Emit data path
      VM->getDataPathBuffer(2) << "assign " << Name << " = ";
    }
    // Emit the data path
    visit(*OF);
  }
}

std::string RTLWriter::getRegPrefix(HWResType::Types T) {
  switch (T) {
  case HWResType::MemoryBus:  return "membus_out";
  case HWResType::SHL:        return "shl_result";
  case HWResType::ASR:        return "asr_result";
  case HWResType::LSR:        return "lsr_result";
  case HWResType::AddSub:     return "addsub_result";
  case HWResType::Mult:       return "mult_result";
  case HWResType::Trivial:    return "R";
  }

  llvm_unreachable("Unknown ResType!");
  return "<Unknown>";
}

void RTLWriter::emitResourceDeclForBinOpRes(HWFUnit *FU,
                                            const std::string &OpPrefix,
                                            const std::string &Operator) {
  unsigned FUID = FU->getUnitID();
  std::string OpA = OpPrefix + "_a" + utostr(FUID);
  std::string OpB = OpPrefix + "_b" + utostr(FUID);
  std::string Res = getRegPrefix(FU->getResType()) + utostr(FUID);

  vlang->declSignal(VM->getSignalDeclBuffer(), OpA, FU->getInputBitwidth(0), 0);
  vlang->declSignal(VM->getSignalDeclBuffer(), OpB, FU->getInputBitwidth(1), 0);
  vlang->declSignal(VM->getSignalDeclBuffer(), Res, FU->getOutputBitwidth(), 0, false);

  // vlang->alwaysBegin(DataPath, 2);
  // vlang->resetRegister(DataPath.indent(6), Res, FU->getOutputBitwidth());
  // vlang->ifElse(DataPath.indent(4));
  VM->getDataPathBuffer(2) << "assign "<< Res << " = " << OpA << Operator << OpB << ";\n";
  // vlang->alwaysEnd(DataPath, 2);
}

template<>
void RTLWriter::emitResourceDecl<HWAddSub>(HWFUnit *FU) {
  unsigned FUID = FU->getUnitID();
  std::string OpA = "addsub_a" + utostr(FUID);
  std::string OpB = "addsub_b" + utostr(FUID);
  std::string Mode = "addsub_mode" + utostr(FUID);
  std::string Res = getRegPrefix(FU->getResType()) + utostr(FUID);

  vlang->declSignal(VM->getSignalDeclBuffer(), OpA, FU->getInputBitwidth(0), 0);
  vlang->declSignal(VM->getSignalDeclBuffer(), OpB, FU->getInputBitwidth(1), 0);
  vlang->declSignal(VM->getSignalDeclBuffer(), Res, FU->getOutputBitwidth(), 0, false);
  vlang->declSignal(VM->getSignalDeclBuffer(), Mode, 1, 0);

  vlang->comment(VM->getDataPathBuffer(2)) << "Add/Sub Unit: " << FUID << '\n';
  // vlang->alwaysBegin(DataPath, 2);
  // vlang->resetRegister(VM->getDataPathBuffer(6), Res, FU->getOutputBitwidth());
  // vlang->ifElse(VM->getDataPathBuffer(4));
  VM->getDataPathBuffer(2) << "assign " << Res << " = " << Mode << " ? ";
  VM->getDataPathBuffer()           << "(" << OpA << " + " << OpB << ") : ";
  VM->getDataPathBuffer()           << "(" << OpA << " - " << OpB << ");\n";
  // vlang->alwaysEnd(VM->getDataPathBuffer(), 2);
}

template<>
void RTLWriter::emitResourceDecl<HWMult>(HWFUnit *FU) {
  emitResourceDeclForBinOpRes(FU, "mult", " * ");
}

template<>
void RTLWriter::emitResourceDecl<HWSHL>(HWFUnit *FU) {
  emitResourceDeclForBinOpRes(FU, "shl", " * ");
}

template<>
void RTLWriter::emitResourceDecl<HWASR>(HWFUnit *FU) {
  emitResourceDeclForBinOpRes(FU, "asr", " * ");
}

template<>
void RTLWriter::emitResourceDecl<HWLSR>(HWFUnit *FU) {
  emitResourceDeclForBinOpRes(FU, "lsr", " * ");
}

template<>
void RTLWriter::emitResourceDecl<HWMemBus>(HWFUnit *FU) {
  unsigned DataWidth = FU->getInputBitwidth(0),
           AddrWidth = FU->getInputBitwidth(1),
           FUID = FU->getUnitID();

  // Emit the ports;
  //vlang->comment(VM->getModDeclBuffer()) << "Memory bus " << FUID << '\n';
  VM->addInputPort("membus_out" + utostr(FUID), DataWidth);
  VM->addOutputPort("membus_in" + utostr(FUID), DataWidth);
  VM->addOutputPort("membus_addr" + utostr(FUID), DataWidth);
  VM->addOutputPort("membus_we" + utostr(FUID), 1);
  VM->addOutputPort("membus_en" + utostr(FUID), 1);
}

void RTLWriter::emitResourceOpForBinOpRes(HWAOpFU *A, const std::string &OpPrefix) {
  unsigned FUID = A->getUnitID();
  std::string OpA = OpPrefix + "_a" + utostr(FUID);
  std::string OpB = OpPrefix + "_b" + utostr(FUID);

  VM->getDataPathBuffer(6) <<  OpA << " = "
    << getAsOperand(A->getValDep(0)) << ";\n";
  VM->getDataPathBuffer(6) <<  OpB << " = "
    << getAsOperand(A->getValDep(1)) << ";\n";
}

template<>
void RTLWriter::emitResourceOp<HWAddSub>(HWAOpFU *A) {
  unsigned FUID = A->getUnitID();
  std::string Mode = "addsub_mode" + utostr(FUID);

  Instruction *Inst = &(A->getInst<Instruction>());
  VM->getDataPathBuffer(6) << Mode;
  if (Inst->getOpcode() == Instruction::Sub)
    VM->getDataPathBuffer() << " = 1'b0;\n";
  else
    VM->getDataPathBuffer() << " = 1'b1;\n";

  emitResourceOpForBinOpRes(A, "addsub");
}

template<>
void RTLWriter::emitResourceOp<HWMult>(HWAOpFU *A) {
  emitResourceOpForBinOpRes(A, "mult");
}

template<>
void RTLWriter::emitResourceOp<HWSHL>(HWAOpFU *A) {
  emitResourceOpForBinOpRes(A, "shl");
}

template<>
void RTLWriter::emitResourceOp<HWASR>(HWAOpFU *A) {
  emitResourceOpForBinOpRes(A, "asr");
}

template<>
void RTLWriter::emitResourceOp<HWLSR>(HWAOpFU *A) {
  emitResourceOpForBinOpRes(A, "lsr");
}

template<>
void RTLWriter::emitResourceOp<HWMemBus>(HWAOpFU *A) {
  unsigned DataWidth = A->getInputBitwidth(0),
           AddrWidth = A->getInputBitwidth(1);

  unsigned FUID = A->getUnitID();
  Instruction *Inst = &(A->getInst<Instruction>());
  // Enable the memory
  VM->getDataPathBuffer(6) << "membus_en" << FUID << " = 1'b1;\n";
  // Send the address.
  VM->getDataPathBuffer(6) << "membus_addr" << FUID << " = ";

  // Emit the operation
  if (LoadInst *L = dyn_cast<LoadInst>(Inst)) {
    VM->getDataPathBuffer() << getAsOperand(A->getValDep(LoadInst::getPointerOperandIndex()))
      << ";\n";
    VM->getDataPathBuffer(6) << "membus_we" << FUID << " = 1'b0;\n";
    VM->getDataPathBuffer(6) << "membus_in" << FUID
      << " = " << vlang->printConstantInt(0, DataWidth, false) << ";\n";
  } else { // It must be a store
    VM->getDataPathBuffer() << getAsOperand(A->getValDep(StoreInst::getPointerOperandIndex()))
      << ";\n";
    VM->getDataPathBuffer(6) << "membus_we" << FUID << " = 1'b1;\n";
    VM->getDataPathBuffer(6) << "membus_in" << FUID
      << " = " << getAsOperand(A->getValDep(0)) << ";\n";
  }
}

void RTLWriter::emitResourceDefaultOpForBinOpRes(HWFUnit *FU, const std::string &OpPrefix) {
  unsigned FUID = FU->getUnitID();
  std::string OpA = OpPrefix + "_a" + utostr(FUID);
  std::string OpB = OpPrefix + "_b" + utostr(FUID);

  VM->getDataPathBuffer(6) << OpA << " = "
    << vlang->printConstantInt(0, FU->getInputBitwidth(0), false) << ";\n";
  VM->getDataPathBuffer(6) << OpB << " = "
    << vlang->printConstantInt(0, FU->getInputBitwidth(1), false) << ";\n";
  vlang->end(VM->getDataPathBuffer(4));
}

template<>
void RTLWriter::emitResourceDefaultOp<HWAddSub>(HWFUnit *FU) {
  std::string Mode = "addsub_mode" + utostr(FU->getUnitID());
  VM->getDataPathBuffer(6) << Mode << " = 1'b0;\n";
  emitResourceDefaultOpForBinOpRes(FU, "addsub");
}

template<>
void RTLWriter::emitResourceDefaultOp<HWMult>(HWFUnit *FU) {
  emitResourceDefaultOpForBinOpRes(FU, "mult");
}

template<>
void RTLWriter::emitResourceDefaultOp<HWSHL>(HWFUnit *FU) {
  emitResourceDefaultOpForBinOpRes(FU, "shl");
}

template<>
void RTLWriter::emitResourceDefaultOp<HWASR>(HWFUnit *FU) {
  emitResourceDefaultOpForBinOpRes(FU, "asr");
}

template<>
void RTLWriter::emitResourceDefaultOp<HWLSR>(HWFUnit *FU) {
  emitResourceDefaultOpForBinOpRes(FU, "lsr");
}

template<>
void RTLWriter::emitResourceDefaultOp<HWMemBus>(HWFUnit *FU) {
  unsigned DataWidth = FU->getInputBitwidth(0),
           AddrWidth = FU->getInputBitwidth(1),
           FUID = FU->getUnitID();

  VM->getDataPathBuffer(6) << "membus_en" << FUID << " = 1'b0;\n";
  VM->getDataPathBuffer(6) << "membus_addr" << FUID
    << " = " << vlang->printConstantInt(0, AddrWidth, false) << ";\n";
  VM->getDataPathBuffer(6) << "membus_we" << FUID << " = 1'b0;\n";
  VM->getDataPathBuffer(6) << "membus_in" << FUID
    << " = " << vlang->printConstantInt(0, DataWidth, false) << ";\n";
  vlang->end(VM->getDataPathBuffer(4));
}

template<class ResType>
void RTLWriter::emitResource(HWAPreBindVecTy &Atoms) {
  HWAOpFU *FirstAtom = Atoms[0];
  HWFUnit *FU = FirstAtom->getFUnit();
  unsigned FUID = FU->getUnitID();

  std::string SelName = ResType::getTypeName() + utostr(FUID) + "Mux";
  unsigned SelBitWidth = Atoms.size();
  vlang->declSignal(VM->getSignalDeclBuffer(), SelName, SelBitWidth, 0, false);

  emitResourceDecl<ResType>(FU);
  VM->getDataPathBuffer(2) << "always @(*)\n";

  unsigned AtomCounter = SelBitWidth;
  vlang->switchCase(VM->getDataPathBuffer(4), SelName);
  raw_string_ostream SelEval(SelName);
  SelEval << " = { ";

  // Emit all resource operation
  for (HWAPreBindVecTy::iterator I = Atoms.begin(), E = Atoms.end();
      I != E; ++I) {
    HWAOpFU *A = *I;
    Instruction *Inst = &(A->getInst<Instruction>());
    BasicBlock *BB = Inst->getParent();
    std::string SelCase = vlang->printConstantInt(1 << (--AtomCounter),
                                                  SelBitWidth, false);
    vlang->matchCase(VM->getDataPathBuffer(4), SelCase);

    vlang->comment(VM->getDataPathBuffer(6)) << *Inst << '\n';

    SelEval << getMircoStateEnable(A->getParent(), A->getSlot(), false);

    emitResourceOp<ResType>(A);
  
    // Else for other atoms
    vlang->end(VM->getDataPathBuffer(4));
    // Next atom
    if (AtomCounter > 0)
      SelEval << ", ";
  }
  VM->getDataPathBuffer(4) << "default: begin\n";
  // Else for Resource is idle
  emitResourceDefaultOp<ResType>(FU);
  vlang->endSwitch(VM->getDataPathBuffer(4));
  //
  VM->getDataPathBuffer(2) << "assign " << SelEval.str() << " };\n";
}

void RTLWriter::emitResources() {
  for (ResourceMapType::iterator I = ResourceMap.begin(), E = ResourceMap.end();
      I != E; ++I) {
    switch (I->first->getResType()) {
    case HWResType::MemoryBus:
      emitResource<HWMemBus>(I->second);
      break;
    case HWResType::SHL:
      emitResource<HWSHL>(I->second);
      break;
    case HWResType::ASR:
      emitResource<HWASR>(I->second);
      break;
    case HWResType::LSR:
      emitResource<HWLSR>(I->second);
      break;
    case HWResType::AddSub:
      emitResource<HWAddSub>(I->second);
      break;
    case HWResType::Mult:
      emitResource<HWMult>(I->second);
      break;
    default:
      break;
    }
  }
}

//===----------------------------------------------------------------------===//
void RTLWriter::emitNextFSMState(raw_ostream &ss, BasicBlock &BB)
{
  ss << "NextFSMState <= " << vlang->GetValueName(&BB) << ";\n";
}


void RTLWriter::createMircoStateEnable(FSMState *State) {
  std::string StateName = vlang->GetValueName(State->getBasicBlock());
  unsigned totalSlot = State->getTotalSlot();

  // Next state
  vlang->declSignal(VM->getSignalDeclBuffer(),
    "next_" + StateName + "_enable", totalSlot + 1, 0);
  vlang->resetRegister(VM->getResetBlockBuffer(),
    "next_" + StateName + "_enable", totalSlot + 1, 0);

    // current state
  vlang->declSignal(VM->getSignalDeclBuffer(),
    "cur_" + StateName + "_enable", totalSlot + 1, 0);
  vlang->resetRegister(VM->getResetBlockBuffer(),
    "cur_" + StateName + "_enable", totalSlot + 1, 0);

  VM->getSeqComputeBuffer(6) << "cur_" << StateName << "_enable <= next_"
                       << StateName << "_enable;\n";
}

std::string RTLWriter::getMircoStateEnableName(FSMState *State,
                                               bool InFSMBlock) {
  std::string StateName = vlang->GetValueName(State->getBasicBlock());

  if (InFSMBlock)
    StateName = "next_" + StateName;
  else
    StateName = "cur_" + StateName;

  return StateName + "_enable";
}

std::string RTLWriter::getMircoStateEnable(FSMState *State, unsigned Slot,
                                           bool InFSMBlock) {
  std::string StateName = getMircoStateEnableName(State, InFSMBlock);
  raw_string_ostream ss(StateName);

  if (State->getTotalSlot() != 0)
    ss << "[" << (Slot - State->getSlot()) << "]";

  return ss.str();
}

void RTLWriter::emitNextMicroState(raw_ostream &ss, BasicBlock &BB,
                                   const std::string &NewState) {
  FSMState *State = HI->getStateFor(BB);
  unsigned totalSlot = State->getTotalSlot();
  std::string StateName = getMircoStateEnableName(State, true);
  ss << StateName << " <= ";
  if (totalSlot > 0)
    ss << "{ " << StateName << "[" <<  totalSlot - 1 << ": 0], ";

  ss << NewState;

  if (totalSlot > 0)
    ss << " }";

  ss << ";\n";
}

std::string  RTLWriter::computeSelfLoopEnable(FSMState *State) {
  unsigned IISlot = State->getIISlot();

  std::string MircoState = getMircoStateEnable(State, IISlot, true);
  
  BasicBlock *BB = State->getBasicBlock();
  BranchInst *Br = cast<BranchInst>(BB->getTerminator());

  MircoState += " & ";
  // Invert the pred if necessary.
  if (Br->getSuccessor(1) == BB)
    MircoState += "~";
  
  Value *Cnd = Br->getCondition();
  if (Instruction *IPred = dyn_cast<Instruction>(Cnd)) {
    HWAOpFU *Pred = cast<HWAOpFU>(HI->getAtomFor(*IPred));
    assert(Pred->getFinSlot() <= IISlot && "Pred can not finish in time!");
    if (Pred->getFinSlot() == IISlot)
      return MircoState + getAsOperand(Pred);
    
    if ((Pred->getFinSlot() + 1 == IISlot) && (Pred->isBinded())) {
      assert(isa<HWAWrReg>(Pred->use_back()) && "Expect write to register!");
      return MircoState + getAsOperand(Pred->use_back());
    }

        
    HWRegister *PredReg = HI->lookupRegForValue(&Pred->getValue());
    return MircoState + getAsOperand(PredReg);
  } else if (ConstantInt *CI = dyn_cast<ConstantInt>(Cnd))
    return MircoState + vlang->printConstant(CI);

  assert(0 && "Not support Pred!");
  return "<Not Support>";
}


//===----------------------------------------------------------------------===//
// Emit instructions
void RTLWriter::visitICmpInst(HWAOpFU &A) {
  ICmpInst &I = A.getInst<ICmpInst>();
  VM->getDataPathBuffer() << "(" << getAsOperand(A.getValDep(0));
  switch (I.getPredicate()) {
      case ICmpInst::ICMP_EQ:  VM->getDataPathBuffer() << " == "; break;
      case ICmpInst::ICMP_NE:  VM->getDataPathBuffer() << " != "; break;
      case ICmpInst::ICMP_ULE:
      case ICmpInst::ICMP_SLE: VM->getDataPathBuffer() << " <= "; break;
      case ICmpInst::ICMP_UGE:
      case ICmpInst::ICMP_SGE: VM->getDataPathBuffer() << " >= "; break;
      case ICmpInst::ICMP_ULT:
      case ICmpInst::ICMP_SLT: VM->getDataPathBuffer() << " < "; break;
      case ICmpInst::ICMP_UGT:
      case ICmpInst::ICMP_SGT: VM->getDataPathBuffer() << " > "; break;
      default: VM->getDataPathBuffer() << "Unknown icmppredicate";
  }
  VM->getDataPathBuffer() << getAsOperand(A.getValDep(1)) << ");\n";
}

void RTLWriter::visitExtInst(HWAOpFU &A) {
  unsigned TyWidth = A.getBitWidth();
  unsigned ChTyWidth = A.getValDep(0)->getBitWidth();

  int DiffBits = TyWidth - ChTyWidth;	
  assert(DiffBits > 0 && "Bad ext!");
  VM->getDataPathBuffer() << "{{" << DiffBits << "{";

  HWEdge &Op = A.getValDep(0);

  CastInst &I = A.getInst<CastInst>();
  if(I.getOpcode() == Instruction::ZExt)
    VM->getDataPathBuffer() << "1'b0";
  else
    VM->getDataPathBuffer() << getAsOperand(Op) << "["<< (ChTyWidth - 1) << "]";

  VM->getDataPathBuffer() <<"}}," << getAsOperand(Op) << "}" <<";\n";   
}


void esyn::RTLWriter::visitReturnInst(HWAOpFU &A) {
  // Operation finish.
  VM->getControlBlockBuffer(10) << "fin <= 1'h1;\n";
  VM->getControlBlockBuffer(10) << "NextFSMState <= state_idle;\n";
  // If returing a value
  ReturnInst &Ret = A.getInst<ReturnInst>();
  if (Ret.getNumOperands() != 0)
        // Emit data path
   VM->getControlBlockBuffer(10) << "return_value" << " <= "
                           << getAsOperand(A.getValDep(0)) << ";\n";
}

void esyn::RTLWriter::visitGetElementPtrInst(HWAOpFU &A) {
  GetElementPtrInst &I = A.getInst<GetElementPtrInst>();
  if (I.getNumIndices() > 1) {
    assert(I.hasAllZeroIndices() && "Too much indices in GEP!");
    // Only emit the pointer.
    VM->getDataPathBuffer() << getAsOperand(A.getValDep(0)) << ";\n";
    return;
  }

  // InstLowering pass already take care of the element size.
  VM->getDataPathBuffer() << getAsOperand(A.getValDep(0)) << " + "
           << getAsOperand(A.getValDep(1)) << ";\n";
}

void esyn::RTLWriter::visitBinaryOperator(HWAOpFU &A) {
  VM->getDataPathBuffer() << getAsOperand(A.getValDep(0));
  Instruction &I = A.getInst<Instruction>();
  switch (I.getOpcode()) {
  case Instruction::Add:  VM->getDataPathBuffer() << " + "; break;
  case Instruction::Sub:  VM->getDataPathBuffer() << " - "; break;
  case Instruction::Mul:  VM->getDataPathBuffer() << " * "; break;
  case Instruction::And:  VM->getDataPathBuffer() << " & "; break;
  case Instruction::Or:   VM->getDataPathBuffer() << " | "; break;
  case Instruction::Xor:  VM->getDataPathBuffer() << " ^ "; break;
  case Instruction::Shl : VM->getDataPathBuffer() << " << "; break;
  case Instruction::LShr: VM->getDataPathBuffer() << " >> "; break;
  case Instruction::AShr: VM->getDataPathBuffer() << " >>> ";  break;
  default: VM->getDataPathBuffer() << " <unsupport> "; break;
  }
  VM->getDataPathBuffer() << getAsOperand(A.getValDep(1)) << ";\n";
}

void esyn::RTLWriter::visitSelectInst(HWAOpFU &A) {
  VM->getDataPathBuffer() << getAsOperand(A.getValDep(0)) << " ? "
           << getAsOperand(A.getValDep(1)) << " : "
           << getAsOperand(A.getValDep(2)) << ";\n";
}

void esyn::RTLWriter::visitIntCastInst(HWAOpFU &A) {
  VM->getDataPathBuffer() << getAsOperand(A.getValDep(0)) << ";\n";
}

void RTLWriter::visitTruncInst(HWAOpFU &A) {
  VM->getDataPathBuffer() << getAsOperand(A.getValDep(0))
           << vlang->printBitWitdh(A.getBitWidth(), 0, true) << ";\n";
}


void RTLWriter::visitBranchInst(HWAOpFU &A) {
  BranchInst &I = A.getInst<BranchInst>();
  if (I.isConditional()) {
    BasicBlock &NextBB0 = *(I.getSuccessor(0)), 
               &NextBB1 = *(I.getSuccessor(1)), 
               &CurBB = *(I.getParent());
    HWEdge &Cnd = A.getValDep(0);
    vlang->ifBegin(VM->getControlBlockBuffer(10), getAsOperand(Cnd));

    if (&NextBB0 != &CurBB) {
      emitNextFSMState(VM->getControlBlockBuffer(12), NextBB0);
      emitNextMicroState(VM->getControlBlockBuffer(12), NextBB0, "1'b1");
    }

    vlang->ifElse(VM->getControlBlockBuffer(10));

    if (&NextBB1 != &CurBB) {
      emitNextFSMState(VM->getControlBlockBuffer(12), NextBB1);
      emitNextMicroState(VM->getControlBlockBuffer(12), NextBB1, "1'b1");
    }

    vlang->end(VM->getControlBlockBuffer(10));
  } else {
    BasicBlock &NextBB = *(I.getSuccessor(0)), &CurBB = *(I.getParent());
    emitNextFSMState(VM->getControlBlockBuffer(10), NextBB);
    emitNextMicroState(VM->getControlBlockBuffer(10), NextBB, "1'b1");
  }
}
