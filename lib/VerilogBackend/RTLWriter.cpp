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

  // Emit control register and idle state
  unsigned totalFSMStates = F.size() + 1;
  TotalFSMStatesBit = Log2_32_Ceil(totalFSMStates);

  vlang->declSignal(getSignalDeclBuffer(), "NextFSMState", TotalFSMStatesBit, 0);
  vlang->resetRegister(getResetBlockBuffer(), "NextFSMState", TotalFSMStatesBit, 0);
  
  // Idle state
  vlang->param(getStateDeclBuffer(), "state_idle", TotalFSMStatesBit, 0);
  vlang->matchCase(ControlBlock.indent(6), "state_idle");
  // Idle state is always ready.
  ControlBlock.indent(8) << "fin <= 1'h0;\n";
  vlang->ifBegin(ControlBlock.indent(8), "start");
  // The module is busy now
  emitNextFSMState(ControlBlock.indent(10), F.getEntryBlock());
  emitNextMicroState(ControlBlock.indent(10), F.getEntryBlock(), "1'b1");
  
  // Remember the parameters
  emitFunctionSignature(F);

  //
  vlang->ifElse(ControlBlock.indent(8));
  ControlBlock.indent(10) << "NextFSMState <= state_idle;\n";
  vlang->end(ControlBlock.indent(8));
  vlang->end(ControlBlock.indent(6));

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

  if (DesignName.empty())
    DesignName = F.getNameStr();

  // Write buffers to output
  vlang->moduleBegin(Out, DesignName);
  Out << ModDecl.str();
  vlang->endModuleDecl(Out);
  Out << "\n\n";
  // States
  vlang->comment(Out.indent(2)) << "States\n";
  Out << StateDecl.str();
  Out << "\n\n";
  // Reg and wire
  vlang->comment(Out.indent(2)) << "Reg and wire decl\n";
  Out << SignalDecl.str();
  Out << "\n\n";

  // Datapath
  vlang->comment(Out.indent(2)) << "Datapath\n";
  Out << DataPath.str();

  Out << "\n\n";
  vlang->comment(Out.indent(2)) << "Always Block\n";
  vlang->alwaysBegin(Out, 2);
  Out << ResetBlock.str();
  vlang->ifElse(Out.indent(4));

  vlang->comment(Out.indent(6)) << "SeqCompute:\n";
  Out << SeqCompute.str();

  vlang->comment(Out.indent(6)) << "FSM\n";
  vlang->switchCase(Out.indent(6), "NextFSMState");
  Out << ControlBlock.str();
  vlang->endSwitch(Out.indent(6));
  vlang->alwaysEnd(Out, 2);

  vlang->endModule(Out);

  return false;
}

void RTLWriter::clear() {
  // Clear buffers
  ModDecl.str().clear();
  StateDecl.str().clear();
  SignalDecl.str().clear();
  DataPath.str().clear();
  ControlBlock.str().clear();
  ResetBlock.str().clear();
  SeqCompute.str().clear();

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
    //
    vlang->declSignal((getModDeclBuffer() << "input "),
      Name, BitWidth, 0, false, PAL.paramHasAttr(Idx, Attribute::SExt), ",");

    UsedRegs.insert(std::make_pair(ArgReg->getRegNum(), ArgReg));
    // Assign the register
    ControlBlock.indent(10) << RegName << " <= " << Name << ";\n";
    ++Idx;
  }

  const Type *RetTy = F.getReturnType();
  if (RetTy->isVoidTy()) {
    // Do something?
    //vlang->indent(ModDecl) << "/*return void*/";
  } else {
    assert(RetTy->isIntegerTy() && "Only support return integer now!");
    vlang->declSignal((getModDeclBuffer() << "output "), "return_value",
      cast<IntegerType>(RetTy)->getBitWidth(), 0, true, false, ",");
    // reset the register
    vlang->resetRegister(getResetBlockBuffer(), "return_value",
                         cast<IntegerType>(RetTy)->getBitWidth());
  }
}

void RTLWriter::emitBasicBlock(BasicBlock &BB) {
  FSMState *State = HI->getStateFor(BB);
  std::string StateName = vlang->GetValueName(&BB);

  unsigned StartSlot = State->getSlot(), EndSlot = State->getEndSlot();

  vlang->comment(getStateDeclBuffer()) << "State for " << StateName << '\n';
  vlang->param(getStateDeclBuffer(), StateName, TotalFSMStatesBit,
               ++CurFSMStateNum);

  // State information.
  vlang->comment(ControlBlock.indent(6)) << StateName 
    << " Total Slot: " << State->getTotalSlot()
    << " II: " << State->getII() <<  '\n';
  // Mirco state enable.
  createMircoStateEnable(State);

  FSMState::ScheduleMapType Atoms;
  typedef FSMState::ScheduleMapType::iterator cycle_iterator;

  State->getScheduleMap(Atoms);

  // Case begin
  vlang->matchCase(ControlBlock.indent(6), StateName);

  for (unsigned i = StartSlot, e = EndSlot + 1; i != e; ++i) {
    vlang->ifBegin(ControlBlock.indent(8), getMircoStateEnable(State, i, true));
    // Emit all atoms at cycle i

    vlang->comment(DataPath.indent(2)) << "at cycle: " << i << '\n';
    for (cycle_iterator CI = Atoms.lower_bound(i), CE = Atoms.upper_bound(i);
        CI != CE; ++CI)
      emitAtom(CI->second);
    vlang->end(ControlBlock.indent(8));
  }// end for

  // Emit Self Loop logic.
  if (State->haveSelfLoop()) {
    vlang->comment(ControlBlock.indent(8)) << "For self loop:\n";
    std::string SelfLoopEnable = computeSelfLoopEnable(State);
    emitNextMicroState(ControlBlock.indent(8), BB, SelfLoopEnable);
  } else
    emitNextMicroState(ControlBlock.indent(8), BB, "1'b0");
  // Case end
  vlang->end(ControlBlock.indent(6));
}

void RTLWriter::emitCommonPort() {
  ModDecl << '\n';
  vlang->comment(getModDeclBuffer()) << "Common ports\n";
  getModDeclBuffer() << "input wire " << "clk" << ",\n";
  getModDeclBuffer() << "input wire " << "rstN" << ",\n";
  getModDeclBuffer() << "input wire " << "start" << ",\n";
  getModDeclBuffer() << "output reg " << "fin";
  // Reset fin
  vlang->resetRegister(getResetBlockBuffer(), "fin", 1);
}

RTLWriter::~RTLWriter() {
  delete &(ModDecl.str());
  delete &(StateDecl.str());
  delete &(SignalDecl.str());
  delete &(DataPath.str());
  delete &(ControlBlock.str());
  delete &(ResetBlock.str());
}


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
        vlang->comment(ControlBlock.indent(10)) << "Emit: "
          << A->getValue() << '\n';
        emitOpFU(cast<HWAOpFU>(A));
        break;
      case atomWrReg:
        emitWrReg(cast<HWAWrReg>(A));
        break;
      case atomLIReg:
        vlang->comment(ControlBlock.indent(10)) << "Import:"
          << A->getValue() << '\n';
        emitLIReg(cast<HWALIReg>(A));
        break;
      case atomDelay:
        vlang->comment(ControlBlock.indent(10));
        A->print(ControlBlock);
        ControlBlock << '\n';
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
    vlang->comment(ControlBlock.indent(10)) << "Read:"
    << DR->getValue() << '\n';

  std::string Name = getAsOperand(DR);
  ControlBlock.indent(10) << Name << " <= " << getAsOperand(E) << ";\n";
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

    vlang->declSignal(getSignalDeclBuffer(), Name, BitWidth, 0);
    vlang->resetRegister(getResetBlockBuffer(), Name, BitWidth, 0);
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
      vlang->declSignal(getSignalDeclBuffer(), Name, OF->getBitWidth(), 0, false);
      vlang->comment(DataPath.indent(2)) << OF->getValue() << '\n';
      // Emit data path
      DataPath.indent(2) << "assign " << Name << " = ";
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

  vlang->declSignal(getSignalDeclBuffer(), OpA, FU->getInputBitwidth(0), 0);
  vlang->declSignal(getSignalDeclBuffer(), OpB, FU->getInputBitwidth(1), 0);
  vlang->declSignal(getSignalDeclBuffer(), Res, FU->getOutputBitwidth(), 0, false);

  // vlang->alwaysBegin(DataPath, 2);
  // vlang->resetRegister(DataPath.indent(6), Res, FU->getOutputBitwidth());
  // vlang->ifElse(DataPath.indent(4));
  DataPath.indent(2) << "assign "<< Res << " = " << OpA << Operator << OpB << ";\n";
  // vlang->alwaysEnd(DataPath, 2);
}

template<>
void RTLWriter::emitResourceDecl<HWAddSub>(HWFUnit *FU) {
  unsigned FUID = FU->getUnitID();
  std::string OpA = "addsub_a" + utostr(FUID);
  std::string OpB = "addsub_b" + utostr(FUID);
  std::string Mode = "addsub_mode" + utostr(FUID);
  std::string Res = getRegPrefix(FU->getResType()) + utostr(FUID);

  vlang->declSignal(getSignalDeclBuffer(), OpA, FU->getInputBitwidth(0), 0);
  vlang->declSignal(getSignalDeclBuffer(), OpB, FU->getInputBitwidth(1), 0);
  vlang->declSignal(getSignalDeclBuffer(), Res, FU->getOutputBitwidth(), 0, false);
  vlang->declSignal(getSignalDeclBuffer(), Mode, 1, 0);

  vlang->comment(DataPath.indent(2)) << "Add/Sub Unit: " << FUID << '\n';
  // vlang->alwaysBegin(DataPath, 2);
  // vlang->resetRegister(DataPath.indent(6), Res, FU->getOutputBitwidth());
  // vlang->ifElse(DataPath.indent(4));
  DataPath.indent(2) << "assign " << Res << " = " << Mode << " ? ";
  DataPath           << "(" << OpA << " + " << OpB << ") : ";
  DataPath           << "(" << OpA << " - " << OpB << ");\n";
  // vlang->alwaysEnd(DataPath, 2);
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
  ModDecl << '\n';
  vlang->comment(getModDeclBuffer()) << "Memory bus " << FUID << '\n';
  getModDeclBuffer() << "input wire [" << (DataWidth-1) << ":0] membus_out"
    << FUID <<",\n";
  getModDeclBuffer() << "output reg [" << (DataWidth - 1) << ":0] membus_in"
    << FUID << ",\n";
  getModDeclBuffer() << "output reg [" << (AddrWidth - 1) <<":0] membus_addr"
    << FUID << ",\n";

  getModDeclBuffer() << "output reg membus_we" << FUID << ",\n";
  getModDeclBuffer() << "output reg membus_en" << FUID << ",\n";
}

void RTLWriter::emitResourceOpForBinOpRes(HWAOpFU *A, const std::string &OpPrefix) {
  unsigned FUID = A->getUnitID();
  std::string OpA = OpPrefix + "_a" + utostr(FUID);
  std::string OpB = OpPrefix + "_b" + utostr(FUID);

  DataPath.indent(6) <<  OpA << " <= "
    << getAsOperand(A->getValDep(0)) << ";\n";
  DataPath.indent(6) <<  OpB << " <= "
    << getAsOperand(A->getValDep(1)) << ";\n";
}

template<>
void RTLWriter::emitResourceOp<HWAddSub>(HWAOpFU *A) {
  unsigned FUID = A->getUnitID();
  std::string Mode = "addsub_mode" + utostr(FUID);

  Instruction *Inst = &(A->getInst<Instruction>());
  DataPath.indent(6) << Mode;
  if (Inst->getOpcode() == Instruction::Sub)
    DataPath << " <= 1'b0;\n";
  else
    DataPath << " <= 1'b1;\n";

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
  DataPath.indent(6) << "membus_en" << FUID << " <= 1'b1;\n";
  // Send the address.
  DataPath.indent(6) << "membus_addr" << FUID << " <= ";

  // Emit the operation
  if (LoadInst *L = dyn_cast<LoadInst>(Inst)) {
    DataPath << getAsOperand(A->getValDep(LoadInst::getPointerOperandIndex()))
      << ";\n";
    DataPath.indent(6) << "membus_we" << FUID << " <= 1'b0;\n";
    DataPath.indent(6) << "membus_in" << FUID
      << " <= " << vlang->printConstantInt(0, DataWidth, false) << ";\n";
  } else { // It must be a store
    DataPath << getAsOperand(A->getValDep(StoreInst::getPointerOperandIndex()))
      << ";\n";
    DataPath.indent(6) << "membus_we" << FUID << " <= 1'b1;\n";
    DataPath.indent(6) << "membus_in" << FUID
      << " <= " << getAsOperand(A->getValDep(0)) << ";\n";
  }
}

void RTLWriter::emitResourceDefaultOpForBinOpRes(HWFUnit *FU, const std::string &OpPrefix) {
  unsigned FUID = FU->getUnitID();
  std::string OpA = OpPrefix + "_a" + utostr(FUID);
  std::string OpB = OpPrefix + "_b" + utostr(FUID);

  DataPath.indent(6) << OpA << " <= "
    << vlang->printConstantInt(0, FU->getInputBitwidth(0), false) << ";\n";
  DataPath.indent(6) << OpB << " <= "
    << vlang->printConstantInt(0, FU->getInputBitwidth(1), false) << ";\n";
  vlang->end(DataPath.indent(4));
}

template<>
void RTLWriter::emitResourceDefaultOp<HWAddSub>(HWFUnit *FU) {
  std::string Mode = "addsub_mode" + utostr(FU->getUnitID());
  DataPath.indent(6) << Mode << " <= 1'b0;\n";
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

  DataPath.indent(6) << "membus_en" << FUID << " <= 1'b0;\n";
  DataPath.indent(6) << "membus_addr" << FUID
    << " <= " << vlang->printConstantInt(0, AddrWidth, false) << ";\n";
  DataPath.indent(6) << "membus_we" << FUID << " <= 1'b0;\n";
  DataPath.indent(6) << "membus_in" << FUID
    << " <= " << vlang->printConstantInt(0, DataWidth, false) << ";\n";
  vlang->end(DataPath.indent(4));
}

template<class ResType>
void RTLWriter::emitResource(HWAPreBindVecTy &Atoms) {
  HWAOpFU *FirstAtom = Atoms[0];
  HWFUnit *FU = FirstAtom->getFUnit();
  unsigned FUID = FU->getUnitID();

  std::string SelName = ResType::getTypeName() + utostr(FUID) + "Mux";
  unsigned SelBitWidth = Atoms.size();
  vlang->declSignal(getSignalDeclBuffer(), SelName, SelBitWidth, 0, false);

  emitResourceDecl<ResType>(FU);
  DataPath.indent(2) << "always @(*)\n";

  unsigned AtomCounter = SelBitWidth;
  vlang->switchCase(DataPath.indent(4), SelName);
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
    vlang->matchCase(DataPath.indent(4), SelCase);

    vlang->comment(DataPath.indent(6)) << *Inst << '\n';

    SelEval << getMircoStateEnable(A->getParent(), A->getSlot(), false);

    emitResourceOp<ResType>(A);
  
    // Else for other atoms
    vlang->end(DataPath.indent(4));
    // Next atom
    if (AtomCounter > 0)
      SelEval << ", ";
  }
  DataPath.indent(4) << "default: begin\n";
  // Else for Resource is idle
  emitResourceDefaultOp<ResType>(FU);
  vlang->endSwitch(DataPath.indent(4));
  //
  DataPath.indent(2) << "assign " << SelEval.str() << " };\n";
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
  vlang->declSignal(getSignalDeclBuffer(),
    "next_" + StateName + "_enable", totalSlot + 1, 0);
  vlang->resetRegister(getResetBlockBuffer(),
    "next_" + StateName + "_enable", totalSlot + 1, 0);

    // current state
  vlang->declSignal(getSignalDeclBuffer(),
    "cur_" + StateName + "_enable", totalSlot + 1, 0);
  vlang->resetRegister(getResetBlockBuffer(),
    "cur_" + StateName + "_enable", totalSlot + 1, 0);

  SeqCompute.indent(6) << "cur_" << StateName << "_enable <= next_"
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
  DataPath << "(" << getAsOperand(A.getValDep(0));
  switch (I.getPredicate()) {
      case ICmpInst::ICMP_EQ:  DataPath << " == "; break;
      case ICmpInst::ICMP_NE:  DataPath << " != "; break;
      case ICmpInst::ICMP_ULE:
      case ICmpInst::ICMP_SLE: DataPath << " <= "; break;
      case ICmpInst::ICMP_UGE:
      case ICmpInst::ICMP_SGE: DataPath << " >= "; break;
      case ICmpInst::ICMP_ULT:
      case ICmpInst::ICMP_SLT: DataPath << " < "; break;
      case ICmpInst::ICMP_UGT:
      case ICmpInst::ICMP_SGT: DataPath << " > "; break;
      default: DataPath << "Unknown icmppredicate";
  }
  DataPath << getAsOperand(A.getValDep(1)) << ");\n";
}

void RTLWriter::visitExtInst(HWAOpFU &A) {
  unsigned TyWidth = A.getBitWidth();
  unsigned ChTyWidth = A.getValDep(0)->getBitWidth();

  int DiffBits = TyWidth - ChTyWidth;	
  DataPath << "{{" << DiffBits << "{";

  HWEdge &Op = A.getValDep(0);

  CastInst &I = A.getInst<CastInst>();
  if(I.getOpcode() == Instruction::ZExt)
    DataPath << "1'b0";
  else
    DataPath << getAsOperand(Op) << "["<< (ChTyWidth - 1) << "]";

  DataPath <<"}}," << getAsOperand(Op) << "}" <<";\n";   
}


void esyn::RTLWriter::visitReturnInst(HWAOpFU &A) {
  // Operation finish.
  ControlBlock.indent(10) << "fin <= 1'h1;\n";
  ControlBlock.indent(10) << "NextFSMState <= state_idle;\n";
  // If returing a value
  ReturnInst &Ret = A.getInst<ReturnInst>();
  if (Ret.getNumOperands() != 0)
        // Emit data path
   ControlBlock.indent(10) << "return_value" << " <= "
                           << getAsOperand(A.getValDep(0)) << ";\n";
}

void esyn::RTLWriter::visitGetElementPtrInst(HWAOpFU &A) {
  GetElementPtrInst &I = A.getInst<GetElementPtrInst>();
  if (I.getNumIndices() > 1) {
    assert(I.hasAllZeroIndices() && "Too much indices in GEP!");
    // Only emit the pointer.
    DataPath << getAsOperand(A.getValDep(0)) << ";\n";
    return;
  }

  // InstLowering pass already take care of the element size.
  DataPath << getAsOperand(A.getValDep(0)) << " + "
           << getAsOperand(A.getValDep(1)) << ";\n";
}

void esyn::RTLWriter::visitBinaryOperator(HWAOpFU &A) {
  DataPath << getAsOperand(A.getValDep(0));
  Instruction &I = A.getInst<Instruction>();
  switch (I.getOpcode()) {
  case Instruction::Add:  DataPath << " + "; break;
  case Instruction::Sub:  DataPath << " - "; break;
  case Instruction::Mul:  DataPath << " * "; break;
  case Instruction::And:  DataPath << " & "; break;
  case Instruction::Or:   DataPath << " | "; break;
  case Instruction::Xor:  DataPath << " ^ "; break;
  case Instruction::Shl : DataPath << " << "; break;
  case Instruction::LShr: DataPath << " >> "; break;
  case Instruction::AShr: DataPath << " >>> ";  break;
  default: DataPath << " <unsupport> "; break;
  }
  DataPath << getAsOperand(A.getValDep(1)) << ";\n";
}

void esyn::RTLWriter::visitSelectInst(HWAOpFU &A) {
  DataPath << getAsOperand(A.getValDep(0)) << " ? "
           << getAsOperand(A.getValDep(1)) << " : "
           << getAsOperand(A.getValDep(2)) << ";\n";
}

void esyn::RTLWriter::visitIntCastInst(HWAOpFU &A) {
  DataPath << getAsOperand(A.getValDep(0)) << ";\n";
}

void RTLWriter::visitTruncInst(HWAOpFU &A) {
  DataPath << getAsOperand(A.getValDep(0))
           << vlang->printBitWitdh(A.getBitWidth(), 0, true) << ";\n";
}


void RTLWriter::visitBranchInst(HWAOpFU &A) {
  BranchInst &I = A.getInst<BranchInst>();
  if (I.isConditional()) {
    BasicBlock &NextBB0 = *(I.getSuccessor(0)), 
               &NextBB1 = *(I.getSuccessor(1)), 
               &CurBB = *(I.getParent());
    HWEdge &Cnd = A.getValDep(0);
    vlang->ifBegin(ControlBlock.indent(10), getAsOperand(Cnd));

    if (&NextBB0 != &CurBB) {
      emitNextFSMState(ControlBlock.indent(12), NextBB0);
      emitNextMicroState(ControlBlock.indent(12), NextBB0, "1'b1");
    }

    vlang->ifElse(ControlBlock.indent(10));

    if (&NextBB1 != &CurBB) {
      emitNextFSMState(ControlBlock.indent(12), NextBB1);
      emitNextMicroState(ControlBlock.indent(12), NextBB1, "1'b1");
    }

    vlang->end(ControlBlock.indent(10));
  } else {
    BasicBlock &NextBB = *(I.getSuccessor(0)), &CurBB = *(I.getParent());
    emitNextFSMState(ControlBlock.indent(10), NextBB);
    emitNextMicroState(ControlBlock.indent(10), NextBB, "1'b1");
  }
}
