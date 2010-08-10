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

using namespace esyn;

char RTLWriter::ID = 0;

bool RTLWriter::runOnFunction(Function &F) {
  TD = &getAnalysis<TargetData>();
  vlang = &getAnalysis<VLang>();
  HI = &getAnalysis<HWAtomInfo>();

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


  // Write buffers to output
  vlang->moduleBegin(Out, F.getNameStr());
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
    Argument &Arg = *I;
    unsigned BitWidth = vlang->getBitWidth(Arg);

    std::string Name = vlang->GetValueName(&Arg), 
                RegName = getAsOperand(HI->lookupRegForValue(&Arg));
    //
    vlang->declSignal((getModDeclBuffer() << "input "),
      Name, BitWidth, 0, false, PAL.paramHasAttr(Idx, Attribute::SExt), ",");
    // Declare the register
    vlang->declSignal(getSignalDeclBuffer(), RegName, BitWidth, 0);
    // Reset the register
    vlang->resetRegister(getResetBlockBuffer(), RegName, BitWidth, 0);
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
    getModDeclBuffer()
      << VLang::printType(RetTy, false, "return_value", "reg ", "output ")
      << ",\n";
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
    vlang->ifBegin(ControlBlock.indent(8), SelfLoopEnable);
    emitPHICopiesForSucc(BB, BB, 0);
    vlang->end(ControlBlock.indent(8));
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
    case atomPreBind:
      return getFURegisterName(cast<HWAPreBind>(A)->getFunUnitID());
    case atomPostBind:
      return getAsOperand(V, "_w");
    case atomWrReg:
      return "/*" + vlang->GetValueName(&A->getValue()) + "*/"
        + getAsOperand(cast<HWAWrReg>(A)->getReg());
    case atomRdReg:
      return "/*" + vlang->GetValueName(&A->getValue()) + "*/"
        + getAsOperand(cast<HWARdReg>(A)->getReg());
    case atomDelay:
      return getAsOperand(cast<HWADelay>(A)->getDep(0).getSrc());
    default:
      llvm_unreachable("Do not use other atom as operand!");
      return "<Unknown Atom>";
  }
}

std::string RTLWriter::getFURegisterName(HWFUnitID FUID) {
  return "FU" + utostr(FUID.getRawData()) + "Reg";
}

std::string RTLWriter::getAsOperand(HWRegister *R) {
  if (R->isFuReg())
    return getFURegisterName(R->getFUnit());

  // Else not a function unit Reg.
  return "Reg"+utostr(R->getRegNum());
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
      case atomPreBind:
        // Emit the origin IR as comment.
        vlang->comment(ControlBlock.indent(10)) << "Emit: "
          << A->getValue() << '\n';
        emitPreBind(cast<HWAPreBind>(A));
        break;
      case atomPostBind:
        // Emit the origin IR as comment.
        vlang->comment(ControlBlock.indent(10)) << "Emit: "
          << A->getValue() << '\n';
        emitPostBind(cast<HWAPostBind>(A));
        break;
      case atomWrReg:
        vlang->comment(ControlBlock.indent(10)) << "Read:"
          << A->getValue() << '\n';
        emitWrReg(cast<HWAWrReg>(A));
        break;
      case atomRdReg:
        vlang->comment(ControlBlock.indent(10)) << "Import:"
          << A->getValue() << '\n';
        emitRdReg(cast<HWARdReg>(A));
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
  
  UsedRegs.insert(R);
  
  std::string Name = getAsOperand(DR);
  ControlBlock.indent(10) << Name << " <= " << getAsOperand(DR->getDep(0)) << ";\n";
}

void RTLWriter::emitRdReg(HWARdReg *DR) {
  if (DR->isPHINode())
    UsedRegs.insert(DR->getReg());
}

void RTLWriter::emitAllRegisters() {
  for (std::set<const HWRegister*>::iterator I = UsedRegs.begin(), E = UsedRegs.end();
      I != E; ++I) {
    const HWRegister *R = *I;
    unsigned BitWidth = vlang->getBitWidth(R->getType());
    std::string Name = "Reg"+utostr(R->getRegNum());

    vlang->declSignal(getSignalDeclBuffer(), Name, BitWidth, 0);
    vlang->resetRegister(getResetBlockBuffer(), Name, BitWidth, 0);
  } 
}

void RTLWriter::emitPostBind(HWAPostBind *PostBind) {
  Instruction &Inst = cast<Instruction>(PostBind->getValue());
  assert(!isa<PHINode>(Inst) && "PHINode is not PostBind atom!");
  std::string Name = getAsOperand(PostBind);
  // Do not decl signal for void type
  // And do not emit data path for phi node.
  if (!Inst.getType()->isVoidTy()) {
    // Declare the signal
    vlang->declSignal(getSignalDeclBuffer(), Name, vlang->getBitWidth(Inst), 0, false);
    // Emit data path
    DataPath.indent(2) << "assign " << Name << " = ";
  }
  // Emit the data path
  visit(*PostBind);
}

void RTLWriter::emitPreBind(HWAPreBind *PreBind) {
  // Remember this atom
  ResourceMap[PreBind->getFunUnitID()].push_back(PreBind);

  switch (PreBind->getResClass()) {
  case HWResource::MemoryBus:
    opMemBus(PreBind);
    break;
  case HWResource::AddSub:
    opAddSub(PreBind);
    break;
  default:
    assert(!"Unexcept resource type!");
  }
}

void RTLWriter::opAddSub(HWAPreBind *PreBind) {
}

template<>
void RTLWriter::emitResourceDecl<HWAddSub>(HWAPreBindVecTy &Atoms) {
  HWAPreBind *FirstAtom = Atoms[0];
  HWFUnitID FUID = FirstAtom->getFunUnitID();
  unsigned ResourceId = FUID.getUnitNum();
  // Dirty Hack: Resource only share inside bb at this moment.
  FSMState *State = FirstAtom->getParent();
  BasicBlock *BB = State->getBasicBlock();
  unsigned SlotWidth = State->getTotalSlot() + 1;
  unsigned StartSlot = State->getSlot();

  // FIXME: Do mix difference type in a function unit.
  unsigned MaxBitWidth = vlang->getBitWidth(FirstAtom->getValue());
  //AddSub.getMaxBitWidth();

  std::string OpA = "addsub_a" + utostr(ResourceId);
  std::string OpB = "addsub_b" + utostr(ResourceId);
  std::string Mode = "addsub_mode" + utostr(ResourceId);
  std::string Res = getFURegisterName(FUID);

  vlang->declSignal(getSignalDeclBuffer(), OpA, MaxBitWidth, 0);
  
  vlang->declSignal(getSignalDeclBuffer(), OpB, MaxBitWidth, 0);

  vlang->declSignal(getSignalDeclBuffer(), Res, MaxBitWidth, 0);
  
  vlang->declSignal(getSignalDeclBuffer(), Mode, 1, 0);

  vlang->comment(DataPath.indent(2)) << "Add/Sub Unit: "
                                     << FUID.getRawData() << '\n';
  vlang->alwaysBegin(DataPath, 2);
  vlang->resetRegister(DataPath.indent(6), Res, MaxBitWidth);
  vlang->ifElse(DataPath.indent(4));
  DataPath.indent(6) << Res << " <= " << Mode << " ? ";
  DataPath           << "(" << OpA << " + " << OpB << ") : ";
  DataPath           << "(" << OpA << " - " << OpB << ");\n";
  vlang->alwaysEnd(DataPath, 2);
}


template<>
void RTLWriter::emitResourceOp<HWAddSub>(HWAPreBind *A) {
  unsigned ResourceId = A->getUnitNum();
  std::string OpA = "addsub_a" + utostr(ResourceId);
  std::string OpB = "addsub_b" + utostr(ResourceId);
  std::string Mode = "addsub_mode" + utostr(ResourceId);
  std::string Res = getFURegisterName(A->getFunUnitID());

  Instruction *Inst = &(A->getInst<Instruction>());
  DataPath.indent(6) << Mode;
  if (Inst->getOpcode() == Instruction::Sub)
    DataPath << " <= 1'b0;\n";
  else
    DataPath << " <= 1'b1;\n";

  DataPath.indent(6) <<  OpA << " <= "
    << getAsOperand(A->getValDep(0)) << ";\n";
  DataPath.indent(6) <<  OpB << " <= "
    << getAsOperand(A->getValDep(1)) << ";\n";
}


template<>
void RTLWriter::emitResourceDefaultOp<HWAddSub>(HWFUnit FU) {
  unsigned ResourceId = FU.getUnitNum();
  std::string OpA = "addsub_a" + utostr(ResourceId);
  std::string OpB = "addsub_b" + utostr(ResourceId);
  std::string Mode = "addsub_mode" + utostr(ResourceId);
  // FIXME: Bitwidth is not correct!
  DataPath.indent(6) << OpA << " <= "
    << vlang->printConstantInt(0, 64, false) << ";\n";
  DataPath.indent(6) << OpB << " <= "
    << vlang->printConstantInt(0, 64, false) << ";\n";
  DataPath.indent(6) << Mode << " <= 1'b0;\n";

  vlang->end(DataPath.indent(4));
}


template<>
void RTLWriter::emitResourceDecl<HWMemBus>(HWAPreBindVecTy &Atoms) {
  HWMemBus *MemBus = cast<HWMemBus>(RC->getResource(HWResource::MemoryBus));
  HWAPreBind *FirstAtom = Atoms[0];
  // Dirty Hack: Resource only share inside bb at this moment.
  FSMState *State = FirstAtom->getParent();
  BasicBlock *BB = State->getBasicBlock();
  unsigned SlotWidth = State->getTotalSlot() + 1;
  unsigned StartSlot = State->getSlot();

  unsigned DataWidth = MemBus->getDataWidth(),
           AddrWidth = MemBus->getAddrWidth();
  unsigned ResourceId = FirstAtom->getUnitNum();

  // Emit the ports;
  ModDecl << '\n';
  vlang->comment(getModDeclBuffer()) << "Memory bus " << ResourceId << '\n';
  getModDeclBuffer() << "input wire [" << (DataWidth-1) << ":0] membus_out"
    << ResourceId <<",\n";
  getModDeclBuffer() << "output reg [" << (DataWidth - 1) << ":0] membus_in"
    << ResourceId << ",\n";

  getModDeclBuffer() << "output reg [" << (AddrWidth - 1) <<":0] membus_addr"
    << ResourceId << ",\n";

  getModDeclBuffer() << "output reg membus_mode" << ResourceId << ",\n";

}

template<>
void RTLWriter::emitResourceOp<HWMemBus>(HWAPreBind *A) {
  HWMemBus *MemBus = cast<HWMemBus>(RC->getResource(HWResource::MemoryBus));
  unsigned DataWidth = MemBus->getDataWidth(),
           AddrWidth = MemBus->getAddrWidth();

  unsigned ResourceId = A->getUnitNum();
  Instruction *Inst = &(A->getInst<Instruction>());
  DataPath.indent(6) << "membus_addr" << ResourceId << " <= ";

  // Emit the operation
  if (LoadInst *L = dyn_cast<LoadInst>(Inst)) {
    DataPath << getAsOperand(A->getValDep(LoadInst::getPointerOperandIndex()))
      << ";\n";
    DataPath.indent(6) << "membus_mode" << ResourceId << " <= 1'b0;\n";
    DataPath.indent(6) << "membus_in" << ResourceId
      << " <= " << vlang->printConstantInt(0, DataWidth, false) << ";\n";
  } else { // It must be a store
    DataPath << getAsOperand(A->getValDep(StoreInst::getPointerOperandIndex()))
      << ";\n";
    DataPath.indent(6) << "membus_mode" << ResourceId << " <= 1'b1;\n";
    DataPath.indent(6) << "membus_in" << ResourceId
      << " <= " << getAsOperand(A->getValDep(0)) << ";\n";
  }
}

template<>
void RTLWriter::emitResourceDefaultOp<HWMemBus>(HWFUnit FU) {
  HWMemBus *MemBus = cast<HWMemBus>(RC->getResource(HWResource::MemoryBus));
  unsigned DataWidth = MemBus->getDataWidth(),
           AddrWidth = MemBus->getAddrWidth();

  unsigned ResourceId = FU.getUnitNum();

  DataPath.indent(6) << "membus_addr" << ResourceId
    << " <= " << vlang->printConstantInt(0, AddrWidth, false) << ";\n";
  DataPath.indent(6) << "membus_mode" << ResourceId << " <= 1'b0;\n";
  DataPath.indent(6) << "membus_in" << ResourceId
    << " <= " << vlang->printConstantInt(0, DataWidth, false) << ";\n";
}

template<class ResType>
void RTLWriter::emitResource(HWAPreBindVecTy &Atoms) {
  HWAPreBind *FirstAtom = Atoms[0];
  unsigned FUNum = FirstAtom->getUnitNum();

  std::string SelName = ResType::getResourceName() + utostr(FUNum) + "Mux";
  unsigned SelBitWidth = Atoms.size();
  vlang->declSignal(getSignalDeclBuffer(), SelName, SelBitWidth, 0, false);

  emitResourceDecl<ResType>(Atoms);
  DataPath.indent(2) << "always @(*)\n";

  unsigned AtomCounter = SelBitWidth;
  vlang->switchCase(DataPath.indent(4), SelName);
  raw_string_ostream SelEval(SelName);
  SelEval << " = { ";

  // Emit all resource operation
  for (HWAPreBindVecTy::iterator I = Atoms.begin(), E = Atoms.end();
      I != E; ++I) {
    HWAPreBind *A = *I;
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
  emitResourceDefaultOp<ResType>(FirstAtom->getFunUnit());
  vlang->endSwitch(DataPath.indent(4));
  //
  DataPath.indent(4) << "assign " << SelEval.str() << " };\n";
}

void RTLWriter::emitResources() {
  for (ResourceMapType::iterator I = ResourceMap.begin(), E = ResourceMap.end();
      I != E; ++I) {
    switch (I->first.getResType()) {
    case HWResource::MemoryBus:
      emitResource<HWMemBus>(I->second);
      break;
    case HWResource::AddSub:
      emitResource<HWAddSub>(I->second);
      break;
    default:
      break;
    }
  }
}

void RTLWriter::opMemBus(HWAPreBind *PreBind) {
  unsigned MemBusInst = PreBind->getUnitNum();
  if (LoadInst *L = dyn_cast<LoadInst>(&PreBind->getValue())) {
    std::string Name = getAsOperand(PreBind);
    // Declare the signal
    vlang->declSignal(getSignalDeclBuffer(), Name,
                      vlang->getBitWidth(*L), 0, false);
    // Emit the datapath
    DataPath.indent(2) <<  "assign " << getAsOperand(PreBind) 
      << " = membus_out" << MemBusInst <<";\n";
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
    HWAOpInst *Pred = cast<HWAOpInst>(HI->getAtomFor(*IPred));
    assert(Pred->getFinSlot() <= IISlot && "Pred can not finish in time!");
    assert(isa<HWAPostBind>(Pred) && "Prebind predicate not support yet!");
    if (Pred->getFinSlot() == IISlot)
      return MircoState + getAsOperand(Pred);
    
    if ((Pred->getFinSlot() + 1 == IISlot) && (isa<HWAPreBind>(Pred)))
      return MircoState +
             getFURegisterName(cast<HWAPreBind>(Pred)->getFunUnitID());

        
    HWRegister *PredReg = HI->lookupRegForValue(&Pred->getValue());
    return MircoState + getAsOperand(PredReg);
  } else if (ConstantInt *CI = dyn_cast<ConstantInt>(Cnd))
    return MircoState + vlang->printConstant(CI);

  assert(0 && "Not support Pred!");
  return "<Not Support>";
}


//===----------------------------------------------------------------------===//
// Emit instructions
void RTLWriter::visitICmpInst(HWAPostBind &A) {
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

void RTLWriter::visitExtInst(HWAPostBind &A) {
  CastInst &I = A.getInst<CastInst>();
  const IntegerType *Ty = cast<IntegerType>(I.getType());

  Value *V = I.getOperand(0);
  const IntegerType *ChTy = cast<IntegerType>(V->getType());

  int DiffBits = Ty->getBitWidth() - ChTy->getBitWidth();	
  DataPath << "{{" << DiffBits << "{";

  HWEdge &Op = A.getValDep(0);

  if(I.getOpcode() == Instruction::ZExt)
    DataPath << "1'b0";
  else
    DataPath << getAsOperand(Op) << "["<< (ChTy->getBitWidth()-1)<<"]";

  DataPath <<"}}," << getAsOperand(Op) << "}" <<";\n";   
}


void esyn::RTLWriter::visitReturnInst(HWAPostBind &A) {
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

void esyn::RTLWriter::visitGetElementPtrInst(HWAPostBind &A) {
  GetElementPtrInst &I = A.getInst<GetElementPtrInst>();
  const Type *Ty = I.getOperand(0)->getType();
  assert(isa<SequentialType>(Ty) && "GEP type not support yet!");
  assert(I.getNumIndices() < 2 && "Too much indices in GEP!");

  DataPath << getAsOperand(A.getValDep(0)) << " + "
           << getAsOperand(A.getValDep(1)) << ";\n"; // FIXME multipy the element size.
}

void esyn::RTLWriter::visitBinaryOperator(HWAPostBind &A) {
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

void esyn::RTLWriter::visitSelectInst(HWAPostBind &A) {
  DataPath << getAsOperand(A.getValDep(0)) << " ? "
           << getAsOperand(A.getValDep(1)) << " : "
           << getAsOperand(A.getValDep(2)) << ";\n";
}

void RTLWriter::visitTruncInst(HWAPostBind &A) {
  TruncInst &I = A.getInst<TruncInst>();
  const IntegerType *Ty = cast<IntegerType>(I.getType());
  DataPath << getAsOperand(A.getValDep(0)) << vlang->printBitWitdh(Ty, 0, true) << "\n";
}


void RTLWriter::visitBranchInst(HWAPostBind &A) {
  BranchInst &I = A.getInst<BranchInst>();
  if (I.isConditional()) {
    BasicBlock &NextBB0 = *(I.getSuccessor(0)), 
               &NextBB1 = *(I.getSuccessor(1)), 
               &CurBB = *(I.getParent());
    HWEdge &Cnd = A.getValDep(0);
    vlang->ifBegin(ControlBlock.indent(10), getAsOperand(Cnd));

    if (&NextBB0 != &CurBB) {
      emitPHICopiesForSucc(CurBB, NextBB0, 2);
      emitNextFSMState(ControlBlock.indent(12), NextBB0);
      emitNextMicroState(ControlBlock.indent(12), NextBB0, "1'b1");
    }

    vlang->ifElse(ControlBlock.indent(10));

    if (&NextBB1 != &CurBB) {
      emitPHICopiesForSucc(CurBB, NextBB1, 2);
      emitNextFSMState(ControlBlock.indent(12), NextBB1);
      emitNextMicroState(ControlBlock.indent(12), NextBB1, "1'b1");
    }

    vlang->end(ControlBlock.indent(10));
  } else {
    BasicBlock &NextBB = *(I.getSuccessor(0)), &CurBB = *(I.getParent());
    emitPHICopiesForSucc(CurBB, NextBB);
    emitNextFSMState(ControlBlock.indent(10), NextBB);
    emitNextMicroState(ControlBlock.indent(10), NextBB, "1'b1");
  }
}

void RTLWriter::emitPHICopiesForSucc(BasicBlock &CurBlock, BasicBlock &Succ,
                                     unsigned ind) {
  FSMState *CurStage = HI->getStateFor(CurBlock);
  bool SelfLoop = (&CurBlock == &Succ);

  vlang->comment(ControlBlock.indent(10 + ind)) << "Phi Node:\n";
  Instruction *NotPhi = Succ.getFirstNonPHI();
  
  for (BasicBlock::iterator I = Succ.begin(), E = Succ.getFirstNonPHI();
      I != E; ++I) {
    PHINode *PN = cast<PHINode>(I);
    HWRegister *PR = HI->lookupRegForValue(PN);

    Value *IV = PN->getIncomingValueForBlock(&CurBlock);
    ControlBlock.indent(10 + ind) << getAsOperand(PR)
                                 << "/*" << vlang->GetValueName(PN) << "*/";

    if (Constant *C = dyn_cast<Constant>(IV)) {
      ControlBlock << " <= " << vlang->printConstant(C) << ";\n";
      continue;
    } else if (SelfLoop && isa<Instruction>(IV)) {
      // Loop up the self phi source table.
      Instruction *Inst = cast<Instruction>(IV);
      ControlBlock << " <= " << getAsOperand(CurStage->getSelfPHISrc(Inst))
                   << ";\n";
    } else {
      // Just loop up the register.
      HWRegister *PHISrc = HI->lookupRegForValue(IV);
      ControlBlock << " <= " << getAsOperand(PHISrc) << ";\n";
    }
  }
}
