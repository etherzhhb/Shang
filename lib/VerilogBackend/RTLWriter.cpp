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
  unsigned totalStatesBits = HI->getTotalCycleBitWidth();
  vlang->declSignal(getSignalDeclBuffer(), "NextState", totalStatesBits, 0);
  vlang->resetRegister(getResetBlockBuffer(), "NextState", totalStatesBits, 0);
  vlang->declSignal(getSignalDeclBuffer(), "CurState", totalStatesBits, 0);
  vlang->resetRegister(getResetBlockBuffer(), "CurState", totalStatesBits, 0);
  
  // Idle state
  vlang->param(getStateDeclBuffer(), "state_idle", HI->getTotalCycleBitWidth(), 0);
  vlang->matchCase(ControlBlock.indent(6), "state_idle");
  // Idle state is always ready.
  ControlBlock.indent(8) << "fin <= 1'h0;\n";
  vlang->ifBegin(ControlBlock.indent(8), "start");
  // The module is busy now
  emitNextState(ControlBlock.indent(10), F.getEntryBlock());
  
  // Remember the parameters
  emitFunctionSignature(F);

  //
  vlang->ifElse(ControlBlock.indent(8));
  ControlBlock.indent(10) << "NextState <= state_idle;\n";
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
  Out.indent(6) << "CurState <= NextState;\n";
  vlang->switchCase(Out.indent(6), "NextState");
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
  FSMState &State = HI->getStateFor(BB);
  std::string StateName = vlang->GetValueName(&BB);
  
  unsigned totalStatesBits = HI->getTotalCycleBitWidth();
  vlang->comment(getStateDeclBuffer()) << "State for " << StateName << '\n';

  //
  FSMState::ScheduleMapType Atoms;
  typedef FSMState::ScheduleMapType::iterator cycle_iterator;

  State.getScheduleMap(Atoms);
  HWAVRoot &Entry = State.getEntryRoot();
  HWAOpInst &Exit = State.getExitRoot();

  unsigned StartSlot = Entry.getSlot(), EndSlot = Exit.getSlot();
  //
  vlang->comment(ControlBlock.indent(6)) << StateName << '\n';
  for (unsigned i = StartSlot, e = EndSlot + 1; i != e; ++i) {
    vlang->param(getStateDeclBuffer(),
                 StateName + utostr(i),
                 totalStatesBits, i);
    // Case begin
    vlang->matchCase(ControlBlock.indent(6), StateName + utostr(i));

    // Emit all atoms at cycle i

    vlang->comment(DataPath.indent(2)) << "at cycle: " << i << '\n';
    for (cycle_iterator CI = Atoms.lower_bound(i), CE = Atoms.upper_bound(i);
        CI != CE; ++CI)
      emitAtom(CI->second);

    // Transfer to next state
    if (i != EndSlot)
      emitNextState(ControlBlock.indent(8), BB, (i - StartSlot) + 1);
    // Case end
    vlang->end(ControlBlock.indent(6));
  }// end for
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
    case atomPostBind:
      return getAsOperand(V, "_w");
    case atomWrStg:
      return getAsOperand(cast<HWAWrStg>(A)->getReg())
        + " /*" + vlang->GetValueName(&A->getValue()) + "*/";
    case atomImpStg:
      return getAsOperand(cast<HWAImpStg>(A)->getReg())
        + " /*" + vlang->GetValueName(&A->getValue()) + "*/";
    default:
      llvm_unreachable("Do not use other atom as operand!");
      return "<Unknown Atom>";
  }
}

std::string RTLWriter::getFURegisterName(HWFUnitID FUID) {
  return "FU" + utostr(FUID.getRawData()) + "Reg";
}

std::string RTLWriter::getAsOperand(HWReg *R) {
  if (R->isFuReg())
    return getFURegisterName(R->getFUnit());

  // Else not a function unit Reg.
  return "Reg"+utostr(R->getRegNum());
}

std::string RTLWriter::getAsOperand(HWEdge *E) {
  switch (E->getEdgeType()) {
  case edgeConst:
    return vlang->printConstant(cast<HWConst>(E)->getConstant());
  case edgeValDep:
    return getAsOperand(cast<HWValDep>(E)->getDagSrc());
  default:
    llvm_unreachable("Do not use other edge as operand!");
    return "<Unknown edge>";
  }
}

void RTLWriter::emitAtom(HWAtom *A) {
  switch (A->getHWAtomType()) {
      case atomPreBind:
        // Emit the origin IR as comment.
        vlang->comment(ControlBlock.indent(8)) << "Emit: "
          << A->getValue() << '\n';
        emitPreBind(cast<HWAPreBind>(A));
        break;
      case atomPostBind:
        // Emit the origin IR as comment.
        vlang->comment(ControlBlock.indent(8)) << "Emit: "
          << A->getValue() << '\n';
        emitPostBind(cast<HWAPostBind>(A));
        break;
      case atomWrStg:
        emitDrvReg(cast<HWAWrStg>(A));
        break;
      // Do nothing.
      case atomImpStg:
      case atomDelay:
      case atomVRoot:
        break;
      default:
        llvm_unreachable("Unknow Atom!");
        break;
  }
}

void RTLWriter::emitDrvReg(HWAWrStg *DR) {
  const HWReg *R = DR->getReg();
  UsedRegs.insert(R);
  
  std::string Name = getAsOperand(DR);
  ControlBlock.indent(8) << Name << " <= " << getAsOperand(DR->getDep(0)) << ";\n";
}

void RTLWriter::emitAllRegisters() {
  for (std::set<const HWReg*>::iterator I = UsedRegs.begin(), E = UsedRegs.end();
      I != E; ++I) {
    const HWReg *R = *I;
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

void RTLWriter::emitResources() {
  ResourceConfig &RC = getAnalysis<ResourceConfig>();

  for (ResourceMapType::iterator I = ResourceMap.begin(), E = ResourceMap.end();
      I != E; ++I) {
    HWResource &Res = *(RC.getResource(I->first.getResType()));
    switch (Res.getResourceType()) {
    case HWResource::MemoryBus:
      emitMemBus(cast<HWMemBus>(Res), I->second);
      break;
    case HWResource::AddSub:
      emitAddSub(cast<HWAddSub>(Res), I->second);
      break;
    default:
      break;
    }
  }
}

void RTLWriter::opAddSub(HWAPreBind *PreBind) {
  std::string Name = getAsOperand(PreBind);
  // Declare the signal
  vlang->declSignal(getSignalDeclBuffer(), Name,
    vlang->getBitWidth(PreBind->getValue()), 0, false);

  DataPath.indent(2) <<  "assign " << getAsOperand(PreBind)
    << " = addsub_res" << PreBind->getUnitNum() << ";\n";
}

void RTLWriter::emitAddSub(HWAddSub &AddSub, HWAPreBindVecTy &Atoms) {
  unsigned ResourceId = Atoms[0]->getUnitNum();

  unsigned MaxBitWidth = AddSub.getMaxBitWidth();

  std::string OpA = "addsub_a" + utostr(ResourceId);
  std::string OpB = "addsub_b" + utostr(ResourceId);
  std::string Mode = "addsub_mode" + utostr(ResourceId);
  std::string Res = "addsub_res" + utostr(ResourceId);

  vlang->declSignal(getSignalDeclBuffer(), OpA, MaxBitWidth, 0);
  
  vlang->declSignal(getSignalDeclBuffer(), OpB, MaxBitWidth, 0);

  vlang->declSignal(getSignalDeclBuffer(), Res, MaxBitWidth, 0, false);
  
  vlang->declSignal(getSignalDeclBuffer(), Mode, 1, 0);

  DataPath.indent(2) << "assign " << Res << " = " << Mode << " ? ";
  DataPath           << "(" << OpA << " + " << OpB << ") : ";
  DataPath           << "(" << OpA << " - " << OpB << ");\n";

  DataPath.indent(2) << "always @(*)\n";
  vlang->switchCase(DataPath.indent(4), "CurState");
  // Emit all resource operation
  for (HWAPreBindVecTy::iterator I = Atoms.begin(), E = Atoms.end(); I != E; ++I) {
    HWAPreBind *A = *I;
    Instruction *Inst = &(A->getInst<Instruction>());

    BasicBlock *BB = Inst->getParent();
    vlang->matchCase(DataPath.indent(4),
      vlang->GetValueName(BB) + utostr(A->getSlot()));

    DataPath.indent(6) << Mode;
    if (Inst->getOpcode() == Instruction::Sub)
      DataPath << " <= 1'b0;\n";
    else
      DataPath << " <= 1'b1;\n";

    DataPath.indent(6) <<  OpA << " <= "
      << getAsOperand(A->getValDep(0)) << ";\n";
    DataPath.indent(6) <<  OpB << " <= "
      << getAsOperand(A->getValDep(1)) << ";\n";

    // Else for other atoms
    vlang->end(DataPath.indent(4));
  }
  DataPath.indent(4) << "default: begin\n";
  // Else for Resource is idle
  DataPath.indent(6) << OpA << " <= "
    << vlang->printConstantInt(0, MaxBitWidth, false) << ";\n";
  DataPath.indent(6) << OpB << " <= "
    << vlang->printConstantInt(0, MaxBitWidth, false) << ";\n";
  DataPath.indent(6) << Mode << " <= 1'b0;\n";

  vlang->end(DataPath.indent(4));
  vlang->endSwitch(DataPath.indent(4));
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

void RTLWriter::emitMemBus(HWMemBus &MemBus,  HWAPreBindVecTy &Atoms) {
  unsigned DataWidth = MemBus.getDataWidth(),
    AddrWidth = MemBus.getAddrWidth();
  unsigned ResourceId = Atoms[0]->getUnitNum();

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

  // Emit all resource operation
  DataPath.indent(2) << "always @(*)\n";
  vlang->switchCase(DataPath.indent(4), "CurState");
  for (HWAPreBindVecTy::iterator I = Atoms.begin(), E = Atoms.end(); I != E; ++I) {
    HWAPreBind *A = *I;
    Instruction *Inst = &(A->getInst<Instruction>());
    BasicBlock *BB = Inst->getParent();
    vlang->matchCase(DataPath.indent(4),
      vlang->GetValueName(BB) + utostr(A->getSlot()));

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
    // Else for other atoms
    vlang->end(DataPath.indent(4));
  }
  DataPath.indent(4) << "default: begin\n";
  // Else for Resource is idle
  DataPath.indent(6) << "membus_addr" << ResourceId
    << " <= " << vlang->printConstantInt(0, AddrWidth, false) << ";\n";
  DataPath.indent(6) << "membus_mode" << ResourceId << " <= 1'b0;\n";
  DataPath.indent(6) << "membus_in" << ResourceId
    << " <= " << vlang->printConstantInt(0, DataWidth, false) << ";\n";
  vlang->end(DataPath.indent(4));
  vlang->endSwitch(DataPath.indent(4));
}

//===----------------------------------------------------------------------===//
void RTLWriter::emitNextState(raw_ostream &ss, BasicBlock &BB, unsigned offset) {
  FSMState &State = HI->getStateFor(BB);
  unsigned stateCycle = State.getEntryRoot().getSlot() + offset;
  assert(stateCycle <= State.getExitRoot().getSlot() 
         && "Offest out of range!");
  ss << "NextState <= " << vlang->GetValueName(&BB) << stateCycle << ";\n";
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

  if(I.getOpcode() == Instruction::ZExt)
    DataPath << "1'b0";
  else
    DataPath << getAsOperand(&A) << "["<< (ChTy->getBitWidth()-1)<<"]";

  DataPath <<"}}," << getAsOperand(A.getValDep(0)) << "}" <<";\n";   
}


void esyn::RTLWriter::visitReturnInst(HWAPostBind &A) {
  // Operation finish.
  ControlBlock.indent(8) << "fin <= 1'h1;\n";
  ControlBlock.indent(8) << "NextState <= state_idle;\n";
  // If returing a value
  ReturnInst &Ret = A.getInst<ReturnInst>();
  if (Ret.getNumOperands() != 0)
        // Emit data path
   ControlBlock.indent(8) << "return_value" << " <= "
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
    HWEdge *Cnd = A.getValDep(0);
    vlang->ifBegin(ControlBlock.indent(8), getAsOperand(Cnd));
    emitPHICopiesForSucc(CurBB, NextBB0, 2);
    emitNextState(ControlBlock.indent(10), NextBB0, 0);
    vlang->ifElse(ControlBlock.indent(8));
    emitPHICopiesForSucc(CurBB, NextBB1, 2);
    emitNextState(ControlBlock.indent(10), NextBB1, 0);
    vlang->end(ControlBlock.indent(8));
  } else {
    BasicBlock &NextBB = *(I.getSuccessor(0)), &CurBB = *(I.getParent());
    emitPHICopiesForSucc(CurBB, NextBB);
    emitNextState(ControlBlock.indent(8), NextBB, 0);
  }
}

void RTLWriter::emitPHICopiesForSucc(BasicBlock &CurBlock, BasicBlock &Succ,
                                     unsigned ind) {
  vlang->comment(ControlBlock.indent(8 + ind)) << "Phi Node:\n";
  Instruction *NotPhi = Succ.getFirstNonPHI();
  BasicBlock::iterator I = Succ.begin();
  while (&*I != NotPhi) {
    PHINode *PN = cast<PHINode>(I);
    HWReg *PR = HI->lookupRegForValue(PN);

    Value *IV = PN->getIncomingValueForBlock(&CurBlock);
    ControlBlock.indent(8 + ind) << getAsOperand(PR)
                                 << "/*" << vlang->GetValueName(PN) << "*/";
    if (Constant *C = dyn_cast<Constant>(IV))
      ControlBlock << " <= " << vlang->printConstant(C) << ";\n";      
    else
      ControlBlock << " <= " << getAsOperand(HI->getLiveOutRegAtTerm(IV))
                   << ";\n";    
    ++I;
  }
}
