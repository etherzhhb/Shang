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
  ControlBlock.indent(10) << "CurState <= state_idle;\n";
  vlang->end(ControlBlock.indent(8));
  vlang->end(ControlBlock.indent(6));

  // Emit basicblocks
  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
    BasicBlock &BB = *I;
    emitBasicBlock(BB);
  }

  // Emit resouces
  emitResources();

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
  vlang->switchCase(Out.indent(6), "CurState");
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
                RegName = getAsOperand(&Arg);
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
  ExecStage &State = HI->getStateFor(BB);
  std::string StateName = vlang->GetValueName(&BB);
  
  unsigned totalStatesBits = HI->getTotalCycleBitWidth();
  vlang->comment(getStateDeclBuffer()) << "State for " << StateName << '\n';

  //
  ExecStage::ScheduleMapType Atoms;
  typedef ExecStage::ScheduleMapType::iterator cycle_iterator;

  State.getScheduleMap(Atoms);
  HWAVRoot &Entry = State.getEntryRoot();
  HWAPostBind &Exit = State.getExitRoot();

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
  if (ConstantInt *CI = dyn_cast<ConstantInt>(V))
    return vlang->printConstant(CI);

  if (PHINode *PHI = dyn_cast<PHINode>(V))
    return vlang->GetValueName(PHI) + "_phi_r";

  if (Argument *Arg = dyn_cast<Argument>(V))
    return vlang->GetValueName(V) + "_pr";

  return vlang->GetValueName(V) + postfix;
}
std::string RTLWriter::getAsOperand(HWAtom *A) {
  Value *V = &A->getValue();
  switch (A->getHWAtomType()) {
    case atomRegister:
      return getAsOperand(V, "_r");
    case atomPreBind:
    case atomPostBind:
      return getAsOperand(V, "_w");
    case atomConst:
      return getAsOperand(V);
    case atomSignedPrefix:
      return getAsOperand(V) + "_signed_w";
    default:
      return "<Unknown Atom>";
  }
}

void RTLWriter::emitAtom(HWAtom *A) {
  switch (A->getHWAtomType()) {
      case atomRegister:
        // Emit the origin IR as comment.
        vlang->comment(ControlBlock.indent(8)) << "Finish: "
          << A->getValue() << '\n';
        emitRegister(cast<HWARegister>(A));
        break;
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
      case atomSignedPrefix:
        emitSigned(cast<HWASigned>(A));
        break;
      case atomVRoot:
        break;
      case atomConst:
        if (isa<PHINode>(A->getValue())) {
          vlang->comment(ControlBlock.indent(8)) << "Emit: "
            << A->getValue() << '\n';
          visitPHINode(cast<HWAConst>(A));
        }
        break;
      default:
        llvm_unreachable("Unknow Atom!");;
  }
}

void RTLWriter::emitSigned(HWASigned *Signed) {
  std::string Name = getAsOperand(Signed);
  Value &V = Signed->getValue();

  // Declare the signal
  vlang->declSignal(getSignalDeclBuffer(), Name,
                    vlang->getBitWidth(V), 0, false, true);
  DataPath.indent(2) << "assign " <<
    Name << " = " << getAsOperand(Signed->getDep(0)) << ";\n";
}

void RTLWriter::emitRegister(HWARegister *Register) {
  HWAtom *Val = Register->getRefVal();

  Value &V = Register->getValue();
  unsigned BitWidth = vlang->getBitWidth(V);
  
  std::string Name = getAsOperand(Register);

  // Declare the register
  vlang->declSignal(getSignalDeclBuffer(), Name, BitWidth, 0);
  // Reset the register
  vlang->resetRegister(getResetBlockBuffer(), Name, BitWidth, 0);

  // assign the register
  ControlBlock.indent(8) << Name << " <= " << getAsOperand(Val) << ";\n";
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
  ResourceMap[PreBind->getResourceId()].push_back(PreBind);
  //
  switch (PreBind->getResClass()) {
  case HWResource::MemoryBus:
    opMemBus(PreBind);
    break;
  }
}

void RTLWriter::emitResources() {
  ResourceConfig &RC = getAnalysis<ResourceConfig>();

  for (ResourceMapType::iterator I = ResourceMap.begin(), E = ResourceMap.end();
      I != E; ++I) {
    HWResource &Res = *(RC.getResource(HWResource::extractResType(I->first)));
    switch (Res.getResourceType()) {
    case HWResource::MemoryBus:
      emitMemBus(cast<HWMemBus>(Res), I->second);
    default:
      return;
    }
  }
}

void RTLWriter::opMemBus(HWAPreBind *PreBind) {
  unsigned MemBusInst = PreBind->getAllocatedInstance();
  if (LoadInst *L = dyn_cast<LoadInst>(&PreBind->getValue())) {
    std::string Name = getAsOperand(PreBind);
    // Declare the signal
    vlang->declSignal(getSignalDeclBuffer(), Name,
                      vlang->getBitWidth(*L), 0, false);
    // Emit the datapath
    DataPath.indent(2) <<  "assign " << getAsOperand(PreBind) 
      << " = membus_out" << MemBusInst <<";\n";
  } else { // It must be a store.
    StoreInst &S = PreBind->getInst<StoreInst>();
    ControlBlock.indent(8) << "membus_in" << MemBusInst
      << " <= " << getAsOperand(PreBind->getOperand(0)) << ";\n";
  }
}

void RTLWriter::emitMemBus(HWMemBus &MemBus,  HWAPreBindVecTy &Atoms) {
  unsigned DataWidth = MemBus.getDataWidth(),
    AddrWidth = MemBus.getAddrWidth();
  unsigned ResourceId = Atoms[0]->getAllocatedInstance();

  // Emit the ports;
  ModDecl << '\n';
  vlang->comment(getModDeclBuffer()) << "Memory bus " << ResourceId << '\n';
  getModDeclBuffer() << "input wire [" << (DataWidth-1) << ":0] membus_out"
                     << ResourceId <<",\n";
  getModDeclBuffer() << "output reg [" << (DataWidth - 1) << ":0] membus_in"
                     << ResourceId << ",\n";
  vlang->resetRegister(getResetBlockBuffer(), "membus_in" + utostr(ResourceId),
                       DataWidth, 0);
  getModDeclBuffer() << "output reg [" << (AddrWidth - 1) <<":0] membus_addr"
                     << ResourceId << ",\n";
  vlang->resetRegister(getResetBlockBuffer(),
                       "membus_addr" + utostr(ResourceId), AddrWidth);

  getModDeclBuffer() << "output reg membus_mode" << ResourceId << ",\n";
  vlang->resetRegister(getResetBlockBuffer(), 
                       "membus_mode" + utostr(ResourceId), 1, 0);

  // Emit all resource operation
  for (HWAPreBindVecTy::iterator I = Atoms.begin(), E = Atoms.end(); I != E; ++I) {
    HWAPreBind *A = *I;
    Instruction *Inst = &(A->getInst<Instruction>());
    BasicBlock *BB = Inst->getParent();
    vlang->ifBegin(SeqCompute.indent(6),
      "CurState == " + vlang->GetValueName(BB) + utostr(A->getSlot()));

    SeqCompute.indent(8) << "membus_addr" << ResourceId << " <= ";

    // Emit the operation
    if (LoadInst *L = dyn_cast<LoadInst>(Inst)) {
      SeqCompute << getAsOperand(A->getOperand(LoadInst::getPointerOperandIndex()))
                << ";\n";
      SeqCompute.indent(8) << "membus_mode" << ResourceId << " <= 1'b0;\n";
    } else { // It must be a store
      SeqCompute << getAsOperand(A->getOperand(StoreInst::getPointerOperandIndex()))
        << ";\n";
      SeqCompute.indent(8) << "membus_mode" << ResourceId << "<= 1'b1;\n";
    }
    // Else for other atoms
    vlang->ifElse(SeqCompute.indent(6), false);
  }
  vlang->begin(SeqCompute.indent(6));
  // Else for Resource is idle
  SeqCompute.indent(8) << "membus_addr" << ResourceId
    << " <= " << vlang->printConstantInt(0, AddrWidth, false) << ";\n";
  SeqCompute.indent(8) << "membus_mode" << ResourceId << " <= 1'b0;\n";

  vlang->end(SeqCompute.indent(6));
}

//===----------------------------------------------------------------------===//
void RTLWriter::emitNextState(raw_ostream &ss, BasicBlock &BB, unsigned offset) {
  ExecStage &State = HI->getStateFor(BB);
  unsigned stateCycle = State.getEntryRoot().getSlot() + offset;
  assert(stateCycle <= State.getExitRoot().getSlot() 
         && "Offest out of range!");
  ss << "CurState <= " << vlang->GetValueName(&BB) << stateCycle << ";\n";
}

//===----------------------------------------------------------------------===//
// Emit instructions
void RTLWriter::visitICmpInst(HWAPostBind &A) {
  ICmpInst &I = A.getInst<ICmpInst>();
  DataPath << "(" << getAsOperand(A.getOperand(0));
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
  DataPath << getAsOperand(A.getOperand(1)) << ");\n";
}


void RTLWriter::visitPHINode(HWAConst *A) {
  PHINode &I = cast<PHINode>(A->getValue());
  unsigned BitWidth = vlang->getBitWidth(I);

  std::string Name = getAsOperand(A);

  // Declare the register
  vlang->declSignal(getSignalDeclBuffer(), Name, BitWidth, 0);
  // Reset the register
  vlang->resetRegister(getResetBlockBuffer(), Name, BitWidth, 0);
  // Phi node will be assign at terminate state
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

  DataPath <<"}}," << getAsOperand(A.getOperand(0)) << "}" <<";\n";   
}


void esyn::RTLWriter::visitReturnInst(HWAPostBind &A) {
  // Operation finish.
  ControlBlock.indent(8) << "fin <= 1'h1;\n";
  ControlBlock.indent(8) << "CurState <= state_idle;\n";
  // If returing a value
  ReturnInst &Ret = A.getInst<ReturnInst>();
  if (Ret.getNumOperands() != 0)
        // Emit data path
   ControlBlock.indent(8) << "return_value" << " <= "
                          << getAsOperand(A.getOperand(0)) << ";\n";
}

void esyn::RTLWriter::visitGetElementPtrInst(HWAPostBind &A) {
  GetElementPtrInst &I = A.getInst<GetElementPtrInst>();
  const Type *Ty = I.getOperand(0)->getType();
  assert(isa<SequentialType>(Ty) && "GEP type not support yet!");
  assert(I.getNumIndices() < 2 && "Too much indices in GEP!");

  DataPath << getAsOperand(A.getOperand(0)) << " + "
           << getAsOperand(A.getOperand(1)) << ";\n"; // FIXME multipy the element size.
}

void esyn::RTLWriter::visitBinaryOperator(HWAPostBind &A) {
  DataPath << getAsOperand(A.getOperand(0));
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
  DataPath << getAsOperand(A.getOperand(1)) << ";\n";
}

void esyn::RTLWriter::visitSelectInst(HWAPostBind &A) {
  DataPath << getAsOperand(A.getOperand(0)) << " ? "
           << getAsOperand(A.getOperand(1)) << " : "
           << getAsOperand(A.getOperand(2)) << ";\n";
}

void RTLWriter::visitTruncInst(HWAPostBind &A) {
  TruncInst &I = A.getInst<TruncInst>();
  const IntegerType *Ty = cast<IntegerType>(I.getType());
  DataPath << getAsOperand(A.getOperand(0)) << vlang->printBitWitdh(Ty, 0, true) << "\n";
}


void RTLWriter::visitBranchInst(HWAPostBind &A) {
  BranchInst &I = A.getInst<BranchInst>();
  if (I.isConditional()) {
    BasicBlock &NextBB0 = *(I.getSuccessor(0)), 
               &NextBB1 = *(I.getSuccessor(1)), 
               &CurBB = *(I.getParent());
    vlang->ifBegin(ControlBlock.indent(8), getAsOperand(A.getOperand(0)));
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
    Value *IV = PN->getIncomingValueForBlock(&CurBlock);
    HWAtom *Incoming = HI->getAtomFor(*IV);
    while (isa<HWARegister>(Incoming) 
          && Incoming->getSlot() == HI->getStateFor(CurBlock).getExitRoot().getSlot())
      Incoming = cast<HWARegister>(Incoming)->getRefVal();
    
    ControlBlock.indent(8 + ind) << getAsOperand(HI->getAtomFor(*I))
      << " <= " << getAsOperand(Incoming) << ";\n";      
    ++I;
  }
}
