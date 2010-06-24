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

  emitFunctionSignature(F);

  // Emit resources
  for(HWAtomInfo::resource_iterator I = HI->resource_begin(),
    E = HI->resource_end();I != E; ++I) {
      HWResource &Resource = **I;
      // Ignore the infinite resources, we will emit them on the fly.
      if (Resource.isInfinite())
        continue;

      emitResources(Resource);
  }
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
  vlang->ifElse(ControlBlock.indent(8));
  ControlBlock.indent(10) << "CurState <= state_idle\n";
  vlang->end(ControlBlock.indent(8));
  vlang->end(ControlBlock.indent(6));

  // Emit basicblocks
  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
    BasicBlock &BB = *I;
    emitBasicBlock(BB);
  }

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
  vlang->switchCase(Out.indent(6), "CurState");
  Out << ControlBlock.str();
  vlang->endSwitch(Out.indent(6));
  vlang->alwaysEnd(Out, 2);

  vlang->endModule(Out);

  return false;
}

void RTLWriter::clear() {
  ModDecl.str().clear();

  StateDecl.str().clear();

  SignalDecl.str().clear();

  DataPath.str().clear();

  ControlBlock.str().clear();

  ResetBlock.str().clear();
}

void RTLWriter::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<VLang>();
  AU.addRequired<TargetData>();
  AU.setPreservesAll();
}

void RTLWriter::print(raw_ostream &O, const Module *M) const {

}

void RTLWriter::emitFunctionSignature(const Function &F) {
  const AttrListPtr &PAL = F.getAttributes();
  unsigned Idx = 1;
  for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();
    I != E; ++I) {
      // integers:
      const Type *ArgTy = I->getType();
      if (ArgTy->isPointerTy()) {
        // Get the pointer bit witdh, only print the input parameter now.
        unsigned PtrBitWitdh = TD->getPointerSizeInBits();
        getModDeclBuffer()
          << VLang::printType(IntegerType::get(F.getContext(), PtrBitWitdh),
                              false,
                              vlang->GetValueName(I), "wire ", "input ")
          << ",\n";
      } else if(ArgTy->isIntegerTy()) {
        getModDeclBuffer() <<
          VLang::printType(ArgTy, PAL.paramHasAttr(Idx, Attribute::SExt),
                           vlang->GetValueName(I), "wire ", "input ") << ",\n";
        ++Idx;
      } else {
        vlang->comment(getModDeclBuffer()) << "Unsopport Type\n";
      }
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
    vlang->resetRegister(ResetBlock, "return_value",
                         cast<IntegerType>(RetTy)->getBitWidth());
  }
}


void RTLWriter::emitBasicBlock(BasicBlock &BB) {
  HWAState &State = HI->getStateFor(BB);
  std::string StateName = vlang->GetValueName(&BB);
  
  unsigned totalStatesBits = HI->getTotalCycleBitWidth();
  vlang->comment(getStateDeclBuffer()) << "State for " << StateName << '\n';

  //
  HWAState::ScheduleMapType Atoms;
  typedef HWAState::ScheduleMapType::iterator cycle_iterator;

  State.getScheduleMap(Atoms);
  HWAOpInst *End = State.getStateEnd();

  unsigned StartSlot = State.getSlot(), EndSlot = End->getSlot();
  //
  vlang->comment(ControlBlock.indent(6)) << StateName << '\n';
  for (unsigned i = State.getSlot(), e = End->getSlot() + 1; i != e; ++i) {
    vlang->param(getStateDeclBuffer(),
                 StateName + utostr(i),
                 totalStatesBits, i);
    // Case begin
    vlang->matchCase(ControlBlock.indent(6), StateName + utostr(i));

    // Emit all atoms at cycle i
    for (cycle_iterator CI = Atoms.lower_bound(i), CE = Atoms.upper_bound(i);
        CI != CE; ++CI) {
      HWAtom *A = CI->second;
      switch (A->getHWAtomType()) {
      case atomRegister:
        // Emit the origin IR as comment.
        vlang->comment(ControlBlock.indent(8)) << "Write Result to Reg: "
          << A->getValue() << '\n';
        emitRegister(cast<HWARegister>(A));
        break;
      case atomOpRes:
        // Emit the origin IR as comment.
        vlang->comment(ControlBlock.indent(8)) << A->getValue() << '\n';
        emitOpRes(cast<HWAOpRes>(A));
        break;
      case atomOpInst:
        // Emit the origin IR as comment.
        vlang->comment(ControlBlock.indent(8)) << A->getValue() << '\n';
        emitOpInst(cast<HWAOpInst>(A));
        break;
      case atomSignedPrefix:
        emitSigned(cast<HWASigned>(A));
        break;
      default:
        ;
      }
    }

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

void RTLWriter::emitResources(HWResource &Resource) {
  assert(!Resource.isInfinite() && "Resource expect to be finite!");
  switch (Resource.getResourceType()) {
  case MemoryBus:
    emitMemBus(cast<HWMemBus>(Resource));
  default:
    return;
  }
}

void RTLWriter::emitMemBus(HWMemBus &MemBus) {
  unsigned DataWidth = MemBus.getDataWidth(),
           AddrWidth = MemBus.getAddrWidth();
  // TODO: only emit the used ports
  for (unsigned i = 1, e = MemBus.getTotalRes() + 1; i != e; ++i) {
    ModDecl << '\n';
    vlang->comment(getModDeclBuffer()) << "Memory bus " << i << '\n';
    getModDeclBuffer()
      << "input wire [" << (DataWidth-1) << ":0] membus_out" << i <<",\n";

    getModDeclBuffer()
      << "output reg [" << (DataWidth - 1) << ":0] membus_in" << i << ",\n";
    vlang->resetRegister(getResetBlockBuffer(), "membus_in" + utostr(i),
                        DataWidth);
  
    getModDeclBuffer()
      << "output reg [" << (AddrWidth - 1) <<":0] membus_addr"<< i << ",\n";
    vlang->resetRegister(getResetBlockBuffer(), "membus_addr" + utostr(i),
                        AddrWidth);

    getModDeclBuffer() << "output reg membus_mode" << i << ",\n";
    vlang->resetRegister(getResetBlockBuffer(), "membus_mode" + utostr(i),
                        1);
  }
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
std::string RTLWriter::getAsOperand(Value *V) {
  // Phi node is a register.
  //if (PHINode *PHI = dyn_cast<PHINode>(V))
  //  return vlang->GetValueName(PHI) + "_phi_r";

  return getAsOperand(HI->getAtomFor(*V));
}
std::string RTLWriter::getAsOperand(HWAtom *A) {
  Value *V = &A->getValue();
  switch (A->getHWAtomType()) {
    case atomRegister:
      return vlang->GetValueName(&A->getValue()) + "_r";
    case atomOpRes:
    case atomOpInst:
      // Phi node is a register.
      if (PHINode *PHI = dyn_cast<PHINode>(V))
        return vlang->GetValueName(PHI) + "_phi_r";

      return vlang->GetValueName(&A->getValue()) + "_w";
    case atomConst: {
      if (ConstantInt *CI = dyn_cast<ConstantInt>(V))
        return vlang->printConstant(CI);

      if (Argument *Arg = dyn_cast<Argument>(V))
        return vlang->GetValueName(V);

      llvm_unreachable("Broken Constant atom find!");
      return "<Unkown Constant>";
    }
    case atomSignedPrefix:
      return vlang->GetValueName(&A->getValue()) + "_signed_w";
    default:
      return "<Unknown Atom>";
  }
}


void RTLWriter::emitSigned(HWASigned *Signed) {
  std::string Name = getAsOperand(Signed);
  Value &V = Signed->getValue();

  // TODO: move this to a function
  unsigned BitWidth = 0;
  if (const IntegerType *IntTy = dyn_cast<IntegerType>(V.getType()))
    BitWidth = IntTy->getBitWidth();
  else if (V.getType()->isPointerTy())
    BitWidth =  TD->getPointerSizeInBits();
  // Declare the signal
  vlang->declSignal(getSignalDeclBuffer(), Name, BitWidth, 0, false, true);
  DataPath.indent(2) << "assign " <<
    Name << " = " << getAsOperand(Signed->getDep(0)) << ";\n";
}

void RTLWriter::emitRegister(HWARegister *Register) {
  HWAtom *Val = Register->getDep(0);
  // Do not emit the dump register
  if (isa<HWAState>(Val))
    return;

  Instruction &Inst = cast<Instruction>(Register->getValue());
  unsigned BitWidth = 0;
  if (const IntegerType *IntTy = dyn_cast<IntegerType>(Inst.getType()))
    BitWidth = IntTy->getBitWidth();
  else if (Inst.getType()->isPointerTy())
    BitWidth =  TD->getPointerSizeInBits();
  
  std::string Name = getAsOperand(Register);

  // Declare the register
  vlang->declSignal(getSignalDeclBuffer(), Name, BitWidth, 0);
  // Reset the register
  vlang->resetRegister(getResetBlockBuffer(), Name, BitWidth, 0);

  // assign the register
  ControlBlock.indent(8) << Name << " <= " << getAsOperand(Val) << ";\n";
}

void RTLWriter::emitOpInst(HWAOpInst *OpInst) {
  Instruction &Inst = cast<Instruction>(OpInst->getValue());
  std::string Name = getAsOperand(OpInst);
  unsigned BitWidth = 0;
  if (const IntegerType *IntTy = dyn_cast<IntegerType>(Inst.getType()))
    BitWidth = IntTy->getBitWidth();
  else if (Inst.getType()->isPointerTy())
    BitWidth =  TD->getPointerSizeInBits();

  // Declare the signal
  vlang->declSignal(getSignalDeclBuffer(), Name, BitWidth, 0, false);
  //
  DataPath.indent(2) << "assign " << Name << " = ";
  // Emit the data path
  visit(*OpInst);
  //
  DataPath << "\n";
}

void RTLWriter::emitOpRes(HWAOpRes *OpRes) {

}

//===----------------------------------------------------------------------===//
void RTLWriter::emitNextState(raw_ostream &ss, BasicBlock &BB, unsigned offset) {
  HWAState &State = HI->getStateFor(BB);
  unsigned stateCycle = State.getSlot() + offset;
  assert(stateCycle <= State.getStateEnd()->getSlot() 
         && "Offest out of range!");
  ss << "CurState <= " << vlang->GetValueName(&BB) << stateCycle << ";\n";
}

//===----------------------------------------------------------------------===//
// Emit instructions
void RTLWriter::visitICmpInst(HWAOpInst &A) {
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


void RTLWriter::visitPHINode(HWAOpInst &A) {
  PHINode &I = A.getInst<PHINode>();//A.getInst<PHINode>();
  unsigned BitWidth = 0;
  if (const IntegerType *IntTy = dyn_cast<IntegerType>(I.getType()))
    BitWidth = IntTy->getBitWidth();
  else if (I.getType()->isPointerTy())
    BitWidth =  TD->getPointerSizeInBits();

  std::string Name = getAsOperand(&A);

  // Declare the register
  vlang->declSignal(getSignalDeclBuffer(), Name, BitWidth, 0);
  // Reset the register
  vlang->resetRegister(getResetBlockBuffer(), Name, BitWidth, 0);
  // Phi node will be assign at terminate state
}

void RTLWriter::visitExtInst(HWAOpInst &A) {
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

  DataPath <<"}}," << getAsOperand(&A) << "}" <<";\n";   
}


void esyn::RTLWriter::visitReturnInst(HWAOpInst &A) {
  ControlBlock.indent(8) << "fin <= 1'h0;\n";
  ControlBlock.indent(8) << "CurState <= state_idle\n";
}

void RTLWriter::visitTruncInst(HWAOpInst &A) {
  TruncInst &I = A.getInst<TruncInst>();
  const IntegerType *Ty = cast<IntegerType>(I.getType());
  DataPath << getAsOperand(A.getOperand(0)) << vlang->printBitWitdh(Ty, 0, true) << "\n";
}


void RTLWriter::visitBranchInst(HWAOpInst &A) {
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
    ControlBlock.indent(8 + ind) << getAsOperand(I)
      << " <= " << getAsOperand(IV) << ";\n";      
    ++I;
  }
}
