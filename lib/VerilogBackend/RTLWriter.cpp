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

#include "RTLWriter.h"
#include "HWAtomInfo.h"

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
  vlang->ifBegin(ControlBlock.indent(8), "start");
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

  //
  vlang->comment(ControlBlock.indent(6)) << StateName << '\n';

  for (unsigned i = State.getSlot(), e = State.getStateEnd()->getSlot();
      i != e; ++i) {
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
        emitRegister(cast<HWARegister>(A));
        break;
      case atomOpRes:
        break;
      default:
        ;
      }
    }

    // Case end
    vlang->end(ControlBlock.indent(6));
  }
}

void RTLWriter::emitCommonPort() {
  ModDecl << '\n';
  vlang->comment(getModDeclBuffer()) << "Common ports\n";
  getModDeclBuffer() << "input wire " << "clk" << ",\n";
  getModDeclBuffer() << "input wire " << "rstN" << ",\n";
  getModDeclBuffer() << "input wire " << "start" << ",\n";
  getModDeclBuffer() << "output reg " << "rdy";
  // Reset rdy
  vlang->resetRegister(getResetBlockBuffer(), "rdy", 1);
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

void RTLWriter::emitRegister(HWARegister *Register) {
  Instruction &Inst = cast<Instruction>(Register->getValue());
  unsigned BitWidth = 0;
  if (const IntegerType *IntTy = dyn_cast<IntegerType>(Inst.getType()))
    BitWidth = IntTy->getBitWidth();
  else if (Inst.getType()->isPointerTy())
    BitWidth =  TD->getPointerSizeInBits();
  
  std::string Name = vlang->GetValueName(&Inst);

  // Declare the register
  vlang->declSignal(getSignalDeclBuffer(), Name, BitWidth, 0);
  // Reset the register
  vlang->resetRegister(getResetBlockBuffer(), Name, BitWidth, 0);
}

void RTLWriter::emitOpRes(HWAOpRes *OpRes) {
  Instruction &Inst = cast<Instruction>(OpRes->getValue());
  visit(Inst);
}

void RTLWriter::emitNextState(raw_ostream &ss, BasicBlock &BB, unsigned offset) {
  HWAState &State = HI->getStateFor(BB);
  unsigned stateCycle = State.getSlot() + offset;
  assert( stateCycle < State.getStateEnd()->getSlot() 
         && "Offest out of range!");
  ss << "CurState <= " << vlang->GetValueName(&BB) << stateCycle << ";\n";
}
