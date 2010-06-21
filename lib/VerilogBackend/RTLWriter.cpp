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

  emitFunctionSignature(ModDecl, vlang, F);


  // Emit resources
  for(HWAtomInfo::resource_iterator I = HI->resource_begin(),
    E = HI->resource_end();I != E; ++I) {
      HWResource &Resource = **I;
      // Ignore the infinite resources, we will emit them on the fly.
      if (Resource.isInfinite())
        continue;

      emitFiniteResources(Resource);
  }

  // Emit basicblocks
  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I) {
    BasicBlock &BB = *I;

  }

  // Finish the buffers.
  vlang->emitEndModuleDecl(ModDecl);

  // Write buffers to output
  Out << ModDecl.str()
    << StateDecl.str()
    << SignalDecl.str()
    << DataPath.str()
    << AlwaysBlock.str();

  vlang->emitEndModule(Out);

  return false;
}

void RTLWriter::clear() {
  ModDecl.str().clear();
  ModDecl << "//Design module\n";

  StateDecl.str().clear();
  StateDecl << "  // State Decl\n";

  SignalDecl.str().clear();
  SignalDecl << "  // Signal Decl\n";

  DataPath.str().clear();
  DataPath << "  // Data Path\n";

  AlwaysBlock.str().clear();
  AlwaysBlock << "  // AlwaysBlock\n";
}

void RTLWriter::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<HWAtomInfo>();
  AU.addRequired<VLang>();
  AU.addRequired<TargetData>();
  AU.setPreservesAll();
}

void RTLWriter::print(raw_ostream &O, const Module *M) const {

}

void RTLWriter::emitFunctionSignature(raw_ostream &ss, VLang *vlang,
                                       const Function &F) {
  vlang->emitModuleBegin(ss, F.getNameStr());

  vlang->indent(ss) << "input wire " << "clk" << ",\n";
  vlang->indent(ss) << "input wire " << "rstN" << ",\n";
  vlang->indent(ss) << "input wire " << "start" << ",\n";

  const AttrListPtr &PAL = F.getAttributes();
  unsigned Idx = 1;
  for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();
    I != E; ++I) {
      // integers:
      const Type *ArgTy = I->getType();
      if (ArgTy->isPointerTy()) {
        // Get the pointer bit witdh, only print the input parameter now.
        unsigned PtrBitWitdh = TD->getPointerSizeInBits();
        vlang->indent(ss) 
          << VLang::printType(IntegerType::get(F.getContext(), PtrBitWitdh),
                              false,
                              vlang->GetValueName(I), "wire ", "input ")
          << ",\n";
      } else if(ArgTy->isIntegerTy()) {
        vlang->indent(ss) <<
          VLang::printType(ArgTy,
          /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt),
          vlang->GetValueName(I), "wire ", "input ") << ",\n";
        ++Idx;
      } else {
        vlang->emitCommentBegin(ss) << "Unsopport Type\n";
      }
  }

  // Ready signal

  vlang->indent(ss) << "output reg " << "rdy";
  const Type *RetTy = F.getReturnType();
  if (RetTy->isVoidTy()) {
    // Do something?
    //vlang->indent(ss) << "/*return void*/";
  } else {
    // End rdy declare.
    ss << ",\n";
    assert(RetTy->isIntegerTy() && "Only support return integer now!");
    vlang->indent(ss)
      << VLang::printType(RetTy, false, "return_value", "reg ", "output ");
  }
}

void RTLWriter::emitFiniteResources(HWResource &Resource) {
  assert(!Resource.isInfinite() && "Resource expect to be finite!");
  // TODO: Use getType
  if (Resource.getName() == "MemoryBus") {
    //ModDecl << 
  }
}

RTLWriter::~RTLWriter() {
  delete &(ModDecl.str());
  delete &(StateDecl.str());
  delete &(SignalDecl.str());
  delete &(DataPath.str());
  delete &(AlwaysBlock.str());
}
