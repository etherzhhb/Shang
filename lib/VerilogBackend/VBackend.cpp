/*
* Copyright: 2008 by Nadav Rotem. all rights reserved.
* IMPORTANT: This software is supplied to you by Nadav Rotem in consideration
* of your agreement to the following terms, and your use, installation, 
* modification or redistribution of this software constitutes acceptance
* of these terms.  If you do not agree with these terms, please do not use, 
* install, modify or redistribute this software. You may not redistribute, 
* install copy or modify this software without written permission from 
* Nadav Rotem. 
*/
#include "VTargetMachine.h"
#include "llvm/CallingConv.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Module.h"
#include "llvm/Instructions.h"
#include "llvm/Pass.h"
#include "llvm/PassManager.h"
#include "llvm/TypeSymbolTable.h"
#include "llvm/Intrinsics.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/InlineAsm.h"
#include "llvm/Analysis/ConstantsScanner.h"
#include "llvm/Analysis/FindUsedTypes.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/CodeGen/IntrinsicLowering.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Target/TargetRegistry.h "
#include "llvm/Target/TargetData.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"
#include "llvm/Support/InstVisitor.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Config/config.h"
#include <algorithm>
#include <sstream>
#include "llvm/Support/raw_ostream.h"


#include "llvm/Support/Debug.h"

#include "HWAtomInfo.h"
#include "ScheduleDriver.h"
#include "RTLWriter.h"
#include "TestBenchWriter.h"
#include "vbe/ResourceConfig.h"

using namespace llvm;
using namespace esyn;

extern "C" void LLVMInitializeVerilogBackendTarget() { 
  // Register the target.
  RegisterTargetMachine<VTargetMachine> X(TheVBackendTarget);
}

//===----------------------------------------------------------------------===//
//                       External Interface declaration
//===----------------------------------------------------------------------===//

bool VTargetMachine::addPassesToEmitWholeFile(PassManager &PM,
                                              formatted_raw_ostream &Out,
                                              CodeGenFileType FileType,
                                              CodeGenOpt::Level OptLevel,
                                              bool DisableVerify) {
    if (FileType != TargetMachine::CGFT_AssemblyFile) return true;

    // Resource config
    ResourceConfig *RC = new ResourceConfig();
    PM.add(RC);
    // Add the language writer.
    PM.add(new VLang());
    //
    PM.add(new HWAtomInfo(*RC));
    PM.add(new ListScheduler());
    //
    PM.add(new RTLWriter(Out));
    PM.add(new TestbenchWriter(Out));
    return false;
}
