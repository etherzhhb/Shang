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
#include "llvm/PassManager.h"
#include "llvm/Target/TargetRegistry.h"

#include "llvm/Support/raw_ostream.h"

#include "HWAtomInfo.h"
#include "ScheduleDriver.h"
#include "RTLWriter.h"
#include "TestBenchWriter.h"
#include "HWAtomPasses.h"
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
    // Memory dependencies analysis
    PM.add(new HWAtomInfo());
    PM.add(createListSchedulePass());
    // Resource binding
    // Region Base global resource binding
    PM.add(createRegisterReductionPass());
    //
    PM.add(new RTLWriter(Out));
    PM.add(new TestbenchWriter(Out));
    return false;
}
