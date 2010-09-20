##===- lib/Target/VBackend/Makefile ------------------------*- Makefile -*-===##
# 
#                     The LLVM Compiler Infrastructure
#
# This file was developed by the LLVM research group and is distributed under
# the University of Illinois Open Source License. See LICENSE.TXT for details.
# 
##===----------------------------------------------------------------------===##

LEVEL = ../../../..
LIBRARYNAME = LLVMSynthesis
TARGET=Verilog
LOADABLE_MODULE = 1
BUILT_SOURCES=../params.cpp
include $(LEVEL)/Makefile.common


CompileCommonOpts += -Wno-format 
#CXX=icc
#CC=icc
# DO NOT DELETE

abstractHWOpcode.o: abstractHWOpcode.h ../params.h globalVarsRegistry.h
designScorer.o: designScorer.h listScheduler.h ../utils.h ../params.h
designScorer.o: abstractHWOpcode.h globalVarsRegistry.h
globalVarsRegistry.o: globalVarsRegistry.h
instPriority.o: instPriority.h ../utils.h abstractHWOpcode.h ../params.h
instPriority.o: globalVarsRegistry.h
intrinsics.o: intrinsics.h
listScheduler.o: listScheduler.h ../utils.h ../params.h abstractHWOpcode.h
listScheduler.o: globalVarsRegistry.h instPriority.h
Opcodes.o: Opcodes.h ../params.h globalVarsRegistry.h
VBackend.o: VTargetMachine.h listScheduler.h ../utils.h ../params.h
VBackend.o: abstractHWOpcode.h globalVarsRegistry.h verilogLang.h
VBackend.o: designScorer.h
verilogLang.o: verilogLang.h listScheduler.h ../utils.h ../params.h
verilogLang.o: abstractHWOpcode.h globalVarsRegistry.h intrinsics.h
