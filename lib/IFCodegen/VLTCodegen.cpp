//===----- VLTCodegen.cpp - Interface generation for Verilator --*- C++ -*-===//
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
// This file implement the class that generate the interface between the
// software part and the rtl module of the generated software.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/VerilogAST.h"
#include "vtm/VFInfo.h"
#include "vtm/LangSteam.h"
#include "vtm/Utilities.h"

#include "llvm/Function.h"
#include "llvm/Module.h"
#include "llvm/DerivedTypes.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

#include "llvm/ADT/StringExtras.h"

#include "llvm/Support/Format.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/ErrorHandling.h"

#include "llvm/Support/Debug.h"

#include <map>

using namespace llvm;

//===----------------------------------------------------------------------===//
// Helper functions Copy from CCodegen.cpp to help printing functions.
static std::string GetValueName(const Value *Operand) {
  std::string Name = Operand->getName();

  assert(!Name.empty() && "Unexpected empty name!");

  std::string VarName;
  VarName.reserve(Name.capacity());

  for (std::string::iterator I = Name.begin(), E = Name.end();
       I != E; ++I) {
    char ch = *I;

    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
      (ch >= '0' && ch <= '9') || ch == '_')) {
        char buffer[5];
        sprintf(buffer, "_%x_", ch);
        VarName += buffer;
    } else
      VarName += ch;
  }

  return VarName;
}

static
raw_ostream &printSimpleType(raw_ostream &Out, const Type *Ty, bool isSigned,
                             const std::string &NameSoFar = "") {
  assert((Ty->isPrimitiveType() || Ty->isIntegerTy() || Ty->isVectorTy()) &&
         "Invalid type for printSimpleType");
  switch (Ty->getTypeID()) {
  case Type::VoidTyID:   return Out << "void " << NameSoFar;
  case Type::IntegerTyID: {
    unsigned NumBits = cast<IntegerType>(Ty)->getBitWidth();
    if (NumBits == 1)
      return Out << "bool " << NameSoFar;
    else if (NumBits <= 8)
      return Out << (isSigned?"signed":"unsigned") << " char " << NameSoFar;
    else if (NumBits <= 16)
      return Out << (isSigned?"signed":"unsigned") << " short " << NameSoFar;
    else if (NumBits <= 32)
      return Out << (isSigned?"signed":"unsigned") << " int " << NameSoFar;
    else if (NumBits <= 64)
      return Out << (isSigned?"signed":"unsigned") << " long long "<< NameSoFar;
    else {
      assert(NumBits <= 128 && "Bit widths > 128 not implemented yet");
      return Out << (isSigned?"llvmInt128":"llvmUInt128") << " " << NameSoFar;
    }
  }
  case Type::FloatTyID:  return Out << "float "   << NameSoFar;
  case Type::DoubleTyID: return Out << "double "  << NameSoFar;
  // Lacking emulation of FP80 on PPC, etc., we assume whichever of these is
  // present matches host 'long double'.
  case Type::X86_FP80TyID:
  case Type::PPC_FP128TyID:
  case Type::FP128TyID:  return Out << "long double " << NameSoFar;

  case Type::X86_MMXTyID:
    return printSimpleType(Out, Type::getInt32Ty(Ty->getContext()), isSigned,
                     " __attribute__((vector_size(64))) " + NameSoFar);

  case Type::VectorTyID: {
    assert(0 && "Unsupported Type!");
    return Out << "Bad type!";
    //const VectorType *VTy = cast<VectorType>(Ty);
    //return printSimpleType(Out, VTy->getElementType(), isSigned,
    //                 " __attribute__((vector_size(" +
    //                 utostr(TD->getTypeAllocSize(VTy)) + " ))) " + NameSoFar);
  }

  default:
#ifndef NDEBUG
    errs() << "Unknown primitive type: " << *Ty << "\n";
#endif
    llvm_unreachable(0);
  }
}

// Pass the Type* and the variable name and this prints out the variable
// declaration.
//
static raw_ostream &printType(raw_ostream &Out, const Type *Ty,
                              bool isSigned = false,
                              const std::string &NameSoFar = "",
                              bool IgnoreName = false,
                              const AttrListPtr &PAL = AttrListPtr()) {
  if (Ty->isPrimitiveType() || Ty->isIntegerTy() || Ty->isVectorTy()) {
    printSimpleType(Out, Ty, isSigned, NameSoFar);
    return Out;
  }

  // Check to see if the type is named.
  //if (!IgnoreName || Ty->isOpaqueTy()) {
  //  assert(0 && "Unsupported Type!");
  //  return Out << "Bad type!";
  //}

  switch (Ty->getTypeID()) {
  case Type::FunctionTyID: {
    const FunctionType *FTy = cast<FunctionType>(Ty);
    std::string tstr;
    raw_string_ostream FunctionInnards(tstr);
    FunctionInnards << " (" << NameSoFar << ") (";
    unsigned Idx = 1;
    for (FunctionType::param_iterator I = FTy->param_begin(),
           E = FTy->param_end(); I != E; ++I) {
      const Type *ArgTy = *I;
      if (PAL.paramHasAttr(Idx, Attribute::ByVal)) {
        assert(ArgTy->isPointerTy());
        ArgTy = cast<PointerType>(ArgTy)->getElementType();
      }
      if (I != FTy->param_begin())
        FunctionInnards << ", ";
      printType(FunctionInnards, ArgTy,
        /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt), "");
      ++Idx;
    }
    if (FTy->isVarArg()) {
      if (!FTy->getNumParams())
        FunctionInnards << " int"; //dummy argument for empty vaarg functs
      FunctionInnards << ", ...";
    } else if (!FTy->getNumParams()) {
      FunctionInnards << "void";
    }
    FunctionInnards << ')';
    printType(Out, FTy->getReturnType(),
      /*isSigned=*/PAL.paramHasAttr(0, Attribute::SExt), FunctionInnards.str());
    return Out;
  }
  case Type::StructTyID: {
    const StructType *STy = cast<StructType>(Ty);
    Out << NameSoFar + " {\n";
    unsigned Idx = 0;
    for (StructType::element_iterator I = STy->element_begin(),
           E = STy->element_end(); I != E; ++I) {
      Out << "  ";
      printType(Out, *I, false, "field" + utostr(Idx++));
      Out << ";\n";
    }
    Out << '}';
    if (STy->isPacked())
      Out << " __attribute__ ((packed))";
    return Out;
  }

  case Type::PointerTyID: {
    const PointerType *PTy = cast<PointerType>(Ty);
    std::string ptrName = "*" + NameSoFar;

    //if (PTy->getElementType()->isArrayTy() ||
     //   PTy->getElementType()->isVectorTy())
      //ptrName = "(" + ptrName + ")";

    if (!PAL.isEmpty())
      // Must be a function ptr cast!
      return printType(Out, PTy->getElementType(), false, ptrName, true, PAL);
    return printType(Out, PTy->getElementType(), false, ptrName);
  }

  case Type::ArrayTyID: {
    const ArrayType *ATy = cast<ArrayType>(Ty);
    unsigned NumElements = ATy->getNumElements();
    if (NumElements == 0) NumElements = 1;
    // Arrays are wrapped in structs to allow them to have normal
    // value semantics (avoiding the array "decay").
    //Out << NameSoFar << " { ";
    printType(Out, ATy->getElementType(), false,
              NameSoFar+"[" + utostr(NumElements) + "]");
    return Out;// << "; }";
  }

  case Type::OpaqueTyID: {
    assert(0 && "Unsupported Type!");
    return Out << "Bad type!";
  }
  default:
    llvm_unreachable("Unhandled case in getTypeProps!");
  }

  return Out;
}

static void printFunctionSignature(raw_ostream &Out, const Function *F) {
  /// isStructReturn - Should this function actually return a struct by-value?
  bool isStructReturn = F->hasStructRetAttr();

  if (F->hasLocalLinkage()) Out << "static ";
  if (F->hasDLLImportLinkage()) Out << "__declspec(dllimport) ";
  if (F->hasDLLExportLinkage()) Out << "__declspec(dllexport) ";
  switch (F->getCallingConv()) {
   case CallingConv::X86_StdCall:
     Out << "__attribute__((stdcall)) ";
     break;
   case CallingConv::X86_FastCall:
     Out << "__attribute__((fastcall)) ";
     break;
   case CallingConv::X86_ThisCall:
     Out << "__attribute__((thiscall)) ";
     break;
   default:
     break;
  }

  // Loop over the arguments, printing them...
  const FunctionType *FT = cast<FunctionType>(F->getFunctionType());
  const AttrListPtr &PAL = F->getAttributes();

  std::string tstr;
  raw_string_ostream FunctionInnards(tstr);

  // Print out the name...
  FunctionInnards << GetValueName(F) << '(';

  bool PrintedArg = false;
  if (!F->isDeclaration()) {
    if (!F->arg_empty()) {
      Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
      unsigned Idx = 1;

      // If this is a struct-return function, don't print the hidden
      // struct-return argument.
      if (isStructReturn) {
        assert(I != E && "Invalid struct return function!");
        ++I;
        ++Idx;
      }

      std::string ArgName;
      for (; I != E; ++I) {
        if (PrintedArg) FunctionInnards << ", ";
        
        ArgName = GetValueName(I);

        const Type *ArgTy = I->getType();
        if (PAL.paramHasAttr(Idx, Attribute::ByVal)) {
          ArgTy = cast<PointerType>(ArgTy)->getElementType();
          //ByValParams.insert(I);
        }
        printType(FunctionInnards, ArgTy,
          /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt),
          ArgName);
        PrintedArg = true;
        ++Idx;
      }
    }
  } else {
    // Loop over the arguments, printing them.
    FunctionType::param_iterator I = FT->param_begin(), E = FT->param_end();
    unsigned Idx = 1;

    // If this is a struct-return function, don't print the hidden
    // struct-return argument.
    if (isStructReturn) {
      assert(I != E && "Invalid struct return function!");
      ++I;
      ++Idx;
    }

    for (; I != E; ++I) {
      if (PrintedArg) FunctionInnards << ", ";
      const Type *ArgTy = *I;
      if (PAL.paramHasAttr(Idx, Attribute::ByVal)) {
        assert(ArgTy->isPointerTy());
        ArgTy = cast<PointerType>(ArgTy)->getElementType();
      }
      printType(FunctionInnards, ArgTy,
        /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt));
      PrintedArg = true;
      ++Idx;
    }
  }

  if (!PrintedArg && FT->isVarArg()) {
    FunctionInnards << "int vararg_dummy_arg";
    PrintedArg = true;
  }

  // Finish printing arguments... if this is a vararg function, print the ...,
  // unless there are no known types, in which case, we just emit ().
  //
  if (FT->isVarArg() && PrintedArg) {
    FunctionInnards << ",...";  // Output varargs portion of signature!
  } else if (!FT->isVarArg() && !PrintedArg) {
    FunctionInnards << "void"; // ret() -> ret(void) in C.
  }
  FunctionInnards << ')';

  // Get the return tpe for the function.
  const Type *RetTy;
  if (!isStructReturn)
    RetTy = F->getReturnType();
  else {
    // If this is a struct-return function, print the struct-return type.
    RetTy = cast<PointerType>(FT->getParamType(0))->getElementType();
  }

  // Print out the return type and the signature built above.
  printType(Out, RetTy,
    /*isSigned=*/PAL.paramHasAttr(0, Attribute::SExt),
    FunctionInnards.str());
}

//===----------------------------------------------------------------------===//
// Verilator interface writer.
namespace {
struct VLTIfCodegen : public MachineFunctionPass {
  static char ID;

  lang_raw_ostream<CppTraits> Out;

  std::string VLTClassName, VLTModInstName;
  VASTModule *RTLMod;
  const VFInfo *FInfo;

  VLTIfCodegen(raw_ostream &O) : MachineFunctionPass(ID), Out(O),
    RTLMod(0), FInfo(0) {}

  void assignInPort(unsigned T, const std::string &Val) {
    // TODO: Assert the port must be an input port.
    Out << VLTModInstName << '.'
           << RTLMod->getPortName(T)
           << " = (" << Val << ");\n";
  }

  void assignInPort(unsigned T, uint64_t Val, bool isNeg = false) {
    assignInPort(T, utostr(Val, isNeg));
  }

  std::string getModMember(const std::string &MemberName) const {
    return "(" + VLTModInstName + "." + MemberName + ")";
  }

  std::string getPortVal(unsigned T) const {
    return getModMember(RTLMod->getPortName(T));
  }

  void evalHalfCycle() {
    Out << "//Increase clk by half cycle.\n";
    assignInPort(VASTModule::Clk, "sim_time++ & 0x1");

    Out << "// Evaluate model.\n";
    Out << VLTModInstName << ".eval();\n";
  }

  const char *getCurClk() const {
    // We can simply read the clk value from sim_time, but the sim_time
    // is half cycle faster than the current clock.
    return "( (~sim_time) & 0x1)";
  }

  void isClkEdgeBegin(bool posedge) {
    Out << "// Check for if the clk "
           << (posedge ? "rising" : "falling")
           << '\n';

    Out.if_begin(format("%s == %c", getCurClk(), '0' + posedge));
  }

  void isClkEdgeEnd(bool posedge) {
    if (posedge)
      Out.exit_block("// end clk rising\n");
    else
      Out.exit_block("// end clk falling\n");
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    MachineFunctionPass::getAnalysisUsage(AU);
    AU.setPreservesAll();
  }

  bool doInitialization(Module &M) {
    for (Module::iterator I = M.begin(), E = M.end(); I != E; ++I) {
      if (I->empty()) continue;

      SynSettings *Settings = getSynSetting(I->getName());
      assert(Settings
        && "Synthesis settings of a hardware function not available!");
      // Ignore submodules.
      if (!Settings->isTopLevelModule()) continue;

      std::string RTLModName = Settings->getModName();

      // Setup the Name of the module in verilator.
      VLTClassName = "V" + RTLModName;
      // And the name of the intance of this class.
      VLTModInstName = VLTClassName + "_Inst";

      Out << "// Include the verilator header.\n"
            "#include \"verilated.h\"\n"
            "// And the header file of the generated module.\n"
            "#include \"" << VLTClassName << ".h\"\n\n\n"
            "// Instantiation of module\n"
            "static " << VLTClassName << " " << VLTModInstName
            << "(\"" << RTLModName << "\");\n\n\n";
    }

    Out << "// Current simulation time\n"
              "static long sim_time = 0;\n\n\n"
              "// Called by $time in Verilog\n"
              "double sc_time_stamp () {\n"
              "  return sim_time;\n"
              "}\n\n\n"
              "// Dirty Hack: Only C function is supported now.\n"
              "#ifdef __cplusplus\n"
              "extern \"C\" {\n"
              "#endif\n\n";

    for (Module::global_iterator GI = M.global_begin(), E = M.global_end();
         GI != E; ++GI ){
      GlobalVariable *GV = GI;
      const Type *Ty = cast<PointerType>(GV->getType())->getElementType();
      if (GV->hasLocalLinkage())
        Out << "static ";
      else
        Out << "extern ";

      printType(Out, Ty, false, VBEMangle(GV->getName())) << ";\n";
      // TODO: The initializer.

      Out << "void *verilator_get_gv"
          << VBEMangle(GV->getNameStr())<<"() {\n";
      Out << "  return (void *)";
      // Take the address for non-array type.
      if (!Ty->isArrayTy()) Out << '&';
      Out << VBEMangle(GV->getNameStr())<<";\n"
             "}\n\n\n";
    }

    Out.flush();

    return false;
  }

  bool doFinalization(Module &M) {
    Out << "// Dirty Hack: Only C function is supported now.\n"
              "#ifdef __cplusplus\n"
              "}\n"
              "#endif\n";
    return false;
  }

  bool runOnMachineFunction(MachineFunction &MF);

  template<bool SimRead>
  void simulateMemBusImpl(unsigned MemPortStartIdx, LLVMContext &Context) {
    unsigned Address  = MemPortStartIdx + 2;
    unsigned InData   = MemPortStartIdx + 3;
    unsigned OutData  = MemPortStartIdx + 4;
    unsigned ByteEnable = MemPortStartIdx + 5;

    // Compute the membus data size (in bytes).
    // FIXME: Every membus may have different data size.
    VFUMemBus *MemBusDesc = getFUDesc<VFUMemBus>();
    unsigned DataPortBytes = MemBusDesc->getDataWidth() / 8;

    // Simulate the read operation on memory bus.
    Out << "switch (" << getPortVal(ByteEnable) << " & 0xff)";
    Out.enter_block();

    for (unsigned Size = 1, EndSize = (DataPortBytes * 2);
         Size < EndSize; Size *= 2) {
      Out << "case " << getByteEnable(Size) << ": ";
      // Write the value to input port from memory if this is a read.
      if (SimRead)
        Out << getPortVal(InData) << " = ";
      
      // Cast the pointer and dereference it.
      // The size is in bytes, convert it to bits.
      Out << "*((";
      printSimpleType(Out, Type::getIntNTy(Context, Size * 8), false);
      Out << "*)" << getPortVal(Address) << ")";

      // Other Write the value from output port to memory.
      if (!SimRead) {
        Out << " = ((";
        printSimpleType(Out, Type::getIntNTy(Context, Size * 8), false);
        Out << ") " << getPortVal(OutData) << ")";
      }

      Out << "; break;\n";
    }
    Out << "default: assert(0 && \"Unsupported size!\"); break;\n";
    Out.exit_block();
  }

  inline void simulateMemRead(unsigned MemPortStartIdx, LLVMContext &Context) {
    simulateMemBusImpl<true>(MemPortStartIdx, Context);
  }

  inline void simulateMemWrite(unsigned MemPortStartIdx, LLVMContext &Context) {
    simulateMemBusImpl<false>(MemPortStartIdx, Context);
  }
};
} //end anonymous namespace

Pass *llvm::createVLTIfCodegenPass(raw_ostream &O) {
  return new VLTIfCodegen(O);
}

char VLTIfCodegen::ID = 0;

bool VLTIfCodegen::runOnMachineFunction(MachineFunction &MF) {
  const Function *F = MF.getFunction();
  // Only Generate the interface for top level modules.
  if (!getSynSetting(F->getName())->isTopLevelModule())
    return false;

  FInfo = MF.getInfo<VFInfo>();

  RTLMod = FInfo->getRtlMod();
  TargetData *TD = getAnalysisIfAvailable<TargetData>();
  assert(TD && "Where is TD?");
  LLVMContext &Context = F->getContext();

  // Print the interface function.
  const Type *retty = F->getReturnType();
  
  printFunctionSignature(Out, F);
  // And the function body.
  Out.enter_block();

  // TODO:
  // Verilated::commandArgs(argc, argv); // Remember args

  // Reset if necessary.
  Out << "// Reset the module if we first time invoke the module.\n";
  Out.if_begin("sim_time == 0");
  assignInPort(VASTModule::RST, 0);
  Out.exit_block();

  // Run several cycles.
  Out << '\n';
  evalHalfCycle();
  Out << "  // Deassert reset\n";
  assignInPort(VASTModule::RST, 1);
  evalHalfCycle();

  Out << '\n';
  evalHalfCycle();
  evalHalfCycle();
  Out << "assert(!" << getPortVal(VASTModule::Finish)
         << "&& \"Module finished before start!\");\n";

  // Setup the parameters.
  Out << '\n';
  Out<< "// Setup the parameters.\n";
  for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
       I != E; ++I) {
    std::string ArgName = GetValueName(I);
    Out << VLTModInstName << '.' << ArgName << " = ";
    // Cast the argument to integer type if necessary.
    const Type *ArgTy = I->getType();
    if (ArgTy->isPointerTy()) {
      Out << '(';
      printType(Out, TD->getIntPtrType(Context));
      Out << ')';
    }

    Out << ArgName << ";\n";
      
    Out << "#ifdef __DEBUG_IF\n"
              "printf(\"Passing " << ArgName << "->%x\\n\", " << ArgName << ");\n"
              "#endif\n";
  }

  Out << '\n';
  Out << "// Start the module.\n";
  assignInPort(VASTModule::Start, 1);

  Out << "// Remember the start time.\n"
            "long start_time = sim_time;\n"
            "long ready_time = sim_time;\n";

  Out<< "// Commit the signals.\n";
  evalHalfCycle();

  // The main evaluation loop.
  Out << "\n\n\n"
            "// The main evaluation loop.\n"
            "do";
  Out.enter_block();

  // TODO: Allow the user to custom the clock edge.
  isClkEdgeBegin(true);
  // Check membuses.
  unsigned MemBusLatency = getFUDesc<VFUMemBus>()->getLatency();
  typedef VFInfo::const_id_iterator id_iterator;
  for (id_iterator I = FInfo->id_begin(VFUs::MemoryBus),
       E = FInfo->id_end(VFUs::MemoryBus); I != E; ++I) {
    FuncUnitId ID = *I;
    unsigned FUNum = ID.getFUNum();

    unsigned Enable = RTLMod->getFUPortOf(ID);

    unsigned BusRdy = Enable + 6;
    assignInPort(BusRdy, "(sim_time >= ready_time)");

    Out << "if (" << getPortVal(Enable) << ")";
    Out.enter_block(format("// If membus%d active\n", FUNum));

    unsigned Address = Enable + 2;

    Out << "#ifdef __DEBUG_IF\n";
    // Read the address
    printType(Out, TD->getIntPtrType(Context), false, "Addr");
    Out << " = " << getPortVal(Address) << ";\n"
              "printf(\"Bus active with address %x\\n\", Addr);\n"
              "#endif\n";

    unsigned WriteEnable = Enable + 1;
    Out << "if (" << getPortVal(WriteEnable) << ")";
    Out.enter_block("// This is a write\n");
    simulateMemWrite(Enable, Context);
    Out.else_begin("// This is a read\n");
    simulateMemRead(Enable, Context);
    Out.exit_block("// end read/write\n");
    Out.exit_block(format("// end membus%d\n", FUNum));

    // Dirty hack: All membus share a ready_time.
    Out << "// Simulate the ready port of the membus.\n";
    // FIXME: Use a random ready time.
    Out << "if (" << getPortVal(BusRdy) << " && " << getPortVal(Enable) << ")";
    Out.enter_block(format("// If membus%d ready for next transaction\n",FUNum));

    // FIXME: use a random uncertain delay.
    unsigned TransactionUncertain = 5;

    Out << "// Update the ready time of membus.\n"
      // Note: Simulate advance half clock cycle when sim_time increased.
      "ready_time = sim_time + " << MemBusLatency * 2
        << " + " << TransactionUncertain <<";\n";

    Out << "#ifdef __DEBUG_IF\n"
      "printf(\"Memory Active at %x going to ready at %x\\n\","
      " sim_time, ready_time);\n"
      "#endif\n";
    Out.exit_block(format("// end next transaction membus%d\n", FUNum));
  }

  isClkEdgeEnd(true);

  Out << "\n"
            "// Check if the module finish its job at last.\n";
  Out.if_begin(getPortVal(VASTModule::Finish));
  // TODO: Print the total spent cycle number if necessary.

  Out << "return";

  // If the function return something?
  if (!retty->isVoidTy()) {
    Out << " (";
    // Cast the return value, and we only support simple type as return type now.
    printSimpleType(Out, retty, false);
    // FIXME: Find the correct name for the return value port.
    Out << ")" << getModMember("return_value") << ";\n";  
  } else
    Out << ";\n";

  Out.exit_block();

  Out << '\n';

  evalHalfCycle();
  assignInPort(VASTModule::Start, 0);

  // TODO: allow user to custom the maximum simulation time.
  // FIXME: The result is wrong if sim_time just overflow.
  Out.exit_block("while(!Verilated::gotFinish()"
                    " && (sim_time - start_time) < 0x1000);\n");

  Out << '\n';
  // TODO: Allow user to custom the error handling.
  Out << "assert(0 && \"Something went wrong during the simulation!\");\n";
  if (!retty->isVoidTy()) {
    Out << "return 0;\n";
  }
  // End function body.
  Out.exit_block() << "\n";
  Out.flush();

  return false;
}
