//==- ClangIfCodegen.cpp - The base class for c interface code generation --==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file define the ClangIfCodegen pass, which is the common base of the
// interface code generation pass.
//
//===----------------------------------------------------------------------===//

#include "ClangIfCodegen.h"
#include "vtm/SynSettings.h"
#include "vtm/VerilogAST.h"
#include "vtm/Utilities.h"

#include "llvm/Function.h"
#include "llvm/Type.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

std::string ClangIfCodegen::GetValueName(const Value *Operand) {
  std::string Name = Operand->getName();
  assert(!Name.empty() && "Unexpected empty name!");
  return VBEMangle(Name);
}

raw_ostream &ClangIfCodegen::printSimpleType(raw_ostream &Out, const Type *Ty,
                                             bool isSigned,
                                             const std::string &NameSoFar) {
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

raw_ostream &ClangIfCodegen::printType(raw_ostream &Out, const Type *Ty,
                                       bool isSigned, const std::string &NameSoFar,
                                       bool IgnoreName, const AttrListPtr &PAL) {
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

void ClangIfCodegen::printFunctionSignature(raw_ostream &Out,
                                            const Function *F) {
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
  FunctionInnards << GetValueName(F) << SynSettings::getIfPostfix() << '(';

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

void ClangIfCodegen::printConstant(raw_ostream &Out, Constant *CPV,
                                   bool Static) {
  if (isa<UndefValue>(CPV) && CPV->getType()->isSingleValueType()) {
    Out << "((";
    printType(Out, CPV->getType()); // sign doesn't matter
    Out << ")/*UNDEF*/";
    if (!CPV->getType()->isVectorTy()) {
      Out << "0)";
    } else {
      Out << "{})";
    }
    return;
  }

  if (ConstantInt *CI = dyn_cast<ConstantInt>(CPV)) {
    const Type* Ty = CI->getType();
    if (Ty == Type::getInt1Ty(CPV->getContext()))
      Out << (CI->getZExtValue() ? '1' : '0');
    else if (Ty == Type::getInt32Ty(CPV->getContext()))
      Out << CI->getZExtValue() << 'u';
    else if (Ty->getPrimitiveSizeInBits() > 32)
      Out << CI->getZExtValue() << "ull";
    else {
      Out << "((";
      printSimpleType(Out, Ty, false) << ')';
      if (CI->isMinValue(true))
        Out << CI->getZExtValue() << 'u';
      else
        Out << CI->getSExtValue();
      Out << ')';
    }
    return;
  }

  switch (CPV->getType()->getTypeID()) {
  case Type::ArrayTyID:
    // Use C99 compound expression literal initializer syntax.
    if (!Static) {
      Out << "(";
      printType(Out, CPV->getType());
      Out << ")";
    }
    //Out << "{ "; // Arrays are wrapped in struct types.
    if (ConstantArray *CA = dyn_cast<ConstantArray>(CPV)) {
      printConstantArray(Out, CA, Static);
    } else {
      assert(isa<ConstantAggregateZero>(CPV) || isa<UndefValue>(CPV));
      const ArrayType *AT = cast<ArrayType>(CPV->getType());
      Out << '{';
      if (AT->getNumElements()) {
        Out << ' ';
        Constant *CZ = Constant::getNullValue(AT->getElementType());
        printConstant(Out, CZ, Static);
        for (unsigned i = 1, e = AT->getNumElements(); i != e; ++i) {
          Out << ", ";
          printConstant(Out, CZ, Static);
        }
      }
      Out << " }";
    }
    // Out << " }"; // Arrays are wrapped in struct types.
    break;
  default:
#ifndef NDEBUG
    errs() << "Unknown constant type: " << *CPV << "\n";
#endif
    llvm_unreachable(0);
  }
}

void ClangIfCodegen::printConstantArray(raw_ostream &Out, ConstantArray *CPA,
                                        bool Static) {
  // As a special case, print the array as a string if it is an array of
  // ubytes or an array of sbytes with positive values.
  //
  const Type *ETy = CPA->getType()->getElementType();
  bool isString = (ETy == Type::getInt8Ty(CPA->getContext()) ||
    ETy == Type::getInt8Ty(CPA->getContext()));

  // Make sure the last character is a null char, as automatically added by C
  if (isString && (CPA->getNumOperands() == 0 ||
    !cast<Constant>(*(CPA->op_end()-1))->isNullValue()))
    isString = false;

  if (isString) {
    Out << '\"';
    // Keep track of whether the last number was a hexadecimal escape
    bool LastWasHex = false;

    // Do not include the last character, which we know is null
    for (unsigned i = 0, e = CPA->getNumOperands()-1; i != e; ++i) {
      unsigned char C = cast<ConstantInt>(CPA->getOperand(i))->getZExtValue();

      // Print it out literally if it is a printable character.  The only thing
      // to be careful about is when the last letter output was a hex escape
      // code, in which case we have to be careful not to print out hex digits
      // explicitly (the C compiler thinks it is a continuation of the previous
      // character, sheesh...)
      //
      if (isprint(C) && (!LastWasHex || !isxdigit(C))) {
        LastWasHex = false;
        if (C == '"' || C == '\\')
          Out << "\\" << (char)C;
        else
          Out << (char)C;
      } else {
        LastWasHex = false;
        switch (C) {
        case '\n': Out << "\\n"; break;
        case '\t': Out << "\\t"; break;
        case '\r': Out << "\\r"; break;
        case '\v': Out << "\\v"; break;
        case '\a': Out << "\\a"; break;
        case '\"': Out << "\\\""; break;
        case '\'': Out << "\\\'"; break;
        default:
          Out << "\\x";
          Out << (char)(( C/16  < 10) ? ( C/16 +'0') : ( C/16 -10+'A'));
          Out << (char)(((C&15) < 10) ? ((C&15)+'0') : ((C&15)-10+'A'));
          LastWasHex = true;
          break;
        }
      }
    }
    Out << '\"';
  } else {
    Out << '{';
    if (CPA->getNumOperands()) {
      Out << ' ';
      printConstant(Out, cast<Constant>(CPA->getOperand(0)), Static);
      for (unsigned i = 1, e = CPA->getNumOperands(); i != e; ++i) {
        Out << ", ";
        printConstant(Out, cast<Constant>(CPA->getOperand(i)), Static);
      }
    }
    Out << " }";
  }
}
