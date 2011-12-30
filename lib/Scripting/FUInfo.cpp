//===----- VFunctionUnit.cpp - VTM Function Unit Information ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains implementation of the function unit class in
// Verilog target machine.
//
//===----------------------------------------------------------------------===//

#include "vtm/FUInfo.h"
#include "vtm/SynSettings.h" // DiryHack: Also implement the SynSetting class.
#include "vtm/LuaScript.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/VInstrInfo.h"
#include "vtm/VTM.h"

#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SourceMgr.h"
#define DEBUG_TYPE "vtm-fu-info"
#include "llvm/Support/Debug.h"
using namespace llvm;

//===----------------------------------------------------------------------===//
// Helper functions for reading function unit table from script.
template<typename PropType, typename IdxType>
static PropType getProperty(luabind::object &FUTable, IdxType PropName,
  PropType DefaultRetVal = PropType()) {
    if (luabind::type(FUTable) != LUA_TTABLE) return DefaultRetVal;

    boost::optional<PropType> Result =
      luabind::object_cast_nothrow<PropType>(FUTable[PropName]);

    if (!Result) return DefaultRetVal;

    return Result.get();
}

//===----------------------------------------------------------------------===//
/// Hardware resource.
void VFUDesc::print(raw_ostream &OS) const {
}

namespace llvm {
  namespace VFUs {
    const char *VFUNames[] = {
      "Trivial", "AddSub", "Shift", "Mult", "MemoryBus", "BRam",
      "ICmp", "CalleeFN"
    };

    const TargetRegisterClass *getRepRegisterClass(unsigned OpCode){
      switch (OpCode) {
      default:                  return VTM::WireRegisterClass;
      case VTM::VOpAdd:         return VTM::RADDRegisterClass;
      case VTM::VOpSRA:         return VTM::RASRRegisterClass;
      case VTM::VOpSRL:         return VTM::RLSRRegisterClass;
      case VTM::VOpSHL:         return VTM::RSHLRegisterClass;
      case VTM::VOpMult:        return VTM::RMULRegisterClass;
      case VTM::VOpMultLoHi:    return VTM::RMULLHRegisterClass;
      case VTM::VOpCmdSeq:
      case VTM::VOpMemTrans:    return VTM::RINFRegisterClass;
      case VTM::VOpInternalCall:return VTM::RCFNRegisterClass;
      case VTM::VOpBRam:        return VTM::RBRMRegisterClass;
      // allocate unsigned comparison fu by default.
      case VTM::VOpICmp:        return VTM::RUCMPRegisterClass;
      }

      return 0;
    }

    // Default area cost parameter.
    unsigned LUTCost = 64;
    unsigned RegCost = 64;
    unsigned MUXCost = 72;
    unsigned AddCost = 64;
    unsigned MulCost = 96;
    unsigned ShiftCost = 192;
    unsigned ICmpCost = 64;
    unsigned MuxSizeCost = 96;

    unsigned MaxLutSize = 4;
    unsigned MaxMuxPreLut = 4;

    // Default value of Latency tables.         8bit 16bit 32bit 64bit
    double AdderLatencies[]     = { 1.0, 1.0,  1.0,  1.0 };
    double CmpLatencies[]       = { 1.0, 1.0,  1.0,  1.0 };
    double MultLatencies[]      = { 1.0, 1.0,  1.0,  1.0 };
    double ShiftLatencies[]     = { 1.0, 1.0,  1.0,  1.0 };
    double MemBusLatency = 1.0;
    double BRamLatency = 1.0;
    double LutLatency = 0.0;
    double ClkEnSelLatency = 0.0;

    void initLatencyTable(luabind::object LuaLatTable, double *LatTable,
                          unsigned Size) {
      for (unsigned i = 0; i < Size; ++i)
        // Lua array starts from 1
        LatTable[i] = getProperty<double>(LuaLatTable, i + 1, LatTable[i]);
    }

    double getReductionLatency(unsigned Size) {
      if (Size < 2) return 0;

      unsigned Level = ceil(double(Log2_32_Ceil(Size))
                            / double(Log2_32_Ceil(MaxLutSize)));

      return Level * LutLatency;
    }

    double getMuxLatency(unsigned Size) {
      if (Size < 2) return 0;

      unsigned Level = ceil(double(Log2_32_Ceil(Size))
                            / double(Log2_32_Ceil(MaxMuxPreLut)));

      return Level * LutLatency;
    }
  }
}

VFUDesc::VFUDesc(VFUs::FUTypes type, luabind::object FUTable, double *latencies)
  : ResourceType(type),
    StartInt(getProperty<unsigned>(FUTable, "StartInterval")),
    Cost(getProperty<unsigned>(FUTable, "Cost")), LatencyTable(latencies),
    ChainingThreshold(getProperty<unsigned>(FUTable, "ChainingThreshold")) {
  luabind::object LatTable = FUTable["Latencies"];
  VFUs::initLatencyTable(LatTable, latencies, 4);
}

VFUMemBus::VFUMemBus(luabind::object FUTable)
  : VFUDesc(VFUs::MemoryBus,
            getProperty<unsigned>(FUTable, "StartInterval"),
            &VFUs::MemBusLatency),
    AddrWidth(getProperty<unsigned>(FUTable, "AddressWidth")),
    DataWidth(getProperty<unsigned>(FUTable, "DataWidth")){
  *LatencyTable = getProperty<double>(FUTable, "Latency");
}

VFUBRam::VFUBRam(luabind::object FUTable)
  : VFUDesc(VFUs::BRam,
            getProperty<unsigned>(FUTable, "StartInterval"),
            &VFUs::BRamLatency),
    DataWidth(getProperty<unsigned>(FUTable, "DataWidth")),
    Template(getProperty<std::string>(FUTable, "Template")),
    InitFileDir(getProperty<std::string>(FUTable, "InitFileDir")){
  *LatencyTable = getProperty<double>(FUTable, "Latency");
}

// Dirty Hack: anchor from SynSettings.h
SynSettings::SynSettings(StringRef Name, SynSettings &From)
  : PipeAlg(From.PipeAlg), SchedAlg(From.SchedAlg),
  ModName(Name), InstName(""), IsTopLevelModule(false) {}

SynSettings::SynSettings(luabind::object SettingTable)
  : PipeAlg(SynSettings::DontPipeline),
    SchedAlg(SynSettings::ILP), IsTopLevelModule(true) {
  if (luabind::type(SettingTable) != LUA_TTABLE)
    return;

  if (boost::optional<std::string> Result =
      luabind::object_cast_nothrow<std::string>(SettingTable["ModName"]))
    ModName = Result.get();

  if (boost::optional<std::string> Result =
      luabind::object_cast_nothrow<std::string>(SettingTable["InstName"]))
    InstName = Result.get();

  if (boost::optional<ScheduleAlgorithm> Result =
    luabind::object_cast_nothrow<ScheduleAlgorithm>(SettingTable["Scheduling"]))
    SchedAlg = Result.get();

  if (boost::optional<PipeLineAlgorithm> Result =
    luabind::object_cast_nothrow<PipeLineAlgorithm>(SettingTable["Pipeline"]))
    PipeAlg = Result.get();

  if (boost::optional<bool> Result =
    luabind::object_cast_nothrow<bool>(SettingTable["isTopMod"]))
    IsTopLevelModule = Result.get();
}

void FuncUnitId::print(raw_ostream &OS) const {
  OS << VFUs::VFUNames[getFUType()];
  // Print the function unit id if necessary.
  if (isBound()) OS << " Bound to " << getFUNum();
}

void FuncUnitId::dump() const {
  print(dbgs());
}

std::string VFUBRam::generateCode(const std::string &Clk, unsigned Num,
                                  unsigned DataWidth, unsigned AddrWidth,
                                  std::string Filename) const {
  std::string Script;
  raw_string_ostream ScriptBuilder(Script);

  std::string ResultName = "bram" + utostr_32(Num) + "_"
                           + utostr_32(DataWidth) + "x" + utostr_32(AddrWidth)
                           + "_result";
  // FIXME: Use LUA api directly?
  // Call the preprocess function.
  ScriptBuilder <<
    /*"local " <<*/ ResultName << ", message = require \"luapp\" . preprocess {"
  // The inpute template.
                << "input=[=[" << Template <<"]=],"
  // And the look up.
                << "lookup={ "
                << "datawidth=" << DataWidth << ", addrwidth=" << AddrWidth
                << ", num=" << Num << ", filepath="
                << "[[" << InitFileDir << "]]" << ", filename=" << "[[" << Filename
                << "]]" << ", empty=" << "[[" << " " << "]]"
                << ", clk='" << Clk

  // End the look up and the function call.
                << "'}}\n";
  DEBUG(ScriptBuilder << "print(" << ResultName << ")\n");
  DEBUG(ScriptBuilder << "print(message)\n");
  ScriptBuilder.flush();
  DEBUG(dbgs() << "Going to execute:\n" << Script);

  SMDiagnostic Err;
  if (!scriptEngin().runScriptStr(Script, Err))
    report_fatal_error("Block Ram code generation:" + Err.getMessage());

  return scriptEngin().getValueStr(ResultName);
}

static void printConstant(raw_ostream &Out, Constant *CPV,
  unsigned DataWidth);

static void printConstantArray(raw_ostream &Out, ConstantArray *CPA,
  unsigned DataWidth);

static void printConstantArray(raw_ostream &Out, ConstantArray *CPA,
  unsigned DataWidth) {
    // As a special case, print the array as a string if it is an array of
    // ubytes or an array of sbytes with positive values.
    //
    const Type *ETy = CPA->getType()->getElementType();

    // Make sure the last character is a null char, as automatically added by C
    /*if (isString && (CPA->getNumOperands() == 0 ||
      !cast<Constant>(*(CPA->op_end()-1))->isNullValue()))
      isString = false;*/

    assert (CPA->getNumOperands() && "Constant has no operands!");
    printConstant(Out, cast<Constant>(CPA->getOperand(0)), DataWidth);
    for (unsigned i = 1, e = CPA->getNumOperands(); i != e; ++i) {
      Out << "\n";
      printConstant(Out, cast<Constant>(CPA->getOperand(i)), DataWidth);
    }
}

// Helper Functions: print constant for initializing bram.
// The printed constant is of hex format and separated by '\n'
// It can be read by readmemh$()
static void printConstant(raw_ostream &Out, Constant *CPV,
                     unsigned DataWidth) {
  if (ConstantInt *CI = dyn_cast<ConstantInt>(CPV)) {
    const Type* Ty = CI->getType();
    std::string tempstr;
    if (Ty == Type::getInt1Ty(CPV->getContext())) {
      tempstr = (CI->getZExtValue() ? '1' : '0');
    } else {
      tempstr = utohexstr(CI->getSExtValue());
    }
 /* if (ConstantInt *CI = dyn_cast<ConstantInt>(CPV)) {
    const Type* Ty = CI->getType();
    std::string tempstr;
    if (Ty == Type::getInt1Ty(CPV->getContext()))
      tempstr = (CI->getZExtValue() ? '1' : '0');
    else if (Ty == Type::getInt32Ty(CPV->getContext()))
      tempstr = utohexstr(CI->getZExtValue());
    else if (Ty->getPrimitiveSizeInBits() > 32)
      tempstr = utohexstr(CI->getZExtValue());
    else {
      if (CI->isMinValue(true))
        tempstr = utohexstr(CI->getZExtValue());
      else
        tempstr = utohexstr(CI->getSExtValue());
    }*/

    if (tempstr.size() > DataWidth/4) {
      std::string str(tempstr.end()-DataWidth/4, tempstr.end());
      Out << str;
    } else {
      Out << tempstr;
    }
    return;
  }

  switch (CPV->getType()->getTypeID()) {
  case Type::ArrayTyID:
    if (ConstantArray *CA = dyn_cast<ConstantArray>(CPV)) {
      printConstantArray(Out, CA, DataWidth);
    } else {
      assert(isa<ConstantAggregateZero>(CPV) || isa<UndefValue>(CPV));
      const ArrayType *AT = cast<ArrayType>(CPV->getType());
      if (AT->getNumElements()) {
        Constant *CZ = Constant::getNullValue(AT->getElementType());
        printConstant(Out, CZ, DataWidth);
        for (unsigned i = 1, e = AT->getNumElements(); i != e; ++i) {
          Out << "\n";
          printConstant(Out, CZ, DataWidth);
        }
      }
    }
    break;
  default:
    errs() << "Unknown constant type: " << *CPV << "\n";
    llvm_unreachable(0);
  }
}

static void printZeros(raw_ostream &Out, unsigned int NumElement,
  unsigned int Bytes){
    std::string element = utostr_32(Bytes)+"'h";
    std::string S;
    for(unsigned int i = 0; i < NumElement; ++i) 
      S += element;
    Out << S;
}

std::string VFUBRam::generateInitFile(unsigned DataWidth, const Value* Initializer,
                     unsigned NumElem) {
  GlobalVariable *GV = const_cast<GlobalVariable*>
                               (cast<GlobalVariable>(Initializer));
  //if the initializer is null, as is the case with real bram, give it an empty file name
  //so that the lua template can skip the initial statement.

  std::string Filename;  // Template for the readmemh file name

  //if the initializer has already been written in a .txt, skip it and keep the filename
  if (GVSet.count(GV)) {
    std::string GVName = GV->getNameStr();
    Filename = "bram" + GVName + ".txt";
    return Filename;
  }
  if (GV) {
    std::string GVName = GV->getNameStr();
    Filename = "bram" + GVName + ".txt";
    GVSet.insert(GV);
  } else {
    Filename = "empty";
  }
  //write the initializer to a .txt file
  std::string File = InitFileDir + Filename;
  raw_ostream& InitS = scriptEngin().getOutputFileStream(File);

  if (GV) {
    Constant* CPV = GV->getInitializer();
    //There is initial value, print the constant array.
    printConstant(InitS, CPV, DataWidth);
  } else {
    //There is no initial value, print Zeros to the InitS.
    printZeros(InitS, NumElem, DataWidth/8); 
  }
  return Filename;
}

std::string VFUs::instantiatesModule(const std::string &ModName, unsigned ModNum,
                                     ArrayRef<std::string> Ports) {
  std::string Script;
  raw_string_ostream ScriptBuilder(Script);

  luabind::object ModTemplate = scriptEngin().getModTemplate(ModName);
  std::string Template = getProperty<std::string>(ModTemplate, "InstTmplt");

  std::string ResultName = ModName + utostr_32(ModNum) + "_inst";
  // FIXME: Use LUA api directly?
  // Call the preprocess function.
  ScriptBuilder <<
    /*"local " <<*/ ResultName << ", message = require \"luapp\" . preprocess {"
  // The inpute template.
                << "input=[=[";
  if (Template.empty()) {
    ScriptBuilder << "// " << ModName << " not available!\n";
    errs() << "Instantiation template for external module :" << ModName
           << " not available!\n";
    // Dirty Hack: create the finish signal.
    return "parameter " + Ports[3] + "= 1'b1;\n";
  } else
    ScriptBuilder << Template;
  ScriptBuilder <<"]=],"
  // And the look up.
                   "lookup={ num=" << ModNum << ", clk='" << Ports[0]
                << "', rst = '" <<  Ports[1] << "', en = '" <<  Ports[2]
                << "', fin = '" <<  Ports[3];
  // The output ports.
  for (unsigned i = 4, e = Ports.size(); i < e; ++i)
    ScriptBuilder << "', out" << (i - 4) << " = '" <<  Ports[i];

  // End the look up and the function call.
  ScriptBuilder << "'}}\n";
  DEBUG(ScriptBuilder << "print(" << ResultName << ")\n");
  DEBUG(ScriptBuilder << "print(message)\n");
  ScriptBuilder.flush();
  DEBUG(dbgs() << "Going to execute:\n" << Script);

  SMDiagnostic Err;
  if (!scriptEngin().runScriptStr(Script, Err))
    report_fatal_error("External module instantiation:" + Err.getMessage());

  return scriptEngin().getValueStr(ResultName);
}

std::string VFUs::startModule(const std::string &ModName, unsigned ModNum,
                              ArrayRef<std::string> InPorts) {
  std::string Script;
  raw_string_ostream ScriptBuilder(Script);

  luabind::object ModTemplate = scriptEngin().getModTemplate(ModName);
  std::string Template = getProperty<std::string>(ModTemplate, "StartTmplt");

  std::string ResultName = ModName + utostr_32(ModNum) + "_start";
  // FIXME: Use LUA api directly?
  // Call the preprocess function.
  ScriptBuilder <<
    /*"local " <<*/ ResultName << ", message = require \"luapp\" . preprocess {"
  // The inpute template.
                   "input=[=[";
  if (Template.empty()) {
    ScriptBuilder << "// " << ModName << " not available!\n";
    errs() << "Start template for external Module :" << ModName
           << " not available!\n";
  } else
    ScriptBuilder << Template;
  ScriptBuilder << "]=],"
  // And the look up.
                   "lookup={ num=" << ModNum;
  // The input ports.
  for (unsigned i = 0, e = InPorts.size(); i < e; ++i)
    ScriptBuilder << ", in" << i << " = [=[" <<  InPorts[i] << "]=]";

  // End the look up and the function call.
  ScriptBuilder << "}}\n";
  DEBUG(ScriptBuilder << "print(" << ResultName << ")\n");
  DEBUG(ScriptBuilder << "print(message)\n");
  ScriptBuilder.flush();
  DEBUG(dbgs() << "Going to execute:\n" << Script);

  SMDiagnostic Err;
  if (!scriptEngin().runScriptStr(Script, Err))
    report_fatal_error("External module starting:" + Err.getMessage());

  return scriptEngin().getValueStr(ResultName);
}

unsigned VFUs::getModuleOperands(const std::string &ModName, unsigned FNNum,
                                 SmallVectorImpl<ModOpInfo> &OpInfo) {
  luabind::object O = scriptEngin().getModTemplate(ModName);
  O = O["TimingInfo"];
  if (luabind::type(O) != LUA_TTABLE) return 0;

  unsigned NumOperands = getProperty<unsigned>(O, "NumOperands");
  if (NumOperands == 0)  return 0;

  unsigned Latency = getProperty<unsigned>(O, "Latency");
  if (Latency == 0) return 0;

  O = O["OperandInfo"];
  if (luabind::type(O) != LUA_TTABLE) return 0;

  std::string Script;
  raw_string_ostream ScriptBuilder(Script);

  std::string ResultName = ModName + utostr_32(FNNum) + "_operand";
  for (unsigned i = 1; i <= NumOperands; ++i) {
    luabind::object OpTab = O[i];
    // FIXME: Use LUA api directly?
    // Call the preprocess function.
    ScriptBuilder <<
      /*"local " <<*/ ResultName << ", message = require \"luapp\" . preprocess {"
      // The inpute template.
      "input=[=[" << getProperty<std::string>(OpTab, "Name") << "]=],"
      // And the look up.
      "lookup={ num=" << FNNum << "}}\n";
    DEBUG(ScriptBuilder << "print(" << ResultName << ")\n");
    DEBUG(ScriptBuilder << "print(message)\n");
    ScriptBuilder.flush();
    DEBUG(dbgs() << "Going to execute:\n" << Script);

    SMDiagnostic Err;
    if (!scriptEngin().runScriptStr(Script, Err))
      report_fatal_error("External module starting:" + Err.getMessage());

    std::string OpName = scriptEngin().getValueStr(ResultName);
    unsigned OpSize = getProperty<unsigned>(OpTab, "SizeInBits");
    OpInfo.push_back(ModOpInfo(OpName, OpSize));

    Script.clear();
  }

  return Latency;
}
