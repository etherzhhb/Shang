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

#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SourceMgr.h"
#define DEBUG_TYPE "vtm-fu-info"
#include "llvm/Support/Debug.h"
using namespace llvm;

//===----------------------------------------------------------------------===//
/// Hardware resource.
void VFUDesc::print(raw_ostream &OS) const {
  // OS << "Resource: " << Name << '\n';
  OS.indent(2) << "TotalNum: " << TotalRes << '\n';
  OS.indent(2) << "Latency: " << Latency << '\n';
  OS.indent(2) << "StartInterval: " << StartInt << '\n';
}

namespace llvm {
  namespace VFUs {
   const char *VFUNames[] = {
      "Trivial", "AddSub", "Shift", "Mult", "MemoryBus", "BRam",
      "CalleeFN", "FSMFinish"
    };

    const TargetRegisterClass *getRepRegisterClass(enum FUTypes T) {
      static const TargetRegisterClass *RepRCTable[] = {
        VTM::WireRegisterClass, //Trivial = 0,
        VTM::RADDRegisterClass, //AddSub = 1,
        VTM::RSHTRegisterClass, //Shift = 2,
        VTM::RMULRegisterClass, //Mult = 3,
        VTM::RINFRegisterClass, //MemoryBus = 4,
        VTM::RBRMRegisterClass, //BRam = 5,
        VTM::RCFNRegisterClass, //CalleeFN = 6,
        VTM::WireRegisterClass  //FSMFinish = 7,
      };
      return RepRCTable[T];
    }
  }
}

//===----------------------------------------------------------------------===//
// Helper functions for reading function unit table from script.
template<class PropType>
static PropType getProperty(luabind::object &FUTable,
                              const std::string &PropName) {
  //luabind::object FUTable =
  //  Script->getRawObject("FU" + std::string(FUType::getTypeName()));

  if (luabind::type(FUTable) != LUA_TTABLE) return PropType();

  boost::optional<PropType> Result =
    luabind::object_cast_nothrow<PropType>(FUTable[PropName]);

  if (!Result) return PropType();

  return Result.get();
}

VFUDesc::VFUDesc(VFUs::FUTypes type, luabind::object FUTable)
  : ResourceType(type),
    Latency(getProperty<unsigned>(FUTable, "Latency")),
    StartInt(getProperty<unsigned>(FUTable, "StartInterval")),
    TotalRes(getProperty<unsigned>(FUTable, "TotalNumber")),
    MaxBitWidth(getProperty<unsigned>(FUTable, "OperandWidth")){}

VFUMemBus::VFUMemBus(luabind::object FUTable)
  : VFUDesc(VFUs::MemoryBus,
            getProperty<unsigned>(FUTable, "Latency"),
            getProperty<unsigned>(FUTable, "StartInterval"),
            getProperty<unsigned>(FUTable, "TotalNumber"),
            getProperty<unsigned>(FUTable, "DataWidth")),
    AddrWidth(getProperty<unsigned>(FUTable, "AddressWidth")) {}


VFUBRam::VFUBRam(luabind::object FUTable)
  : VFUDesc(VFUs::BRam,
            getProperty<unsigned>(FUTable, "Latency"),
            getProperty<unsigned>(FUTable, "StartInterval"),
            getProperty<unsigned>(FUTable, "TotalNumber"),
            getProperty<unsigned>(FUTable, "DataWidth")),
  Template(getProperty<std::string>(FUTable, "Template")) {}


// Dirty Hack: anchor from SynSettings.h
SynSettings::SynSettings(StringRef Name, SynSettings &From)
  : PipeAlg(From.PipeAlg), SchedAlg(From.SchedAlg),
  ModName(Name), HierPrefix(""), IsTopLevelModule(false) {}

SynSettings::SynSettings(luabind::object SettingTable)
  : PipeAlg(SynSettings::DontPipeline),
    SchedAlg(SynSettings::ILP), IsTopLevelModule(true) {
  if (luabind::type(SettingTable) != LUA_TTABLE)
    return;

  if (boost::optional<std::string> Result =
      luabind::object_cast_nothrow<std::string>(SettingTable["ModName"]))
    ModName = Result.get();

  if (boost::optional<std::string> Result =
      luabind::object_cast_nothrow<std::string>(SettingTable["HierPrefix"]))
    HierPrefix = Result.get();

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

unsigned FuncUnitId::getTotalFUs() const {
  // If the function unit is binded, there is only one function unit with
  // the specific function unit id available.
  if (isBound()) return 1;

  // Else we can just choose a function unit from all available function units.
  return getFUDesc(getFUType())->getTotalRes();
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
                                  unsigned DataWidth, unsigned AddrWidth) const {
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
                << ", num=" << Num << ", clk='" << Clk
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
