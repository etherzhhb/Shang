//===----- VFunctionUnit.cpp - VTM Function Unit Information ----*- C++ -*-===//
//
// Copyright: 2011 by SYSU EDA Group. all rights reserved.
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
// This file contains implementation of the function unit class in
// Verilog target machine.
//
//===----------------------------------------------------------------------===//

#include "vtm/FUInfo.h"
#include "vtm/SynSettings.h" // DiryHack: Also implement the SynSetting class.
#include "vtm/LuaScript.h"

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

static unsigned ComputeOperandSizeInByteLog2Ceil(unsigned SizeInBits) {
  return std::max(Log2_32_Ceil(SizeInBits), 3u) - 3;
}

//===----------------------------------------------------------------------===//
/// Hardware resource.
void VFUDesc::print(raw_ostream &OS) const {
}

namespace llvm {
  namespace VFUs {
    const char *VFUNames[] = {
      "Trivial", "AddSub", "Shift", "Mult", "ICmp", "Sel", "Reduction", 
      "MemoryBus", "BRam", "Mux", "CalleeFN"
    };

    // Default area cost parameter.
    unsigned LUTCost = 64;
    unsigned RegCost;

    ////////////////////////////////////
    unsigned AddCost[64] ;
    unsigned MulCost[64] ;
    unsigned ShiftCost[64] ;
    unsigned ICmpCost[64] ;
    unsigned SelCost[64] ;
    unsigned ReductionCost[64] ;
    //////////////////////////////////
    //FIX ME: This can be initialized the MuxCost from lua.
    unsigned MaxLutSize = 4;
    unsigned MaxMuxPerLut = 4;
    unsigned MaxAllowedMuxSize = 8;

    // Default value of Latency tables.         8bit 16bit 32bit 64bit
    float AdderLatencies[]     = { 1.0f,  1.0f,  1.0f, 1.0f };
    float CmpLatencies[]       = { 1.0f,  1.0f,  1.0f, 1.0f };
    float MultLatencies[]      = { 1.0f,  1.0f,  1.0f, 1.0f };
    float ShiftLatencies[]     = { 1.0f,  1.0f,  1.0f, 1.0f };
    float SelLatencies[]       = { 1.0f,  1.0f,  1.0f, 1.0f };
    float ReductionLatencies[]       = { 1.0f,  1.0f,  1.0f, 1.0f };
    float MemBusLatency = 1.0f;
    float BRamLatency = 1.0f;
    float LutLatency = 0.0f;
    float ClkEnSelLatency = 0.0f;

    void initLatencyTable(luabind::object LuaLatTable, float *LatTable,
                          unsigned Size) {
      for (unsigned i = 0; i < Size; ++i)
        // Lua array starts from 1
        LatTable[i] = getProperty<float>(LuaLatTable, i + 1, LatTable[i]);
    }

    void computeCost(unsigned StartY, unsigned EndY, int Size,
                     unsigned StartX, unsigned Index, unsigned *CostTable){
      float Slope = float((EndY - StartY)) / float((Size - 1));
      int Intercept = StartY - Slope * StartX;
      for (int i = 0; i < Size; ++i){
        CostTable[Index + i] = Slope * (StartX + i) + Intercept;
      }
    }

    void initCostTable(luabind::object LuaCostTable, unsigned *CostTable,
                       unsigned Size) {
        SmallVector<unsigned, 8> CopyTable(Size);
        for (unsigned i = 0; i < Size; ++i)
          // Lua array starts from 1
          CopyTable[i] = getProperty<unsigned>(LuaCostTable, i + 1, CopyTable[i]);

        //Initial the array form a[0] to a[6]
        computeCost(CopyTable[0], CopyTable[1], 7, 1, 0, CostTable);
        //Initial the array form a[7] to a[14]
        computeCost(CopyTable[1], CopyTable[2], 8, 8, 7, CostTable);
        //Initial the array form a[15] to a[30]
        computeCost(CopyTable[2], CopyTable[3], 16, 16, 15, CostTable);
        //Initial the array form a[31] to a[63]
        computeCost(CopyTable[3], CopyTable[4], 33, 32, 31, CostTable);
    }

    float lookupLatency(const float *Table, unsigned SizeInBits) {
      int i = ComputeOperandSizeInByteLog2Ceil(SizeInBits);
      float RoundUpLatency   = Table[i],
            RoundDownLatency = i ? Table[i - 1] : 0.0f;
      unsigned SizeRoundUpToByteInBits = 8 << i;
      unsigned SizeRoundDownToByteInBits = i ? (8 << (i - 1)) : 0;
      float PerBitLatency =
        (RoundUpLatency - RoundDownLatency)
        / (SizeRoundUpToByteInBits - SizeRoundDownToByteInBits);
      // Scale the latency according to the actually width.
      return
        RoundDownLatency
        + PerBitLatency * float(SizeInBits - SizeRoundDownToByteInBits);
    }

    float getMuxLatency(unsigned Size) {
      if (Size < 2) return 0;

      unsigned Level = ceil(float(Log2_32_Ceil(Size))
                            / float(Log2_32_Ceil(MaxMuxPerLut)));

      return Level * LutLatency;
    }

    float getMuxCost(unsigned Size) {
      if (Size < 2) return 0;

      // Every MUX will eliminates inputs number by (MaxMuxPerLut - 1), how
      // many MUX need to reduce the input number from Size to 1?
      return (Size - 1) / (MaxMuxPerLut - 1) * LUTCost;
    }
  }
}

VFUDesc::VFUDesc(VFUs::FUTypes type, luabind::object FUTable, unsigned *costs, float *latencies)
  : ResourceType(type),
    StartInt(getProperty<unsigned>(FUTable, "StartInterval")),
    Costs(costs), LatencyTable(latencies),
    ChainingThreshold(getProperty<unsigned>(FUTable, "ChainingThreshold")) {
  luabind::object LatTable = FUTable["Latencies"];
  VFUs::initLatencyTable(LatTable, latencies, 4);
  luabind::object CostTable = FUTable["Costs"];
  VFUs::initCostTable(CostTable, costs, 5);
}

VFUMemBus::VFUMemBus(luabind::object FUTable)
  : VFUDesc(VFUs::MemoryBus,
            getProperty<unsigned>(FUTable, "StartInterval"),
            0, &VFUs::MemBusLatency),
    AddrWidth(getProperty<unsigned>(FUTable, "AddressWidth")),
    DataWidth(getProperty<unsigned>(FUTable, "DataWidth")){
  *LatencyTable = getProperty<float>(FUTable, "Latency");
}

VFUBRAM::VFUBRAM(luabind::object FUTable)
  : VFUDesc(VFUs::BRam,
            getProperty<unsigned>(FUTable, "StartInterval"),
            0, &VFUs::BRamLatency),
    DataWidth(getProperty<unsigned>(FUTable, "DataWidth")),
    Template(getProperty<std::string>(FUTable, "Template")),
    InitFileDir(getProperty<std::string>(FUTable, "InitFileDir")){
  *LatencyTable = getProperty<float>(FUTable, "Latency");
}

// Dirty Hack: anchor from SynSettings.h
SynSettings::SynSettings(StringRef Name, SynSettings &From)
  : PipeAlg(From.PipeAlg), SchedAlg(From.SchedAlg),
  ModName(Name), InstName(""), IsTopLevelModule(false) {}

SynSettings::SynSettings(luabind::object SettingTable)
  : PipeAlg(SynSettings::DontPipeline),
    SchedAlg(SynSettings::SDC), IsTopLevelModule(true) {
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

std::string VFUBRAM::generateCode(const std::string &Clk, unsigned Num,
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

static bool generateOperandNames(const std::string &ModName, luabind::object O,
                                 unsigned FNNum,
                                 SmallVectorImpl<VFUs::ModOpInfo> &OpInfo) {

  unsigned NumOperands = getProperty<unsigned>(O, "NumOperands");
  if (NumOperands == 0)  return false;

  O = O["OperandInfo"];
  if (luabind::type(O) != LUA_TTABLE) return false;

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
    OpInfo.push_back(VFUs::ModOpInfo(OpName, OpSize));

    Script.clear();
  }
  return true;
}

unsigned VFUs::getModuleOperands(const std::string &ModName, unsigned FNNum,
                                 SmallVectorImpl<ModOpInfo> &OpInfo) {
  luabind::object O = scriptEngin().getModTemplate(ModName);
  O = O["TimingInfo"];
  if (luabind::type(O) != LUA_TTABLE) return 0;

  unsigned Latency = getProperty<unsigned>(O, "Latency");

  if (generateOperandNames(ModName, O, FNNum, OpInfo))
    return Latency;

  return 0;
}
