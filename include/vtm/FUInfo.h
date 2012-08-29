//===------- VFunctionUnit.h - VTM Function Unit Information ----*- C++ -*-===//
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
// This file contains define the function unit class in Verilog target machine.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_FUNCTION_UNIT_H
#define VTM_FUNCTION_UNIT_H

#include "vtm/SynSettings.h"

#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Constants.h"
#include "llvm/Function.h"
#include "llvm/GlobalVariable.h"
#include "llvm/DerivedTypes.h"

#include<set>

namespace luabind {
  namespace adl {
    class object;
  }
  using adl::object;
}

namespace llvm {
class TargetRegisterClass;
class MachineInstr;

namespace VFUs {
  enum FUTypes {
    Trivial = 0,
    AddSub = 1,
    Shift = 2,
    Mult = 3,
    ICmp = 4,
    Sel = 5,
    Reduction = 6,
    MemoryBus = 7,
    BRam = 8,
    Mux = 9,
    FirstFUType = Trivial,
    FirstNonTrivialFUType = AddSub,
    LastBitLevelChainingFUType = Mult,
    LastPostBindFUType = Reduction,
    NumPostBindFUs = LastPostBindFUType - FirstNonTrivialFUType + 1,
    LastCommonFUType = Mux,
    NumCommonFUs = LastCommonFUType - FirstFUType + 1,
    NumNonTrivialCommonFUs = LastCommonFUType - FirstNonTrivialFUType + 1,
    // Special function unit.
    // RTL module corresponding to callee functions of function corresponding to
    // current RTL module.
    CalleeFN = 10,
    LastFUType = CalleeFN,
    NumFUs = LastFUType - FirstFUType + 1,
    // Helper enumeration value, just for internal use as a flag to indicate
    // all kind of function units are selected.
    AllFUType = 0xf
  };

  extern const char *VFUNames[];

  enum ICmpFUType {
    CmpEQ, CmpSigned, CmpUnsigned
  };

  static const int RetPortOffset = 1;

  // Ports layout: Clk, Rst, En, Fin, ouput0, output1 ...
  std::string instantiatesModule(const std::string &ModName, unsigned ModNum,
                                 ArrayRef<std::string> Ports);
  std::string startModule(const std::string &ModName, unsigned ModNum,
                          ArrayRef<std::string> InPorts);

  typedef std::pair<std::string, unsigned> ModOpInfo;
  unsigned getModuleOperands(const std::string &ModName, unsigned FNNum,
                             SmallVectorImpl<ModOpInfo> &OpInfo);
  // Cost parameters.
  extern unsigned AddCost[64], MulCost[64], ShiftCost[64], ICmpCost[64],
                  SelCost[64], ReductionCost[64];

  extern unsigned LUTCost;
  extern unsigned RegCost;
  extern unsigned MaxLutSize;

  // Latency tables
  extern float AdderLatencies[4], CmpLatencies[4], MultLatencies[4],
               ShiftLatencies[4],SelLatencies[4], ReductionLatencies[4];

  extern float BRamLatency, MemBusLatency, LutLatency,
                // Latency of clock enable multiplexer selector
                ClkEnSelLatency;

  float lookupLatency(const float *Table, unsigned SizeInBits);
  unsigned lookupCost(const unsigned *Table, unsigned SizeInBits);
}

class FuncUnitId {
  union {
    struct {
      unsigned Type  : 4;
      unsigned Num : 12;
    } ID;

    uint16_t data;
  } UID;

public:
  // The general FUId of a given type.
  inline explicit FuncUnitId(VFUs::FUTypes T, unsigned N) {
    UID.ID.Type = T;
    UID.ID.Num = N;
    assert(unsigned(T) == UID.ID.Type && UID.ID.Num == N
           && "Data overflow!");
  }

  /*implicit*/ FuncUnitId(VFUs::FUTypes T = VFUs::Trivial) {
    UID.ID.Type = T;
    UID.ID.Num = 0xfff;
  }

  explicit FuncUnitId(uint16_t Data) {
    UID.data = Data;
  }

  inline VFUs::FUTypes getFUType() const { return VFUs::FUTypes(UID.ID.Type); }
  inline unsigned getFUNum() const { return UID.ID.Num; }
  inline unsigned getData() const { return UID.data; }
  inline bool isUnknownInstance() const { return getFUNum() == 0xfff; }

  inline bool isTrivial() const { return getFUType() == VFUs::Trivial; }
  inline bool isBound() const {
    return !isTrivial() && getFUNum() != 0xfff;
  }

  inline bool operator==(const FuncUnitId X) const { return UID.data == X.UID.data; }
  inline bool operator!=(const FuncUnitId X) const { return !operator==(X); }
  inline bool operator< (const FuncUnitId X) const { return UID.data < X.UID.data; }

  void print(raw_ostream &OS) const;
  void dump() const;
};

inline static raw_ostream &operator<<(raw_ostream &O, const FuncUnitId &ID) {
  ID.print(O);
  return O;
}

/// @brief The description of Verilog target machine function units.
class VFUDesc {
  VFUDesc(const VFUDesc &);            // DO NOT IMPLEMENT
  void operator=(const VFUDesc &);  // DO NOT IMPLEMENT
protected:
  // The HWResource baseclass this node corresponds to
  const unsigned ResourceType;
  // Start interval
  const unsigned StartInt;
  // Function unit cost for resource allocation and binding.
  unsigned *const Costs;
  // Latency table of the function unit.
  float *const LatencyTable;
  // Chain the operation if its size smaller than the threshold;
  unsigned ChainingThreshold;
  VFUDesc(VFUs::FUTypes type, unsigned startInt, unsigned *costs, float *latencies)
    : ResourceType(type), StartInt(startInt), Costs(costs),
      LatencyTable(latencies), ChainingThreshold(0) {}

public:
  VFUDesc(VFUs::FUTypes type, luabind::object FUTable, unsigned *costs, float *latencies);

  static const char *getTypeName(VFUs::FUTypes FU) {
    return VFUs::VFUNames[FU];
  }

  unsigned getType() const { return ResourceType; }
  const char *getTypeName() const {
    return getTypeName((VFUs::FUTypes)getType());
  }

  unsigned getStartInt() const { return StartInt; }
  const unsigned *getCost() const { return Costs; }

  bool shouldBeChained(unsigned FUSize) const {
    return FUSize <= ChainingThreshold;
  }

  virtual void print(raw_ostream &OS) const;
};

class VFUMux : public VFUDesc {
  unsigned MuxCost[31][64];
  float MuxLatencies[31][4];

public:
  const unsigned MaxAllowedMuxSize;

  VFUMux(luabind::object FUTable);

  float getMuxLatency(unsigned Size, unsigned BitWidth);
  unsigned getMuxCost(unsigned Size, unsigned BitWidth);

  static VFUs::FUTypes getType() { return VFUs::Mux; };
  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VFUMux *A) { return true; }
  static inline bool classof(const VFUDesc *A) {
    return A->getType() == VFUs::Mux;
  }
};

class VFUMemBus : public VFUDesc {
  unsigned AddrWidth;
  unsigned DataWidth;
public:
  VFUMemBus(luabind::object FUTable);

  unsigned getAddrWidth() const { return AddrWidth; }
  unsigned getDataWidth() const { return DataWidth; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  static inline bool classof(const VFUMemBus *A) { return true; }
  static inline bool classof(const VFUDesc *A) {
    return A->getType() == VFUs::MemoryBus;
  }

  static VFUs::FUTypes getType() { return VFUs::MemoryBus; }
  static const char *getTypeName() { return VFUs::VFUNames[getType()]; }

  // Signal names of the function unit.
  inline static std::string getAddrBusName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "addr";
  }

  inline static std::string getInDataBusName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "in";
  }

  inline static std::string getOutDataBusName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "out";
  }

  // Dirty Hack: This should be byte enable.
  inline static std::string getByteEnableName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "be";
  }

  inline static std::string getCmdName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "cmd";
  }

  inline static std::string getEnableName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "en";
  }

  inline static std::string getReadyName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "rdy";
  }

  static const int CMDWidth = 4;

  enum Cmds {
    // Load and store.
    CmdLoad = 0, CmdStore = 1,
    CmdFirstNoLoadStore = 2,
    // Memset/Memcpy/Memmove
    CmdMemSet = 2, CmdMemCpy = 3, CmdMemMove = 4
  };

  enum CmdSeqs {
    SeqBegin = 0, Seq = 1, SeqEnd = 2
  };
};

template<enum VFUs::FUTypes T>
class VSimpleFUDesc : public VFUDesc {
public:
  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  template<enum VFUs::FUTypes OtherT>
  static inline bool classof(const VSimpleFUDesc<OtherT> *A) {
    return T == OtherT;
  }
  static inline bool classof(const VFUDesc *A) {
    return A->getType() == T;
  }

  static VFUs::FUTypes getType() { return T; };
  static const char *getTypeName() { return VFUs::VFUNames[getType()]; }
};

typedef VSimpleFUDesc<VFUs::AddSub>  VFUAddSub;
typedef VSimpleFUDesc<VFUs::Shift>   VFUShift;
typedef VSimpleFUDesc<VFUs::Mult>    VFUMult;
typedef VSimpleFUDesc<VFUs::ICmp>    VFUICmp;
typedef VSimpleFUDesc<VFUs::Sel>     VFUSel;
typedef VSimpleFUDesc<VFUs::Reduction>     VFUReduction;

class VFUBRAM : public  VFUDesc {
  unsigned DataWidth;
  std::string Template; // Template for inferring block ram.
  std::string InitFileDir; // Template for readmemh dir.
  std::set<GlobalVariable*> GVSet;
public:
  VFUBRAM(luabind::object FUTable);

  std::string generateCode(const std::string &Clk, unsigned Num,
                           unsigned DataWidth, unsigned AddrWidth,
                           std::string Filename) const;

  static inline bool classof(const VFUBRAM *A) {
    return true;
  }

  template<enum VFUs::FUTypes OtherT>
  static inline bool classof(const VSimpleFUDesc<OtherT> *A) {
    return getType() == OtherT;
  }

  static inline bool classof(const VFUDesc *A) {
    return A->getType() == VFUs::BRam;
  }

  static VFUs::FUTypes getType() { return VFUs::BRam; };
  static const char *getTypeName() { return VFUs::VFUNames[getType()]; }

  // Signal names of the function unit.
  inline static std::string getAddrBusName(unsigned FUNum) {
    return "bram" + utostr(FUNum) + "addr";
  }

  inline static std::string getInDataBusName(unsigned FUNum) {
    return "bram" + utostr(FUNum) + "in";
  }

  inline static std::string getOutDataBusName(unsigned FUNum) {
    return "bram" + utostr(FUNum) + "out";
  }

  // Dirty Hack: This should be byte enable.
  inline static std::string getByteEnableName(unsigned FUNum) {
    return "bram" + utostr(FUNum) + "be";
  }

  inline static std::string getWriteEnableName(unsigned FUNum) {
    return "bram" + utostr(FUNum) + "we";
  }

  inline static std::string getEnableName(unsigned FUNum) {
    return "bram" + utostr(FUNum) + "en";
  }
};

struct CommonFUIdentityFunctor
  : public std::unary_function<enum VFUs::FUTypes, unsigned>{

  unsigned operator()(enum VFUs::FUTypes T) const {
    return (unsigned)T - (unsigned)VFUs::FirstFUType;
  }
};

VFUDesc *getFUDesc(enum VFUs::FUTypes T);

template<class ResType>
ResType *getFUDesc() {
  return cast<ResType>(getFUDesc(ResType::getType()));
}
}

#endif
