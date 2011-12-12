//===------- VFunctionUnit.h - VTM Function Unit Information ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
    MemoryBus = 4,
    BRam = 5,
    ICmp = 6,
    FirstFUType = Trivial,
    FirstNonTrivialFUType = AddSub,
    LastPostBindFUType = Mult,
    NumPostBindFUs = LastPostBindFUType - FirstNonTrivialFUType + 1,
    LastCommonFUType = ICmp,
    NumCommonFUs = LastCommonFUType - FirstFUType + 1,
    NumNonTrivialCommonFUs = LastCommonFUType - FirstNonTrivialFUType + 1,
    // Special function unit.
    // RTL module corresponding to callee functions of function corresponding to
    // current RTL module.
    CalleeFN = 7,
    LastFUType = CalleeFN,
    NumFUs = LastFUType - FirstFUType + 1,
    // Helper enumeration value, just for internal use as a flag to indicate
    // all kind of function units are selected.
    AllFUType = 0xf
  };

  extern const char *VFUNames[];
  const TargetRegisterClass *getRepRegisterClass(unsigned OpCode);

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
  extern unsigned LUTCost, RegCost, MUXCost, AddCost, MulCost,
                  ShiftCost, ICmpCost, MuxSizeCost;

  // Latency tables
  extern double AdderLatencies[4], CmpLatencies[4], MultLatencies[4],
                ShiftLatencies[4], ReductionLatencies[4];
  // Mux latency from 2-1 mux to 16-1 mux.
  extern double MuxLatencies[16];
  double getMuxLatency(unsigned MuxSize);

  extern double BRamLatency, MemBusLatency, LutLatency,
                // Latency of clock enable multiplexer selector
                ClkEnSelLatency;

  void initLatencyTable(luabind::object LuaLatTable, double *LatTable,
                        unsigned Size);
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
  const unsigned Cost;
  // Latency table of the function unit.
  double *LatencyTable;
  // Chain the operation if its size smaller than the threshold;
  unsigned ChainingThreshold;
  VFUDesc(VFUs::FUTypes type, unsigned startInt, double *latencies)
    : ResourceType(type), StartInt(startInt), Cost(~0),
      LatencyTable(latencies), ChainingThreshold(0) {}

public:
  VFUDesc(VFUs::FUTypes type, luabind::object FUTable, double *latencies);

  static const char *getTypeName(VFUs::FUTypes FU) {
    return VFUs::VFUNames[FU];
  }

  unsigned getType() const { return ResourceType; }
  const char *getTypeName() const {
    return getTypeName((VFUs::FUTypes)getType());
  }

  unsigned getStartInt() const { return StartInt; }
  unsigned getCost() const { return Cost; }

  bool shouldBeChained(unsigned FUSize) const {
    return FUSize <= ChainingThreshold;
  }

  virtual void print(raw_ostream &OS) const;
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

class VFUBRam : public  VFUDesc {
  unsigned DataWidth;
  std::string Template; // Template for inferring block ram.
  std::string InitFileDir; // Template for readmemh dir.
  std::set<GlobalVariable*> GVSet;
public:
  VFUBRam(luabind::object FUTable);

  std::string generateCode(const std::string &Clk, unsigned Num,
                           unsigned DataWidth, unsigned AddrWidth, std::string Filename) const;

  std::string generateInitFile(unsigned DataWidth, const Value* Initializer,
                           unsigned NumElem);

  static inline bool classof(const VFUBRam *A) {
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
