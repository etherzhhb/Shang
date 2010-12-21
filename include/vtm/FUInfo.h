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
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"

namespace llvm {

namespace VFUs {
  enum FUTypes {
    Trivial = 0,
    MemoryBus = 1,
    Shift = 2,
    AddSub = 3,
    Mult = 4,

    FirstFUType = Trivial,
    LastCommonFUType = Mult,
    NumCommonFUs = LastCommonFUType - FirstFUType + 1,
    // Special function unit.
    // Finite state machine finish.
    FSMFinish = 7,
    LastFUType = FSMFinish,
    NumFUs = LastFUType - FirstFUType + 1,
    // Helper enumeration value, just for internal use as a flag to indicate
    // all kind of function units are selected.
    AllFUType = 0xf
  };

  extern const char *VFUNames[];
}

class FuncUnitId {
  union {
    struct {
      VFUs::FUTypes Type  : 4;
      unsigned  Num : 12;
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
    UID.ID.Num = (T == VFUs::FSMFinish) ? 0 : 0xfff;
  }

  explicit FuncUnitId(uint16_t Data) {
    UID.data = Data;
  }

  inline VFUs::FUTypes getFUType() const { return UID.ID.Type; }
  inline unsigned getFUNum() const { return UID.ID.Num; }
  inline unsigned getData() const { return UID.data; }

  inline bool isTrivial() const { return getFUType() == VFUs::Trivial; }
  inline bool isBinded() const {
    return !isTrivial() && getFUNum() != 0xfff;
  }
 
  // Get the total avaliable number of this kind of function unit.
  unsigned getTotalFUs() const;

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
  // The HWResource baseclass this node corresponds to
  unsigned ResourceType;
  // How many cycles to finish?
  const unsigned Latency;
  // Start interval
  const unsigned StartInt;
  // How many resources available?
  const unsigned TotalRes;

  //// Use a map mapping instance to count?
  //typedef std::vector<unsigned> UsingCountVec;
  //UsingCountVec UsingCount;

  VFUDesc(const VFUDesc &);            // DO NOT IMPLEMENT
  void operator=(const VFUDesc &);  // DO NOT IMPLEMENT
protected:
  VFUDesc(VFUs::FUTypes type, unsigned latency, unsigned startInt,
          unsigned totalRes)
    : ResourceType(type), Latency(latency), StartInt(startInt),
    TotalRes(totalRes) {}
public:
  unsigned getType() const { return ResourceType; }
  const char *getTypeName() { return VFUs::VFUNames[getType()]; }
  
  unsigned getLatency() const { return Latency; }
  unsigned getTotalRes() const { return TotalRes; }
  unsigned getStartInt() const { return StartInt; }

  virtual void print(raw_ostream &OS) const;
}; 

class VFUMemBus : public VFUDesc {
  unsigned AddrWidth;
  unsigned DataWidth;

  VFUMemBus(unsigned latency, unsigned startInt, unsigned totalRes,
    unsigned addrWidth, unsigned dataWidth)
    : VFUDesc(VFUs::MemoryBus, latency, startInt, totalRes),
    AddrWidth(addrWidth), DataWidth(dataWidth) {}

  friend class FUInfo;
public:
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
  inline static std::string getDataSizeName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "datasize";
  }

  inline static std::string getWriteEnableName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "we";
  }

  inline static std::string getEnableName(unsigned FUNum) {
    return "mem" + utostr(FUNum) + "en";
  }
};



template<enum VFUs::FUTypes T>
class VFUBinOpFUDesc : public VFUDesc {
  unsigned MaxBitWidth;

  VFUBinOpFUDesc(unsigned latency, unsigned startInt, unsigned totalRes,
                 unsigned maxBitWidth)
    : VFUDesc(T, latency, startInt, totalRes), MaxBitWidth(maxBitWidth) {}
    
  friend class FUInfo;
public:
  unsigned getMaxBitWidth() const { return MaxBitWidth; }

  /// Methods for support type inquiry through isa, cast, and dyn_cast:
  template<enum VFUs::FUTypes OtherT>
  static inline bool classof(const VFUBinOpFUDesc<OtherT> *A) {
    return T == OtherT;
  }
  static inline bool classof(const VFUDesc *A) {
    return A->getType() == T;
  }

  static VFUs::FUTypes getType() { return T; };
  static const char *getTypeName() { return VFUs::VFUNames[getType()]; }
};

typedef VFUBinOpFUDesc<VFUs::Mult>    VFUMult;
typedef VFUBinOpFUDesc<VFUs::AddSub>  VFUAddSub;
typedef VFUBinOpFUDesc<VFUs::Shift> VFUShift;
class FUInfo {
  // DO NOT IMPLEMENT
  FUInfo(const FUInfo&);
  // DO NOT IMPLEMENT
  const FUInfo &operator=(const FUInfo&);

  /// mapping allocated instences to atom
  VFUDesc *ResSet[VFUs::NumCommonFUs];

  // Configuration function.
  void setupMemBus(unsigned latency, unsigned startInt, unsigned totalRes) {
    ResSet[VFUs::MemoryBus - VFUs::FirstFUType]
      = new VFUMemBus(latency, startInt, totalRes,
                      64,
                      //DataLayout.getPointerSizeInBits(),
                      // Dirty Hack.
                      64);
  }

  template<enum VFUs::FUTypes T>
  void setupBinOpRes(unsigned latency, unsigned startInt, unsigned totalRes) {
    ResSet[T - VFUs::FirstFUType]
      = new VFUBinOpFUDesc<T>(latency, startInt, totalRes,
                          // Dirty Hack.
                          64);
  }

  friend struct LuaConstraints;
public:
  
  FUInfo();

  ~FUInfo();

  template<class ResType>
  ResType *getFUDesc() const {
    return cast<ResType>(getFUDesc(ResType::getType()));
  }

  VFUDesc *getFUDesc(enum VFUs::FUTypes T) const {
    unsigned idx = (unsigned)T - (unsigned)VFUs::FirstFUType;
    assert(ResSet[idx] && "Bad resource!");
    return ResSet[idx];
  }

  typedef VFUDesc *const * iterator;
  typedef const VFUDesc *const * const_iterator;

  iterator begin() { return &ResSet[0]; }
  const_iterator begin() const { return &ResSet[0]; }

  iterator end() { 
    return begin() + (size_t)VFUs::LastCommonFUType -
      (size_t)VFUs::FirstFUType;
  }

  const_iterator end() const { 
    return begin() + (size_t)VFUs::LastCommonFUType -
      (size_t)VFUs::FirstFUType;
  }
};


FUInfo &vtmfus();

}

#endif
