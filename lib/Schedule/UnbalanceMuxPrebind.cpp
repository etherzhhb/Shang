//=- PrebindUnbalanceMux.cpp- Allocate Multiplexer for Prebound Function units - C++ -=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Allocate Multiplexer for Prebound Function units.
//
//===----------------------------------------------------------------------===//
#include "MuxPrebinding.h"
#include <algorithm>
#include <vector>
#include "llvm/ADT/SmallVector.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/CodeGen/Passes.h"

using namespace llvm;

namespace {

struct PrebindUnbalanceMux : public PrebindMuxBase {
  static char ID;
  std::vector<std::pair<ucOperand,unsigned> > TmpOpSet;
  PrebindUnbalanceMux() : PrebindMuxBase(ID){
    initializeMachineBlockFrequencyInfoPass(*PassRegistry::getPassRegistry());
  }
  void getFreq(PortFanInMapTy &TmpFanInInfo,MachineBasicBlock *MBB,
               std::pair<unsigned, unsigned> InstInfoPair,ucOperand &MO);
  void rmPortInTmpFanIn(PortFanInMapTy &TmpFanInInfo,unsigned MaxMuxSize);
  void sortFanInByFreq();
  void allocateBalanceMux();
};
}
char PrebindUnbalanceMux::ID = 0;

Pass *llvm::createPrebindUnbalanceMuxPass() {
  return new PrebindUnbalanceMux();
}

INITIALIZE_PASS_BEGIN(PrebindUnbalanceMux, "Bind-Mux",
  "Bind Mux", true, false)
INITIALIZE_PASS_END(PrebindUnbalanceMux, "Bind-Mux",
  "Bind Mux", true, false)

static inline bool sort_type(std::pair<ucOperand,unsigned> LHS,
                             std::pair<ucOperand,unsigned> RHS) {
    return LHS.second > RHS.second;
}

void PrebindUnbalanceMux::getFreq(PortFanInMapTy &TmpFanInInfo,MachineBasicBlock *MBB,
                                  std::pair<unsigned, unsigned> InstInfoPair,ucOperand &MO){
  //Get Block's Freq
  MachineBlockFrequencyInfo &BFI = getAnalysis<MachineBlockFrequencyInfo>();
  BlockFrequency  InsertFreq = BFI.getBlockFreq(MBB);
  // Remember the fan-in for the port.
  TmpFanInInfo[InstInfoPair].FindAndConstruct(MO);
  //Insert the Frequency to FanInInfo.OpSet.second
  TmpFanInInfo[InstInfoPair].FindAndConstruct(MO).second = InsertFreq.getFrequency();
}

void PrebindUnbalanceMux::rmPortInTmpFanIn(PortFanInMapTy &TmpFanInInfo,unsigned MaxMuxSize){
  // Remove the ports with mux that can fit into a single cycle.
  for (PortFanInMapTy::iterator I = TmpFanInInfo.begin(), E = TmpFanInInfo.end();
    I != E; /*++I*/) {
      PortFanInMapTy::iterator at = I;
      ++I;

      if (at->second.size() <= MaxMuxSize)
        TmpFanInInfo.erase(at);
  }
}

void PrebindUnbalanceMux::sortFanInByFreq(){
  //Sort the Vector then put back in TmpFanInInfo.OpSet
  for (PortFanInMapTy::iterator I = TmpFanInInfo.begin(), E = TmpFanInInfo.end();
    I != E; ++I) {
      OpSet &FIs = I->second;
      DEBUG(FuncUnitId(I->first.first).print(dbgs());
      dbgs() << "@" << I->first.second << ": #Fan-in "
        << FIs.size() << '\n';);

      //Put OpSet into the smallvector to sort by type
      for (OpSet::iterator OI = FIs.begin(), OE = FIs.end(); OI != OE; ++OI) {
      //  dbgs()<<" the Freq in Map OpSet  "<<OI->second<<'\n';
        TmpOpSet.push_back(std::make_pair(OI->first,OI->second));
      }

      //Rank ordering the Freq in TmpOpSet
      std::sort(TmpOpSet.begin(), TmpOpSet.end(), sort_type);

      unsigned ivec=0;
      for (OpSet::iterator OI = FIs.begin(), OE = FIs.end(); OI != OE; ++OI) {
        //Order the type OpSet in TmpFanInInfo
        OI->first = TmpOpSet[ivec].first;
        OI->second = TmpOpSet[ivec].second;
        ++ivec;
      }
      TmpOpSet.clear();
  }
}

void PrebindUnbalanceMux::allocateBalanceMux() {
  sortFanInByFreq();
  // For each port.
  for (PortFanInMapTy::iterator I = TmpFanInInfo.begin(), E = TmpFanInInfo.end();
       I != E; ++I) {
    OpSet &FIs = I->second;

    // the pair(Id.getData(), i) in TmpFanInInfo
    FUPortTy Port = I->first;
    //Get the same iterator in FanInInfo
    PortFanInMapTy::iterator FanInInfoNumAt = FanInInfo.find(Port);
    OpSet &FIs2 = FanInInfoNumAt->second;

    DEBUG(FuncUnitId(I->first.first).print(dbgs());
          dbgs() << "@" << I->first.second << ": #Fan-in "
                 << FIs.size() << '\n';);

    unsigned FICounter = 0;
    //MuxCounter = 1;
    unsigned NumMuxFanins = (FIs.size() + MaxMuxSize) / MaxMuxSize;
    assert(NumMuxFanins > 1 && "Small fan-in set not eliminated?");
    //Get the summary of all the frequency in TmpFanInInfo.OpSet
    unsigned FreqSum = 0;
    for (OpSet::iterator OI = FIs.begin(), OE = FIs.end(); OI != OE; ++OI) {
      FreqSum = FreqSum + OI->second;
    }

    unsigned int DbgNum = 0;
    unsigned int CoutNumForMux = 0;
    for (OpSet::iterator OI = FIs.begin(), OE = FIs.end(); OI != OE; ++OI) {
      ++DbgNum;
  //    dbgs()<<"The Number is : "<<DbgNum<<" ! Each OpSet in TmpFanInInfo has The Freq "<<OI->second<<'\n';
      // the ucOperand in TmpFanInInfo.OpSet.ucOperand
      ucOperand &Oprd = OI->first;

      float CompSum = (float) OI->second/FreqSum;
    //  dbgs()<<"Float CompSum is "<<CompSum<<'\n';
      //Get the iterator in FanInInfo.OpSet
      OpSet::iterator OpSetNumAt = FIs2.find(Oprd);
      if ((CompSum >= 0.15)&&(CoutNumForMux < (MaxMuxSize-1)) ){ //
        OpSetNumAt->second = 0;
        CoutNumForMux++;
      } else {
        OpSetNumAt->second = MuxCounter;
        // Allocate and assign the mux number.
        // OI->second = MuxCounter;
        // Increase the allocted mux size.
        ++MuxSizeInfo[MuxCounter];
        // If the fanin number exceed the maximum mux size, allocate a new mux.
        if (++FICounter >= NumMuxFanins) {
          ++MuxCounter;
          FICounter = 0;
        }
      }
      // Remember the port bitwidth, there maybe fanins with different width.
      PortBitwidthInfo[I->first] = std::max(PortBitwidthInfo[I->first],
                                            OI->first.getBitWidth());
    }

    //for (OpSet::iterator OI = FIs2.begin(), OE = FIs2.end(); OI != OE; ++OI) {
    //  dbgs()<<"The MuxNumber in FanInInfo a OpSet "<<OI->second<<'\n';
    //}

    // Allocate a new mux for other ports.
    ++MuxCounter;
  }
  TmpFanInInfo.clear();
}
