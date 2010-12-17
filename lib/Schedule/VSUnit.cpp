//===------------ VSUnit.cpp - Translate LLVM IR to VSUnit  -----*- C++ -*-===//
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
// This file implement the VSUnit class, which represent the basic atom
// operation in hardware.
//
//===----------------------------------------------------------------------===//

#include "VSUnit.h"
#include "ForceDirectedScheduling.h"
#include "ScheduleDOT.h"

#include "vtm/PartitionInfo.h"
#include "vtm/MicroState.h"
#include "vtm/VFuncInfo.h"
#include "vtm/VTargetMachine.h"
#include "vtm/BitLevelInfo.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"

#include "llvm/ADT/SCCIterator.h"

#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "vtm-sunit"
#include "llvm/Support/Debug.h"

using namespace llvm;

namespace {
// Helper class to build Micro state.
struct MicroStateBuilder {
  MicroStateBuilder(const MicroStateBuilder&);     // DO NOT IMPLEMENT
  void operator=(const MicroStateBuilder&); // DO NOT IMPLEMENT

  VSchedGraph &State;
  MachineBasicBlock &MBB;
  MachineBasicBlock::iterator InsertPos;

  unsigned WireNum;
  unsigned OpId;
  LLVMContext &VMContext;
  const TargetInstrInfo &TII;
  MachineRegisterInfo &MRI;
  VFuncInfo &VFI;
  BitLevelInfo &BLI;

  SmallVector<VSUnit*, 8> SUnitsToEmit;
  SmallVector<VSUnit*, 8> DeferredSUnits;
  
  std::vector<MachineInstr*> InstsToDel;
  struct WireDef {
    unsigned WireNum;
    const char *SymbolName;
    MachineOperand *Op;
    unsigned EmitSlot;
    unsigned WriteSlot;

    WireDef(unsigned wireNum, const char *Symbol, MachineOperand *op,
            unsigned emitSlot, unsigned writeSlot)
      : WireNum(wireNum), SymbolName(Symbol), Op(op), EmitSlot(emitSlot),
      WriteSlot(writeSlot) {}

    bool isSymbol() const { return SymbolName != 0; }
    
    MachineOperand *getOperand() const { return Op; }
  };

  inline WireDef createWireDef(unsigned WireNum, VSUnit *A, MachineOperand *MO,
                               unsigned OpNum, unsigned emitSlot,
                               unsigned writeSlot){
    const char *Symbol = 0;
    if (A->getFUId().isBinded()) {
      assert(A->getFUType() == VFUs::MemoryBus
             && "Only support Membus at this moment!");
      assert(OpNum == 0 && "Bad Operand!");
      Symbol = VFI.allocateSymbol(VFUMemBus::getInDataBusName(A->getFUNum()));
    }
    
    return WireDef(WireNum, Symbol, MO, emitSlot, writeSlot);
  }
  
  typedef std::vector<WireDef*> DefVector;
  std::vector<DefVector> DefToEmit;
  
  // register number -> wire define.
  typedef std::map<unsigned, WireDef> SWDMapTy;
  SWDMapTy StateWireDefs;

  MicroStateBuilder(VSchedGraph &S, LLVMContext& Context, const VTargetMachine &TM,
                    BitLevelInfo &BitInfo)
  : State(S), MBB(*S.getMachineBasicBlock()), InsertPos(MBB.end()),
  WireNum(MBB.getNumber()), OpId(MBB.getNumber() << 24),
  VMContext(Context), TII(*TM.getInstrInfo()),
  MRI(MBB.getParent()->getRegInfo()),
  VFI(*MBB.getParent()->getInfo<VFuncInfo>()), BLI(BitInfo),
  DefToEmit(State.getTotalSlot() + 2 /*Dirty hack: The last slot never use!*/) {}

  DefVector &getDefsToEmitAt(unsigned Slot) {
    return DefToEmit[Slot - State.getStartSlot()];
  }

  MachineBasicBlock::iterator getInsertPos() { return InsertPos; }

  void emitSUnit(VSUnit *A) { SUnitsToEmit.push_back(A); }
  bool emitQueueEmpty() const { return SUnitsToEmit.empty(); }

  void defereSUnit(VSUnit *A) { DeferredSUnits.push_back(A); }

  MachineInstr *buildMicroState(unsigned Slot, bool IsLastSlot = false);

  void emitDeferredInsts() {
    // Emit the  deferred atoms before data path need it.
    while (!DeferredSUnits.empty()) {
      VSUnit *A = DeferredSUnits.pop_back_val();
      for (VSUnit::instr_iterator I = A->instr_begin(), E = A->instr_end();
          I != E; ++I)
        MBB.insert(InsertPos, *I);
    }
  }

  void fuseInstr(MachineInstr &Inst, VSUnit *A, bool IsLastSlot,
                 MachineInstrBuilder &DPInst, MachineInstrBuilder &CtrlInst);

  unsigned advanceToSlot(unsigned CurSlot, unsigned TargetSlot,
                         bool IsLastSlot = false) {
    assert(TargetSlot > CurSlot && "Bad target slot!");
    
    buildMicroState(CurSlot, IsLastSlot);
    SUnitsToEmit.clear();
    
    // Some states may not emit any atoms, but it may read the result from
    // previous atoms.
    // Note that SUnitsToEmit is empty now, so we do not emitting any new
    // atoms.
    while (++CurSlot != TargetSlot && !IsLastSlot)
      buildMicroState(CurSlot);

    return CurSlot;
  }

  // Clean up the basic block by remove all unused instructions.
  void clearUp() {
    while (!InstsToDel.empty()) {
      InstsToDel.back()->eraseFromParent();
      InstsToDel.pop_back();
    }
  }

};
}

//===----------------------------------------------------------------------===//

MachineInstr*
MicroStateBuilder::buildMicroState(unsigned Slot, bool IsLastSlot) {
  MachineBasicBlock &MBB = *State.getMachineBasicBlock();

  const TargetInstrDesc &TID = IsLastSlot ? TII.get(VTM::Terminator)
                                          : TII.get(VTM::Control);
  MachineInstrBuilder CtrlInst
    = BuildMI(MBB, InsertPos, DebugLoc(), TID).addImm(Slot);

  emitDeferredInsts();

  MachineInstrBuilder DPInst;
  if (!IsLastSlot) {
    DPInst
      = BuildMI(MBB, InsertPos, DebugLoc(), TII.get(VTM::Datapath)).addImm(Slot);
  }

  for (SmallVectorImpl<VSUnit*>::iterator I = SUnitsToEmit.begin(),
       E = SUnitsToEmit.end(); I !=E; ++I) {
    VSUnit *A = *I;
    for (VSUnit::instr_iterator II = A->instr_begin(), IE = A->instr_end();
        II != IE; ++II)
      fuseInstr(**II, A, IsLastSlot, DPInst, CtrlInst);
  }

  DefVector &DefsAtSlot = getDefsToEmitAt(Slot);
  // Emit the exported registers at current slot.
  for (DefVector::iterator I = DefsAtSlot.begin(), E = DefsAtSlot.end();
       I != E; ++I) {
    WireDef *WD = *I;

    MachineOperand *MO = WD->getOperand();

    // This operand will delete with its origin instruction.
    // Eliminate the dead register.
    if (MRI.use_empty(MO->getReg())) continue;

    // Export the register.
    CtrlInst.addMetadata(MetaToken::createDefReg(++OpId, WD->WireNum, VMContext));
    CtrlInst.addOperand(*MO);
  }

  return 0;
}

void MicroStateBuilder::fuseInstr(MachineInstr &Inst, VSUnit *A, bool IsLastSlot,
                                  MachineInstrBuilder &DPInst,
                                  MachineInstrBuilder &CtrlInst) {
  VTFInfo VTID = Inst;
  // FIXME: Inline datapath is allow in last slot.
  assert(!(IsLastSlot && VTID.hasDatapath())
    && "Unexpect datapath in last slot!");
  MachineInstrBuilder &Builder = VTID.hasDatapath() ? DPInst : CtrlInst;

  // Add the opcode metadata and the function unit id.
  Builder.addMetadata(MetaToken::createInstr(++OpId, Inst, A->getFUNum(),
                                             VMContext));
  typedef SmallVector<MachineOperand*, 8> OperandVector;
  OperandVector Ops(Inst.getNumOperands());

  // Remove all operand of Instr.
  while (Inst.getNumOperands() != 0) {
    unsigned i = Inst.getNumOperands() - 1;
    MachineOperand *MO = &Inst.getOperand(i);
    Inst.RemoveOperand(i);
    Ops[i] = MO;
  }

  unsigned EmitSlot = A->getSlot(),
           WriteSlot = A->getFinSlot();
  unsigned ReadSlot = EmitSlot;

  bool isReadAtEmit = VTID.isReadAtEmit();

  // We can not write the value to a register at the same moment we emit it.
  // Unless we read at emit.
  // FIXME: Introduce "Write at emit."
  if (WriteSlot == EmitSlot && !isReadAtEmit) ++WriteSlot;
  // Write to register operation need to wait one more slot if the result is
  // written at the moment (clock event) that the atom finish.
  if (VTID.isWriteUntilFinish()) ++WriteSlot;


  // We read the values after we emit it unless the value is read at emit.
  if (!isReadAtEmit) ++ReadSlot;

  DefVector &Defs = getDefsToEmitAt(WriteSlot);


  for (unsigned i = 0, e = Ops.size(); i != e; ++i) {
    MachineOperand *MO = Ops[i];

    if (!MO->isReg() || !MO->getReg()) {
      Builder.addOperand(*MO);
      continue;
    }

    unsigned RegNo = MO->getReg();

    // Remember the defines.
    if (MO->isDef() && EmitSlot != WriteSlot) {
      ++WireNum;
      WireDef WDef = createWireDef(WireNum, A, MO, i, EmitSlot, WriteSlot);

      SWDMapTy::iterator mapIt;
      bool inserted;
      tie(mapIt, inserted) = StateWireDefs.insert(std::make_pair(RegNo, WDef));

      assert(inserted && "Instructions not in SSA form!");
      WireDef *NewDef = &mapIt->second;

      unsigned BitWidth = BLI.getBitWidth(*MO);

      // Do not emit write to register unless it not killed in the current state.
      // FIXME: Emit the wire only if the value is not read in a function unit port.
      // if (!NewDef->isSymbol()) {
        MDNode *WireDefOp = MetaToken::createDefWire(WireNum, BitWidth, VMContext);
        Builder.addMetadata(WireDefOp);
        // Remember to emit this wire define if necessary.
        Defs.push_back(NewDef);
      // }
      continue;
    }

    // Else this is a use.
    SWDMapTy::iterator at = StateWireDefs.find(RegNo);
    // Using regster from previous state.
    if (at == StateWireDefs.end()) {
      Builder.addOperand(*MO);
      continue;
    }

    WireDef &WDef = at->second;

    // We need the value after it is written to register.
    if (WDef.WriteSlot < ReadSlot) {
      Builder.addOperand(*MO);
      continue;
    }

    assert(WDef.EmitSlot <= ReadSlot && "Dependencies broken!");

    if (WDef.isSymbol())
      Builder.addExternalSymbol(WDef.SymbolName);
    else
      Builder.addMetadata(MetaToken::createReadWire(WDef.WireNum, VMContext));
  }

  // Remove this instruction since they are fused to uc state.
  InstsToDel.push_back(&Inst);
}

//===----------------------------------------------------------------------===//

static inline bool top_sort_start(const VSUnit* LHS, const VSUnit* RHS) {
  if (LHS->getSlot() != RHS->getSlot())
    return LHS->getSlot() < RHS->getSlot();

  return LHS->getIdx() < RHS->getIdx();
}

static inline bool top_sort_finish(const VSUnit* LHS, const VSUnit* RHS) {
  if (LHS->getFinSlot() != RHS->getFinSlot())
    return LHS->getFinSlot() < RHS->getFinSlot();

  return LHS->getIdx() < RHS->getIdx();
}

MachineBasicBlock *VSchedGraph::emitSchedule(BitLevelInfo &BLI) {
  unsigned CurSlot = startSlot;
  VFuncInfo *VFI = MBB->getParent()->getInfo<VFuncInfo>();

  std::sort(SUnits.begin(), SUnits.end(), top_sort_start);

  // Build bundle from schedule units.
  MicroStateBuilder BTB(*this, MBB->getBasicBlock()->getContext(), TM, BLI);

  for (iterator I = begin(), E = end(); I != E; ++I) {
    VSUnit *A = *I;

    FuncUnitId FUId = A->getFUId();
    // Remember the active slot.
    if (FUId.isBinded())
      VFI->rememberAllocatedFU(FUId, A->getSlot(), A->getFinSlot());

    // Special case: Ret instruction use the function unit "FSMFinish".
    if (A->getOpcode() == VTM::VOpRet)
      VFI->rememberAllocatedFU(VFUs::FSMFinish, A->getSlot(), A->getSlot()+1);

    if (A->getSlot() != CurSlot)
      CurSlot = BTB.advanceToSlot(CurSlot, A->getSlot());
    
    if (MachineInstr *Inst = A->getFirstInstr()) {
      // Ignore some instructions.
      switch (Inst->getOpcode()) {
      case TargetOpcode::PHI:
        assert(BTB.emitQueueEmpty() && "Unexpected atom before PHI.");
        // Do not touch the PHIs, leave them at the beginning of the BB.
        continue;
      case TargetOpcode::COPY:
        // TODO: move this to MicroStateBuilder.
        MBB->remove(Inst);
        BTB.defereSUnit(A);
        continue;
      }

      BTB.emitSUnit(A);
    }
  }
  // Build last state.
  assert(!BTB.emitQueueEmpty() && "Expect atoms for last state!");
  BTB.advanceToSlot(CurSlot, CurSlot + 1, true);
  // Remove all unused instructions.
  BTB.clearUp();

  DEBUG(
  dbgs() << "After schedule emitted:\n";
  for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end();
      I != E; ++I) {
    MachineInstr *Instr = I;
    switch (Instr->getOpcode()) {
    case VTM::Control:
    case VTM::Datapath:
    case VTM::Terminator:
      ucState(*Instr).dump();
      break;
    default:
      Instr->dump();
      break;
    }
  }
  );

  return MBB;
}

//===----------------------------------------------------------------------===//
namespace {
class SubGraph;

struct SubGraphNode {
  unsigned FirstNode;
  const VSUnit *U;
  SubGraph *subGraph;

  SubGraphNode(unsigned First, const VSUnit *SU, SubGraph *SG)
    : FirstNode(First), U(SU), subGraph(SG) {}

  SubGraphNode(const SubGraphNode &O)
    : FirstNode(O.FirstNode), U(O.U), subGraph(O.subGraph) {}

  const SubGraphNode &operator=(const SubGraphNode &RHS);

  const VSUnit *getSUnit() const { return U; }
  unsigned getIdx() const { return U->getIdx(); }

  typedef SubGraphNode *result_type;
  result_type operator()(const VSUnit *U) const;

  typedef mapped_iterator<VSUnit::const_dep_iterator, SubGraphNode> ChildIt;

  ChildIt child_begin() const;
  ChildIt child_end() const;

  void dump() const {
    if (U) {
      dbgs() << U->getIdx() << " {";

      for (VSUnit::const_dep_iterator DI = U->dep_begin(), DE = U->dep_end();
           DI != DE;++DI)
        dbgs() << " [" << DI->getIdx() << "]";

      dbgs() << "}\n";
    } else
      dbgs() << "dummy\n";
  }
};

class SubGraph {
  typedef std::vector<SubGraphNode*> SubGrapNodeVec;
  typedef std::set<SubGraphNode*> SubGrapNodeSet;

  const VSchedGraph *G;
  const VSUnit *GraphEntry;
  // ModuloScheduleInfo *MSInfo;

  //Set of blocked nodes
  SubGrapNodeSet blocked;
  //Stack holding current circuit
  SubGrapNodeVec stack;
  //Map for B Lists
  std::map<SubGraphNode*, SubGrapNodeSet> B;
  //
  SubGrapNodeSet Visited;
  // SCC with least vertex.
  SubGrapNodeVec Vk;

  // SubGraph stuff
  typedef std::map<const VSUnit*, SubGraphNode*> CacheTy;
  CacheTy ExtCache;
  
  unsigned CurIdx;
  unsigned RecMII;
public:
  SubGraph(VSchedGraph *SG)
    : G(SG), GraphEntry(SG->getEntryRoot()), CurIdx(G->getEntryRoot()->getIdx()),
    RecMII(0) {}

  unsigned getRecMII() const { return RecMII; }

  ~SubGraph() { releaseCache(); }

  VSUnit::const_dep_iterator dummy_end() const { return GraphEntry->dep_end(); }

  // Create or loop up a node.
  SubGraphNode *getNode(const VSUnit *SU) {
    CacheTy::iterator at = ExtCache.find(SU);
    if (at != ExtCache.end())
      return at->second;

    SubGraphNode *NewNode = new SubGraphNode(CurIdx, SU, this);

    ExtCache.insert(std::make_pair(SU, NewNode));
    return NewNode;
  }

  void releaseCache() {
    // Release the temporary nodes in subgraphs.
    for (CacheTy::iterator I = ExtCache.begin(),
      E = ExtCache.end(); I != E; ++I) {
        delete I->second;
    }
    ExtCache.clear();
  }

  // Iterate over the subgraph start from idx.
  // Node: The atom list of FSMState already sort by getIdx.
  typedef mapped_iterator<VSchedGraph::const_iterator, SubGraphNode>
    nodes_iterator;

  nodes_iterator sub_graph_begin() {
    VSchedGraph::const_iterator I = G->begin();
    while ((*I)->getIdx() < CurIdx && I != G->end())
      ++I;

    return nodes_iterator(I, *getNode(*I));
  }
  nodes_iterator sub_graph_end() {
    return nodes_iterator(G->end(), *getNode(0));
  }

  void findAllCircuits();
  bool circuit(SubGraphNode *CurNode, SubGraphNode *LeastVertex);
  void addRecurrence();
  void unblock(SubGraphNode *N);
};
}

// GraphTraits
namespace llvm {
  template<> struct GraphTraits<SubGraphNode*> {
    typedef SubGraphNode NodeType;
    typedef SubGraphNode::ChildIt ChildIteratorType;
    static NodeType *getEntryNode(NodeType* N) { return N; }
    static inline ChildIteratorType child_begin(NodeType *N) {
      return N->child_begin();
    }
    static inline ChildIteratorType child_end(NodeType *N) {
      return N->child_end();
    }
  };
}

typedef GraphTraits<SubGraphNode*> VSUSccGT;

typedef scc_iterator<SubGraphNode*, VSUSccGT> dep_scc_iterator;

void SubGraph::unblock(SubGraphNode *N) {
  blocked.erase(N);

  while (!B[N].empty()) {
    SubGraphNode *W = *B[N].begin();
    B[N].erase(W);
    if(blocked.count(W))
      unblock(W);
  }
}

void SubGraph::addRecurrence() {
  DEBUG(dbgs() << "\nRecurrence:\n");
  //std::vector<VSUnit*> Recurrence;
  unsigned TotalLatency = 0;
  unsigned TotalDistance = 0;
  const VSUnit *LastAtom = stack.back()->getSUnit();
  
  for (SubGrapNodeVec::iterator I = stack.begin(), E = stack.end(); I != E; ++I) {
    SubGraphNode *N = *I;

    const VSUnit *A = N->getSUnit();
    TotalLatency += A->getLatency();
    
    VDEdge *Edge = LastAtom->getEdgeFrom(A);
    if (Edge->isBackEdge()) {
      //  assert(TotalDistance == 0 && "Multiple back edge?"); 
      DEBUG(dbgs() << "Backedge --> ");
    }
    TotalDistance += Edge->getItDst();
  
    DEBUG(N->dump());
    LastAtom = A;
    // Dirty Hack.
    //Recurrence.push_back(const_cast<VSUnit*>(A));
  }

  unsigned RecII = TotalLatency / TotalDistance;
  //MSInfo->addRecurrence(RecII, Recurrence);
  RecMII = std::max(RecMII, RecII);
  DEBUG(dbgs() << "RecII: " << RecII << '\n');
}

bool SubGraph::circuit(SubGraphNode *CurNode, SubGraphNode *LeastVertex) {
  bool ret = false;

  stack.push_back(CurNode);
  blocked.insert(CurNode);

  SubGrapNodeSet AkV;
  for (SubGraphNode::ChildIt I = CurNode->child_begin(),
       E = CurNode->child_end(); I != E; ++I) {
    SubGraphNode *N = *I;
    if (std::find(Vk.begin(), Vk.end(), N) != Vk.end())
      AkV.insert(N);
  }

  for (SubGrapNodeSet::iterator I = AkV.begin(), E = AkV.end(); I != E; ++I) {
    SubGraphNode *N = *I;
    if (N == LeastVertex) {
      //We have a circuit, so add it to recurrent list.
      addRecurrence();
      ret = true;
    } else if (!blocked.count(N) && circuit(N, LeastVertex))
      ret = true;
  }

  if (ret)
    unblock(CurNode);
  else
    for (SubGrapNodeSet::iterator I = AkV.begin(), E = AkV.end(); I != E; ++I) {
      SubGraphNode *N = *I;
      B[N].insert(CurNode);
    }
  
  // Pop current node.
  stack.pop_back();

  return ret;
}

void SubGraph::findAllCircuits() {
  VSUnit *ExitRoot = G->getExitRoot();
  unsigned ExitIdx = ExitRoot->getIdx();

  // While the subgraph not empty.
  while (CurIdx < ExitIdx) {
    DEBUG(dbgs() << "Current Idx: " << CurIdx << '\n');
    // Initialize the subgraph induced by {CurIdx, ...., ExitIdx}
    SubGraphNode *RootNode = getNode(ExitRoot);

    //Iterate over all the SCCs in the graph
    Visited.clear();
    Vk.clear();
    // least vertex in Vk
    SubGraphNode *LeastVertex = 0;

    //Find scc with the least vertex
    for (nodes_iterator I = sub_graph_begin(), E = sub_graph_end();
        I != E; ++I) {
      SubGraphNode *Node = *I;
      // If the Node visited.
      if (!Visited.insert(Node).second)
        continue;

      for (dep_scc_iterator SCCI = dep_scc_iterator::begin(RootNode),
           SCCE = dep_scc_iterator::end(RootNode); SCCI != SCCE; ++SCCI) {
        SubGrapNodeVec &nextSCC = *SCCI;  
        SubGraphNode *FirstNode = nextSCC.front();
        // If FirstNode visited.
        if (!Visited.insert(FirstNode).second)
          continue;
        
        if (nextSCC.size() == 1) {
          assert(!SCCI.hasLoop() && "No self loop expect in DDG!");
          continue;
        }

        // The entire SCC visited.
        Visited.insert(nextSCC.begin() + 1, nextSCC.end());
        // Find the lest vetex
        SubGraphNode *OldLeastVertex = LeastVertex;
        for (SubGrapNodeVec::iterator I = (*SCCI).begin(),
             E = (*SCCI).end();I != E; ++I) {
          SubGraphNode *CurNode = *I;
          if (!LeastVertex || CurNode->getIdx() < LeastVertex->getIdx())
            LeastVertex = CurNode;
        }
        // Update Vk if leastVe
        if (OldLeastVertex != LeastVertex)
          Vk = nextSCC; 
      }
    }

    // No SCC?
    if (Vk.empty())
      break;
    
    // Now we have the SCC with the least vertex.
    CurIdx = LeastVertex->getIdx();
    // Do some clear up.
    for (SubGrapNodeVec::iterator I = Vk.begin(), E = Vk.end(); I != E; ++I) {
      SubGraphNode *N = *I;
      blocked.erase(N);
      B[N].clear();
    }
    // Find the circuit.
    circuit(LeastVertex, LeastVertex);

    // Move forward.
    ++CurIdx;
    
    //
    releaseCache();
  }
}

//===----------------------------------------------------------------------===//
SubGraphNode::result_type SubGraphNode::operator()(const VSUnit *U) const {
  return U->getIdx() < FirstNode ? subGraph->getNode(0) 
                                 : subGraph->getNode(U); 
}

const SubGraphNode &SubGraphNode::operator=(const SubGraphNode &RHS) {
  FirstNode = RHS.FirstNode;
  U = RHS.U;
  subGraph = RHS.subGraph;
  return *this;
}

SubGraphNode::ChildIt SubGraphNode::child_begin() const {
  if (U) return ChildIt(U->dep_begin(), *this);
  // The node outside the subgraph.
  return ChildIt(subGraph->dummy_end(), *this);
}

SubGraphNode::ChildIt SubGraphNode::child_end() const {
  if (U) return ChildIt(U->dep_end(), *this);

  // The node outside the subgraph.
  return ChildIt(subGraph->dummy_end(), *this);
}
//===----------------------------------------------------------------------===//
unsigned VSchedGraph::computeRecMII() {
  //// Find all recurrents with Johnson's algorithm.
  SubGraph SG(this);
  SG.findAllCircuits();
  unsigned MaxRecII = SG.getRecMII();
  DEBUG(dbgs() << "RecMII: " << MaxRecII << '\n');
  return MaxRecII;
}
unsigned VSchedGraph::computeResMII() {
  std::map<FuncUnitId, unsigned> TotalResUsage;
  for (VSchedGraph::iterator I = begin(), E = end(); I != E; ++I) {
    VSUnit *SU = *I;
    if (SU->getFUId().isTrivial()) continue;
    
    ++TotalResUsage[SU->getFUId()];
  }

  unsigned MaxResII = 0;
  typedef std::map<FuncUnitId, unsigned>::iterator UsageIt;
  for (UsageIt I = TotalResUsage.begin(), E = TotalResUsage.end(); I != E; ++I){
      MaxResII = std::max(MaxResII,
                          I->second / I->first.getTotalFUs());
  }
  DEBUG(dbgs() << "ResMII: " << MaxResII << '\n');
  return MaxResII;
}

unsigned VSchedGraph::computeMII() {
  unsigned RecMII = computeRecMII();
  unsigned ResMII = computeResMII();
  return std::max(RecMII, ResMII);
}

void VSchedGraph::print(raw_ostream &OS) const {
}

void VSchedGraph::dump() const {
  print(dbgs());
}

void VSchedGraph::preSchedTopSort() {
  std::sort(SUnits.begin(), SUnits.end(), top_sort_start);
}

bool llvm::VSchedGraph::trySetLoopOp(VTFInfo &VTID) {
  assert(VTID->isTerminator() && "Bad instruction!");

  if (VTID->getOpcode() != VTM::VOpToState) return false;

  if (VTID.get().getOperand(1).getMBB() != MBB) return false;

  // Ok, rememeber this instruction as self eanble.
  LoopOp.setPointer(&VTID.get());
  return true;
}

static FDSBase *createLinearScheduler(VSchedGraph &G) {
  MachineFunction *F = G.getMachineBasicBlock()->getParent();
  const ConstraintsInfo &I = F->getInfo<VFuncInfo>()->getConstraints();

  switch (I.getScheduleAlgorithm()) {
  case ConstraintsInfo::FDS:   return new FDScheduler(G);
  case ConstraintsInfo::FDLS:  return new FDListScheduler(G);
  }
  return 0;
}

static FDSBase *createLoopScheduler(VSchedGraph &G) {
  MachineFunction *F = G.getMachineBasicBlock()->getParent();
  const ConstraintsInfo &I = F->getInfo<VFuncInfo>()->getConstraints();
  if (I.getPipeLineAlgorithm() == ConstraintsInfo::IMS)
    return new IteractiveModuloScheduling(G);
  
  return createLinearScheduler(G);
}


void VSchedGraph::schedule() {
  if (enablePipeLine())
    scheduleLoop();
  else
    scheduleLinear();
}


void VSchedGraph::scheduleLinear() {
  OwningPtr<FDSBase> Scheduler(createLinearScheduler(*this));

  while (!Scheduler->scheduleState())
    Scheduler->lengthenCriticalPath();

  DEBUG(Scheduler->dumpTimeFrame());
}

void VSchedGraph::scheduleLoop() {
  OwningPtr<FDSBase> Scheduler(createLoopScheduler(*this));
  unsigned II = computeMII();

  DEBUG(dbgs() << "MII: " << II << "...");
  // Ensure us can schedule the critical path.
  while (!Scheduler->scheduleCriticalPath(true))
    Scheduler->lengthenCriticalPath();

  Scheduler->setMII(II);
  while (!Scheduler->scheduleCriticalPath(true))
    Scheduler->increaseMII();

  // The point of current solution.
  typedef std::pair<unsigned, unsigned> SolutionPoint;
  SolutionPoint CurPoint
    = std::make_pair(Scheduler->getMII(), Scheduler->getCriticalPathLength());
  SmallVector<SolutionPoint, 3> NextPoints;

  double lastReq = 1e9;

  while (!Scheduler->scheduleState()) {
    double CurReq = Scheduler->getExtraResReq();
    if (lastReq > CurReq) {
      CurPoint = std::make_pair(Scheduler->getMII(),
        Scheduler->getCriticalPathLength());
      lastReq = CurReq;
      NextPoints.clear();
    }

    if (NextPoints.empty()) {
      NextPoints.push_back(std::make_pair(CurPoint.first + 1, CurPoint.second  + 1));
      if (Scheduler->getCriticalPathLength() >= Scheduler->getMII())
        NextPoints.push_back(std::make_pair(CurPoint.first + 1, CurPoint.second));
      NextPoints.push_back(std::make_pair(CurPoint.first, CurPoint.second  + 1));
      // Add both by default.
      CurPoint = std::make_pair(CurPoint.first + 1, CurPoint.second  + 1);
    }

    Scheduler->setMII(NextPoints.back().first);
    Scheduler->setCriticalPathLength(NextPoints.back().second);
    NextPoints.pop_back();
  }
  DEBUG(dbgs() << "SchedII: " << Scheduler->getMII()
               << " Latency: " << getTotalSlot() << '\n');
  unsigned FinalII = Scheduler->getMII();
  VSUnit *LoopOp = getLoopOp();
  assert(LoopOp && "Where is Loop op?");
  // Get finish slot?
  assert(LoopOp->getSlot() <= getStartSlot() + FinalII
         && "Loop can not restart in time!");

  // Ditry Hack: Fix the schedule of loop op.
  LoopOp->resetSchedule();
  LoopOp->scheduledTo(getStartSlot() + FinalII);
}

void VSchedGraph::viewGraph() {
  ViewGraph(this, this->getMachineBasicBlock()->getName());
}

//===----------------------------------------------------------------------===//

void VSUnit::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

void VDMemDep::print(raw_ostream &OS) const {

}

void VDCtrlDep::print(raw_ostream &OS) const {
}

void VDValDep::print(raw_ostream &OS) const {
}

unsigned VSUnit::getOpcode() const {
  if (MachineInstr *I =getFirstInstr())
    return I->getOpcode();

  return VTM::INSTRUCTION_LIST_END;
}

void VSUnit::scheduledTo(unsigned slot) {
  assert(slot && "Can not schedule to slot 0!");
  SchedSlot = slot;
}

void VSUnit::dropAllReferences() {
  for (dep_iterator I = dep_begin(), E = dep_end(); I != E; ++I)
    I->removeFromList(this);
}

void VSUnit::replaceAllUseBy(VSUnit *A) {
  while (!use_empty()) {
    VSUnit *U = use_back();

    U->setDep(U->getDepIt(this), A);
  }
}

VFUs::FUTypes VSUnit::getFUType() const {
  if (MachineInstr *Instr = getFirstInstr())
    return VTFInfo(*Instr).getFUType();

  return VFUs::Trivial;
}

void VSUnit::print(raw_ostream &OS) const {
  OS << "[" << getIdx() << "] ";

  for (const_instr_iterator I = instr_begin(), E = instr_end(); I != E; ++I) {
    MachineInstr *Instr = *I;

    VTFInfo VTID = *Instr;
    OS << Instr->getDesc().getName() << '\n';
    DEBUG(OS << *Instr << '\n');
  }

  OS << getFUId() << "\nAt slot: " << getSlot();
}
