//======- BitLevelABCOpt.cpp - Verilog target machine bit level ABC Opt-======//
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
// This file implement Verilog target machine-specific bit level optimization
// using ABC.
//
//===----------------------------------------------------------------------===//

#include "vtm/Passes.h"
#include "vtm/FUInfo.h"
#include "vtm/VFInfo.h"
#include "vtm/VRegisterInfo.h"
#include "vtm/VInstrInfo.h"
#include "vtm/VerilogBackendMCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Allocator.h"
#define DEBUG_TYPE "vtm-logic-synthesis"
#include "llvm/Support/Debug.h"

// The header of ABC
#define ABC_DLL
//#include "base/main/mainInt.h"
#include "src/base/main/main.h"
#include "src/map/fpga/fpga.h"
extern "C" {
  extern Abc_Ntk_t *Abc_NtkFpga(Abc_Ntk_t *pNtk, float DelayTarget,
                                int fRecovery, int fSwitching, int fLatchPaths,
                                int fVerbose);
}

using namespace llvm;
static cl::opt<bool>
EnableLogicOpt("vtm-enable-logic-optimization",
               cl::desc("Perform pre-schedule logic optimization by ABC"),
               cl::init(false));

namespace {
struct LogicNetwork {
  MachineBasicBlock *BB;
  MachineRegisterInfo &MRI;
  Abc_Ntk_t *Ntk;

  LogicNetwork(MachineBasicBlock *MBB)
    : BB(MBB), MRI(MBB->getParent()->getRegInfo())
  {
    Ntk = Abc_NtkAlloc(ABC_NTK_STRASH, ABC_FUNC_AIG, 1);
    Ntk->pName = Extra_UtilStrsav(BB->getName().str().c_str());
  }

  ~LogicNetwork() {
    Abc_NtkDelete(Ntk);
    // Called by its destructor.
    //NtkObjAllocator.DestroyAll();
  }

  // Helper class
  struct NetworkObj {
    Abc_Obj_t *Obj;
    MachineOperand MO;
    unsigned ExposedUses;

    // The object is exposed by default.
    NetworkObj(Abc_Obj_t *O, MachineOperand &M, unsigned NumUses)
      : Obj(O), MO(M), ExposedUses(NumUses)
    {
      // Clear the flags.
      MO.clearParent();
      if (MO.isReg()) {
        MO.setIsDef(false);
        MO.setIsKill(false);
      }
    }

    NetworkObj(const NetworkObj&);     // DO NOT IMPLEMENT
    void operator=(const NetworkObj&); // DO NOT IMPLEMENT

    unsigned decreaseUses() {
      if (ExposedUses) --ExposedUses;

      return ExposedUses;
    }
  };

  SpecificBumpPtrAllocator<NetworkObj> NtkObjAllocator;
  // Mapping register number to logic network port.
  typedef DenseMap<MachineOperand, NetworkObj*, VMachineOperandValueTrait> ObjMapTy;
  // Nodes.
  ObjMapTy Nodes;

  // The indices of the black box instructions.
  typedef DenseMap<MachineInstr*, unsigned> IdxMapTy;
  IdxMapTy IdxMap;

  // Map the Abc_Obj_t name to Instruction.
  typedef StringMap<MachineInstr*> InstrMapTy;
  InstrMapTy FIMap;
  typedef MachineBasicBlock::iterator IPTy;

  typedef StringMap<MachineOperand> MOMapTy;
  MOMapTy MOMap;

  unsigned getInstIdx(MachineInstr *MI) const {
    return IdxMap.lookup(MI);
  }

  MachineInstr *getDefMI(Abc_Obj_t *FI) const {
    return FIMap.lookup(Abc_ObjName(Abc_ObjRegular(FI)));
  }

  unsigned getDefIdx(Abc_Obj_t *FI) const {
    if (MachineInstr *DefMI = getDefMI(FI))
      return getInstIdx(DefMI);

    return 0;
  }

  //
  NetworkObj *getObj(MachineOperand MO) {
    // Clear the flags berore looking up the object.
    if (MO.isReg()) {
      MO.setIsDef(false);
      MO.setIsKill(false);
    }

    ObjMapTy::iterator inNodes = Nodes.find(MO);

    if (inNodes != Nodes.end()) return inNodes->second;

    return 0;
  }


  bool hasImplicitBitslice(MachineOperand &MO) {
    if (!MO.isReg() || !MO.getReg()) return false;

    MachineRegisterInfo::def_iterator I = MRI.def_begin(MO.getReg());
    assert(I != MRI.def_end() && llvm::next(I) == MRI.def_end() &&
           "Not in SSA form!");
    return VInstrInfo::getBitWidth(MO)!=VInstrInfo::getBitWidth(I.getOperand());
  }

  NetworkObj *getOrCreateObj(MachineOperand &MO) {
    if (MO.isImm()) {
      Abc_Obj_t *Obj = 0;
      if (VInstrInfo::isAllOnes(MO))
        Obj = Abc_AigConst1(Ntk);
      else if (VInstrInfo::isAllZeros(MO))
        Obj = Abc_ObjNot(Abc_AigConst1(Ntk));

      // If we can handle the constant...
      if (Obj) // Constants are not exposed.
        return new (NtkObjAllocator.Allocate()) NetworkObj(Obj, MO, 0);
    }

    NetworkObj *NtkObj = getObj(MO);

    // Object not existed, create a PI for the MO now.
    if (NtkObj == 0) {
      Abc_Obj_t *Obj = Abc_NtkCreatePi(Ntk);
      std::string PIName = "i" + utostr_32(Abc_ObjId(Abc_ObjRegular(Obj)));
      Abc_ObjAssignName(Obj, const_cast<char*>(PIName.c_str()), 0);
      char *Name = Abc_ObjName(Abc_ObjRegular(Obj));

      // Remember the MI that define this MO so we can compute the insert
      // position.
      if (MO.isReg() && MO.getReg()) {
        MachineInstr *DefMI = MRI.getVRegDef(MO.getReg());
        assert(DefMI && "VReg not defined?");
        if (DefMI->getParent() == BB)
          FIMap.GetOrCreateValue(Name, DefMI);
      }
      // PIs are not exposed.
      NtkObj = new (NtkObjAllocator.Allocate()) NetworkObj(Obj, MO, 0);

      // Use the normalized MO.
      // Map the PI to MO.
      MOMap.GetOrCreateValue(Name, NtkObj->MO);
      Nodes.insert(std::make_pair(NtkObj->MO, NtkObj));
      // The kill flags may broken during the rebuil process.
      if (MO.isReg() && MO.getReg()) MRI.clearKillFlags(MO.getReg());
    }

    return NtkObj;
  }

  MachineOperand getOperand(Abc_Obj_t *Obj, unsigned SizeInBits = 0) {
    // Else look it up in the FO map.
    MachineOperand &MO =
      MOMap.GetOrCreateValue(Abc_ObjName(Obj),
                             VInstrInfo::CreateReg(0, SizeInBits)).second;

    // Allocate the register.
    if (MO.isReg() && MO.getReg() == 0)
      MO.ChangeToRegister(MRI.createVirtualRegister(VTM::DRRegisterClass),
                          false);

    assert(VInstrInfo::getBitWidthOrZero(MO)
           && "Instruction defining Obj not inserted yet?");

    return MO;
  }

  void cleanUp() {
    // Build the POs
    typedef ObjMapTy::iterator it;

    for (it I = Nodes.begin(), E = Nodes.end(); I != E; ++I) {
      NetworkObj &Node = *I->second;

      // Only create PO for exposed node.
      if (!Node.ExposedUses) continue;

      Abc_Obj_t *PO = Abc_NtkCreatePo(Ntk);
      Abc_Obj_t *&Obj = Node.Obj;
      Abc_ObjAddFanin(PO, Obj);
      Obj = PO;

      std::string POName = "o" + utostr_32(Abc_ObjId(Abc_ObjRegular(Obj)));
      Abc_ObjAssignName(Obj, const_cast<char*>(POName.c_str()), 0);

      // Remember the MO.
      MOMap.GetOrCreateValue(Abc_ObjName(Abc_ObjRegular(Obj)), Node.MO);
    }

    // Clean up the aig.
    Abc_AigCleanup((Abc_Aig_t *)Ntk->pManFunc);

    // Create default names.
    //Abc_NtkAddDummyPiNames(Ntk);
    //Abc_NtkAddDummyPoNames(Ntk);
    // We do not have boxes.
    //Abc_NtkAddDummyBoxNames(Ntk);

    // Check the Aig
    assert(Abc_NtkCheck(Ntk) && "The AIG construction has failed!");
  }

  // Call abc routine to synthesis the logic network.
  void synthesis() {
    // FIXME: Do not synthesis if the network is very small.
    // FIXME: Call dispatch command to run user script?
    int res;
    // Use the resyn flow, which invoking:
    //  balance
    Ntk = Abc_NtkBalance(Ntk, false, false, false);
    //  rewrite
    res = Abc_NtkRewrite(Ntk, 0, 0, 0, 0, 0);
    assert(res && "Rewrite fail during logic synthesis!");
    //  rewrite -z
    res = Abc_NtkRewrite(Ntk, 0, 1, 0, 0, 0);
    assert(res && "Rewrite fail during logic synthesis!");
    //  balance
    Ntk = Abc_NtkBalance(Ntk, false, false, false);
    //  rewrite -z
    res = Abc_NtkRewrite(Ntk, 0, 1, 0, 0, 0);
    assert(res && "Rewrite fail during logic synthesis!");
    //  balance
    Ntk = Abc_NtkBalance(Ntk, false, false, false);
  }

  void performLUTMapping() {
    // Map the network to LUTs
    Ntk = Abc_NtkFpga(Ntk, 1, 0, 0, 0, 0);
    assert(Ntk && "Fail to perform LUT mapping!");

    // Translate the network to netlist.
    Ntk = Abc_NtkToNetlist(Ntk);
    assert(Ntk && "Network doese't exist!!!");
    assert(Abc_NtkHasBdd(Ntk) && "Expect Bdd after LUT mapping!");
    int res = Abc_NtkBddToSop(Ntk, 0);
    assert(res && "BddToSop fail!");
    (void) res;
  }

  // Function for logic network building.
  template <typename BuildFunc>
  bool buildBinaryOpNode(MachineInstr *MI, BuildFunc F) {
    if (hasImplicitBitslice(MI->getOperand(1)) ||
        hasImplicitBitslice(MI->getOperand(2)))
      return false;

    NetworkObj *Op0 = getOrCreateObj(MI->getOperand(1));
    NetworkObj *Op1 = getOrCreateObj(MI->getOperand(2));

    // They are used in the logic network.
    Op0->decreaseUses();
    Op1->decreaseUses();
    // Create the internal node for this machine instruction.
    Abc_Obj_t *Res = F((Abc_Aig_t *)Ntk->pManFunc, Op0->Obj, Op1->Obj);
    MachineOperand &ResMO = MI->getOperand(0);

    unsigned NumUse = std::distance(MRI.use_begin(ResMO.getReg()), MRI.use_end());
    NetworkObj *NtkObj =
      new (NtkObjAllocator.Allocate()) NetworkObj(Res, ResMO, NumUse);
    Nodes.insert(std::make_pair(NtkObj->MO, NtkObj));
    return true;
  }

  bool buildNotNode(MachineInstr *MI) {
    if (hasImplicitBitslice(MI->getOperand(1)))
      return false;

    NetworkObj *Op0 = getOrCreateObj(MI->getOperand(1));
    // It is used in the logic network.
    Op0->decreaseUses();

    // Create the internal node for this machine instruction.
    Abc_Obj_t *Res = Abc_ObjNot(Op0->Obj);
    MachineOperand &ResMO = MI->getOperand(0);

    unsigned NumUse = std::distance(MRI.use_begin(ResMO.getReg()), MRI.use_end());
    NetworkObj *NtkObj =
      new (NtkObjAllocator.Allocate()) NetworkObj(Res, ResMO, NumUse);
    Nodes.insert(std::make_pair(NtkObj->MO, NtkObj));
    return true;
  }

  bool addInstr(MachineInstr *MI);
  void buildLUTInst(Abc_Obj_t *Obj, VFInfo *VFI,
                      MachineBasicBlock::iterator IP);

  // Sort the nodes in Netlist.
  // Helper data structure to sort the nodes in logic network.
  struct ObjIdx {
    Abc_Obj_t *Obj;
    unsigned MaxFIId;
    // Topological order number.
    unsigned Idx;

    static inline bool sort_asap(const ObjIdx &LHS, const ObjIdx &RHS) {
      return LHS.MaxFIId < RHS.MaxFIId ||
        (LHS.MaxFIId == RHS.MaxFIId && LHS.Idx < RHS.Idx);
    }

    void print(raw_ostream &OS) const {
      OS << "Node: " << Abc_ObjName(Abc_ObjRegular(Obj))
        << " Id: " << Abc_ObjId(Abc_ObjRegular(Obj)) << " FI: {";

      Abc_Obj_t *FI;
      int j;
      Abc_ObjForEachFanin(Obj, FI, j) {
        OS << Abc_ObjName(Abc_ObjRegular(FI)) << ", ";
      }

      OS << "} FO: " << Abc_ObjName(Abc_ObjRegular(Abc_ObjFanout0(Obj)))
        << " Idx: " << Idx << " MaxFIId: " << MaxFIId << '\n';
    }

    void dump() const { print(dbgs()); }
  };

  void sortNodes(SmallVectorImpl<ObjIdx> &ObjIdxList) {
    DenseMap<Abc_Obj_t*, unsigned> NodeIdxMap;

    Abc_Obj_t *Obj;
    int i;

    Abc_NtkForEachNode(Ntk, Obj, i) {
      // Node already visited.
      if (NodeIdxMap.count(Obj))
       continue;

      computeIdx(Obj, ObjIdxList, NodeIdxMap);
    }

    // Sort the nodes by its index.
    std::sort(ObjIdxList.begin(), ObjIdxList.end(), ObjIdx::sort_asap);

    DEBUG(dbgs() << "Sorted ObjList:\n";
      for (unsigned i = 0, e = ObjIdxList.size(); i != e; ++i)
        ObjIdxList[i].dump();
    );
  }

  unsigned computeIdx(Abc_Obj_t *Obj, SmallVectorImpl<ObjIdx> &ObjIdxList,
                      DenseMap<Abc_Obj_t*, unsigned> &NodeIdxMap) {
    // Compute the FI index.
    Abc_Obj_t *FI;
    int j;

    unsigned MaxFIIdx = 0;

    // Compute the max FI index.
    Abc_ObjForEachFanin(Obj, FI, j) {
      DEBUG(dbgs() << "visiting FI: " << Abc_ObjName(Abc_ObjRegular(FI))
                   << " " << Abc_ObjType(FI) << '\n');
      // Get the instruction that defines the PI.
      if (unsigned FIIdx = getDefIdx(FI)) {
        MaxFIIdx = std::max(MaxFIIdx, FIIdx);
        continue;
      }

      FI = Abc_ObjFanin0Ntk(FI);

      // PIs whose position is not important.
      if (Abc_ObjIsPi(FI)) continue;

      // Is the node already visited?
      DenseMap<Abc_Obj_t*, unsigned>::iterator at = NodeIdxMap.find(FI);

      if (at != NodeIdxMap.end()) {
        MaxFIIdx = std::max(at->second, MaxFIIdx);
        continue;
      }

      // Otherwise visit the depending node now, and push all child node
      // of current node into the list.
      MaxFIIdx = std::max(computeIdx(FI, ObjIdxList, NodeIdxMap), MaxFIIdx);
    }

    // Create the index.
    ObjIdx Idx;
    Idx.Obj = Obj;
    // The nodes are pushed into the list in topological order,
    // so use the size of the list as topological order index.
    Idx.Idx = ObjIdxList.size();
    Idx.MaxFIId = MaxFIIdx;

    DEBUG(dbgs() << "Built Idx:\n"; Idx.dump());

    // Remember the index.
    ObjIdxList.push_back(Idx);
    NodeIdxMap.insert(std::make_pair(Obj, MaxFIIdx));

    return MaxFIIdx;
  }
};

struct LogicSynthesis : public MachineFunctionPass {
  static char ID;

  VFInfo *VFI;

  LogicSynthesis() : MachineFunctionPass(ID), VFI(0) {
    Abc_Start();
    // FIXME: Set complex library?
    Fpga_SetSimpleLutLib(VFUs::MaxLutSize);
  }

  ~LogicSynthesis() {
    Abc_Stop();
  }

  const char *getPassName() const { return "Pre-schedule Logic Synthesis"; }

  bool runOnMachineFunction(MachineFunction &MF);
  bool synthesisBasicBlock(MachineBasicBlock *BB);
};
}
//===----------------------------------------------------------------------===//
// Implementation of logic network class.
bool LogicNetwork::addInstr(MachineInstr *MI) {
  switch (MI->getOpcode()) {
  default: break;
  case VTM::VOpAnd:
    if (buildBinaryOpNode(MI, Abc_AigAnd)) return true;
    break;
  case VTM::VOpOr:
    if (buildBinaryOpNode(MI, Abc_AigOr)) return true;
    break;
  case VTM::VOpXor:
    if (buildBinaryOpNode(MI, Abc_AigXor)) return true;
    break;
  case VTM::VOpNot:
    if (buildNotNode(MI)) return true;
    break;
  }

  // Add the black box instruction to index map,
  // make sure the index is no-zero by adding 1 to the size of the map.
  IdxMap.insert(std::make_pair(MI, IdxMap.size() + 1));
  return false;
}

void LogicNetwork::buildLUTInst(Abc_Obj_t *Obj, VFInfo *VFI,
                                MachineBasicBlock::iterator IP) {
  SmallVector<MachineOperand, 2> Ops;
  Abc_Obj_t *FO = Abc_ObjFanout0(Obj), *FI;
  DEBUG(dbgs() << Abc_ObjName(FO) << '\n');
  if (Abc_NodeIsConst(Obj)) {
    MOMapTy::iterator at = MOMap.find(Abc_ObjName(FO));
    assert(at != MOMap.end() && "MachineOperand for constant node not found!");
    MachineOperand MO = at->second;

    // Internal constant nodes should be already optimized away by ABC.
    assert(getObj(MO) && getObj(MO)->ExposedUses
           && "Expected constant node are exposed!");
    uint64_t Imm = Abc_NodeIsConst0(Obj) ? UINT64_C(0) : ~UINT64_C(0);

    // Replace the register by immediate.
    typedef MachineRegisterInfo::use_iterator it;
    for (it I = MRI.use_begin(MO.getReg()); I != MachineRegisterInfo::use_end();
         /*++I*/)
      (I++).getOperand().ChangeToImmediate(Imm);

    return;
  }

  // Get all operands and compute the bit width of the result.
  Ops.clear();
  unsigned SizeInBits = 0;
  int j;
  Abc_ObjForEachFanin(Obj, FI, j) {
    DEBUG(dbgs() << "\tBuilt MO for FI: " << Abc_ObjName(FI) << '\n');
    MachineOperand MO = getOperand(FI);
    assert((SizeInBits == 0 || SizeInBits == VInstrInfo::getBitWidth(MO))
           && "Operand SizeInBits not match!");
    Ops.push_back(MO);
    SizeInBits = VInstrInfo::getBitWidth(MO);
  }

  // Get the result.
  DEBUG(dbgs() << "Built LUT for FO: " << Abc_ObjName(FO) << "\n\n");
  assert(SizeInBits && "Expect non-zero size output!");
  MachineOperand DefMO = getOperand(FO, SizeInBits);
  assert(VInstrInfo::getBitWidth(DefMO) == SizeInBits && "Result SizeInBits not match!");
  DefMO.setIsDef(true);

  // Get the sum of product table.
  // FIXME: data can be also encoded by Abc_SopToTruth.
  char *data = (char*)Abc_ObjData(Obj);
  DEBUG(dbgs() << data << '\n');

  MachineInstrBuilder Builder =
    BuildMI(*BB, IP, DebugLoc(), VInstrInfo::getDesc(VTM::VOpLUT))
    .addOperand(DefMO)
    .addExternalSymbol(VFI->allocateSymbol(data), Abc_ObjFaninNum(Obj))
    .addOperand(VInstrInfo::CreatePredicate())
    .addOperand(VInstrInfo::CreateTrace());

  for (unsigned k = 0, e = Ops.size(); k != e; ++k)
    Builder.addOperand(Ops[k]);

  DEBUG(Builder->dump());
}

//===----------------------------------------------------------------------===//
// Implement of the logic synthesis pass.
char LogicSynthesis::ID = 0;

Pass *llvm::createLogicSynthesisPass() {
  return new LogicSynthesis();
}

bool LogicSynthesis::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  VFI = MF.getInfo<VFInfo>();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I)
    Changed = synthesisBasicBlock(I);

  // Verify the function.
  MF.verify(this);

  return Changed;
}

bool LogicSynthesis::synthesisBasicBlock(MachineBasicBlock *BB) {
  SmallVector<MachineInstr*, 16> InstrsToRewrite;
  LogicNetwork Ntk(BB);

  DEBUG(dbgs() << "Before logic synthesis:\n";
        BB->dump(););

  typedef MachineBasicBlock::iterator it;
  for (it I = BB->begin(), E = BB->end(); I != E; /*++I*/) {
    MachineInstr *MI = I++;

    // Try to add the instruction into the logic network.
    if (!Ntk.addInstr(MI)) continue;

    InstrsToRewrite.push_back(MI);
  }

  // Not change at all.
  if (InstrsToRewrite.empty()) return false;

  // Erase the instructions before rewriting them.
  while (!InstrsToRewrite.empty())
    InstrsToRewrite.pop_back_val()->eraseFromParent();

  // Clean up the network, prepare for logic optimization.
  Ntk.cleanUp();

  // Synthesis the logic network.
  if (EnableLogicOpt) Ntk.synthesis();

  // Map the logic network to LUTs
  Ntk.performLUTMapping();

  SmallVector<LogicNetwork::ObjIdx, 32> ObjIdxList;
  Ntk.sortNodes(ObjIdxList);

  // Build the BB from the logic netlist.
  MachineBasicBlock::iterator IP = BB->getFirstNonPHI();

  for (unsigned i = 0, e = ObjIdxList.size(); i != e; ++i) {
    LogicNetwork::ObjIdx Idx = ObjIdxList[i];

    while (Ntk.getInstIdx(IP) <= Idx.MaxFIId)
      ++IP;

    DEBUG(dbgs() << "For "; Idx.dump(););
    Ntk.buildLUTInst(Idx.Obj, VFI, IP);
  }

  DEBUG(dbgs() << "After logic synthesis:\n";
        BB->dump(););

  return true;
}
