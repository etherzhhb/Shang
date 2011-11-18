//======- BitLevelABCOpt.cpp - Verilog target machine bit level ABC Opt-======//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implement Verilog target machine-specific bit level optimization
// using ABC.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "vtm-abcopt"
#include <time.h>
#include <stack>
#include "vtm/MicroState.h"
#include "vtm/VInstrInfo.h"
#include "vtm/VFInfo.h"
#include "vtm/Passes.h"
#include "vtm/VTM.h"
#include "VGenInstrInfo.inc"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/Debug.h"

// The head file of ABC.
extern "C" {
#include "aig/saig/saig.h"
#include "abc.h"
#include "extra.h"
#include "ioAbc.h"
#include "base/main/main.h"
#include "base/main/mainInt.h"


void   Abc_Start();
void   Abc_Stop();
Abc_Ntk_t *Abc_NtkAlloc(Abc_NtkType_t Type, Abc_NtkFunc_t Func, int fUseMemMan);
}

using namespace llvm;

namespace {

  class BitLevelABCOpt : public MachineFunctionPass {

  public:

    static char ID;

    BitLevelABCOpt();

    void getAnalysisUsage(AnalysisUsage &AU) const;

    bool runOnMachineFunction(MachineFunction &MF);

    // Store the information of the MachineOperand.
    struct RegInfo {
      MachineOperand MO;            // The MO.
      MachineInstr *MI;             // The MachineInstr which the output MO is MOOut.
      unsigned width;               // The width of the MO.
      unsigned pointerInVec;       // The pointer of the vector which stores Abc_Obj_t,
      // it will be used when we construct MIs.
      int numInput;                // The number of MO as input.
      int numOutput;               // The number of MO as output.

      // Default construct function.
      RegInfo()
        : MO(MachineOperand::CreateReg(0, false)),width(0),
          pointerInVec(0), numInput(0), numOutput(0) {}
    };

    // Store the information of the newly constructed MI.
    struct InstrInfo {
      unsigned MIOpcode;        // Opcode.
      int MOIn0;                // First input reg number.
      int MOIn1;                // Second input reg number, it is useless for inv
                                 // instruction.
    };

    // Get reg number from the name of Abc_Obj_t.
    int getRegNo(Abc_Obj_t *obj);

    // Dump ABC_Ntk_t.
    void dumpNtk(Abc_Ntk_t *pABC, bool blif);

    // Dump MOInfoMap, just for debugging.
    void dumpMOInfoMap(std::map<unsigned, RegInfo>MOInfoMap);

    // Dump MOOutInfo, just for debugging.
    void dumpMOOutInfo(std::map<int, InstrInfo>MOOutInfo);

    // Store output MO information.
    unsigned storeOutputMOInfo(MachineInstr *Instr, MachineOperand &MO,
                               std::map<unsigned, RegInfo> &MOInfoMap,
                               SmallVector<Abc_Obj_t*, 128> &NodeVec);

    // Store input MO information.
    unsigned storeInputMOInfo(MachineInstr *Instr, MachineOperand &MO,
                              std::map<unsigned, RegInfo> &MOInfoMap,
                              SmallVector<Abc_Obj_t*, 128> &NodeVec);

    // Construct Abc_Ntk_t from a MBB, and store the information into different maps.
    void constructABCNtk(MachineBasicBlock *MBB, Abc_Ntk_t* pABC,
                         std::map<unsigned, RegInfo> &MOInfoMap,
                         SmallVector<Abc_Obj_t*, 128> &NodeVec,
                         SmallVector<MachineInstr*, 16> &MIDeleted);

    // Construct part of Abc_Ntk_t from a 2-input MI.
    void constructABCInstr(MachineInstr *Instr, Abc_Ntk_t *pABC,
                           std::map<unsigned, RegInfo> &MOInfoMap,
                           SmallVector<Abc_Obj_t*, 128> &NodeVec);

    // Construct part of Abc_Ntk_t from a inv MI.
    void constructInvABCInstr(MachineInstr *Instr, Abc_Ntk_t *pABC,
                              std::map<unsigned, RegInfo> &MOInfoMap,
                              SmallVector<Abc_Obj_t*, 128> &NodeVec);

    // Traverse the binary tree of Abc_Ntk_t and get all the node of Abc_Ntk_t.
    void traverseBinaryTree(Abc_Obj_t *objOut, SmallVector<Abc_Obj_t*, 16> &NodeStack);

    // Construct MI from Abc_Ntk_t.
    void constructMachineInstr(const TargetInstrInfo *TII, MachineRegisterInfo *MRI,
                               MachineBasicBlock &MBB, MachineInstr *Instr,
                               std::map<Abc_Obj_t*, int> &NodeMapMO,
                               std::map<int, InstrInfo> &MOOutInfo,
                               std::map<unsigned, RegInfo> &MOInfoMap,
                               SmallVectorImpl<Abc_Obj_t*> &NodeStack);

    // Get the corresponding reg from the reg of the Abc_Ntk_t.
    int getRegFromABCReg(const TargetInstrInfo *TII, MachineRegisterInfo *MRI,
                         MachineBasicBlock &MBB, MachineInstr *Instr,
                         Abc_Obj_t *child,
                         std::map<unsigned, RegInfo> &MOInfoMap,
                         std::map<int, InstrInfo> &MOOutInfo);

    // Get the corresponding reg from the node of the Abc_Ntk_t.
    int getRegFromABCNode(const TargetInstrInfo *TII, MachineRegisterInfo *MRI,
                          MachineBasicBlock &MBB, MachineInstr *Instr,
                          Abc_Obj_t *child,
                          std::map<unsigned, RegInfo> &MOInfoMap,
                          std::map<int, InstrInfo> &MOOutInfo,
                          std::map<Abc_Obj_t*, int> &NodeMapMO);

    // Create inv MI.
    void createInvMachineInstr(const TargetInstrInfo *TII, MachineRegisterInfo *MRI,
                               MachineBasicBlock &MBB, MachineInstr *Instr,
                               Abc_Obj_t *father,
                               int inRegNo, int outRegNo,
                               std::map<unsigned, RegInfo> &MOInfoMap,
                               std::map<int, InstrInfo> &MOOutInfo,
                               std::map<Abc_Obj_t*, int> &NodeMapMO);

    // Create and MI.
    void createAndMachineInstr(const TargetInstrInfo *TII, MachineRegisterInfo *MRI,
                               MachineBasicBlock &MBB, MachineInstr *Instr,
                               Abc_Obj_t *father,
                               int inRegNo0, int inRegNo1, int outRegNo,
                               std::map<unsigned, RegInfo> &MOInfoMap,
                               std::map<int, InstrInfo> &MOOutInfo,
                               std::map<Abc_Obj_t*, int> &NodeMapMO);
  };
}

INITIALIZE_PASS(BitLevelABCOpt, "vtm-abcopt", "Verilog Target Machine - "
  "Bit Level ABC Optimization", false, true)

Pass *llvm::createBitLevelABCOptPass() {
  return new BitLevelABCOpt();
}

char BitLevelABCOpt::ID = 0;

BitLevelABCOpt::BitLevelABCOpt() : MachineFunctionPass(ID) {
  initializeBitLevelABCOptPass(*PassRegistry::getPassRegistry());
}

void BitLevelABCOpt::getAnalysisUsage(AnalysisUsage &AU) const {
  MachineFunctionPass::getAnalysisUsage(AU);
  AU.setPreservesAll();
}

bool BitLevelABCOpt::runOnMachineFunction(MachineFunction &MF) {
  // Something useful.
  const TargetInstrInfo *TII = MF.getTarget().getInstrInfo();
  MachineRegisterInfo *MRI = &MF.getRegInfo();
  // Start ABC.
  Abc_Start();
  for (MachineFunction::iterator BI = MF.begin(), BE = MF.end(); BI != BE; ++BI) {
    // BasicBlock by BasicBlock.
    // The preparation of constructing ABC network.
    DEBUG(dbgs() << "New BasicBlock begins!\n\n"
                  << "\n----------------------------------------------------\n"
                  << "----------------------------------------------------\n\n");
    Abc_Ntk_t *pABC;
    pABC = Abc_NtkAlloc(ABC_NTK_STRASH, ABC_FUNC_AIG, 1);
    pABC->pName = Extra_UtilStrsav("Opt");

    // Store all the information of MachineOperand.
    // Key: reg number, value: struct RegInfo.
    std::map<unsigned, RegInfo> MOInfoMap;
    // Store all the node of ABC network.
    SmallVector<Abc_Obj_t*, 128> NodeVec;
    // MIDeleted : store all the MachineInstructions
    // which will be replaced and deleted.
    SmallVector<MachineInstr*, 16> MIDeleted;
    constructABCNtk(BI, pABC, MOInfoMap, NodeVec, MIDeleted);


    // Get the information from AIG network and construct the MachineInstr.

    MachineBasicBlock &MBB = *BI;

    // After constructing new MachineInstruction, store the information of new MI.
    // Key: output reg, value: opcode & input regs.
    std::map<int, InstrInfo> MOOutInfo;
    // Stack to store all the nodes of AIG.
    SmallVector<Abc_Obj_t*, 16> NodeStack;
    // Store the relationship of AIG node and MachineOperands.
    // Key: internal nodes of AIG, value: MachineOperands.
    std::map<Abc_Obj_t*, int> NodeMapMO;

    //Iterate all the primary outputs and traverse the binary tree.
    int PoNum;
    for (PoNum = 0; PoNum < Abc_NtkPoNum(pABC); PoNum++) {
      // The root obj.
      Abc_Obj_t *objOut = Abc_NtkPo(pABC, PoNum);
      // The original MachineInstruction,
      // before which the newly created ones will insert.
      MachineInstr *Instr = MOInfoMap[getRegNo(objOut)].MI;

      // Do the binary tree traversal.
      traverseBinaryTree(objOut,NodeStack);
      DEBUG(dbgs() << "\n----------------------------------------------------\n\n");
      // construct MachineInstruction based on the information from the binary tree.
      constructMachineInstr(TII, MRI, MBB, Instr,
                            NodeMapMO, MOOutInfo,
                            MOInfoMap, NodeStack);
    }
    //Delete all the MachineInstructions in the vector MIDeleted.
    while (!MIDeleted.empty()) {
      MachineInstr *MI = MIDeleted.pop_back_val();
      MI->eraseFromParent();
    }
  }
  // Stop ABC.
  Abc_Stop();
  return true;
}

int BitLevelABCOpt::getRegNo(Abc_Obj_t *obj) {
  // Get the reg number from the name of Abc_Obj_t.
  // The format of the name: regXX.
  // XX is the reg number.
  char *Name = Abc_ObjName(obj);
  int RegNo;
  if (Name[0] == 'r' && Name[1] == 'e' && Name[2] == 'g') {
    if (Name[4] =='\0') {
      RegNo = Name[3] - '0';
    }
    else
      if (Name[5] == '\0') {
        RegNo = (Name[3] - '0') * 10 + Name[4] - '0';
      }
      else
        if (Name[6] == '\0') {
          RegNo = (Name[3] - '0') * 100 + (Name[4] - '0') * 10
            + Name[5] - '0';
        }
        else
          if (Name[7] == '\0') {
            RegNo = (Name[3] - '0') * 1000 + (Name[4] - '0') * 100
              + (Name[5] - '0') * 10 + Name[6] - '0';
          }
  }
  else
    assert(0 && "Something is wrong when getting the RegNo of the output.");
  return RegNo;
}

void BitLevelABCOpt::dumpNtk(Abc_Ntk_t *pABC, bool blif) {
  std::string dstpath = "D:\\cygwin\\home\\eric\\work\\Debug\\ABC\\";
  if (blif) {
    dstpath += "1.blif";
    DEBUG(Io_Write(pABC, const_cast<char*>(dstpath.c_str()), IO_FILE_BLIF));
  }
  else {
    dstpath += "2.blif";
    DEBUG(Io_Write(pABC, const_cast<char*>(dstpath.c_str()), IO_FILE_BLIF));
  }
}

void BitLevelABCOpt::dumpMOInfoMap(std::map<unsigned, RegInfo>MOInfoMap) {
  DEBUG(dbgs() << "The detail of MOInfoMap(the size is " << MOInfoMap.size() << "): \n");
  int i = 1;
  for (std::map<unsigned, RegInfo>::iterator IMap = MOInfoMap.begin(), EMap = MOInfoMap.end();
       IMap != EMap; IMap++, i++) {
    if (IMap->second.numInput) {
      MachineOperand MO = IMap->second.MO;
      bool isKill = MO.isKill();
      bool isDef = MO.isDef();
      int MOOutReg = TargetRegisterInfo::virtReg2Index(MO.getReg());
      DEBUG(dbgs() << i << " : The key is " << IMap->first
                    << " and the input value of MO is " << MOOutReg
                    << ". Attribute: isKill: " << isKill
                    << " isDef: " << isDef << "\n");
    }
    if (IMap->second.numOutput) {
      MachineOperand MO = IMap->second.MO;
      bool isKill = MO.isKill();
      bool isDef = MO.isDef();
      int MOOutReg = TargetRegisterInfo::virtReg2Index(MO.getReg());
      DEBUG(dbgs() << i << " : The key is " << IMap->first
                    << " and the output value of MO is " << MOOutReg
                    << ". Attribute: isKill: " << isKill
                    << " isDef: " << isDef << "\n");
    }
  }
}

void BitLevelABCOpt::dumpMOOutInfo(std::map<int, InstrInfo>MOOutInfo) {
   DEBUG(dbgs() << "The detail of MOOutInfo(the size is " << MOOutInfo.size() << "): \n");
  int i = 1;
  for (std::map<int, InstrInfo>::iterator I = MOOutInfo.begin(), E = MOOutInfo.end();
       I != E; ++I, i++) {
    DEBUG(dbgs() << i << ": The output MO is reg" << I->first << ". ");
    if (I->second.MIOpcode == VTM::VOpNot) {
      DEBUG(dbgs() << "The opcode is VTM::VOpNot. And the input is reg"
                    << I->second.MOIn0 << " .\n");
    }
    else {
      DEBUG(dbgs() << "The opcode is VTM::VOpAnd. And the inputs are reg"
                    << I->second.MOIn0 << " and reg" << I->second.MOIn1 << " .\n");
    }
  }
}

unsigned BitLevelABCOpt::storeOutputMOInfo(MachineInstr *Instr, MachineOperand &MO,
                                           std::map<unsigned, RegInfo> &MOInfoMap,
                                           SmallVector<Abc_Obj_t*, 128> &NodeVec) {
  unsigned MONum = TargetRegisterInfo::virtReg2Index(MO.getReg());
  int MOWidth = cast<ucOperand>(MO).getBitWidth();
  if (!MOInfoMap.count(MONum)) {
    // Store the information of MO.
    MO.setIsDef(false);
    MOInfoMap[MONum].MO = MO;
    MOInfoMap[MONum].MI = Instr;
    MOInfoMap[MONum].numOutput += 1;
    MOInfoMap[MONum].width = MOWidth;
  }
  return MONum;
}

unsigned BitLevelABCOpt::storeInputMOInfo(MachineInstr *Instr, MachineOperand &MO,
                                          std::map<unsigned, RegInfo> &MOInfoMap,
                                          SmallVector<Abc_Obj_t*, 128> &NodeVec) {
  unsigned MONum = TargetRegisterInfo::virtReg2Index(MO.getReg());
  int MOWidth = cast<ucOperand>(MO).getBitWidth();
  if (!MOInfoMap.count(MONum)) {
    MO.setIsKill(false);
    MOInfoMap[MONum].MO = MO;
    MOInfoMap[MONum].numInput += 1;
    MOInfoMap[MONum].width = MOWidth;
  }
  else {
    MOInfoMap[MONum].numInput += 1;
  }
  return MONum;
}

void BitLevelABCOpt::constructABCNtk(MachineBasicBlock *MBB, Abc_Ntk_t* pABC,
                                     std::map<unsigned, RegInfo> &MOInfoMap,
                                     SmallVector<Abc_Obj_t*, 128> &NodeVec,
                                     SmallVector<MachineInstr*, 16> &MIDeleted) {
  for (MachineBasicBlock::iterator I = MBB->begin(), E = MBB->end(); I != E; ++I) {
    MachineInstr *Instr = I;
    switch (Instr->getOpcode()) {
    case VTM::VOpNot: {
      MIDeleted.push_back(Instr);
      constructInvABCInstr(Instr, pABC, MOInfoMap, NodeVec);
      break;
    }
    case VTM::VOpOr:
    case VTM::VOpAnd:
    case VTM::VOpXor: {
      MIDeleted.push_back(Instr);
      constructABCInstr(Instr, pABC, MOInfoMap, NodeVec);
      break;
    }
    default:break;
    }
  }
  Abc_AigCleanup((Abc_Aig_t *)pABC->pManFunc);
}

void BitLevelABCOpt::constructABCInstr(MachineInstr *Instr,
                                       Abc_Ntk_t *pABC,
                                       std::map<unsigned, RegInfo> &MOInfoMap,
                                       SmallVector<Abc_Obj_t*, 128> &NodeVec
                                       ) {
  // Output MO.
  MachineOperand MO0 = Instr->getOperand(0);
  unsigned MO0Num = storeOutputMOInfo(Instr, MO0, MOInfoMap, NodeVec);

  // First input MO.
  MachineOperand MO1 = Instr->getOperand(1);
  unsigned MO1Num = storeInputMOInfo(Instr, MO1, MOInfoMap, NodeVec);

  // Second input MO.
  MachineOperand MO2 = Instr->getOperand(2);
  unsigned MO2Num = storeInputMOInfo(Instr, MO2, MOInfoMap, NodeVec);

  // construct ABC instruction.
  Abc_Obj_t *objIn1, *objIn2, *objOut, *Output;
  // The width of the instruction is 1 bit, here we don't take care
  // the width information of the reg, so it can represent the relationship
  // of the input and output reg.
  //
  // Input obj.
  // When the numInput is larger than 1 or the numOutput is not 0,
  // it means that the Abc_Obj_t has been created, use it.
  // Else, create a new one.
  if (MOInfoMap[MO1Num].numInput > 1 || MOInfoMap[MO1Num].numOutput) {
    objIn1 = NodeVec[MOInfoMap[MO1Num].pointerInVec];
  }
  else {
    objIn1 = Abc_NtkCreatePi(pABC);
    // Set the name of the obj.
    std::string RegIn1Name = "reg" + utostr_32(MO1Num);
    Abc_ObjAssignName(objIn1, const_cast<char*>(RegIn1Name.c_str()), NULL);
    // Store the obj into the vector.
    MOInfoMap[MO1Num].pointerInVec = NodeVec.size();
    NodeVec.push_back(objIn1);
  }
  if (MOInfoMap[MO2Num].numInput > 1 || MOInfoMap[MO2Num].numOutput)
    objIn2 = NodeVec[MOInfoMap[MO2Num].pointerInVec];
  else {
    objIn2 = Abc_NtkCreatePi(pABC);
    std::string RegIn2Name = "reg" + utostr_32(MO2Num);
    Abc_ObjAssignName(objIn2, const_cast<char*>(RegIn2Name.c_str()), NULL);
    MOInfoMap[MO2Num].pointerInVec = NodeVec.size();
    NodeVec.push_back(objIn2);
  }
  // Create output obj.
  switch(Instr->getOpcode()) {
  case VTM::VOpAnd:
    objOut = Abc_AigAnd((Abc_Aig_t *)pABC->pManFunc, objIn1, objIn2);
    break;
  case VTM::VOpXor:
    objOut = Abc_AigXor((Abc_Aig_t *)pABC->pManFunc, objIn1, objIn2);
    break;
  case VTM::VOpOr:
    objOut = Abc_AigOr((Abc_Aig_t *)pABC->pManFunc, objIn1, objIn2);
    break;
  default:
    break;
  }
  // Store the obj into the vector.
  MOInfoMap[MO0Num].pointerInVec = NodeVec.size();
  NodeVec.push_back(objOut);
  // Connect the output obj to primary output.
  Output = Abc_NtkCreatePo(pABC);
  Abc_ObjAddFanin(Output, objOut);
  // Set the name of the PO.
  std::string RegOutName = "reg" + utostr_32(MO0Num);
  Abc_ObjAssignName(Output, const_cast<char*>(RegOutName.c_str()), NULL);
}

void BitLevelABCOpt::constructInvABCInstr(MachineInstr *Instr,
                                          Abc_Ntk_t *pABC,
                                          std::map<unsigned, RegInfo> &MOInfoMap,
                                          SmallVector<Abc_Obj_t*, 128> &NodeVec
                                          ) {
  // Output MO.
  MachineOperand MO0 = Instr->getOperand(0);
  unsigned MO0Num = storeOutputMOInfo(Instr, MO0, MOInfoMap, NodeVec);

  // Input MO.
  MachineOperand MO1 = Instr->getOperand(1);
  unsigned MO1Num = storeInputMOInfo(Instr, MO1, MOInfoMap, NodeVec);

  // Construct ABC instruction.
  Abc_Obj_t *objIn, *objOut, *Output;
  // The width of the instruction is 1 bit, here we don't take care
  // the width information of the reg, so it can represent the relationship
  // of the input and output reg.
  //
  // Input obj.
  // When the numInput is larger than 1 or the numOutput is not 0,
  // it means that the Abc_Obj_t has been created, use it.
  // Else, create a new one.
  if (MOInfoMap[MO1Num].numInput > 1 || MOInfoMap[MO1Num].numOutput) {
    objIn = NodeVec[MOInfoMap[MO1Num].pointerInVec];
  }
  else {
    objIn = Abc_NtkCreatePi(pABC);
    // Set the name of the obj.
    std::string RegIn1Name = "reg" + utostr_32(MO1Num);
    Abc_ObjAssignName(objIn, const_cast<char*>(RegIn1Name.c_str()), NULL);
    // Store the obj into the vector.
    MOInfoMap[MO1Num].pointerInVec = NodeVec.size();
    NodeVec.push_back(objIn);
  }
  // Create the output obj.
  objOut = Abc_ObjNot(objIn);
  // Store the obj into the vector.
  MOInfoMap[MO0Num].pointerInVec = NodeVec.size();
  NodeVec.push_back(objOut);
  // Create the primary output and connect it to the output obj.
  Output = Abc_NtkCreatePo(pABC);
  Abc_ObjAddFanin(Output, objOut);
  // Set the name of the PO.
  std::string RegOutName = "reg" + utostr_32(MO0Num);
  Abc_ObjAssignName(Output, const_cast<char*>(RegOutName.c_str()), NULL);
}


void BitLevelABCOpt::traverseBinaryTree(Abc_Obj_t *objOut,
                                        SmallVector<Abc_Obj_t*, 16> &NodeStack) {
  // Stack and obj which will help traversing the tree.
  SmallVector<Abc_Obj_t*, 16> ObjStack;
  Abc_Obj_t *temp = objOut;
  // Store the PO first.
  ObjStack.push_back(objOut);
  NodeStack.push_back(objOut);
  bool Inv;
  while (!ObjStack.empty() || temp != NULL) {
    if (temp != NULL) {
      DEBUG(dbgs() << "The root is " << Abc_ObjName(temp) << "\n");
      if (temp != objOut) {
        ObjStack.push_back(temp);
        // Push the nodes into stack only,
        // then use it to construct MIs afterward.
        if (Abc_ObjFaninNum(temp) != 0)
          NodeStack.push_back(temp);
      }
      switch(Abc_ObjFaninNum(temp)) {
      case 1:
        Inv = Abc_ObjFaninC0(temp);
        temp = Abc_ObjFanin0(temp);
        DEBUG(dbgs() << "Only one fanin, which is " << Abc_ObjName(temp)
                      << " and the polarity is "<< Inv << "\n");
        break;
      case 2:
        Inv = Abc_ObjFaninC0(temp);
        temp = Abc_ObjFanin0(temp);
        DEBUG(dbgs() << "FaninO is " << Abc_ObjName(temp)
                      << " and the polarity is "<< Inv << "\n");
        break;
      default:
        DEBUG(dbgs() << "No FaninO !" << "\n");
        temp = NULL;
        break;
      }
    }

    // Right child of the node.
    else {
      temp = ObjStack.pop_back_val();
      Inv = Abc_ObjFaninC1(temp);
      DEBUG(dbgs() << "root is " << Abc_ObjName(temp) << "\n");
      if (Abc_ObjFaninNum(temp) == 2) {
        temp = Abc_ObjFanin1(temp);
        DEBUG(dbgs() << "Fanin1 is " << Abc_ObjName(temp)
                      << " and the polarity is "<< Inv << "\n");
      }
      else {
        temp = NULL;
        DEBUG(dbgs() << "No Fanin1 !!" << "\n");
      }
    }
  }
}

void BitLevelABCOpt::constructMachineInstr(const TargetInstrInfo *TII, MachineRegisterInfo *MRI,
                                           MachineBasicBlock &MBB,
                                           MachineInstr *Instr,
                                           std::map<Abc_Obj_t*, int> &NodeMapMO,
                                           std::map<int, InstrInfo> &MOOutInfo,
                                           std::map<unsigned, RegInfo> &MOInfoMap,
                                           SmallVectorImpl<Abc_Obj_t*> &NodeStack) {
  int inRegNo0, inRegNo1, outRegNo;
  while(!NodeStack.empty()) {
    inRegNo0 = inRegNo1 = outRegNo = ~0;
    Abc_Obj_t *father = NodeStack.pop_back_val();
    Abc_Obj_t *lChild, *rChild;
    if (Abc_ObjFaninNum(father) == 2) {
      //// Left child.
      lChild = Abc_ObjFanin0(father);
      DEBUG(dbgs() << "First fanin obj is " << Abc_ObjName(lChild) << "\n");
      // Get the corresponding reg of the left child.
      // If polarity of the child is 1,
      // then create a new reg which is the inverse of the reg,
      // else get the original reg.
      // If the child is a node, also find out the corresponding reg.

      // Get the inverse reg.
      if (Abc_ObjFaninC0(father)) {
        // Left child is a reg.
        if (!Abc_ObjIsNode(lChild)) {
          inRegNo0 = getRegFromABCReg(TII, MRI, MBB, Instr, lChild,
                                       MOInfoMap, MOOutInfo);
        }
        // A node.
        else {
          inRegNo0 = getRegFromABCNode(TII, MRI, MBB, Instr, lChild,
                                        MOInfoMap, MOOutInfo, NodeMapMO);
        }
      }
      // Left child is original one.
      else {
        if (!Abc_ObjIsNode(lChild))
          inRegNo0 = getRegNo(lChild);
        else
          inRegNo0 = NodeMapMO[lChild];
      }
      DEBUG(dbgs() << "First input reg is " << inRegNo0 << "\n");

      //
      // Right child.
      rChild = Abc_ObjFanin1(father);
      DEBUG(dbgs() << "Second fanin obj is " << Abc_ObjName(rChild) << "\n");
      if (Abc_ObjFaninC1(father)) {
        // Left child is a reg.
        if (!Abc_ObjIsNode(rChild)) {
          inRegNo1 = getRegFromABCReg(TII, MRI, MBB, Instr, rChild,
                                       MOInfoMap, MOOutInfo);
        }
        else {
          inRegNo1 = getRegFromABCNode(TII, MRI, MBB, Instr, rChild,
                                        MOInfoMap, MOOutInfo, NodeMapMO);
        }
      }
      // The right child is original one.
      else {
        if (!Abc_ObjIsNode(rChild))
          inRegNo1 = getRegNo(rChild);
        else
          inRegNo1 = NodeMapMO[rChild];
      }
      DEBUG(dbgs() << "Second input reg is " << inRegNo1 << "\n");\

      // The father obj.
      DEBUG(dbgs() << "Father obj is " << Abc_ObjName(father) << "\n");
      createAndMachineInstr(TII, MRI, MBB, Instr, father,
                            inRegNo0, inRegNo1, outRegNo,
                            MOInfoMap, MOOutInfo, NodeMapMO);
    }

    // The situation of one Fanin.
    else {
      // The child.
      lChild = Abc_ObjFanin0(father);
      DEBUG(dbgs() << "First fanin obj is " << Abc_ObjName(lChild) << "\n");
      // Get the corresponding reg of the left child.
      // Because we do the inverse operation afterward,
      // so here we do another inverse operation for the original reg/node.
      // If the child is the original version, then get the inverse reg of it,
      // else get the inverse reg itself.
      if (!Abc_ObjFaninC0(father)) {
        // The child is a reg.
        if (!Abc_ObjIsNode(lChild)) {
          inRegNo0 = getRegFromABCReg(TII, MRI, MBB, Instr, lChild,
                                       MOInfoMap, MOOutInfo);
        }
        // A node.
        else {
          inRegNo0 = getRegFromABCNode(TII, MRI, MBB, Instr, lChild,
                                        MOInfoMap, MOOutInfo, NodeMapMO);
        }
      }
      // The child is inverse one.
      else {
        if (!Abc_ObjIsNode(lChild))
          inRegNo0 = getRegNo(lChild);
        else
          inRegNo0 = NodeMapMO[lChild];
      }
      DEBUG(dbgs() << "The only input reg is " << inRegNo0 << "\n");

      // The father obj.
      DEBUG(dbgs() << "Father obj is " << Abc_ObjName(father) << "\n");
      createInvMachineInstr(TII, MRI, MBB, Instr, father, inRegNo0, outRegNo,
                            MOInfoMap, MOOutInfo, NodeMapMO);
    }
  }
}

int BitLevelABCOpt::getRegFromABCReg(const TargetInstrInfo *TII,
                                     MachineRegisterInfo *MRI,
                                     MachineBasicBlock &MBB,
                                     MachineInstr *Instr,
                                     Abc_Obj_t *child,
                                     std::map<unsigned, RegInfo> &MOInfoMap,
                                     std::map<int, InstrInfo> &MOOutInfo) {
  int regNo;
  // Get the final reg. Then remember the corresponding reg.
  for (std::map<int, InstrInfo>::iterator I = MOOutInfo.begin(), E = MOOutInfo.end();
        I != E; ++I) {
    if (I->second.MIOpcode == VTM::VOpNot
        && I->second.MOIn0 == getRegNo(child)) {
      regNo = I->first;
      DEBUG(dbgs() << "Input reg is "
                    << regNo << "\n");
      return regNo;
    }
  }
  // Create a new reg to replace the inverse reg of right child.
  unsigned DstReg = MRI->createVirtualRegister(VTM::DRRegisterClass);
  MachineOperand Dst = MachineOperand::CreateReg(DstReg, true);
  (cast<ucOperand>(Dst)).setBitWidth(MOInfoMap[getRegNo(child)].width);
  MachineInstr *MI = BuildMI(MBB, Instr, DebugLoc(), TII->get(VTM::VOpNot))
    .addOperand(Dst).addOperand(MOInfoMap[getRegNo(child)].MO)
    .addOperand(ucOperand::CreatePredicate())
    .addOperand(ucOperand::CreateTrace(&MBB));
  Dst.setIsDef(false);
  DEBUG(MI->dump());
  regNo = TargetRegisterInfo::virtReg2Index(DstReg);
  MOInfoMap[regNo].MO = Dst;
  MOInfoMap[regNo].numInput += 1;
  MOInfoMap[regNo].width = MOInfoMap[getRegNo(child)].width;
  MOOutInfo[regNo].MIOpcode = VTM::VOpNot;
  MOOutInfo[regNo].MOIn0 = getRegNo(child);
  MOOutInfo[regNo].MOIn1 = ~0;
  DEBUG(dbgs() << "Second input reg is " << regNo << "\n");
  return regNo;
}

int BitLevelABCOpt::getRegFromABCNode(const TargetInstrInfo *TII,
                                      MachineRegisterInfo *MRI,
                                      MachineBasicBlock &MBB,
                                      MachineInstr *Instr,
                                      Abc_Obj_t *child,
                                      std::map<unsigned, RegInfo> &MOInfoMap,
                                      std::map<int, InstrInfo> &MOOutInfo,
                                      std::map<Abc_Obj_t*, int> &NodeMapMO) {
  int regNo;
  int NodeRegNo = NodeMapMO[child];
  // Get the final reg. Then remember the corresponding reg.
  for (std::map<int, InstrInfo>::iterator I = MOOutInfo.begin(), E = MOOutInfo.end();
        I != E; ++I) {
    if (I->second.MIOpcode == VTM::VOpNot
        && I->second.MOIn0 == NodeRegNo) {
      regNo = I->first;
      DEBUG(dbgs() << "Second input reg is " << regNo << "\n");
      return regNo;
    }
  }
  // Create a new reg to replace the inverse reg of right child.
  unsigned DstReg = MRI->createVirtualRegister(VTM::DRRegisterClass);
  MachineOperand Dst = MachineOperand::CreateReg(DstReg, true);
  (cast<ucOperand>(Dst)).setBitWidth(MOInfoMap[NodeRegNo].width);
  DEBUG(dbgs() <<"%%%%The input MachineOperand is " << MOInfoMap[NodeRegNo].MO << "\n");
  MachineInstr *MI = BuildMI(MBB, Instr, DebugLoc(), TII->get(VTM::VOpNot))
    .addOperand(Dst).addOperand(MOInfoMap[NodeRegNo].MO)
    .addOperand(ucOperand::CreatePredicate())
    .addOperand(ucOperand::CreateTrace(&MBB));
  Dst.setIsDef(false);
  DEBUG(MI->dump());
  regNo = TargetRegisterInfo::virtReg2Index(DstReg);
  MOInfoMap[regNo].MO = Dst;
  MOInfoMap[regNo].numInput += 1;
  MOInfoMap[regNo].width = MOInfoMap[NodeRegNo].width;
  MOOutInfo[regNo].MIOpcode = VTM::VOpNot;
  MOOutInfo[regNo].MOIn0 = NodeRegNo;
  MOOutInfo[regNo].MOIn1 = ~0;
  DEBUG(dbgs() << "Second input reg is " << regNo << "\n");
  return regNo;
}

void BitLevelABCOpt::createInvMachineInstr(const TargetInstrInfo *TII,
                                           MachineRegisterInfo *MRI,
                                           MachineBasicBlock &MBB,
                                           MachineInstr *Instr,
                                           Abc_Obj_t *father,
                                           int inRegNo, int outRegNo,
                                           std::map<unsigned, RegInfo> &MOInfoMap,
                                           std::map<int, InstrInfo> &MOOutInfo,
                                           std::map<Abc_Obj_t*, int> &NodeMapMO) {
  // If the father exists in the MOOutInfo map,
  // which means the instruction corresponds to the obj has constructed,
  // so exit, do not construct the same one again.
  //
  // Else, if the father obj is a reg, get it and construct the instruction,
  // if not, that is to say the father is a node, create a new reg
  // to match the father first then construct the instruction.

  bool NewFlag = false;
  DEBUG(dbgs() << "NotOp\n");

  // If the father is a node and the corresponding MachineInstr exists, do nothing.
  if (Abc_ObjIsNode(father)) {
    for (std::map<int, InstrInfo>::iterator I = MOOutInfo.begin(), E = MOOutInfo.end();
         I != E; ++I) {
      if (I->second.MIOpcode == VTM::VOpNot
        && I->second.MOIn0 == inRegNo) {
          return;
      }
    }
  }

  // Get the corresponding reg of output.
  if (!Abc_ObjIsNode(father)) {
    outRegNo = getRegNo(father);
  }
  else {
    outRegNo = TargetRegisterInfo::virtReg2Index(MRI->createVirtualRegister(VTM::DRRegisterClass));
    NewFlag = true;
  }

  // Construct MachineInstruction.
  MachineOperand MOIn0 = MOInfoMap[inRegNo].MO;
  MachineOperand MOOut = MOIn0;
  if (!NewFlag) {
    MOOut = MOInfoMap[outRegNo].MO;
    MOOut.setIsDef(true);
  }
  else {
    MOOut = MachineOperand::CreateReg(TargetRegisterInfo::index2VirtReg(outRegNo), true);
    (cast<ucOperand>(MOOut)).setBitWidth(MOInfoMap[inRegNo].width);
  }
  MachineInstr *MI = BuildMI(MBB, Instr, DebugLoc(), TII->get(VTM::VOpNot))
    .addOperand(MOOut).addOperand(MOIn0)
    .addOperand(ucOperand::CreatePredicate())
    .addOperand(ucOperand::CreateTrace(&MBB));
  DEBUG(MI->dump());
  if (NewFlag) {
    //MOInfoMap[outRegNo].MO = MOOut;
    MOOut.setIsDef(false);
    NodeMapMO[father] = outRegNo;
    MOInfoMap[outRegNo].MO = MOOut;
    MOInfoMap[outRegNo].width = MOInfoMap[inRegNo].width;
  }
  DEBUG(dbgs() << "output reg is " << outRegNo << "\n\n");

  MOOutInfo[outRegNo].MIOpcode = VTM::VOpNot;
  MOOutInfo[outRegNo].MOIn0 = inRegNo;
  MOOutInfo[outRegNo].MOIn1 = ~0;
}

void BitLevelABCOpt::createAndMachineInstr(const TargetInstrInfo *TII,
                                           MachineRegisterInfo *MRI,
                                           MachineBasicBlock &MBB,
                                           MachineInstr *Instr,
                                           Abc_Obj_t *father,
                                           int inRegNo0, int inRegNo1,
                                           int outRegNo,
                                           std::map<unsigned, RegInfo> &MOInfoMap,
                                           std::map<int, InstrInfo> &MOOutInfo,
                                           std::map<Abc_Obj_t*, int> &NodeMapMO) {
  // If the father exists in the MOOutInfo map,
  // which means the instruction corresponds to the obj has constructed,
  // so exit, do not construct the same one again.
  //
  // Else, if the father obj is a reg, get it and construct the instruction,
  // if not, that is to say the father is a node, create a new reg
  // to match the father first then construct the instruction.

  bool NewFlag = false;
  DEBUG(dbgs() << "AndOp\n");
  // If the MI exists and the father is a node, do nothing.
  if (Abc_ObjIsNode(father)) {
    for (std::map<int, InstrInfo>::iterator I = MOOutInfo.begin(), E = MOOutInfo.end();
          I != E; ++I) {
      if (I->second.MIOpcode == VTM::VOpAnd) {
        if (inRegNo0 == I->second.MOIn0 && inRegNo1 == I->second.MOIn1) {
          return;
        }
      }
    }
  }

  // Get the reg of the father node.
  if (!Abc_ObjIsNode(father)) {
    outRegNo = getRegNo(father);
  }
  else {
    outRegNo = TargetRegisterInfo::virtReg2Index(MRI->createVirtualRegister(VTM::DRRegisterClass));
    NewFlag = true;
  }

  // Create the MachineInstruction.
  MachineOperand MOIn0 = MOInfoMap[inRegNo0].MO;
  MachineOperand MOIn1 = MOInfoMap[inRegNo1].MO;
  MachineOperand MOOut = MOIn0;
  if (!NewFlag) {
    MOOut = MOInfoMap[outRegNo].MO;
    MOOut.setIsDef(true);
  }
  else {
    MOOut = MachineOperand::CreateReg(TargetRegisterInfo::index2VirtReg(outRegNo), true);
    (cast<ucOperand>(MOOut)).setBitWidth(MOInfoMap[inRegNo0].width);
  }
  MachineInstr *MI = BuildMI(MBB, Instr, DebugLoc(), TII->get(VTM::VOpAnd))
                     .addOperand(MOOut).addOperand(MOIn0).addOperand(MOIn1)
                     .addOperand(ucOperand::CreatePredicate())
                     .addOperand(ucOperand::CreateTrace(&MBB));
  DEBUG(MI->dump());

  if (NewFlag) {
    //MOInfoMap[outRegNo].MOOut = MOOut;
    MOOut.setIsDef(false);
    NodeMapMO[father] = outRegNo;
    MOInfoMap[outRegNo].MO = MOOut;
    MOInfoMap[outRegNo].width = MOInfoMap[inRegNo0].width;
  }
  DEBUG(dbgs() << "output reg is " << outRegNo << "\n\n");
  MOInfoMap[outRegNo].numInput += 1;
  MOInfoMap[outRegNo].numOutput += 1;

  MOOutInfo[outRegNo].MIOpcode = VTM::VOpAnd;
  MOOutInfo[outRegNo].MOIn0 = inRegNo0;
  MOOutInfo[outRegNo].MOIn1 = inRegNo1;
}
