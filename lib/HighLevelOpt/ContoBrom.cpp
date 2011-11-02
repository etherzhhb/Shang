//==-ContoBrom.cpp ---This pass allocates BRoms to global constants===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file is a pass to transform the global constants' location from main memory 
// to local BRom.
//
//===----------------------------------------------------------------------===//


#include "vtm/Passes.h"
#include "vtm/VIntrinsicsInfo.h"

#include "llvm/Pass.h"
#include "llvm/Module.h"
#include "llvm/Function.h"
#include "llvm/Constants.h"
#include "llvm/GlobalVariable.h"
#include "llvm/Instructions.h"

#include "llvm/Target/TargetData.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunction.h"
#define DEBUG_TYPE "ContoBrom"
#include "llvm/Support/Debug.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"
#include <set>
#include <map>

using namespace llvm;

//===----------------------------------------------------------------------===//
// This pass allocates a BRom for each global constant in the function and replaces
// the load instruction by a call to vtm_access_bram intrinsic function, thus 
// allowing a local BRom access to the global constant.
//

namespace {
 struct ContoBrom: public ModulePass {
  static char ID;
  Module* Mod;
  const TargetIntrinsicInfo &IntrinsicInfo;

  //Every Constant has a relative unsigned BromID
  std::map<Constant*, unsigned> BRom;
  //Use a cast instruction number to mark the cast instruction
  unsigned CastNum;

  ContoBrom(const TargetIntrinsicInfo &II) : ModulePass(ID), Mod(0), CastNum(0),
    IntrinsicInfo(II) {}

  bool runOnModule(Module &M);

  //This function creates a call to  vtm_alloca_brom in the case of GEP-GV node.
  Instruction* AllocaBromGEP(GlobalVariable* GV, Constant* Con, 
    GetElementPtrInst* GEP, SmallVector<Instruction*, 16> &CIVect, Function* F);

  //This function creates a call to  vtm_alloca_brom in the case of Load-GV node.
  Instruction* AllocaBromLoad(GlobalVariable* GV, Constant* Con, 
    LoadInst* Load, SmallVector<Instruction*, 16> &CIVect, Function* F);

  //Based on the information of the LoadInst, access the BRom using the ptr returned 
  //by GEP or access the BRom directly by accessing ALlocaBrom.
  void LoadtoBromAccess(LoadInst* Load, bool IsLoad, Instruction* AllocaBrom, 
    Constant* Con);
  };

}//end of anonymous namespace

bool ContoBrom::runOnModule(Module &M) {
  Mod = &M;
  unsigned ThisBromID = 0;

   //The following variables are for DFS(Depth First Search) for the GV user tree
   SmallVector<std::pair<Value*, Value::use_iterator>, 16>WorkStack;
   SmallVector<std::pair<Value*, Value::use_iterator>, 16>DFSTVect;
   std::set<Value*> Visited;

  //Stores the cast instructions for each new BRom allocation 
  //in the case of GEP-GV node
  SmallVector<Instruction*, 16> CIVect;

  //Stores the Load instructions of Load-GEP nodes
  SmallVector<LoadInst*, 16>LIVect;

  //Stores the load instructions to be erased
  SmallVector<LoadInst*, 16>LIErase;

  //Stores the return value(a call instruction to brom_alloca function)
  Instruction* AllocaBrom = 0;

  //iterate all over the module graph
  for (Module::iterator IF = M.begin(), EF = M.end(); IF != EF; ++IF)  {
    Function &F = *IF;
    for (Function::iterator IBB = F.begin(), EBB = F.end(); IBB != EBB; ++IBB) {
      BasicBlock &BB = *IBB;
      for (BasicBlock::iterator IInst = BB.begin(), EInst = BB.end(); 
            IInst != EInst; ++IInst) {
        Instruction &Inst = *IInst;
        for (Instruction::op_iterator IOP = Inst.op_begin(), EOP = Inst.op_end(); 
              IOP != EOP; ++IOP) {
          GlobalVariable* GV = dyn_cast<GlobalVariable>(IOP);
        
          //Firstly, it should be a GlobalVariable.
          if (!GV) 
            continue; 

          //Secondly, it should be a constant.
          if (!GV->isConstant()) 
            continue; 
          
          //Thirdly, it has internal linkage.
          if (!(GV->hasInternalLinkage()||GV->hasPrivateLinkage()))
            continue; 

          //Fourthly, the user of the GV should be a GEP or a Load
          if (!(isa<GetElementPtrInst>(*GV->use_begin()) ||
                     isa<LoadInst>(*GV->use_begin()))) 
            continue;
              
          //Fifthly, if this Con(GV) has already been visited
          if(Visited.count(GV)) 
            continue;

          //If the Con can go through all the conditions above, it is a Con
          //that needs DFS, and there will be another DFS tree rooted from 
          //this Con.
          Value* V = GV->getOperand(0);
          Constant* Con = cast<Constant>(V);

          //If the Con is a multi-dimension constant array, skip this iteration
          if (const ArrayType* AT = cast<ArrayType>(Con->getType())) {
              const Type* ET = AT->getElementType();
              if (isa<ArrayType>(ET)) continue;
          }                  

          BRom[Con] = ThisBromID++;

          //A GV user tree could look like this:
          /**********************************************************************
                                      GV
                              - - -   ||
                           /        /     \ 
                         /       /    /     \
                      GEP  GEP  Load  Load (in this layer GEP and Load can't coexist)
                       /       /  \
                     /       /      \
                 Load  Load    Load
          **********************************************************************/

          //Prepare the root node.
          WorkStack.push_back(std::make_pair(GV, GV->use_begin()));

          //Start the Depth First Search with this GV as root node.
          //The result DFS Tree is preserved in DFSTVect.
  
          while(!WorkStack.empty()) {
            Value* Node = WorkStack.back().first;
            Value::use_iterator ChildIt = WorkStack.back().second;

            DFSTVect.push_back(std::make_pair(Node, ChildIt));

            if(ChildIt == Node->use_end() || !(isa<LoadInst>(*ChildIt) ||
                                isa<GetElementPtrInst>(*ChildIt))) {
              WorkStack.pop_back();
              DFSTVect.pop_back();
            } else {
              Value* Child = *ChildIt;
              ++WorkStack.back().second;

              WorkStack.push_back(std::make_pair(Child, Child->use_begin()));
            }         
          }
          Visited.insert(GV);

          //The DFSVisit procedure generates a DFS Tree rooted from the GV.
          //Now iterate through the DFS Tree, separate the nodes into tree groups.
          SmallVector<std::pair<GetElementPtrInst*, GlobalVariable*>, 16>VectGEPGV;
          SmallVector<std::pair<LoadInst*, GlobalVariable*>, 16>VectLoadGV;
          SmallVector<std::pair<LoadInst*, GetElementPtrInst*>, 16>VectLoadGEP;
          GetElementPtrInst* GEP = 0;
          LoadInst* Load = 0;
          GlobalVariable* GBV = 0;
          for(SmallVector<std::pair<Value*, Value::use_iterator>, 16>::iterator INode = 
               DFSTVect.begin(), ENode = DFSTVect.end(); INode != ENode; ++INode) {
            if((GEP = dyn_cast<GetElementPtrInst>(*INode->second)) && 
              (GBV = dyn_cast<GlobalVariable>(INode->first)))
              VectGEPGV.push_back(std::make_pair(GEP, GBV));   
            if((Load = dyn_cast<LoadInst>(*INode->second)) && 
              (GBV = dyn_cast<GlobalVariable>(INode->first))) 
              VectLoadGV.push_back(std::make_pair(Load, GBV));
            if((Load = dyn_cast<LoadInst>(*INode->second)) &&
              (GEP = dyn_cast<GetElementPtrInst>(INode->first)))
              VectLoadGEP.push_back(std::make_pair(Load, GEP));
          }

          //Clear the DFSTVect for another DFS Tree rooted from another GV.
          DFSTVect.clear(); 

          //If the GEP is not used by a load, continue and clear the VectGEPGV
          if(VectLoadGV.empty()&&VectLoadGEP.empty()) {
            VectGEPGV.clear();
            continue;
          }

          //Use a Function Set to keep track of the function in which the GV
          //has been allocated a BRom.
          std::set<Function*>FunctionSet;

          //The following three blocks allocate BRoms for the Constant and replace
          //the Load instructions with the brom_access instructions.
          //Use reverse iterator because it fits the instructions' order in the Vectors.

          //Allocate BRoms for the constant if it is used by GetElementPtrInsts.
          //Because the GEP-GV nodes are in order, CastNum-1 can promise that
          //the cast instruction to be setOperand and the GEP are within the same
          //function.
          if(!VectGEPGV.empty()) {
            for(SmallVector<std::pair<GetElementPtrInst*, GlobalVariable*>, 16>::
                 reverse_iterator IN = VectGEPGV.rbegin(), EN = VectGEPGV.rend(); 
                 IN != EN; ++IN) {
              //if within the same function the Con(or GV) has been allocated a BRom,
              //set the first operand of the GEP using the cast instruction directly.
              if(FunctionSet.count(IN->first->getParent ()->getParent()))
                IN->first->setOperand(0, CIVect[CastNum-1]); 
              else {
                Function* Func = IN->first->getParent ()->getParent();
                AllocaBrom = AllocaBromGEP(IN->second, Con, IN->first, CIVect, Func);
                FunctionSet.insert(Func); 
              }
            }
            VectGEPGV.clear();
            FunctionSet.clear();
          }

          //Allocate BRoms for the constant if it is used by LoadInsts.
          //Pay attention that a constant could not be used by both 
          //GetElementPtrInst and LoadInst.

          //Because a LoadGV node has some basic difference from the GEPGV node, 
          //it is dealt separately. Because the Load-GV nodes are in order, the following 
          //can promise the Brom_access instruction access only the AllocaBrom within 
          //the same function.
          if(!VectLoadGV.empty()) {
            for(SmallVector<std::pair<LoadInst*, GlobalVariable*>, 16>::reverse_iterator
                 IN = VectLoadGV.rbegin(), EN = VectLoadGV.rend(); IN != EN; ++IN) {
              if(FunctionSet.count(IN->first->getParent ()->getParent ())) {
                LoadtoBromAccess(IN->first, true, AllocaBrom, Con);
                LIErase.push_back(IN->first);
              } else {
                  Function* Func = IN->first->getParent ()->getParent();
                  AllocaBrom = AllocaBromLoad(IN->second , Con, IN->first , CIVect, Func);
                  LoadtoBromAccess(IN->first, true, AllocaBrom, Con);
                  LIErase.push_back(IN->first);
                  FunctionSet.insert(Func); 
              }
            }
            VectLoadGV.clear();
            FunctionSet.clear();
          }

          //Push back the Load instructions that use GEP in to the LIVect which is to be 
          //replaced by brom_access instructions.
          if(!VectLoadGEP.empty()) {
            for(SmallVector<std::pair<LoadInst*, GetElementPtrInst*>,16>::iterator
                 IN = VectLoadGEP.begin(), EN = VectLoadGEP.end(); IN != EN; ++IN) {
              LIVect.push_back(IN->first);
            }
            VectLoadGEP.clear();
          }

          //Replace the load instructions with the BRom access instructions(For Load-GEP)
          while(!LIVect.empty()) {
            LoadtoBromAccess(LIVect.back(), false, AllocaBrom, Con);
            LIErase.push_back(LIVect.back());
            LIVect.pop_back();
          }        
        }
      }
    }
  }

  //When all that need to be processed finish, erase the load instructions
  while(!LIErase.empty()) {
    LIErase.back()->eraseFromParent();
    LIErase.pop_back();
  }
  return false;
}

Instruction* ContoBrom::AllocaBromGEP(GlobalVariable* GV, Constant* Con, 
                                  GetElementPtrInst* GEP, 
                                  SmallVector<Instruction*, 16> &CIVect,
                                  Function* F) {
  //Gather the parameters of vtm_alloca_brom
  const ArrayType* AT = cast<ArrayType>(Con->getType());
  unsigned NumElems = AT->getNumElements();
  const Type* ET = AT->getElementType();
  unsigned ElemSizeInBytes = ET->getPrimitiveSizeInBits() / 8;
  Instruction* EntryPoint = F->begin()->begin();

  //allocate space for the alloca and get declaration 
  //of the intrinsic function.
  unsigned AllocaAddrSpace = GV->getType()->getAddressSpace();

  //Transform the Element type into a pointer that 
  //points to the Element
  const Type* NewPtrTy = PointerType::get(ET, AllocaAddrSpace);                                                                             
  const Type* CAPtrTy = PointerType::get(ET, AllocaAddrSpace);
  const Type *ValTysAL[] = { NewPtrTy, CAPtrTy}; 
  Function* TheAllocaBromFn = IntrinsicInfo.getDeclaration(Mod,  
        vtmIntrinsic::vtm_alloca_brom, ValTysAL, 2);
  //get the type of the context, construct the Args to the AllocaBromInst,  
  //and creat a callinst to call the intrinsic alloca function
  const Type* Int32TyAL = Type::getInt32Ty(Mod->getContext());
  CastInst* CIET = BitCastInst::CreatePointerCast(GV, CAPtrTy, 
        "const_cast", EntryPoint);

  Value* ArgsAL[] = {CIET, 
        ConstantInt::get(Int32TyAL, BRom[Con]), 
        ConstantInt::get(Int32TyAL, NumElems),
        ConstantInt::get(Int32TyAL, ElemSizeInBytes)};
  Instruction* AllocaBrom = CallInst::Create(TheAllocaBromFn, ArgsAL, 
        array_endof(ArgsAL), GEP->getName(), EntryPoint);

  //replace all the uses of the GV with the AllocaBrom instruction  
  //that calls the vtm_alloca_brom intrinsic function. Because they 
  //return the same type of pointer.
  const Type* ArrayPtrTy = PointerType::get(AT, AllocaAddrSpace);
  CastInst* CIAT = BitCastInst::CreatePointerCast(AllocaBrom, 
        ArrayPtrTy, "cast", EntryPoint);
  //There is a new BRomID, so store the CIAT into the SmallVector.
  //in case there are other GEPs who use the same BRom
  CIVect.push_back(CIAT);
  GEP->setOperand(0, CIAT);
  ++CastNum;

  return AllocaBrom;
}

Instruction* ContoBrom::AllocaBromLoad(GlobalVariable* GV, Constant* Con, 
                                     LoadInst* Load, 
                                     SmallVector<Instruction*, 16> &CIVect,
                                     Function* F) {
  //The GV is inside a Load Instruction
  //Gather the parameters of vtm_alloca_brom
  const IntegerType* IT = cast<IntegerType>(Con->getType());
  unsigned NumElems = 1;
  const Type* ET = IT->getScalarType();
  unsigned ElemSizeInBytes = ET->getPrimitiveSizeInBits() / 8;
  Instruction* EntryPoint = F->begin()->begin();

  //allocate space for the alloca and get declaration 
  //of the intrinsic function.
  unsigned AllocaAddrSpace = GV->getType()->getAddressSpace();

  //Transform the Element type into a pointer that 
  //points to the Element
  const Type* NewPtrTy = PointerType::get(ET, AllocaAddrSpace);                                                                             
  const Type* CAPtrTy = PointerType::get(ET, AllocaAddrSpace);
  const Type *ValTysAL[] = { NewPtrTy, CAPtrTy}; 
  Function* TheAllocaBromFn = IntrinsicInfo.getDeclaration(Mod,  
        vtmIntrinsic::vtm_alloca_brom, ValTysAL, 2);

  //get the type of the context, construct the Args to the AllocaBromInst,  
  //and creat a Callinst to call the intrinsic alloca function
  const Type* Int32TyAL = Type::getInt32Ty(Mod->getContext());
  CastInst* CIET = BitCastInst::CreatePointerCast(GV, CAPtrTy, 
        "const_cast", EntryPoint);

  Value* ArgsAL[] = {CIET, 
        ConstantInt::get(Int32TyAL, BRom[Con]), 
        ConstantInt::get(Int32TyAL, NumElems),
        ConstantInt::get(Int32TyAL, ElemSizeInBytes)};
  Instruction* AllocaBrom = CallInst::Create(TheAllocaBromFn, ArgsAL, 
        array_endof(ArgsAL), Load->getName(), EntryPoint);

  return AllocaBrom;
}

void ContoBrom::LoadtoBromAccess(LoadInst* Load, bool IsLoad, 
                           Instruction* AllocaBrom, Constant* Con) {
  //Use the LoadInst's information to get the declaration of the 
  //vtm_access_brom intrinsic

  Value* Ptr = 0;
  if(!IsLoad) Ptr = Load->getPointerOperand();
  else Ptr = AllocaBrom;

  const Type *ValTysAC[] = {Load->getType (), Ptr->getType()};
  Function* TheLoadBramFn = IntrinsicInfo.getDeclaration(Mod, 
          vtmIntrinsic::vtm_access_bram, ValTysAC, array_lengthof(ValTysAC));
  const Type* Int32TyAC = Type::getInt32Ty(Mod->getContext());
  const Type* Int1TyAC = Type::getInt1Ty(Mod->getContext());

  //Construct the Args of the vtm_access_bram intrinsic and 
  //creat a CallInst to call it.
  Value* ArgsAC[] = {Ptr, UndefValue::get(ValTysAC[0]), 
          ConstantInt::get(Int1TyAC,0),
          ConstantInt::get(Int32TyAC, Load->getAlignment ()),
          ConstantInt::get(Int1TyAC, Load->isVolatile ()),
          ConstantInt::get(Int32TyAC, BRom[Con])}; 
  Instruction* AccessBram = CallInst::Create(TheLoadBramFn, ArgsAC, 
          array_endof(ArgsAC), Load->getName(), Load);
  Load->replaceAllUsesWith (AccessBram);
}

char ContoBrom::ID = 0;
Pass *llvm::createContoBromPass(const TargetIntrinsicInfo &IntrinsicInfo) {
  return new ContoBrom(IntrinsicInfo);
}
