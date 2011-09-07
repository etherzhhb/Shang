//===--------------------------- GVPromotion.cpp --------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This file is to change the GlobalVariables in the functions to arguments, and 
// also modify the function declarations and the callsites of the function.
// After doing this, we can change the IR which uses GV to hardware.
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "vtm-gv-promotion"

#include <map>
#include "vtm/Passes.h"
#include "llvm/Pass.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Module.h"
#include "llvm/CallGraphSCCPass.h"
#include "llvm/Instructions.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Type.h"

#include "llvm/Analysis/CallGraph.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FormattedStream.h"
#include "vtm/HWPartitionInfo.h"
#include "llvm/PassAnalysisSupport.h"


using namespace llvm;


namespace {
  /// GVPromotion - Find out all the GVs in the function and change them to 
  /// arguments.
  struct GVPromotion : public CallGraphSCCPass {

    static char ID; // Pass identification, replacement for typeid

    GVPromotion() : CallGraphSCCPass(ID) {
      initializeGVPromotionPass(*PassRegistry::getPassRegistry());
    }

    virtual bool runOnSCC(CallGraphSCC &SCC);

    void getAnalysisUsage(AnalysisUsage &AU) const {
      AU.addRequired<HWPartitionInfo>();
      AU.addRequired<CallGraph>();   
      AU.setPreservesAll();
      CallGraphSCCPass::getAnalysisUsage(AU);
    }

    // It will be used later in the member function.
    CallGraph *CG;
    llvm::HWPartitionInfo *HWInfo;

  private:

    // Push all the instructions which use GVs as operands into the vector.
    void getInstWithGV(Function *F, SmallVectorImpl<Instruction*> &Inst);

    // Map the GV to the new arguments.
    void buildGVArgMap(SmallVectorImpl<Instruction*> &Inst, 
                     std::map<GlobalVariable*, Argument*> &GVMapArg);

    // Promote the old instructions in the function.
    void promoteInst(SmallVectorImpl<Instruction*> &Inst_old, 
                     std::map<GlobalVariable*, Argument*> &GVMapArg);

    // Create the new function which is ready to replace the old one.
    // In the method do the replacement.
    CallGraphNode *createNewCGN(Function *F, 
                                std::map<GlobalVariable*, Argument*> &GVMapArg, 
                                SmallVectorImpl<Instruction*> &Inst_old);

    // Promote functions that directly use Global Variables into functions that 
    //use pointers of GVs as arguments.
    bool PromoteReturn(CallGraphNode *CGN);

    // Clone the old function to the new function.
    Function *cloneFunction(Function *F);

    // Update all callsites that call F to use NF and reflash the CallGraph.
    bool updateAllCallSites(Function *F, Function *NF,
                                      CallGraphNode *CGN,
                                      std::map<GlobalVariable*, Argument*> &GVMapArg);


  };
}


char GVPromotion::ID = 0;


Pass *llvm::createGVPromotionPass() {
  return new GVPromotion();
}

INITIALIZE_PASS_BEGIN(GVPromotion, "GVPromotion",
  "GlobalVariables Promotion", false, false)
  INITIALIZE_PASS_DEPENDENCY(HWPartitionInfo)
  INITIALIZE_AG_DEPENDENCY(AliasAnalysis)
  INITIALIZE_AG_DEPENDENCY(CallGraph)


INITIALIZE_PASS_END(GVPromotion, "GVPromotion",
  "GlobalVariables Promotion", false, false)


bool GVPromotion::runOnSCC(CallGraphSCC &SCC) {
  HWInfo = &(getAnalysis<HWPartitionInfo>());
  bool PromotionChanged = false;
  for (CallGraphSCC::iterator I = SCC.begin(), E = SCC.end(); I != E; ++I) {
    CallGraphNode *CGN = *I;
    Function *F = CGN->getFunction();
    if (HWInfo->isHW(F)) {
      PromotionChanged |= PromoteReturn(*I);
    }
  }

  return PromotionChanged;
}

bool GVPromotion::PromoteReturn(CallGraphNode *CGN) {
  Function *F = CGN->getFunction();
  if (!F || F->isDeclaration())
    return false;

  DEBUG(dbgs() << "GVPromotion: Looking at function " 
    << F->getName() << "\n");

  // Find out all the instructions that are needed to promote in the function,
  // which is to say that these instrucions use GVs.
  SmallVector<Instruction*, 16> Inst_Promo;
  getInstWithGV(F, Inst_Promo);
  if (Inst_Promo.empty())
    return false;
  
  // When the function has instructions needed to promote, use a map to store 
  // relationship of newly generated arguments and GVs.
  std::map<GlobalVariable*, Argument*> GVMapArg;
  buildGVArgMap(Inst_Promo, GVMapArg);

  // Then change the instructions to use new arguments.
  promoteInst(Inst_Promo, GVMapArg);

  // Create the new function body and insert it into the module.
  Function *NF = cloneFunction(F);

  // Update all call sites to use new function and update the callgraph.
  return updateAllCallSites(F, NF, CGN, GVMapArg);


}

Function *GVPromotion::cloneFunction(Function *F) {
  const FunctionType *FTy = F->getFunctionType();
  std::vector<const Type*> Args_Ty;
  std::vector<Value*> Args;

  // (1) Get the argument types.
  for (Function::arg_iterator I = F->arg_begin(), E = F->arg_end();
       I != E; ) {
    Args_Ty.push_back(I->getType());
    Argument *Arg_Temp = I;
    DEBUG(dbgs() << Arg_Temp->getName() << "\n");      
    Args.push_back(I);
    I++;
  }

  // (2) Copy the attributes from the old function to the new function.
  SmallVector<AttributeWithIndex, 8> AttributesVec;
  const AttrListPtr &PAL = F->getAttributes();

  // First add any return attributes.
  if (Attributes attrs = PAL.getRetAttributes())
    AttributesVec.push_back(AttributeWithIndex::get(0, attrs));

  // Then add the attributes of the arguments.
  // Here we don't need the attributes of the arguments, so do nothing.

  // Finally add any function attributes.
  if (Attributes attrs = PAL.getFnAttributes())
    AttributesVec.push_back(AttributeWithIndex::get(~0, attrs));

  // (3) Now things are ready. Create the new function.
  // Construct the new function type using the new arguments.
  FunctionType *NFTy = FunctionType::get(FTy->getReturnType(), Args_Ty,
    FTy->isVarArg());

  // Create the new function.
  Function *NF = Function::Create(NFTy, F->getLinkage(), F->getName());

  // Then set the attributes.
  NF->copyAttributesFrom(F);
  NF->setAttributes(AttrListPtr::get(AttributesVec.begin(),
                    AttributesVec.end()));

  // Insert the new function into the module.
  F->getParent()->getFunctionList().insert(F, NF);
  NF->takeName(F);

  // Cut and paste the old function body and tell the body to use new 
  // arguments in new function.
  NF->getBasicBlockList().splice(NF->begin(), F->getBasicBlockList());
  for (Function::arg_iterator IOld = F->arg_begin(), EOld = F->arg_end(),
       INew = NF->arg_begin(); IOld != EOld; ++IOld, ++INew) {
    INew->takeName(IOld);
    IOld->replaceAllUsesWith(INew);     
  }

  return NF;
}

bool GVPromotion::updateAllCallSites(Function *F, Function *NF, 
                                               CallGraphNode *CGN,
                                               std::map<GlobalVariable*, Argument*> &GVMapArg) {
  CallGraph &CG = getAnalysis<CallGraph>();

  // Use NF to replace F, at the same time update the callgraph.
  (&CG)->spliceFunction(F, NF);
  CallGraphNode *NF_CGN = CG.getOrInsertFunction(NF);

  // Vectors of arguments and argument attributes of the callsite.
  std::vector<Value*> NewArgs;
  SmallVector<AttributeWithIndex, 8> AttributesVec;

  while(!F->use_empty()) {
    // Clear the vectors of arguments and argument attributes
    // for the following callsite.
    NewArgs.clear();
    AttributesVec.clear();

    // Get the callsite.
    CallSite CS(F->use_back());
    Instruction *Call = CS.getInstruction();

    // [1] Create a new argument vector for the new instruction.
    // (1) Get the old arguments from the old instruction.       
    Function::arg_iterator IOld = F->arg_begin();
    for (CallSite::arg_iterator ICS = CS.arg_begin(), ECS = CS.arg_end(); 
         ICS != ECS; ++ICS, ++IOld) {
      NewArgs.push_back(*ICS);
      DEBUG(dbgs() << "old argument:  " << (*ICS)->getName() << "\n");
    }
    // (2) Put the GVs into the vector.
    for (Function::arg_iterator EOld = F->arg_end(); IOld != EOld; 
         ++IOld) {         
      for (std::map<GlobalVariable*, Argument*>::iterator 
            IMap = GVMapArg.begin(), EMap = GVMapArg.end(); 
            IMap != EMap; ++IMap) {
        Argument *GVArg = IOld;
        Argument *MapArg = IMap->second;
        if (GVArg == MapArg) {
          NewArgs.push_back(IMap->first);
          continue;
        }
      }
    }
    // Check the new vector.
    for (std::vector<Value*>::iterator I = NewArgs.begin(), E = NewArgs.end(); 
         I != E; ++I) {
      DEBUG(dbgs() << "new argument: "<< (*I)->getName() << "\n");
    }

    // [2] Create the attribute vector of the callsites.
    const AttrListPtr &CallPAL = CS.getAttributes();
        
    // (1) Add any return attributes.
    if (Attributes attrs = CallPAL.getRetAttributes())
      AttributesVec.push_back(AttributeWithIndex::get(0, attrs));

    // (2) Argument attribute.    
    // Do nothing.

    // (3) Add any function attributes.
    if (Attributes attrs = CallPAL.getFnAttributes())
      AttributesVec.push_back(AttributeWithIndex::get(~0, attrs));

    // [3] Create a new instruction to replace the old callsite. And at the same
    // time set the attributes.
    Instruction *New;  
    if (InvokeInst *II = dyn_cast<InvokeInst>(Call)) {
      New = InvokeInst::Create(NF, II->getNormalDest(), II->getUnwindDest(),
                                NewArgs.begin(), NewArgs.end(),  "", Call);
      cast<InvokeInst>(New)->setCallingConv(CS.getCallingConv());
      cast<InvokeInst>(New)->setAttributes(AttrListPtr::get(AttributesVec.begin(),
                                                            AttributesVec.end()));
    } else {
      New = CallInst::Create(NF, NewArgs.begin(), NewArgs.end(), "", Call);
      cast<CallInst>(New)->setCallingConv(CS.getCallingConv());
      cast<CallInst>(New)->setAttributes(AttrListPtr::get(AttributesVec.begin(),
                                                          AttributesVec.end()));
      if (dyn_cast<CallInst>(Call)->isTailCall())
        dyn_cast<CallInst>(New)->setTailCall();
    }

    New->takeName(Call);

    // [4] Update the callgraph to know that the callsite has been transformed.
    CallGraphNode *CalleeNode = CG[Call->getParent()->getParent()];
    CalleeNode->replaceCallEdge(Call, New, NF_CGN);
    if (!Call->use_empty()) {
      Call->replaceAllUsesWith(New);
      New->setName(Call->getName());
    }

    // Finally, remove the old call from the program, 
    // reducing the use-count of F.
    Call->eraseFromParent();
  }

  // Update the callgraph.
  F->eraseFromParent();
  return true;    

}

void GVPromotion::getInstWithGV(Function *F, 
                                SmallVectorImpl<Instruction*> &Inst) {

  // The iterating order: Function -> BasicBlock ->Instruction
  // (1) Function iterator, get the BasicBlocks(IF).
  for (Function::iterator IF = F->begin(),EF = F->end();
       IF != EF; ++IF) {
    // (2) BasicBlock iterator, get the instructions(IBB).
    for(BasicBlock::iterator IBB = IF->begin(), EBB = IF->end();
        IBB != EBB; ++IBB) {
      // (3) Instruction iterator, get the operands(I).
      for (Instruction::op_iterator I = IBB->op_begin(), 
           E = IBB->op_end(); I != E; ++I) {
          // If one of the operands in the instruction is GV, 
          // then push the instruction into the vector immediately.
        if(isa<GlobalVariable>(*I)) {
          Inst.push_back(IBB);             
        }
      }
    }
  }
}

void GVPromotion::buildGVArgMap(SmallVectorImpl<Instruction*> &Inst, 
                              std::map<GlobalVariable*, Argument*> &GVMapArg) {
  // The key type of the map is GlobalVariable*, and the mapped type is 
  // Argument*.

  // (1) Vector iterator, get the instructions out from the vector.
  for (SmallVectorImpl<Instruction*>::iterator IVector = Inst.begin(), 
       EVector = Inst.end(); IVector != EVector; ++IVector) {
    Instruction *Inst_temp = *IVector;
  // (2) Instruction iterator, get the operands of the instruction.
    for (Instruction::op_iterator I = Inst_temp->op_begin(), 
         E = Inst_temp->op_end(); I != E; ++I) {   
      // If the operand is GV and has not been found in the map, 
      // create the corresponding mapped value.
      if(!isa<GlobalVariable>(*I)) continue;
      GlobalVariable *GV = dyn_cast<GlobalVariable>(I);
      if (!GVMapArg.count(GV)) {
        GVMapArg[GV] = new Argument(dyn_cast<PointerType>(GV->getType()),
                                    GV->getName()+"_val", 
                                    Inst_temp->getParent()->getParent());
      }           
    }
  }
}

void GVPromotion::promoteInst(SmallVectorImpl<Instruction*> &Inst_Promo, 
                              std::map<GlobalVariable*, Argument*> &GVMapArg) {
        
  for (SmallVectorImpl<Instruction*>::iterator I_Vector = Inst_Promo.begin(), 
       E_Vector = Inst_Promo.end(); I_Vector != E_Vector; ++I_Vector) {
    Instruction *Inst = *I_Vector;
    /*if (Inst->getOpcode() == Instruction::Call)
      continue;*/
    for (Instruction::op_iterator I = Inst->op_begin(), 
         E = Inst->op_end(); I != E; ++I) {   
      if(!isa<GlobalVariable>(*I)) continue;           
      GlobalVariable *GV = dyn_cast<GlobalVariable>(I);

      // Now begin to do the promotion of the GV in the instruction of 
      // the function.
              
      // One way is to directly replace the operand with the new one.
      I->set(GVMapArg[GV]);
    }
  }
}
