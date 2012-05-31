//===------------- VLang.h - Verilog HDL writing engine ---------*- C++ -*-===//
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
// This file implement the VLang class, with provide funtions to complete
// common Verilog HDL writing task.
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilogAST.h"
#include "vtm/Utilities.h"

#include "llvm/Constants.h"
#include "llvm/GlobalVariable.h"
#include "llvm/DerivedTypes.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Format.h"
#define DEBUG_TYPE "verilog-ast"
#include "llvm/Support/Debug.h"

#include <sstream>

using namespace llvm;

static cl::opt<bool>
EnableBBProfile("vtm-enable-bb-profile",
                cl::desc("Generate counters to profile the design"),
                cl::init(false));
static cl::opt<unsigned>
ExprInlineThreshold("vtm-expr-inline-thredhold",
                    cl::desc("Inline the expression which has less than N use"
                             " (0 for always inline)"),
                    cl::init(2));

//===----------------------------------------------------------------------===//
// Value and type printing
static std::string verilogConstToStr(uint64_t Value, unsigned bitwidth,
                                     bool isMinValue) {
  std::string ret;
  ret = utostr_32(bitwidth) + '\'';
  if (bitwidth == 1) ret += "b";
  else               ret += "h";
  // Mask the value that small than 4 bit to prevent printing something
  // like 1'hf out.
  if (bitwidth < 4) Value &= (1 << bitwidth) - 1;

  if(isMinValue) {
    ret += utohexstr(Value);
    return ret;
  }

  std::string ss = utohexstr(Value);
  unsigned int uselength = (bitwidth/4) + (((bitwidth&0x3) == 0) ? 0 : 1);
  if(uselength < ss.length())
    ss = ss.substr(ss.length() - uselength, uselength);
  ret += ss;

  return ret;
}

//----------------------------------------------------------------------------//
// Helper function for Verilog RTL printing.

static raw_ostream &printAssign(raw_ostream &OS, const VASTWire *W) {
  OS << "assign " << W->getName()
     << VASTValue::printBitRange(W->getBitWidth(), 0, false)
     << " = ";
  return OS;
}

void VASTNode::dump() const {
  print(dbgs());
  dbgs() << '\n';
}

//----------------------------------------------------------------------------//
// Classes in Verilog AST.
VASTUse::VASTUse(VASTValPtr v, VASTValue *u) : V(v), User(u) {
  if (User) setUser(User);
}

void VASTUse::removeFromList() {
  get()->removeUseFromList(this);
}

void VASTUse::setUser(VASTValue *User) {
  assert(!ilist_traits<VASTUse>::inAnyList(this)
         && "Not unlink from old list!");
  VASTValue *Use = getAsLValue<VASTValue>();
  assert(Use != User && "Unexpected cycle!");
  this->User = User;
  Use->addUseToList(this);
}

bool VASTUse::operator==(const VASTValPtr RHS) const {
  return V == RHS;
}

void VASTUse::PinUser() const {
  if (VASTSignal *S = dyn_cast<VASTSignal>(getAsLValue<VASTValue>()))
    S->Pin();
}

unsigned VASTUse::getBitWidth() const{ return get()->getBitWidth(); }

VASTSlot::VASTSlot(unsigned slotNum, MachineBasicBlock *BB, VASTModule *VM)
  : VASTNode(vastSlot), SlotReg(0, 0), SlotActive(0, 0), SlotReady(0, 0),
    StartSlot(slotNum), EndSlot(slotNum), II(~0), SlotNum(slotNum) {
  Contents.ParentBB = BB;

  // Create the relative signals.
  SlotReg.set(VM->addSlotRegister(this));
  std::string SlotName = "Slot" + utostr_32(slotNum);

  VASTWire *Ready = VM->addWire(SlotName + "Ready", 1,
                                VASTModule::DirectClkEnAttr.c_str());
  SlotReady.set(Ready);

  VASTWire *Active = VM->addWire(SlotName + "Active", 1,
                                 VASTModule::DirectClkEnAttr.c_str());
  SlotActive.set(Active);
}

void VASTSlot::addSuccSlot(VASTSlot *NextSlot, VASTValPtr Cnd, VASTModule *VM) {
  VASTUse *&U = NextSlots[NextSlot];
  if (U == 0) {
    NextSlot->PredSlots.push_back(this);
    U = new (VM->allocateUse()) VASTUse(Cnd, 0);
  } else
    U->replaceUseBy(VM->buildOrExpr(Cnd, U->unwrap(), 1));
}

void VASTSlot::addEnable(VASTRegister *R, VASTValPtr Cnd, VASTModule *VM) {
  VASTUse *&U = Enables[R];
  if (U == 0)
    U = new (VM->allocateUse()) VASTUse(Cnd, 0);
  else
    U->replaceUseBy(VM->buildOrExpr(Cnd, U->unwrap(), 1));
}

void VASTSlot::addReady(VASTValue *V, VASTValPtr Cnd, VASTModule *VM) {
  VASTUse *&U = Readys[V];
  if (U == 0)
    U = new (VM->allocateUse()) VASTUse(Cnd, 0);
  else
    U->replaceUseBy(VM->buildOrExpr(Cnd, U->unwrap(), 1));
}

void VASTSlot::addDisable(VASTRegister *R, VASTValPtr Cnd, VASTModule *VM) {
  VASTUse *&U = Disables[R];
  if (U == 0)
    U = new (VM->allocateUse()) VASTUse(Cnd, 0);
  else
    U->replaceUseBy(VM->buildOrExpr(Cnd, U->unwrap(), 1));
}

VASTValPtr VASTSlot::buildFUReadyExpr(VASTModule &VM) {
  SmallVector<VASTValPtr, 4> Ops;

  for (VASTSlot::const_fu_rdy_it I = ready_begin(), E = ready_end();I != E; ++I)
    // Print the code for ready signal.
    // If the condition is true then the signal must be 1 to ready.
    Ops.push_back(VM.buildOrExpr(I->first,
                                 VM.buildNotExpr(I->second->getAsInlineOperand()),
                                 1));
  
  // No waiting signal means always ready.
  if (Ops.empty()) Ops.push_back(VM.getBoolImmediate(true));
  
  return VM.buildExpr(VASTExpr::dpAnd, Ops, 1);
}

void VASTSlot::buildReadyLogic(VASTModule &Mod) {
  SmallVector<VASTValPtr, 4> Ops;
  // FU ready for current slot.
  Ops.push_back(buildFUReadyExpr(Mod));

  if (hasAliasSlot()) {
    for (unsigned s = alias_start(), e = alias_end(), ii = alias_ii();
         s < e; s += ii) {
      if (s == SlotNum) continue;

      VASTSlot *AliasSlot = Mod.getSlot(s);

      if (!AliasSlot->readyEmpty()) {
        // FU ready for alias slot, when alias slot register is 1, its waiting
        // signal must be 1.
        Ops.push_back(Mod.buildOrExpr(Mod.buildNotExpr(AliasSlot->
                                                       getRegister()),
                                      AliasSlot->buildFUReadyExpr(Mod), 1));
      }
    }
  }

  // All signals should be 1 before the slot is ready.
  VASTValPtr ReadyExpr = Mod.buildExpr(VASTExpr::dpAnd, Ops, 1);
  Mod.assign(cast<VASTWire>(getReady()), ReadyExpr);
  // The slot is actived when the slot is enable and all waiting signal is ready
  Mod.assign(cast<VASTWire>(getActive()),
             Mod.buildExpr(VASTExpr::dpAnd, SlotReg, ReadyExpr, 1));
}

bool VASTSlot::hasNextSlot(VASTSlot *NextSlot) const {
  if (NextSlots.empty()) return NextSlot->SlotNum == SlotNum + 1;

  return NextSlots.count(NextSlot);
}

void VASTSlot::buildCtrlLogic(VASTModule &Mod) {
  vlang_raw_ostream &CtrlS = Mod.getControlBlockBuffer();
  // TODO: Build the AST for these logic.
  CtrlS.if_begin(getName());
  bool ReadyPresented = !readyEmpty();

  // DirtyHack: Remember the enabled signals in alias slots, the signal may be
  // assigned at a alias slot.
  std::set<const VASTValue *> AliasEnables;
  // A slot may be enable by its alias slot if II of a pipelined loop is 1.
  VASTValPtr PredAliasSlots = 0;

  if (hasAliasSlot()) {
    CtrlS << "// Alias slots: ";

    for (unsigned s = alias_start(), e = alias_end(), ii = alias_ii();
         s < e; s += ii) {
      CtrlS << s << ", ";
      if (s == SlotNum) continue;

      const VASTSlot *AliasSlot = Mod.getSlot(s);
      if (AliasSlot->hasNextSlot(this)) {
        assert(!PredAliasSlots
               && "More than one PredAliasSlots found!");
        PredAliasSlots = AliasSlot->getActive();
      }

      for (VASTSlot::const_fu_ctrl_it I = AliasSlot->enable_begin(),
           E = AliasSlot->enable_end(); I != E; ++I) {
        bool inserted = AliasEnables.insert(I->first).second;
        assert(inserted && "The same signal is enabled twice!");
        (void) inserted;
      }

      ReadyPresented  |= !AliasSlot->readyEmpty();
    }

    CtrlS << '\n';
  } // SS flushes automatically here.

  DEBUG_WITH_TYPE("vtm-codegen-self-verify",
  if (SlotNum != 0)
    CtrlS << "$display(\"" << getName() << " in " << Mod.getName() << " BB#"
          << getParentBB()->getNumber() << ' '
          << getParentBB()->getBasicBlock()->getName()
          << " ready at %d\", $time());\n";
  );

  bool hasSelfLoop = false;
  SmallVector<VASTValPtr, 2> EmptySlotEnCnd;

  if (hasExplicitNextSlots()) {
    CtrlS << "// Enable the successor slots.\n";
    for (VASTSlot::const_succ_cnd_iterator I = succ_cnd_begin(),E = succ_cnd_end();
         I != E; ++I) {
      hasSelfLoop |= I->first->SlotNum == SlotNum;
      VASTRegister *NextSlotReg = I->first->getRegister();
      Mod.addAssignment(NextSlotReg, *I->second, this, EmptySlotEnCnd);
    }
  } else {
    // Enable the default successor slots.
    VASTSlot *NextSlot = Mod.getSlot(SlotNum + 1);
    VASTRegister *NextSlotReg = NextSlot->getRegister();
    Mod.addAssignment(NextSlotReg, Mod.getBoolImmediate(true), this,
                      EmptySlotEnCnd);
    // And connect the fall through edge now.
    Mod.addSlotSucc(this, NextSlot, Mod.getBoolImmediate(true));
  }

  assert(!(hasSelfLoop && PredAliasSlots)
         && "Unexpected have self loop and pred alias slot at the same time.");
  // Do not assign a value to the current slot enable twice.
  if (!hasSelfLoop) {
    // Only disable the current slot if there is no alias slot enable current
    // slot.
    if (PredAliasSlots)
      EmptySlotEnCnd.push_back(Mod.buildNotExpr(PredAliasSlots));

    // Disable the current slot.
    Mod.addAssignment(getRegister(), Mod.getBoolImmediate(false), this,
                      EmptySlotEnCnd);
  }

  if (!ReadyPresented) {
    DEBUG_WITH_TYPE("vtm-codegen-self-verify",
    if (SlotNum != 0) {
      CtrlS << "if (start) begin $display(\"" << getName() << " in "
            << Mod.getName()
            << " bad start %b\\n\", start);  $finish(); end\n";

      CtrlS << "if (Slot0r) begin $display(\"" << getName() << " in "
            << Mod.getName()
            << " bad Slot0 %b\\n\", Slot0r);  $finish(); end\n";
    }

    CtrlS << "if (mem0en_r) begin $display(\"" << getName() << " in "
          << Mod.getName()
          << " bad mem0en_r %b\\n\", mem0en_r);  $finish(); end\n";
    );
  }

  std::string SlotReady = std::string(getName()) + "Ready";
  CtrlS << "// Enable the active FUs.\n";
  for (VASTSlot::const_fu_ctrl_it I = enable_begin(), E = enable_end();
       I != E; ++I) {

    assert(!AliasEnables.count(I->first) && "Signal enabled by alias slot!");
    // No need to wait for the slot ready.
    // We may try to enable and disable the same port at the same slot.
    EmptySlotEnCnd.clear();
    EmptySlotEnCnd.push_back(getRegister());
    Mod.addAssignment(cast<VASTRegister>(I->first),
                      Mod.buildExpr(VASTExpr::dpAnd,
                                    getReady()->getAsInlineOperand(false),
                                    I->second->getAsInlineOperand(), 1),
                      this, EmptySlotEnCnd, false);
  }

  SmallVector<VASTValPtr, 4> DisableAndCnds;
  if (!disableEmpty()) {
    CtrlS << "// Disable the resources when the condition is true.\n";
    for (VASTSlot::const_fu_ctrl_it I = disable_begin(), E = disable_end();
         I != E; ++I) {
      // Look at the current enable set and alias enables set;
      // The port assigned at the current slot, and it will be disabled if
      // The slot is not ready or the enable condition is false. And it is
      // ok that the port is enabled.
      if (isEnabled(I->first)) continue;

      DisableAndCnds.push_back(getRegister());
      // If the port enabled in alias slots, disable it only if others slots is
      // not active.
      bool AliasEnabled = AliasEnables.count(I->first);
      if (AliasEnabled) {
        for (unsigned s = alias_start(), e = alias_end(), ii = alias_ii();
             s < e; s += ii) {
          if (s == SlotNum) continue;

          VASTSlot *ASlot = Mod.getSlot(s);
          assert(!ASlot->isDiabled(I->first)
                 && "Same signal disabled in alias slot!");
          if (ASlot->isEnabled(I->first)) {
            DisableAndCnds.push_back(Mod.buildNotExpr(ASlot->getRegister()));
            continue;
          }
        }
      }

      DisableAndCnds.push_back(*I->second);

      VASTRegister *En = cast<VASTRegister>(I->first);
      Mod.addAssignment(En, Mod.getBoolImmediate(false), this,
                        DisableAndCnds, false);
      DisableAndCnds.clear();
    }
  }
  CtrlS.exit_block("\n\n");
}

const char *VASTSlot::getName() const { return getRegister()->getName(); }

void VASTSlot::print(raw_ostream &OS) const {
  llvm_unreachable("VASTSlot::print should not be called!");
}

VASTRegister::VASTRegister(const char *Name, unsigned BitWidth,
                           uint64_t initVal, VASTRegister::Type T,
                           uint16_t RegData,const char *Attr)
  : VASTSignal(vastRegister, Name, BitWidth, Attr), InitVal(initVal) {
  SignalType = T;
  SignalData = RegData;
}

void VASTRegister::addAssignment(VASTUse *Src, VASTWire *AssignCnd) {
  assert(AssignCnd->getWireType() == VASTWire::AssignCond
         && "Expect wire for assign condition!");
  bool inserted = Assigns.insert(std::make_pair(AssignCnd, Src)).second;
  assert(inserted &&  "Assignment condition conflict detected!");
  Src->setUser(this);
}

//VASTUse VASTRegister::getConstantValue() const {
//  std::set<VASTUse> Srcs;
//  for (assign_itertor I = assign_begin(), E = assign_end(); I != E; ++I)
//    Srcs.insert(*I->second);
//
//  // Only 1 assignment?
//  if (Srcs.size() != 1) return VASTUse();
//
//  VASTUse C = *Srcs.begin();
//
//  if (!C.isImm()) return VASTUse();
//
//  // Some register assignment may have un-match bit-width
//  return VASTUse(C.getImm(), getBitWidth());
//}

void VASTRegister::printCondition(raw_ostream &OS, const VASTSlot *Slot,
                                  const AndCndVec &Cnds) {
  OS << '(';
  if (Slot) {
    VASTValPtr Active = Slot->getActive();
    Active.printAsOperand(OS);
    if (VASTSignal *S = Active.getAsLValue<VASTSignal>()) S->Pin();
  } else      OS << "1'b1";

  typedef AndCndVec::const_iterator and_it;
  for (and_it CI = Cnds.begin(), CE = Cnds.end(); CI != CE; ++CI) {
    OS << " & ";
    CI->printAsOperand(OS);
    if (VASTSignal *S = CI->getAsLValue<VASTSignal>()) S->Pin();
  }

  OS << ')';
}

void VASTRegister::verifyAssignCnd(vlang_raw_ostream &OS,
                                   const VASTModule *Mod) const {
  // Concatenate all condition together to detect the case that more than one
  // case is activated.
  std::string AllPred;
  raw_string_ostream AllPredSS(AllPred);

  AllPredSS << '{';
  for (assign_itertor I = assign_begin(), E = assign_end(); I != E; ++I) {
    I->first->printAsOperand(AllPredSS, false);
    AllPredSS << ", ";
  }
  AllPredSS << "1'b0 }";
  AllPredSS.flush();

  // As long as $onehot0(expr) returns true if at most one bit of expr is high,
  // we can use it to detect if more one case condition is true at the same
  // time.
  OS << "\nif (!$onehot0(" << AllPred << "))"
        " begin $display(\"At time %t, register "
        << getName() << " in module " << ( Mod ? Mod->getName() : "Unknown")
        << " has more than one active assignment: %b!\", $time(), "
        << AllPred << ");\n";

  // Display the conflicted condition and its slot.
  for (assign_itertor I = assign_begin(), E = assign_end(); I != E; ++I) {
    OS.indent(2) << "if (";
    I->first->printAsOperand(OS, false);
    OS << ") begin\n";

    OS.indent(4) << "$display(\"Condition: ";
    I->first->printAsOperand(OS, false);

    unsigned CndSlot = I->first->getSlotNum();
    VASTSlot *S = Mod->getSlot(CndSlot);
    OS << ", current slot: " << CndSlot << ", ";

    if (CndSlot) OS << "in BB#" << S->getParentBB()->getNumber() << ',';

    if (S->hasAliasSlot()) {
      OS << " Alias slots: ";
      for (unsigned s = S->alias_start(), e = S->alias_end(), ii = S->alias_ii();
           s < e; s += ii)
        OS << s << ", ";
    }
    OS << "\");\n";
    OS.indent(2) << "end\n";
  }

  OS.indent(2) << "$finish();\nend\n";
}

void VASTRegister::printReset(raw_ostream &OS) const {
  OS << getName()  << " <= "
     << verilogConstToStr(InitVal, getBitWidth(), false) << ";";
}

void VASTRegister::printAssignment(vlang_raw_ostream &OS,
                                   const VASTModule *Mod) const {
  if (Assigns.empty()) return;

  bool UseSwitch = Assigns.size() > 1;

  typedef std::vector<VASTValPtr> OrVec;
  typedef std::map<VASTValPtr, OrVec> CSEMapTy;
  CSEMapTy SrcCSEMap;

  for (assign_itertor I = assign_begin(), E = assign_end(); I != E; ++I)
    SrcCSEMap[*I->second].push_back(I->first);

  OS << "// Assignment of " << getName() << '\n';
  if (UseSwitch) {
    OS << VASTModule::ParallelCaseAttr << ' ';
    OS.switch_begin("1'b1");
  }

  std::string Pred;
  raw_string_ostream PredSS(Pred);
  typedef CSEMapTy::iterator it;

  for (it I = SrcCSEMap.begin(), E = SrcCSEMap.end(); I != E; ++I) {
    PredSS << '(';

    OrVec &Ors = I->second;
    for (OrVec::iterator OI = Ors.begin(), OE = Ors.end(); OI != OE; ++OI) {
      OI->printAsOperand(PredSS);
      PredSS << '|';
    }

    PredSS << "1'b0)";
    PredSS.flush();
    // Print the assignment under the condition.
    if (UseSwitch) OS.match_case(Pred);
    else OS.if_begin(Pred);
    printAsOperandImpl(OS);
    OS << " <= ";
    I->first.printAsOperand(OS);
    OS << ";\n";
    OS.exit_block();

    Pred.clear();
  }

  if (UseSwitch) OS.switch_end();

  DEBUG_WITH_TYPE("vtm-codegen-self-verify",  verifyAssignCnd(OS, Mod));
}

void VASTRegister::dumpAssignment() const {
  vlang_raw_ostream S(dbgs());
  printAssignment(S, 0);
}

VASTExpr::VASTExpr(Opcode Opc, uint8_t NumOps, unsigned UB,
                   unsigned LB, const FoldingSetNodeIDRef ID)
  : VASTValue(vastExpr, UB - LB), FastID(ID), Opc(Opc), NumOps(NumOps),
    UB(UB), LB(LB) {
  Contents.Name = 0;
  assert(NumOps && "Unexpected empty operand list!");
}

std::string VASTModule::DirectClkEnAttr = "";
std::string VASTModule::ParallelCaseAttr = "";
std::string VASTModule::FullCaseAttr = "";

VASTModule::~VASTModule() {
  // Release all ports.
  Ports.clear();
  Wires.clear();
  Registers.clear();
  Slots.clear();
  Allocator.Reset();
  SymbolTable.clear();
  UniqueExprs.clear();

  delete &(DataPath.str());
  delete &(ControlBlock.str());
}

void VASTModule::printDatapath(raw_ostream &OS) const{
  for (WireVector::const_iterator I = Wires.begin(), E = Wires.end();
       I != E; ++I) {
    VASTWire *W = *I;
    // Do not print the trivial dead data-path.
    if (W->getAssigningValue() && (W->isPinned() || !W->use_empty()))
      W->printAssignment(OS);
  }
}

void VASTModule::printRegisterAssign(vlang_raw_ostream &OS) const {
  for (RegisterVector::const_iterator I = Registers.begin(), E = Registers.end();
       I != E; ++I)
    (*I)->printAssignment(OS, this);
}

void VASTModule::addSlotEnable(VASTSlot *S, VASTRegister *R, VASTValPtr Cnd) {
  S->addEnable(R, Cnd, this);
}

void VASTModule::addSlotDisable(VASTSlot *S, VASTRegister *R, VASTValPtr Cnd) {
  S->addDisable(R, Cnd, this);
}

void VASTModule::addSlotReady(VASTSlot *S, VASTValue *V, VASTValPtr Cnd) {
  S->addReady(V, Cnd, this);
}

void VASTModule::addSlotSucc(VASTSlot *S, VASTSlot *SuccS, VASTValPtr V) {
  S->addSuccSlot(SuccS, V, this);
}

void VASTModule::buildSlotLogic() {
  bool IsFirstSlotInBB = false;
  for (SlotVecTy::const_iterator I = Slots.begin(), E = Slots.end();I != E;++I){
    if (VASTSlot *S = *I) {
      S->buildCtrlLogic(*this);

      // Create a profile counter for each BB.
      if (EnableBBProfile) writeProfileCounters(S, IsFirstSlotInBB);
      IsFirstSlotInBB = false;
      continue;
    }

    // We meet an end slot, The next slot is the first slot in new BB
    IsFirstSlotInBB = true;
  }
}

void VASTModule::printModuleDecl(raw_ostream &OS) const {
  OS << "module " << getName() << "(\n";
  Ports.front()->print(OS.indent(4));
  for (PortVector::const_iterator I = Ports.begin() + 1, E = Ports.end();
       I != E; ++I) {
    OS << ",\n";
    (*I)->print(OS.indent(4));
  }
  OS << ");\n";
}

void VASTModule::printSignalDecl(raw_ostream &OS) {
  unsigned InlineThreshold = ExprInlineThreshold;
  if (InlineThreshold == 0) InlineThreshold = ~0;

  for (WireVector::const_iterator I = Wires.begin(), E = Wires.end();
       I != E; ++I) {
    VASTWire *W = *I;

    // Print the declaration.
    if (W->use_empty() && !W->isPinned()) OS << "//";
    W->printDecl(OS);
    OS << "// uses " << W->num_uses() << " pinned " << W->isPinned() << '\n';
  }

  for (RegisterVector::const_iterator I = Registers.begin(), E = Registers.end();
       I != E; ++I) {
    VASTRegister *R = *I;
    // The output register already declared in module signature.
    if (R->getRegType() == VASTRegister::OutputPort) continue;

    R->printDecl(OS);
    OS << "\n";
  }
}

void VASTModule::printRegisterReset(raw_ostream &OS) {
  for (RegisterVector::const_iterator I = Registers.begin(), E = Registers.end();
       I != E; ++I) {
    (*I)->printReset(OS);
    OS << "\n";
  }
}

VASTValPtr VASTModule::buildNotExpr(VASTValPtr U) {
  U = U.invert();

  if (U.isInverted()) {
    // Try to fold the not expression.
    if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(U.get()))
      return getOrCreateImmediate(~Imm->getValue(), Imm->getBitWidth());
  }

  return U;
}


VASTValPtr VASTModule::foldBitSliceExpr(VASTValPtr U, uint8_t UB, uint8_t LB) {
  unsigned OperandSize = U->getBitWidth();
  // Not a sub bitslice.
  if (UB == OperandSize && LB == 0) return U;

  // Try to fold the bitslice.
  VASTValue *V = U.get();
  bool isInverted = U.isInverted();
  VASTExpr *AssignExpr = 0;
  if (VASTExpr *E = dyn_cast<VASTExpr>(V))
    AssignExpr = E;
  else if (VASTWire *W = dyn_cast<VASTWire>(V)) {
    VASTExprPtr P = W->getExpr().invert(isInverted);
    // DirtyHack: the Invert information will be lost by this way. Allocate a
    // new VASTExprPtr?
    AssignExpr = P.get();
    isInverted = P.isInverted();
  }

  if (AssignExpr) {
    switch(AssignExpr->getOpcode()) {
    default: break;
    case VASTExpr::dpAssign: {
      unsigned Offset = AssignExpr->LB;
      UB += Offset;
      LB += Offset;
      return buildBitSliceExpr(AssignExpr->getOperand(0), UB, LB).invert(isInverted);
    }
    case VASTExpr::dpBitCat: {
      VASTValPtr Hi = AssignExpr->getOperand(0),
                 Lo = AssignExpr->getOperand(1);
      if (Lo->getBitWidth() == LB)
        return buildBitSliceExpr(Hi, UB - LB, 0).invert(isInverted);
      break;
    }
    }
  }

  if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(V)) {
    uint64_t imm = getBitSlice64(Imm->getValue(), UB, LB);
    if (isInverted) imm = ~imm;
    return getOrCreateImmediate(imm, UB - LB);
  }

  return VASTValPtr(0);
}

VASTValPtr VASTModule::buildBitSliceExpr(VASTValPtr U, uint8_t UB, uint8_t LB) {
  // Try to fold the expression.
  if (VASTValPtr P = foldBitSliceExpr(U, UB, LB)) return P;

  assert(UB <= U->getBitWidth() && UB > LB && "Bad bit range!");
  // FIXME: We can name the expression when necessary.
  assert(isa<VASTNamedValue>(U)
         && cast<VASTNamedValue>(U)->getName()
         && *cast<VASTNamedValue>(U)->getName() != '\0'
         && "Cannot get bitslice of value without name!");

  VASTValPtr Ops[] = { U };
  return createExpr(VASTExpr::dpAssign, Ops, UB, LB);
}

VASTValPtr VASTModule::buildBitCatExpr(ArrayRef<VASTValPtr> Ops,
                                       unsigned BitWidth) {
  VASTImmediate *LastImm = dyn_cast<VASTImmediate>(Ops[0]);
  SmallVector<VASTValPtr, 8> NewOps;
  NewOps.push_back(Ops[0]);

  // Merge the constant sequence.
  for (unsigned i = 1; i < Ops.size(); ++i) {
    VASTImmediate *CurImm = dyn_cast<VASTImmediate>(Ops[i]);

    if (!CurImm) {
      LastImm = 0;
      NewOps.push_back(Ops[i]);
      continue;
    }

    if (LastImm) {
      // Merge the constants.
      uint64_t HiVal = LastImm->getValue(), LoVal = CurImm->getValue();
      unsigned HiSizeInBits = LastImm->getBitWidth(),
               LoSizeInBits = CurImm->getBitWidth();
      unsigned SizeInBits = LoSizeInBits + HiSizeInBits;
      assert(SizeInBits <= 64 && "Constant too large!");
      uint64_t Val = (LoVal) | (HiVal << LoSizeInBits);
      LastImm = getOrCreateImmediate(Val, SizeInBits);
      NewOps.back() = LastImm;
    } else {
      LastImm = CurImm;
      NewOps.push_back(Ops[i]);
    }
  }

  if (NewOps.size() == 1) return NewOps.back();

  // FIXME: Flatten bitcat.
  return createExpr(VASTExpr::dpBitCat, NewOps, BitWidth, 0);
}

VASTValPtr VASTModule::buildExpr(VASTExpr::Opcode Opc, VASTValPtr Op,
                                 unsigned BitWidth) {
  VASTValPtr Ops[] = { Op };
  return createExpr(Opc, Ops, BitWidth, 0);
}

// Inline all operands in the expression whose Opcode is the same as Opc
// recursively;
static void flattenExpr(VASTValPtr V, VASTExpr::Opcode Opc,
                        SmallVectorImpl<VASTValPtr> &NewOps) {
  if (VASTExpr *Expr = dyn_cast<VASTExpr>(V)) {
    typedef const VASTUse *op_iterator;
    if (Expr->getOpcode() == Opc) {
      for (op_iterator I = Expr->op_begin(), E = Expr->op_end(); I != E; ++I)
        flattenExpr(I->getAsInlineOperand(), Opc, NewOps);

      return;
    }
  }

  NewOps.push_back(V);
}



VASTValPtr VASTModule::flattenExprTree(VASTExpr::Opcode Opc,
                                       ArrayRef<VASTValPtr> Ops,
                                       unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  typedef const VASTUse *op_iterator;
  bool isCommutative = true;
  unsigned OperandBitWidth = Ops[0]->getBitWidth();

  for (unsigned i = 0; i < Ops.size(); ++i) {
    // Try to flatten the expression tree.
    flattenExpr(Ops[i], Opc, NewOps);

    // The expression is actually commutative only if all its operands have the
    // same bitwidth.
    isCommutative &= (OperandBitWidth == Ops[i]->getBitWidth());
  }

  // The expression that can perform tree flatten should be commutative.
  if (isCommutative)
    return  getOrCreateCommutativeExpr(Opc, NewOps, BitWidth);

  return createExpr(Opc, NewOps, BitWidth, 0);
}

VASTValPtr VASTModule::buildAndExpr(ArrayRef<VASTValPtr> Ops,
                                    unsigned BitWidth) {
  SmallVector<VASTValPtr, 8> NewOps;
  typedef const VASTUse *op_iterator;

  for (unsigned i = 0; i < Ops.size(); ++i) {
    // The expression is actually commutative only if all its operands have the
    // same bitwidth.
    assert(BitWidth == Ops[i]->getBitWidth() && "Bitwidth not match!");

    if (VASTImmediate *Imm = dyn_cast<VASTImmediate>(Ops[i])) {
      // X & 1 = X;
      if (Imm->isAllOnes()) continue;

      // X & 0 = 0;
      if (Imm->isAllZeros()) return Imm;
    }

    // Try to flatten the expression tree.
    flattenExpr(Ops[i], VASTExpr::dpAnd, NewOps);
  }

  if (NewOps.empty())
    return getOrCreateImmediate(getBitSlice64(~0ull, BitWidth), BitWidth);

  // The expression that can perform tree flatten should be commutative.
  return getOrCreateCommutativeExpr(VASTExpr::dpAnd, NewOps, BitWidth);
}

VASTValPtr VASTModule::buildExpr(VASTExpr::Opcode Opc, VASTValPtr LHS,
                                 VASTValPtr RHS, unsigned BitWidth) {
  VASTValPtr Ops[] = { LHS, RHS };
  return buildExpr(Opc, Ops, BitWidth);
}

VASTValPtr VASTModule::buildExpr(VASTExpr::Opcode Opc, VASTValPtr Op0,
                                 VASTValPtr Op1, VASTValPtr Op2,
                                 unsigned BitWidth) {
  VASTValPtr Ops[] = { Op0, Op1, Op2 };
  return buildExpr(Opc, Ops, BitWidth);
}

VASTValPtr VASTModule::buildExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                                 unsigned BitWidth) {
  switch (Opc) {
  default: break;
  case VASTExpr::dpAdd:  return buildAddExpr(Ops, BitWidth);
  case VASTExpr::dpMul:  return buildMulExpr(Ops, BitWidth);
  case VASTExpr::dpAnd:  return buildAndExpr(Ops, BitWidth);
  case VASTExpr::dpBitCat: return buildBitCatExpr(Ops, BitWidth);
  }

  return createExpr(Opc, Ops, BitWidth, 0);
}

VASTValPtr VASTModule::buildMulExpr(ArrayRef<VASTValPtr> Ops,
                                    unsigned BitWidth) {
  return flattenExprTree(VASTExpr::dpMul, Ops, BitWidth);
}

VASTValPtr VASTModule::buildAddExpr(ArrayRef<VASTValPtr> Ops,
                                    unsigned BitWidth) {
  return flattenExprTree(VASTExpr::dpAdd, Ops, BitWidth);
}

VASTValPtr VASTModule::buildOrExpr(ArrayRef<VASTValPtr> Ops,
                                   unsigned BitWidth) {
  if (Ops.size() == 1) return Ops[0];

  assert (Ops.size() > 1 && "There should be more than one operand!!");

  SmallVector<VASTValPtr, 4> NotExprs;
  // Build the operands of Or operation into not Expr.
  for (unsigned i = 0; i < Ops.size(); ++i) {
    VASTValPtr V = buildNotExpr(Ops[i]);
    NotExprs.push_back(V);
  }

  // Build Or operation with the And Inverter Graph (AIG).
  return buildNotExpr(buildAndExpr(NotExprs, BitWidth));
}

VASTValPtr VASTModule::buildXorExpr(ArrayRef<VASTValPtr> Ops,
                                    unsigned BitWidth) {
  assert (Ops.size() == 2 && "There should be more than one operand!!");

  // Build the Xor Expr with the And Inverter Graph (AIG).
  return buildExpr(VASTExpr::dpAnd, buildOrExpr(Ops, BitWidth),
                   buildNotExpr(buildExpr(VASTExpr::dpAnd, Ops, BitWidth)),
                   BitWidth);
}

VASTValPtr
VASTModule::getOrCreateCommutativeExpr(VASTExpr::Opcode Opc,
                                       SmallVectorImpl<VASTValPtr> &Ops,
                                       unsigned BitWidth) {
  std::sort(Ops.begin(), Ops.end());
  return createExpr(Opc, Ops, BitWidth, 0);
}

VASTValPtr VASTModule::createExpr(VASTExpr::Opcode Opc,
                                  ArrayRef<VASTValPtr> Ops,
                                  unsigned UB, unsigned LB) {
  assert(!Ops.empty() && "Unexpected empty expression");
  if (Ops.size() == 1) {
    switch (Opc) {
    default: break;
    case VASTExpr::dpAnd: case VASTExpr::dpAdd: case VASTExpr::dpMul:
      return Ops[0];
    }
  }

  FoldingSetNodeID ID;

  // Profile the elements of VASTExpr.
  ID.AddInteger(Opc);
  ID.AddInteger(UB);
  ID.AddInteger(LB);
  for (unsigned i = 0; i < Ops.size(); ++i) {
    ID.AddPointer(Ops[i].get());
    ID.AddBoolean(Ops[i].isInverted());
  }
  void *IP = 0;
  if (VASTExpr *E = UniqueExprs.FindNodeOrInsertPos(ID, IP))
    return E;
  
  // If the Expression do not exist, allocate a new one.
  // Place the VASTUse array right after the VASTExpr.
  void *P = Allocator.Allocate(sizeof(VASTExpr) + Ops.size() * sizeof(VASTUse),
                               alignOf<VASTExpr>());
  VASTExpr *E = new (P) VASTExpr(Opc, Ops.size(), UB, LB,
                                 ID.Intern(Allocator));
  UniqueExprs.InsertNode(E, IP);

  // Initialize the use list.
  for (unsigned i = 0; i < Ops.size(); ++i)
    (void) new (E->ops() + i) VASTUse(Ops[i], E);

  return E;
}

VASTWire *VASTModule::buildAssignCnd(VASTSlot *Slot,
                                     SmallVectorImpl<VASTValPtr> &Cnds,
                                     bool AddSlotActive) {
  // We only assign the Src to Dst when the given slot is active.
  if (AddSlotActive) Cnds.push_back(Slot->getActive()->getAsInlineOperand(false));
  VASTValPtr AssignAtSlot = buildExpr(VASTExpr::dpAnd, Cnds, 1);
  VASTWire *Wire = Allocator.Allocate<VASTWire>();
  new (Wire) VASTWire(0, AssignAtSlot->getBitWidth(), "");
  assign(Wire, AssignAtSlot, VASTWire::AssignCond);
  Wire->setSlot(Slot->SlotNum);
  // Recover the condition vector.
  if (AddSlotActive) Cnds.pop_back();

  return Wire;
}

void VASTModule::addAssignment(VASTRegister *Dst, VASTValPtr Src, VASTSlot *Slot,
                               SmallVectorImpl<VASTValPtr> &Cnds,
                               bool AddSlotActive) {
  if (Src) {
    VASTWire *Cnd = buildAssignCnd(Slot, Cnds, AddSlotActive);
    Dst->addAssignment(new (Allocator.Allocate<VASTUse>()) VASTUse(Src, 0), Cnd);
  }
}

VASTValPtr VASTModule::assign(VASTWire *W, VASTValPtr V, VASTWire::Type T) {
  if (W->getExpr() != V) W->assign(V, T);

  return W;
}

VASTValPtr VASTModule::assignWithExtraDelay(VASTWire *W, VASTValPtr V,
                                            unsigned latency) {
  if (W->getExpr() != V)
    W->assignWithExtraDelay(V, latency);

  return W;
}

void VASTModule::print(raw_ostream &OS) const {
  // Print the verilog module?
}

VASTPort *VASTModule::addInputPort(const std::string &Name, unsigned BitWidth,
                                   PortTypes T /*= Others*/) {
  // DIRTYHACK: comment out the deceleration of the signal for ports.
  VASTWire *W = addWire(Name, BitWidth, "//");
  VASTRegister *VReg = Allocator.Allocate<VASTRegister>();
  new (VReg) VASTRegister(W->getName(), BitWidth, 0, VASTRegister::Virtual);

  // Do not remove the wire for input port.
  W->setAsInput(VReg);

  VASTPort *Port = new (Allocator.Allocate<VASTPort>()) VASTPort(W, true);
  if (T < SpecialInPortEnd) {
    assert(Ports[T] == 0 && "Special port exist!");
    Ports[T] = Port;
    return Port;
  }

  // Return port is a output port.
  assert(T < RetPort && "Wrong port type!");
  if (T == ArgPort) {
    assert(NumArgPorts == Ports.size() - NumSpecialPort
           && "Unexpected port added before arg port!");
    ++NumArgPorts;
  }

  Ports.push_back(Port);
  return Port;
}

VASTPort *VASTModule::addOutputPort(const std::string &Name, unsigned BitWidth,
                                    PortTypes T /*= Others*/,
                                    bool isReg /*= true*/) {
  VASTSignal *V = 0;
  // DIRTYHACK: comment out the deceleration of the signal for ports.
  if (isReg) V = addRegister(Name, BitWidth, 0, VASTRegister::OutputPort);
  else       V = addWire(Name, BitWidth, "//");

  // Do not remove the signal for output port.
  V->Pin();

  VASTPort *Port = new (Allocator.Allocate<VASTPort>()) VASTPort(V, false);
  if (SpecialInPortEnd <= T && T < SpecialOutPortEnd) {
    assert(Ports[T] == 0 && "Special port exist!");
    Ports[T] = Port;
    return Port;
  }

  assert(T <= RetPort && "Wrong port type!");
  if (T == RetPort) {
    RetPortIdx = Ports.size();
    assert(RetPortIdx == NumArgPorts + NumSpecialPort
      && "Unexpected port added before return port!");
  }

  Ports.push_back(Port);
  return Port;
}

VASTRegister *VASTModule::addRegister(const std::string &Name, unsigned BitWidth,
                                      unsigned InitVal, VASTRegister::Type T,
                                      uint16_t RegData, const char *Attr) {
  SymEntTy &Entry = SymbolTable.GetOrCreateValue(Name);
  assert(Entry.second == 0 && "Symbol already exist!");
  VASTRegister *Reg = Allocator.Allocate<VASTRegister>();
  new (Reg) VASTRegister(Entry.getKeyData(), BitWidth, InitVal, T, RegData,Attr);
  Entry.second = Reg;
  Registers.push_back(Reg);

  return Reg;
}

VASTWire *VASTModule::addWire(const std::string &Name, unsigned BitWidth,
                              const char *Attr) {
  SymEntTy &Entry = SymbolTable.GetOrCreateValue(Name);
  assert(Entry.second == 0 && "Symbol already exist!");
  VASTWire *Wire = Allocator.Allocate<VASTWire>();
  new (Wire) VASTWire(Entry.getKeyData(), BitWidth, Attr);
  Entry.second = Wire;
  Wires.push_back(Wire);

  return Wire;
}

VASTValPtr VASTModule::getOrCreateSymbol(const std::string &Name,
                                         unsigned BitWidth,
                                         bool CreateWrapper) {
  SymEntTy &Entry = SymbolTable.GetOrCreateValue(Name);
  VASTNamedValue *&V = Entry.second;
  if (V == 0) {
    const char *S = Entry.getKeyData();
    // If we are going to create a wrapper, apply the bitwidth to the wrapper.
    unsigned SymbolWidth = CreateWrapper ? 0 : BitWidth;
    V = new (Allocator.Allocate<VASTSymbol>()) VASTSymbol(S, SymbolWidth);
    if (CreateWrapper) {
      // Create the wire for the symbol, and assign the symbol to the wire.
      VASTWire *Wire = addWire(VBEMangle(Name + "_s"), BitWidth);
      Wire->assign(V);
      // Remember the wire.
      V = Wire;
    }
  }

  assert(V->getBitWidth() == BitWidth
          && "Getting symbol with wrong bitwidth!");

  return V;
}

// Out of line virtual function to provide home for the class.
void VASTModule::anchor() {}

void VASTModule::writeProfileCounters(VASTSlot *S, bool isFirstSlot) {
  MachineBasicBlock *BB = S->getParentBB();
  std::string BBCounter = "cnt"+ utostr_32(BB ? BB->getNumber() : 0);
  std::string FunctionCounter = "cnt" + getName();
  vlang_raw_ostream &CtrlS = getControlBlockBuffer();
  
  // Create the profile counter.
  // Write the counter for the function.
  if (S->SlotNum == 0) {
    addRegister(FunctionCounter, 64)->Pin();
    CtrlS.if_begin(getPortName(VASTModule::Finish));
    CtrlS << "$display(\"Module: " << getName();

    CtrlS << " total cycles" << "->%d\"," << FunctionCounter << ");\n";
    CtrlS.exit_block() << "\n";
  } else { // Dont count the ilde state at the moment.
    if (isFirstSlot) {
      addRegister(BBCounter, 64)->Pin();

      CtrlS.if_begin(getPortName(VASTModule::Finish));
      CtrlS << "$display(\"Module: " << getName();
      // Write the parent MBB name.
      if (BB)
        CtrlS << " MBB#" << BB->getNumber() << ": " << BB->getName();

      CtrlS << ' ' << "->%d\"," << BBCounter << ");\n";
      CtrlS.exit_block() << "\n";
    }

    // Increase the profile counter.
    if (S->isLeaderSlot()) {
      CtrlS.if_() << S->getRegister()->getName();
      for (unsigned i = S->alias_start(), e = S->alias_end(),
        k = S->alias_ii(); i < e; i += k) {
          CtrlS << '|' << getSlot(i)->getRegister()->getName();
      }

      CtrlS._then();
      CtrlS << BBCounter << " <= " << BBCounter << " +1;\n";
      CtrlS << FunctionCounter << " <= " << FunctionCounter << " +1;\n";
      CtrlS.exit_block() << "\n";
    }
  }
}

std::string VASTValue::printBitRange(unsigned UB, unsigned LB, bool printOneBit){
  std::string ret;
  assert(UB && UB >= LB && "Bad bit range!");
  --UB;
  if (UB != LB)
    ret = "[" + utostr_32(UB) + ":" + utostr_32(LB) + "]";
  else if(printOneBit)
    ret = "[" + utostr_32(LB) + "]";

  return ret;
}

void VASTValue::print(raw_ostream &OS) const {
  printAsOperandImpl(OS);
}

void VASTValue::printAsOperand(raw_ostream &OS, unsigned UB, unsigned LB,
                               bool isInverted) const{
  if (isInverted) OS << "(~";
  OS << '(';
  printAsOperandImpl(OS, UB, LB);
  OS << ')';
  if (isInverted) OS << ')';
}

void VASTValue::printAsOperand(raw_ostream &OS, bool isInverted) const {
  if (isInverted) OS << "(~";
  OS << '(';
  printAsOperandImpl(OS);
  OS << ')';
  if (isInverted) OS << ')';
}

void VASTValue::printAsOperandImpl(raw_ostream &OS, unsigned UB,
                                   unsigned LB) const {
  assert(0 && "VASTValue::printAsOperand should not be called!");
}

void VASTNamedValue::printAsOperandImpl(raw_ostream &OS, unsigned UB,
                                        unsigned LB) const{
  OS << getName();
  if (UB) OS << VASTValue::printBitRange(UB, LB, getBitWidth() > 1);
}

bool VASTValue::replaceAllUseWith(VASTValue *To) {
  assert(To->getBitWidth() == getBitWidth() && "Bitwidth not match!");

  typedef VASTValue::use_iterator it;
  for (it I = use_begin(), E = use_end(); I != E; /*++I*/) {
    VASTUse *U = I.get();
    ++I;
    assert(U->getBitWidth() == To->getBitWidth() && "Bitwidth not match!");
    VASTValPtr User = U->getUser();
    // Unlink from old list.
    removeUseFromList(U);
    // Move to new list.
    U->set(To);
    U->setUser(User.getAsLValue<VASTValue>());
  }

  return use_empty();
}

VASTValue::dp_dep_it VASTValue::dp_dep_begin(VASTValue *V) {
  switch (V->getASTType()) {
  case VASTNode::vastExpr: return cast<VASTExpr>(V)->op_begin();
  case VASTNode::vastWire: return cast<VASTWire>(V)->op_begin();
  default:  return VASTValue::dp_dep_it(0);
  }
  
}

VASTValue::dp_dep_it VASTValue::dp_dep_end(VASTValue *V) {
  switch (V->getASTType()) {
  case VASTNode::vastExpr: return cast<VASTExpr>(V)->op_end();
  case VASTNode::vastWire: return cast<VASTWire>(V)->op_end();
  default:  return VASTValue::dp_dep_it(0);
  }
}

void VASTImmediate::printAsOperandImpl(raw_ostream &OS, unsigned UB,
                                       unsigned LB) const {
  assert(UB == getBitWidth() && LB == 0 && "Cannot print bitslice of Expr!");
  OS << verilogConstToStr(getValue(), getBitWidth(), false);
}

void VASTPort::print(raw_ostream &OS) const {
  if (IsInput)
    OS << "input ";
  else
    OS << "output ";

  if (isRegister())
    OS << "reg";
  else
    OS << "wire";

  if (getBitWidth() > 1) OS << "[" << (getBitWidth() - 1) << ":0]";

  OS << ' ' << getName();
}

void VASTPort::printExternalDriver(raw_ostream &OS, uint64_t InitVal) const {
  if (IsInput)
    // We need a reg to drive input port.
    OS << "reg";
  else
    // We need a wire to accept the output value from dut.
    OS << "wire";

  if (getBitWidth() > 1)
    OS << "[" << (getBitWidth() - 1) << ":0]";

  OS << ' ' << getName();

  if (IsInput)
    OS << " = " << verilogConstToStr(InitVal, getBitWidth(), false);

  OS << ';';
}

std::string VASTPort::getExternalDriverStr(unsigned InitVal) const {
  std::string ret;
  raw_string_ostream ss(ret);
  printExternalDriver(ss, InitVal);
  ss.flush();
  return ret;
}

// Out of line virtual function to provide home for the class.
void VASTPort::anchor() {}

void VASTSignal::printDecl(raw_ostream &OS) const {
  OS << AttrStr << ' ';
  if (isa<VASTRegister>(this))
    OS << "reg";
  else
    OS << "wire";

  if (getBitWidth() > 1)
    OS << "[" << (getBitWidth() - 1) << ":0]";

  OS << ' ' << getName();

  if (isa<VASTRegister>(this))
    OS << " = " << verilogConstToStr(0, getBitWidth(), false);

  OS << ";";
}

//----------------------------------------------------------------------------//
// Operand printing helper functions.
static void printSignedOperand(raw_ostream &OS, const VASTUse &U) {
  OS << "$signed(";
  U.printAsOperand(OS);
  OS << ")";
}

static void printUnsignedOperand(raw_ostream &OS, const VASTUse &U) {
  U.printAsOperand(OS);
}

template<typename PrintOperandFN>
static void printSimpleOp(raw_ostream &OS, ArrayRef<VASTUse> Ops,
                          const char *Opc, PrintOperandFN &FN) {
  unsigned NumOps = Ops.size();
  assert(NumOps && "Unexpected zero operand!");
  FN(OS, Ops[0]);

  for (unsigned i = 1; i < NumOps; ++i) {
    OS << Opc;
    FN(OS, Ops[i]);
  }
}

static void printSimpleUnsignedOp(raw_ostream &OS, ArrayRef<VASTUse> Ops,
                                  const char *Opc) {
  printSimpleOp(OS, Ops, Opc, printUnsignedOperand);
}

//----------------------------------------------------------------------------//
// Generic datapath printing helper function.
static void printUnaryOp(raw_ostream &OS, const VASTUse &U, const char *Opc) {
  OS << Opc;
  U.printAsOperand(OS);
}

static void printSRAOp(raw_ostream &OS, ArrayRef<VASTUse> Ops) {
  printSignedOperand(OS, Ops[0]);
  OS << " >>> ";
  Ops[1].printAsOperand(OS);
}

template<typename PrintOperandFN>
static void printCmpFU(raw_ostream &OS, ArrayRef<VASTUse> Ops,
                       PrintOperandFN &FN) {
  OS << "{ 3'bx, ((";
  // Port 4: gt.
  printSimpleOp<PrintOperandFN>(OS, Ops, " > ", FN);
  OS << ") ? 1'b1 : 1'b0), ((";
  // Port 3: gt.
  printSimpleOp<PrintOperandFN>(OS, Ops, " >= ", FN);
  OS << ") ? 1'b1 : 1'b0), ((";
  // Port 2: gt.
  printSimpleOp<PrintOperandFN>(OS, Ops, " == ", FN);
  OS << ") ? 1'b1 : 1'b0), ((";
  // Port 2: gt.
  printSimpleOp<PrintOperandFN>(OS, Ops, " != ", FN);
  OS << ") ? 1'b1 : 1'b0), 1'bx }";
}

static void printSel(raw_ostream &OS, ArrayRef<VASTUse> Ops) {
 printUnsignedOperand(OS, Ops[0]);
 OS << '?';
 printUnsignedOperand(OS, Ops[1]);
 OS << ':';
 printUnsignedOperand(OS, Ops[2]);
}

static void printBitCat(raw_ostream &OS, ArrayRef<VASTUse> Ops) {
  OS << '{';
  printSimpleUnsignedOp(OS, Ops, " , ");
  OS << '}';
}

static void printBitRepeat(raw_ostream &OS, ArrayRef<VASTUse> Ops) {
  OS << '{' << cast<VASTImmediate>((Ops[1]).get())->getValue() << '{';
  Ops[0].printAsOperand(OS);
  OS << "}}";
}

static void printCombMux(raw_ostream &OS, const VASTWire *W) {
  assert(!W->getExpr().isInverted() && "Unexpected inverted mux!");
  VASTExpr *E = W->getExpr().get();
  unsigned NumOperands = E->NumOps;
  assert((NumOperands & 0x1) == 0 && "Expect even operand number for CombMUX!");

  // Handle the trivial case trivially: Only 1 input.
  if (NumOperands == 2) {
    printAssign(OS, W);
    E->getOperand(1).printAsOperand(OS);
    OS << ";\n";
    return;
  }

  // Create the temporary signal.
  OS << "// Combinational MUX\n"
     << "reg " << VASTValue::printBitRange(W->getBitWidth(), 0, true)
     << ' ' << W->getName() << "_mux_wire;\n";

  // Assign the temporary signal to the wire.
  printAssign(OS, W) << W->getName() << "_mux_wire;\n";

  // Print the mux logic.
  OS << "always @(*)begin  // begin mux logic\n";
  OS.indent(2) << VASTModule::ParallelCaseAttr << " case (1'b1)\n";
  for (unsigned i = 0; i < NumOperands; i+=2) {
    OS.indent(4);
    E->getOperand(i).printAsOperand(OS);
    OS << ": " << W->getName() << "_mux_wire = ";
    E->getOperand(i + 1).printAsOperand(OS);
    OS << ";\n";
  }
  // Write the default condition, otherwise latch will be inferred.
  OS.indent(4) << "default: " << W->getName() << "_mux_wire = "
               << W->getBitWidth() << "'bx;\n";
  OS.indent(2) << "endcase\nend  // end mux logic\n";
}

void VASTWire::printAsOperandImpl(raw_ostream &OS, unsigned UB,
                                  unsigned LB) const {
  if (getName())
    VASTNamedValue::printAsOperandImpl(OS, UB, LB);
  else {
    VASTValPtr V = getAssigningValue();
    assert(V && "Cannot print wire as operand!");
    V.printAsOperand(OS, UB, LB);
  }
}

void VASTWire::setAsInput(VASTRegister *VReg) {
  assign(VReg, VASTWire::InputPort);
  // Pin the signal to prevent it from being optimized away.
  Pin();
  setTimingUndef();
}

void VASTWire::printAssignment(raw_ostream &OS) const {
  VASTWire::Type T = getWireType();
  // Input ports do not have datapath.
  if (T == VASTWire::InputPort) return;

  VASTValPtr V = getAssigningValue();
  assert(V && "Cannot print the wire!");

  if (VASTExpr *Expr = dyn_cast<VASTExpr>(V)) {
    // Dont know how to print black box.
    if (Expr->getOpcode() == VASTExpr::dpBlackBox) return;

    // MUX need special printing method.
    if (Expr->getOpcode() == VASTExpr::dpMux) {
      printCombMux(OS, this);
      return;
    }
  }

  printAssign(OS, this);
  V.printAsOperand(OS);
  OS << ";\n";
}

void VASTExpr::printAsOperandImpl(raw_ostream &OS, unsigned UB,
                                  unsigned LB) const {
  assert(UB == this->UB && LB == this->LB && "Cannot print bitslice of Expr!");

  printAsOperandInteral(OS);
}

void VASTExpr::printAsOperandInteral(raw_ostream &OS) const {
  OS << '(';
  typedef ArrayRef<VASTUse> UseArray;

  switch (getOpcode()) {
  case dpAnd: printSimpleUnsignedOp(OS, getOperands(), " & "); break;

  case dpRAnd:  printUnaryOp(OS, getOperand(0), "&");  break;
  case dpROr:   printUnaryOp(OS, getOperand(0), "|");  break;
  case dpRXor:  printUnaryOp(OS, getOperand(0), "^");  break;

  case dpSCmp:  printCmpFU(OS, getOperands(), printSignedOperand); break;
  case dpUCmp:  printCmpFU(OS, getOperands(), printUnsignedOperand); break;

  case dpAdd: printSimpleUnsignedOp(OS, getOperands(), " + "); break;
  case dpMul: printSimpleUnsignedOp(OS, getOperands(), " * "); break;
  case dpShl: printSimpleUnsignedOp(OS, getOperands(), " << ");break;
  case dpSRL: printSimpleUnsignedOp(OS, getOperands(), " >> ");break;
  case dpSRA: printSRAOp(OS, getOperands());                   break;

  case dpSel: printSel(OS, getOperands());                     break;

  case dpAssign: getOperand(0).printAsOperand(OS, UB, LB); break;

  case dpBitCat:    printBitCat(OS, getOperands());    break;
  case dpBitRepeat: printBitRepeat(OS, getOperands()); break;

  default: llvm_unreachable("Unknown datapath opcode!"); break;
  }

  OS << ')';
}
