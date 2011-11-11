//===------------- VLang.h - Verilog HDL writing engine ---------*- C++ -*-===//
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
// This file implement the VLang class, with provide funtions to complete
// common Verilog HDL writing task.
//
//===----------------------------------------------------------------------===//

#include "vtm/VerilogAST.h"
#include "vtm/MicroState.h"
#include "vtm/Utilities.h"

#include "llvm/Constants.h"
#include "llvm/GlobalVariable.h"
#include "llvm/DerivedTypes.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/SourceMgr.h"
#define DEBUG_TYPE "verilog-ast"
#include "llvm/Support/Debug.h"

#include <sstream>

using namespace llvm;

//===----------------------------------------------------------------------===//
// Value and type printing

std::string llvm::verilogBitRange(unsigned UB, unsigned LB, bool printOneBit) {
  std::stringstream bw;
  assert(UB && UB >= LB && "Bad bit range!");
  --UB;
  if (UB != LB)
    bw << "[" << UB << ":" << LB << "]";
  else if(printOneBit)
    bw << "[" << LB << "]";

  return bw.str();
}

std::string llvm::verilogConstToStr(Constant *CPV) {
  if (ConstantInt* CI=dyn_cast<ConstantInt>(CPV)) {
    bool isMinValue=CI->isMinValue(true);
    uint64_t v = isMinValue ? CI->getZExtValue() :
                             (uint64_t)CI->getSExtValue();
    return verilogConstToStr(v,CI->getBitWidth(),isMinValue);
  } else if (ConstantExpr *CE = dyn_cast<ConstantExpr>(CPV)) {
    switch (CE->getOpcode()) {
    case Instruction::PtrToInt:
    case Instruction::IntToPtr:
    case Instruction::BitCast:
      return verilogConstToStr(CE->getOperand(0));
    }

  }

  return "??Constant??";
}

std::string llvm::verilogConstToStr(uint64_t value, unsigned bitwidth,
                                    bool isMinValue) {
  std::stringstream pc;
  pc <<bitwidth<< '\'';
  if (bitwidth == 1) pc << 'b';
  else               pc << "h";
  // Mask the value that small than 4 bit to prevent printing something
  // like 1'hf out.
  if (bitwidth < 4)
    value &= (1 << bitwidth) - 1;

  if(isMinValue) {
    pc<<std::hex<<value;
    return pc.str();
  }

  std::stringstream ss;
  ss<<std::hex<<value;
  unsigned int uselength = (bitwidth/4) + (((bitwidth&0x3) == 0) ? 0 : 1);
  std::string sout = ss.str();
  if(uselength < sout.length())
    sout=sout.substr(sout.length()-uselength,uselength);
  pc<<sout;

  return pc.str();
}

//----------------------------------------------------------------------------//
// Helper function for Verilog RTL printing.

static raw_ostream &printAssign(raw_ostream &OS, const VASTWire *W) {
  OS << "assign " << W->getName() << verilogBitRange(W->getBitWidth(), 0, false)
     << " = ";
  return OS;
}

//----------------------------------------------------------------------------//
// Classes in Verilog AST.
void VASTUse::print(raw_ostream &OS) const {
  // Print the bit range if the value is have multiple bits.
  switch (UseKind) {
  case USE_Value:
    OS << Data.V->getName();
    if (UB) OS << verilogBitRange(UB, LB, Data.V->getBitWidth() > 1);
    return;
  case USE_Symbol:
    OS << Data.SymbolName;
    if (UB) OS << verilogBitRange(UB, LB, false);
    return;
  case USE_Immediate:
    OS << verilogConstToStr(Data.ImmVal, UB, false);
    // No need to print bit range for immediate.
    return;
  }
}

VASTUse::iterator VASTUse::dp_src_begin() {
  if (UseKind != USE_Value)  return reinterpret_cast<VASTUse::iterator>(0);

  if (VASTWire *W = dyn_cast<VASTWire>(get()))
    return W->op_begin();

  return reinterpret_cast<VASTUse::iterator>(0);
}

VASTUse::iterator VASTUse::dp_src_end() {
  if (UseKind != USE_Value)  return reinterpret_cast<VASTUse::iterator>(0);

  if (VASTWire *W = dyn_cast<VASTWire>(get()))
    return W->op_end();

  return reinterpret_cast<VASTUse::iterator>(0);
}

void VASTCnd::print(raw_ostream &OS) const {
  OS << '(';
  if (isInverted()) OS << '~';
  VASTUse::print(OS);
  OS << ')';
}

VASTSlot::VASTSlot(unsigned slotNum, unsigned parentIdx, VASTSignal *S[])
  :VASTNode(vastSlot, slotNum), StartSlot(slotNum), EndSlot(slotNum), II(~0),
   ParentIdx(parentIdx) {
  std::uninitialized_copy(S, S + 3, Signals);

  // SlotAcitve = SlotReady & SlotReg
  VASTWire *SlotActive = getActive();
  SlotActive->setOpcode(VASTWire::dpAnd);
  SlotActive->addOperand(getRegister());
  SlotActive->addOperand(getReady());
  // We need alias slot to build the ready signal, keep it as unknown now.

  assert(slotNum >= parentIdx && "Slotnum earlier than parent start slot!");
}

void VASTSlot::addNextSlot(unsigned NextSlotNum, VASTCnd Cnd) {
  bool Inserted = NextSlots.insert(std::make_pair(NextSlotNum, Cnd)).second;
  assert(Inserted && "NextSlot already existed!");
  (void) Inserted;
}

void VASTSlot::addEnable(VASTValue *V, VASTCnd Cnd) {
  bool Inserted = Enables.insert(std::make_pair(V, Cnd)).second;
  assert(Inserted && "NextSlot already existed!");
  (void) Inserted;
}

void VASTSlot::addReady(VASTValue *V, VASTCnd Cnd /* = VASTCnd */) {
  bool Inserted = Readys.insert(std::make_pair(V, Cnd)).second;
  assert(Inserted && "NextSlot already existed!");
  (void) Inserted;
}

void VASTSlot::addDisable(VASTValue *V, VASTCnd Cnd) {
  bool Inserted = Disables.insert(std::make_pair(V, Cnd)).second;
  assert(Inserted && "NextSlot already existed!");
  (void) Inserted;
}

void VASTSlot::buildFUReadyExpr(raw_ostream &OS, VASTSlot *SrcSlot) {
  VASTWire *Ready = SrcSlot->getReady();

  OS << "1'b1";
  for (VASTSlot::const_fu_ctrl_it I = ready_begin(), E = ready_end();
        I != E; ++I) {
    // Print the code for ready signal.
    // If the condition is true then the signal must be 1 to ready.
    OS << " & (" << I->first->getName() << " | ~";
    I->second.print(OS);
    OS << ')';

    // Build the dependence for ready signal.
    Ready->addOperand(I->first);
    Ready->addOperand(I->second);
  }
}

void VASTSlot::buildReadyLogic(raw_ostream &OS, const VASTModule &Mod) {
  printAssign(OS, getReady());
  buildFUReadyExpr(OS, this);

  if (StartSlot != EndSlot) {
    for (unsigned slot = StartSlot; slot < EndSlot; slot += II) {
      if (slot == getSlotNum()) continue;

      VASTSlot *AliasSlot = Mod.getSlot(slot);

      if (!AliasSlot->readyEmpty()) {
        OS << " & ( ~Slot" << slot << " | (";
        AliasSlot->buildFUReadyExpr(OS, this);
        OS << "))";
      }
    }
  }

  OS << ";// Are all waiting resources ready?\n";
}

void VASTSlot::printCtrl(vlang_raw_ostream &CtrlS, const VASTModule &Mod) const{
  // TODO: Build the AST for these logic.
  CtrlS.if_begin(getName());
  std::string SlotReady = std::string(getName()) + "Ready";
  bool ReadyPresented = !readyEmpty();

  // DirtyHack: Remember the enabled signals in alias slots, the signal may be
  // assigned at a alias slot.
  std::set<const VASTValue*> AliasEnables;

  if (StartSlot != EndSlot) {
    CtrlS << "// Alias slots: ";

    for (unsigned slot = StartSlot; slot < EndSlot; slot += II) {
      CtrlS << slot << ", ";
      if (slot == getSlotNum()) continue;

      const VASTSlot *AliasSlot = Mod.getSlot(slot);

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

  // Enable next slot only when resources are ready.
  if (ReadyPresented) CtrlS.if_begin(SlotReady);

  DEBUG(
  if (getSlotNum() != 0)
    CtrlS << "$display(\"" << getName() << " in " << Mod.getName()
          << " ready\");\n";
  );

  bool hasSelfLoop = false;
  if (hasExplicitNextSlots()) {
    CtrlS << "// Enable the successor slots.\n";
    for (VASTSlot::const_succ_iterator I = succ_begin(),E = succ_end();
         I != E; ++I) {
      hasSelfLoop |= I->first == getSlotNum();
      CtrlS << Mod.getSlot(I->first)->getName() << " <= ";
      I->second.print(CtrlS);
      CtrlS << ";\n";
    }
  } else {
    CtrlS << "// Enable the default successor slots.\n";
    CtrlS << Mod.getSlot(getSlotNum() + 1)->getName() << " <= 1'b1;\n";
  }

  // Do not assign a value to the current slot enable twice.
  if (!hasSelfLoop) {
    CtrlS << "// Disable the current slot.\n";
    CtrlS << getName() << " <= 1'b0;\n";
  }

  if (ReadyPresented) {
    DEBUG(CtrlS.else_begin();
          if (getSlotNum() != 0)
            CtrlS << "$display(\"" << getName() << " in " << Mod.getName()
                  <<  " waiting \");\n";
          );
    CtrlS.exit_block("// End resource ready.\n");
  } else {
    // DirtyHack: Check if the memory bus is shutdown.
    DEBUG(
      CtrlS << "if (mem0en_r) begin $display(\"" << getName() << " in "
            << Mod.getName()
            << " bad mem0en_r %b\\n\", mem0en_r);  $finish(); end\n";
    );
  }

  CtrlS << "// Enable the active FUs.\n";
  for (VASTSlot::const_fu_ctrl_it I = enable_begin(), E = enable_end();
       I != E; ++I) {
    // We may try to enable and disable the same port at the same slot.
    CtrlS << I->first->getName() << " <= " << SlotReady << " & ";
    I->second.print(CtrlS);
    CtrlS << ";\n";
  }

  if (!disableEmpty()) {
    CtrlS << "// Disable the resources when the condition is true.\n";
    for (VASTSlot::const_fu_ctrl_it I = disable_begin(), E = disable_end();
         I != E; ++I) {
      // Look at the current enable set and alias enables set;
      // The port assigned at the current slot, and it will be disabled if
      // The slot is not ready or the enable condition is false. And it is
      // ok that the port is enabled.
      if (isEnabled(I->first)) continue;

      // If the port enabled in alias slots, disable it only if others slots is
      // not active.
      bool AliasEnabled = AliasEnables.count(I->first);
      if (AliasEnabled) {
        std::string AliasDisactive = "1'b1";
        raw_string_ostream SS(AliasDisactive);
        for (unsigned slot = StartSlot; slot < EndSlot; slot += II) {
          if (slot == getSlotNum()) continue;

          VASTSlot *ASlot = Mod.getSlot(slot);
          assert(!ASlot->isDiabled(I->first)
                 && "Same signal disabled in alias slot!");
          if (ASlot->isEnabled(I->first)) {
            SS << " & ~" << ASlot->getName();
            continue;
          }
        }

        SS.flush();
        CtrlS.if_begin(AliasDisactive, "// Resolve the conflict\n");
      }

      CtrlS << "if (";
      I->second.print(CtrlS);
      CtrlS << ") "  << I->first->getName() << " <= 1'b0;\n";

      if (AliasEnabled) CtrlS.exit_block();
    }
  }
  CtrlS.exit_block("\n\n");
}

void VASTSlot::print(raw_ostream &OS) const {
  llvm_unreachable("VASTSlot::print should not be called!");
}

VASTRegister::VASTRegister(const char *Name, unsigned BitWidth,
                           unsigned initVal, const char *Attr)
  : VASTSignal(vastRegister, Name, BitWidth, Attr), InitVal(initVal) {}

void VASTRegister::addAssignment(VASTUse Src, AndCndVec Cnd, VASTSlot *S) {
  assert(Src != this && "Self assignemnt not supported yet!");
  Assigns[Src].push_back(std::make_pair(S, Cnd));
  Slots.insert(S);
}

static void bindPath2ScriptEngine(ArrayRef<VASTUse> Path, unsigned Slack) {
  assert(Path.size() >= 2 && "Path vector have less than 2 nodes!");
  // Path table:
  // Datapath: {
  //  unsigned Slack,
  //  table NodesInPath
  // }
  SMDiagnostic Err;

  if (!runScriptStr("RTLDatapath = {}\n", Err))
    llvm_unreachable("Cannot create RTLDatapath table in scripting pass!");

  std::string Script;
  raw_string_ostream SS(Script);
  SS << "RTLDatapath.Slack = " << Slack;
  SS.flush();
  if (!runScriptStr(Script, Err))
    llvm_unreachable("Cannot create slack of RTLDatapath!");

  Script.clear();

  SS << "RTLDatapath.Nodes = {'" << Path[0].get()->getName();
  for (unsigned i = 1; i < Path.size(); ++i) {
    SS << "', '" << Path[i].get()->getName();
  }
  SS << "'}";

  SS.flush();
  if (!runScriptStr(Script, Err))
    llvm_unreachable("Cannot create node table of RTLDatapath!");

  // Get the script from script engine.
  const char *DatapathScriptPath[] = { "Misc", "DatapathScript" };
  if (!runScriptStr(getStrValueFromEngine(DatapathScriptPath), Err))
    report_fatal_error("Error occur while running datapath script:\n"
                       + Err.getMessage());
}

// Traverse the use tree in datapath, stop when we meet a register or other
// leaf node.
void VASTRegister::DepthFristTraverseDataPathUseTree(VASTUse Root,
                                                     const OrCndVec &Cnds) {
  typedef VASTUse::iterator ChildIt;
  // Use seperate node and iterator stack, so we can get the path vector.
  typedef SmallVector<VASTUse, 16> NodeStackTy;
  typedef SmallVector<ChildIt, 16> ItStackTy;
  NodeStackTy NodeWorkStack;
  ItStackTy ItWorkStack;
  // Remember what we had visited.
  std::set<VASTUse> VisitedUses;

  // Put the current node into the node stack, so it will appears in the path.
  NodeWorkStack.push_back(this);

  // Put the root.
  NodeWorkStack.push_back(Root);
  ItWorkStack.push_back(Root.dp_src_begin());

  while (!ItWorkStack.empty()) {
    VASTUse Node = NodeWorkStack.back();
    ChildIt It = ItWorkStack.back();

    // Do we reach the leaf?
    if (Node.is_dp_leaf()) {
      if (VASTValue *V = Node.getOrNull()) {
        DEBUG_WITH_TYPE("rtl-slack-info",
        dbgs() << "Datapath:\t";
        for (NodeStackTy::iterator I = NodeWorkStack.begin(),
             E = NodeWorkStack.end(); I != E; ++I) {
          dbgs() << ", ";
          I->print(dbgs());
        });

        if (VASTRegister *R = dyn_cast<VASTRegister>(V)) {
          unsigned Slack = findSlackFrom(R, Cnds);
          DEBUG_WITH_TYPE("rtl-slack-info",
                           dbgs() << " Slack: " << int(Slack));
          bindPath2ScriptEngine(NodeWorkStack, Slack);
        }

        DEBUG_WITH_TYPE("rtl-slack-info", dbgs() << '\n');
      }

      NodeWorkStack.pop_back();
      ItWorkStack.pop_back();
      continue;
    }

    // All sources of this node is visited.
    if (It == Node.dp_src_end()) {
      NodeWorkStack.pop_back();
      ItWorkStack.pop_back();
      continue;
    }

    // Depth first traverse the child of current node.
    VASTUse ChildNode = *It;
    ++ItWorkStack.back();

    // Had we visited this node?
    if (!VisitedUses.insert(ChildNode).second) continue;

    // If ChildNode is not visit, go on visit it and its childrens.
    NodeWorkStack.push_back(ChildNode);
    ItWorkStack.push_back(ChildNode.dp_src_begin());
  }

  assert(NodeWorkStack.back().get() == this && "Node stack broken!");
}

VASTSlot *VASTRegister::findNearestAssignSlot(VASTSlot *Dst) const {
  VASTSlot *NearestSrc = 0;
  typedef std::set<VASTSlot*, less_ptr<VASTSlot> >::const_iterator SlotIt;
  // FIXME: We can perform a binary search.
  for (SlotIt I = Slots.begin(), E = Slots.end(); I != E; ++I) {
    VASTSlot *Src = *I;

    // Do not mess up with cross state live interval at the moment.
    if (Src->getParentIdx() != Dst->getParentIdx())
      continue;

    if (*Src < *Dst) {
      NearestSrc = Src;
    }
  }

  return NearestSrc;
}

unsigned VASTRegister::findSlackFrom(const VASTRegister *Src,
                                     const OrCndVec &AssignCnds) {
  unsigned Slack = ~0;

  typedef OrCndVec::const_iterator cnd_it;
  for (cnd_it I = AssignCnds.begin(), E = AssignCnds.end(); I != E; ++I) {
    VASTSlot *DstSlot = I->first;
    // Because we ingore cross state live interval, assume all registers assigned
    // when the state start.
    Slack = std::min(Slack, DstSlot->getSlackFromParentStart());

    if (!Src) continue;

    if (VASTSlot *SrcSlot = Src->findNearestAssignSlot(DstSlot))
      // What we got is ASSIGN slot, the data need 1 more cycle to reach the
      // output pin of the register.
      Slack = std::min(Slack, DstSlot->getSlotNum() - (SrcSlot->getSlotNum() + 1));
  }

  return Slack;
}

void VASTRegister::reportAssignmentSlack() {
  // Do we have any assignment information?
  if (Assigns.empty()) return;

  for (AssignMapTy::const_iterator I = Assigns.begin(), E = Assigns.end();
       I != E; ++I) {
    VASTUse Src = I->first;

    VASTValue *SrcValue = Src.getOrNull();

    // Source is immediate or symbol, skip it.
    if (!SrcValue) continue;

    // Trivial case.
    if (VASTRegister *R = dyn_cast<VASTRegister>(SrcValue)) {
      unsigned Slack = findSlackFrom(R, I->second);
      DEBUG_WITH_TYPE("rtl-slack-info",
                       dbgs() << "Datapath:\t" << getName() << ", "
                              << R->getName() << ": "
                              << int(Slack) << '\n');
      VASTUse Path[] = { this, Src };
      bindPath2ScriptEngine(Path, Slack);
      continue;
    }

    DepthFristTraverseDataPathUseTree(Src, I->second);
  }//);
}

void VASTRegister::printCondition(raw_ostream &OS, const VASTSlot *Slot,
                                  const AndCndVec Cnds) {
  OS << '(';
  if (Slot) OS << Slot->getActive()->getName();
  else      OS << "1'b1";

  typedef AndCndVec::const_iterator and_it;
  for (and_it CI = Cnds.begin(), CE = Cnds.end(); CI != CE; ++CI) {
    OS << " & ";
    CI->print(OS);
  }

  OS << ')';
}

void VASTRegister::printReset(raw_ostream &OS) const {
  OS << getName()  << " <= "
     << verilogConstToStr(InitVal, getBitWidth(), false) << ";";
}

void VASTRegister::printAssignment(vlang_raw_ostream &OS) const {
  if (Assigns.empty()) return;

  std::string Pred;
  raw_string_ostream SS(Pred);
  bool UseSwitch = Assigns.size() > 1;

  OS << "\n// Assignment of " << getName() << '\n';
  if (UseSwitch) {
    OS << VASTModule::ParallelCaseAttr << ' ';
    OS.switch_begin("1'b1");
  }

  for (AssignMapTy::const_iterator I = Assigns.begin(), E = Assigns.end();
       I != E; ++I) {
    SS << '(';
    typedef OrCndVec::const_iterator or_it;
    for (or_it OI = I->second.begin(), OE = I->second.end(); OI != OE; ++OI) {
      printCondition(SS, OI->first, OI->second);
      SS << " | ";
    }
    // Build the assign condition.
    SS << "1'b0)";
    SS.flush();
    // Print the assignment under the condition.
    if (UseSwitch) OS.match_case(Pred);
    else OS.if_begin(Pred);
    OS << getName() << verilogBitRange(getBitWidth(), 0, false) << " <= ";
    I->first.print(OS);
    OS << ";\n";
    OS.exit_block();

    Pred.clear();
  }
  if (UseSwitch) OS.switch_end();
}

VASTWire::VASTWire(const char *Name, unsigned BitWidth,
                   const char *Attr)
  : VASTSignal(vastWire, Name, BitWidth, Attr), Opc(dpUnknown) {}

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

  delete &(DataPath.str());
  delete &(ControlBlock.str());
}

void VASTModule::clear() {
  // Clear buffers
  DataPath.str().clear();
  ControlBlock.str().clear();
}

void VASTModule::printDatapath(raw_ostream &OS) const{
  for (WireVector::const_iterator I = Wires.begin(), E = Wires.end();
       I != E; ++I)
    (*I)->print(OS);
}

void VASTModule::printRegisterAssign(vlang_raw_ostream &OS) const {
  for (RegisterVector::const_iterator I = Registers.begin(), E = Registers.end();
       I != E; ++I)
    (*I)->printAssignment(OS);
}

void VASTModule::printSlotCtrls(vlang_raw_ostream &CtrlS) const {
  CtrlS << "\n\n// Slot control flow\n";

  for (SlotVecTy::const_iterator I = Slots.begin(), E = Slots.end();I != E;++I)
    if (VASTSlot *S = *I) S->printCtrl(CtrlS, *this);
}

void VASTModule::buildSlotLogic() {
  raw_ostream &OS = getDataPathBuffer();
  OS << "\n\n// Slot Active Signal\n";

  // DirtyHack: print slot ready logic to data path instead of bulding logic
  // expression for it.
  for (SlotVecTy::const_iterator I = Slots.begin(), E = Slots.end();I != E;++I)
    if (VASTSlot *S = *I)
      S->buildReadyLogic(OS, *this);
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
  for (WireVector::const_iterator I = Wires.begin(), E = Wires.end();
       I != E; ++I) {
    (*I)->printDecl(OS);
    OS << "\n";
  }

  for (RegisterVector::const_iterator I = Registers.begin(), E = Registers.end();
       I != E; ++I) {
    (*I)->printDecl(OS);
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

VASTRegister::AndCndVec
VASTModule::allocateAndCndVec(SmallVectorImpl<VASTCnd> &Cnds) {
  VASTCnd *CndArray = Allocator.Allocate<VASTCnd>(Cnds.size());
  std::uninitialized_copy(Cnds.data(), Cnds.data() + Cnds.size(), CndArray);
  return ArrayRef<VASTCnd>(CndArray, Cnds.size());
}

void VASTModule::addAssignment(VASTRegister *Dst, VASTUse Src, VASTSlot *Slot,
                               SmallVectorImpl<VASTCnd> &Cnds) {
  Dst->addAssignment(Src, allocateAndCndVec(Cnds), Slot);
}

void VASTModule::reportAssignmentSlacks() {
  for (RegisterVector::const_iterator I = Registers.begin(), E = Registers.end();
       I != E; ++I)
    (*I)->reportAssignmentSlack();
}

void VASTModule::print(raw_ostream &OS) const {
  // Print the verilog module?
}

VASTPort *VASTModule::addInputPort(const std::string &Name, unsigned BitWidth,
                                   PortTypes T /*= Others*/) {
  // DIRTYHACK: comment out the deceleration of the signal for ports.
  VASTWire *W = addWire(Name, BitWidth, "//");

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
  if (isReg) V = addRegister(Name, BitWidth, 0, "//");
  else       V = addWire(Name, BitWidth, "//");

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

VASTUse VASTModule::indexVASTValue(unsigned RegNum, VASTUse V) {
  bool Inserted = RegsMap.insert(std::make_pair(RegNum, V)).second;
  assert(Inserted && "ValueIndex already existed!");
  (void) Inserted;
  return V;
}

VASTRegister *VASTModule::addRegister(const std::string &Name, unsigned BitWidth,
                                      unsigned InitVal,
                                      const char *Attr) {
  SymEntTy &Entry = SymbolTable.GetOrCreateValue(Name);
  assert(Entry.second == 0 && "Symbol already exist!");
  VASTRegister *Reg = Allocator.Allocate<VASTRegister>();
  new (Reg) VASTRegister(Entry.first(), BitWidth, InitVal, Attr);
  Entry.second = Reg;
  Registers.push_back(Reg);

  return Reg;
}

VASTRegister *VASTModule::addRegister(unsigned RegNum, unsigned BitWidth,
                                      unsigned InitVal,
                                      const char *Attr) {
  std::string Name;

  if (TargetRegisterInfo::isVirtualRegister(RegNum))
    Name = "reg" + utostr_32(TargetRegisterInfo::virtReg2Index(RegNum));
  else
    Name = "phy_reg" + utostr_32(RegNum);

  VASTRegister *R = addRegister(Name, BitWidth, 0, Attr);
  indexVASTValue(RegNum, R);
  return R;
}

VASTWire *VASTModule::addWire(const std::string &Name, unsigned BitWidth,
                              const char *Attr) {
  SymEntTy &Entry = SymbolTable.GetOrCreateValue(Name);
  assert(Entry.second == 0 && "Symbol already exist!");
  VASTWire *Wire = Allocator.Allocate<VASTWire>();
  new (Wire) VASTWire(Entry.first(), BitWidth, Attr);
  Entry.second = Wire;
  Wires.push_back(Wire);

  return Wire;
}

VASTWire *VASTModule::addWire(unsigned WireNum, unsigned BitWidth,
                              const char *Attr) {
  std::string Name;

  assert(TargetRegisterInfo::isVirtualRegister(WireNum)
         && "Unexpected physics register as wire!");
  Name = "wire" + utostr_32(TargetRegisterInfo::virtReg2Index(WireNum));

  VASTWire *W = addWire(Name, BitWidth, Attr);
  indexVASTValue(WireNum, W);
  return W;
}

// Out of line virtual function to provide home for the class.
void VASTModule::anchor() {}

void VASTValue::print(raw_ostream &OS) const {
  assert(0 && "VASTValue::print should not be called!");
}

void VASTPort::print(raw_ostream &OS) const {
  if (isInput())
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
  if (isInput())
    // We need a reg to drive input port.
    OS << "reg";
  else
    // We need a wire to accept the output value from dut.
    OS << "wire";

  if (getBitWidth() > 1)
    OS << "[" << (getBitWidth() - 1) << ":0]";

  OS << ' ' << getName();

  if (isInput())
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
  if (isRegister())
    OS << "reg";
  else
    OS << "wire";

  if (getBitWidth() > 1)
    OS << "[" << (getBitWidth() - 1) << ":0]";

  OS << ' ' << getName();

  if (isRegister())
    OS << " = " << verilogConstToStr(0, getBitWidth(), false);

  OS << ";";
}

//----------------------------------------------------------------------------//
// Operand printing helper functions.
static void printSignedOperand(raw_ostream &OS, VASTUse U) {
  OS << "$signed(";
  U.print(OS);
  OS << ")";
}

static void printUnsignedOperand(raw_ostream &OS, VASTUse U) {
  U.print(OS);
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

//static void printSimpleSignedOp(raw_ostream &OS, ArrayRef<VASTUse> Ops,
//                                  const char *Opc) {
//  printSimpleOp(OS, Ops, Opc, printSignedOperand);
//}

//----------------------------------------------------------------------------//
// Generic datapath printing helper function.
static void printUnaryOp(raw_ostream &OS, VASTUse U, const char *Opc) {
  OS << Opc;
  U.print(OS);
}

static void printSRAOp(raw_ostream &OS, const VASTWire *W) {
  printSignedOperand(OS, W->getOperand(0));
  OS << " >>> ";
  W->getOperand(1).print(OS);
}

template<typename PrintOperandFN>
static void printCmpFU(raw_ostream &OS, ArrayRef<VASTUse> Ops,
                       PrintOperandFN &FN) {
  OS << "{ ((";
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

static void printBitCat(raw_ostream &OS, ArrayRef<VASTUse> Ops) {
  OS << '{';
  printSimpleUnsignedOp(OS, Ops, " , ");
  OS << '}';
}

static void printBitRepeat(raw_ostream &OS, ArrayRef<VASTUse> Ops) {
  OS << '{';
  Ops[1].print(OS);
  OS << '{';
  Ops[0].print(OS);
  OS << "}}";
}

static void printCombMux(raw_ostream &OS, const VASTWire *W) {
  unsigned NumOperands = W->getNumOperands();
  assert((NumOperands & 0x1) == 0 && "Expect even operand number for CombMUX!");

  // Handle the trivial case trivially: Only 1 input.
  if (NumOperands == 2) {
    printAssign(OS, W);
    W->getOperand(1).print(OS);
    OS << ";\n";
    return;
  }

  // Create the temporary signal.
  OS << "// Combinational MUX\n"
     << "reg " << verilogBitRange(W->getBitWidth()) << ' ' << W->getName()
     << "_mux_wire;\n";

  // Assign the temporary signal to the wire.
  printAssign(OS, W) << W->getName() << "_mux_wire;\n";

  // Print the mux logic.
  OS << "always @(*)begin  // begin mux logic\n";
  OS.indent(2) << VASTModule::ParallelCaseAttr << " case (1'b1)\n";
  for (unsigned i = 0; i < NumOperands; i+=2) {
    OS.indent(4);
    W->getOperand(i).print(OS);
    OS << ": " << W->getName() << "_mux_wire = ";
    W->getOperand(i + 1).print(OS);
    OS << ";\n";
  }
  // Write the default condition, otherwise latch will be inferred.
  OS.indent(4) << "default: " << W->getName() << "_mux_wire = "
               << W->getBitWidth() << "'bx;\n";
  OS.indent(2) << "endcase\nend  // end mux logic\n";
}

void VASTWire::addOperand(VASTUse Op) {
  Operands.push_back(Op);;
}

void VASTWire::print(raw_ostream &OS) const {
  // Skip unknown datapath, it should printed to the datapath buffer of the
  // module
  if (Opc == dpUnknown) return;

  // MUX need special printing method.
  if (Opc == dpMux) {
    printCombMux(OS, this);
    return;
  }

  printAssign(OS, this);

  switch (Opc) {
  case dpNot: printUnaryOp(OS, getOperand(0), " ~ ");  break;
  case dpAnd: printSimpleUnsignedOp(OS, Operands, " & "); break;
  case dpOr:  printSimpleUnsignedOp(OS, Operands, " | "); break;
  case dpXor: printSimpleUnsignedOp(OS, Operands, " ^ "); break;

  case dpRAnd:  printUnaryOp(OS, getOperand(0), "&");  break;
  case dpROr:   printUnaryOp(OS, getOperand(0), "|");  break;
  case dpRXor:  printUnaryOp(OS, getOperand(0), "^");  break;

  case dpSCmp:  printCmpFU(OS, Operands, printSignedOperand); break;
  case dpUCmp:  printCmpFU(OS, Operands, printUnsignedOperand); break;

  case dpAdd: printSimpleUnsignedOp(OS, Operands, " + "); break;
  case dpMul: printSimpleUnsignedOp(OS, Operands, " * "); break;
  case dpShl: printSimpleUnsignedOp(OS, Operands, " << ");break;
  case dpSRL: printSimpleUnsignedOp(OS, Operands, " >> ");break;
  case dpSRA: printSRAOp(OS, this);               break;

  case dpAssign: getOperand(0).print(OS);     break;

  case dpBitCat:    printBitCat(OS, Operands);    break;
  case dpBitRepeat: printBitRepeat(OS, Operands); break;
  default: llvm_unreachable("Unknown datapath opcode!"); break;
  }

  OS << ";\n";
}
