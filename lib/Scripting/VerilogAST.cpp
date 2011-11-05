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

#include "llvm/Constants.h"
#include "llvm/GlobalVariable.h"
#include "llvm/DerivedTypes.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/Support/ErrorHandling.h"
#define DEBUG_TYPE "verilog-ast"
#include "llvm/Support/Debug.h"

#include <set>
#include <algorithm>
#include <sstream>

// Include the lua headers (the extern "C" is a requirement because we're
// using C++ and lua has been compiled as C code)
extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

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

//std::string VLang::GetValueName(const Value *Operand) {
//  // Mangle globals with the standard mangler interface for LLC compatibility.
//  if (const GlobalValue *GV = dyn_cast<GlobalValue>(Operand)) {
//    SmallString<128> Str;
//    Mang->getNameWithPrefix(Str, GV, false);
//    return VLangMangle(Str.str().str());
//  }
//
//  std::string Name = Operand->getName();
//
//  // Constant
//
//  if (Name.empty()) { // Assign unique names to local temporaries.
//    unsigned &No = AnonValueNumbers[Operand];
//    if (No == 0)
//      No = ++NextAnonValueNumber;
//    Name = "tmp__" + utostr(No);
//  }
//
//  std::string VarName;
//  VarName.reserve(Name.capacity());
//
//  for (std::string::iterator I = Name.begin(), E = Name.end();
//      I != E; ++I) {
//    char ch = *I;
//
//  if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
//      (ch >= '0' && ch <= '9') || ch == '_')) {
//    char buffer[5];
//    sprintf(buffer, "_%x_", ch);
//    VarName += buffer;
//  } else
//    VarName += ch;
//  }
//
//  return "esyn_" + VarName;
//}

raw_ostream &llvm::verilogParam(raw_ostream &ss, const std::string &Name,
                                unsigned BitWidth, unsigned Val) {
  ss << "parameter " << Name
    << " = " << verilogConstToStr(Val, BitWidth, false) << ";\n";
  return ss;
}

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
    break;
  case USE_Immediate:
    OS << verilogConstToStr(Data.ImmVal, UB, false);
    // No need to print bit range for immediate.
    return;
  }
}

void VASTCnd::print(raw_ostream &OS) const {
  OS << '(';
  if (isInverted()) OS << '~';
  VASTUse::print(OS);
  OS << ')';
}

void VASTSlot::addNextSlot(unsigned NextSlotNum, VASTCnd Cnd) {
  bool Inserted = NextSlots.insert(std::make_pair(NextSlotNum, Cnd)).second;
  assert(Inserted && "NextSlot already existed!");
  (void) Inserted;
}

void VASTSlot::addEnable(const VASTValue *V, VASTCnd Cnd) {
  bool Inserted = Enables.insert(std::make_pair(V, Cnd)).second;
  assert(Inserted && "NextSlot already existed!");
  (void) Inserted;
}

void VASTSlot::addReady(const VASTValue *V, VASTCnd Cnd /* = VASTCnd */) {
  bool Inserted = Readys.insert(std::make_pair(V, Cnd)).second;
  assert(Inserted && "NextSlot already existed!");
  (void) Inserted;
}

void VASTSlot::addDisable(const VASTValue *V, VASTCnd Cnd) {
  bool Inserted = Disables.insert(std::make_pair(V, Cnd)).second;
  assert(Inserted && "NextSlot already existed!");
  (void) Inserted;
}

void VASTSlot::printReady(raw_ostream &OS) const {
  OS << "1'b1";
  for (VASTSlot::const_fu_ctrl_it I = ready_begin(), E = ready_end();
        I != E; ++I) {
    // If the condition is true then the signal must be 1 to ready.
    OS << " & (" << I->first->getName() << " | ~";
    I->second.print(OS);
    OS << ')';
  }
}

void VASTSlot::printActive(raw_ostream &OS, const VASTModule &Mod) const {
  OS << "wire " << getName() << "Ready = ";
  printReady(OS);

  if (StartSlot != EndSlot) {
    for (unsigned slot = StartSlot; slot < EndSlot; slot += II) {
      if (slot == getSlotNum()) continue;

      const VASTSlot *AliasSlot = Mod.getSlot(slot);

      if (!AliasSlot->readyEmpty()) {
        OS << " & ( ~Slot" << slot << " | (";
        AliasSlot->printReady(OS);
        OS << "))";
      }
    }
  }

  OS << ";// Are all waiting resources ready?\n";
  OS << VASTModule::DirectClkEnAttr << ' ';
  OS << "wire " << getName() << "Active = " << getName() << "Ready & "
     << getName() << ";\n";
}

void VASTSlot::printCtrl(vlang_raw_ostream &CtrlS, const VASTModule &Mod) const{
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

VASTRegister::VASTRegister(const char *Name, unsigned BitWidth,
                           unsigned initVal, const char *Attr)
  : VASTSignal(vastRegister, Name, BitWidth, Attr), InitVal(initVal) {}

void VASTRegister::addAssignment(VASTUse Src, AndCndVec Cnd, VASTSlot *S) {
  assert(Src != this && "Self assignemnt not supported yet!");
  Assigns[Src].push_back(std::make_pair(S, Cnd));
}

void VASTRegister::printCondition(raw_ostream &OS, const VASTSlot *Slot,
                                  const AndCndVec Cnds) {
  OS << '(';
  if (Slot) OS << Slot->getName() << "Active";
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
    OS << getName() << " <= ";
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

VASTModule::~VASTModule() {
  // Release all ports.
  Ports.clear();
  Signals.clear();
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
  for (SignalVector::const_iterator I = Signals.begin(), E = Signals.end();
       I != E; ++I)
    if (VASTWire *W = dyn_cast<VASTWire>(*I))
      W->print(OS);
}

void VASTModule::printRegisterAssign(vlang_raw_ostream &OS) const {
  for (SignalVector::const_iterator I = Signals.begin(), E = Signals.end();
       I != E; ++I)
    if (VASTRegister *R = dyn_cast<VASTRegister>(*I))
      R->printAssignment(OS);
}

void VASTModule::printSlotCtrls(vlang_raw_ostream &CtrlS) const {
  CtrlS << "\n\n// Slot control flow\n";

  for (SlotVecTy::const_iterator I = Slots.begin(), E = Slots.end();I != E;++I)
    if (VASTSlot *S = *I) S->printCtrl(CtrlS, *this);
}

void VASTModule::printSlotActives(raw_ostream &OS) const {
  OS << "\n\n// Slot Active Signal\n";

  for (SlotVecTy::const_iterator I = Slots.begin(), E = Slots.end();I != E;++I)
    if (VASTSlot *S = *I) S->printActive(OS, *this);
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
  for (SignalVector::const_iterator I = Signals.begin(), E = Signals.end();
       I != E; ++I) {
    (*I)->printDecl(OS);
    OS << "\n";
  }
}

void VASTModule::printRegisterReset(raw_ostream &OS) {
  for (SignalVector::const_iterator I = Signals.begin(), E = Signals.end();
       I != E; ++I)
    if (VASTRegister *R = dyn_cast<VASTRegister>(*I)) {
      R->printReset(OS);
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
  Signals.push_back(Reg);

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
  Signals.push_back(Wire);

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

static void printAssign(raw_ostream &OS, const VASTWire *W) {
  OS << "assign " << W->getName() << " = ";
}

static void printSimpleOp(raw_ostream &OS, ArrayRef<VASTUse> Ops,
                          const char *Opc) {
  unsigned NumOps = Ops.size();
  assert(NumOps && "Unexpected zero operand!");
  Ops[0].print(OS);

  for (unsigned i = 1; i < NumOps; ++i) {
    OS << Opc;
    Ops[i].print(OS);
  }
}

static void printUnaryOp(raw_ostream &OS, VASTUse U, const char *Opc) {
  OS << Opc;
  U.print(OS);
}


static void printSRAOp(raw_ostream &OS, const VASTWire *W) {
  VASTSignal *LHS = cast<VASTSignal>(W->getOperand(0));
  // Cast LHS to signed wire.
  OS << "wire signed" << verilogBitRange(LHS->getBitWidth()) << ' '
     << LHS->getName() << "_signed = " << LHS->getName() << ";\n";

  printAssign(OS, W);
  OS << LHS->getName() << "_signed >>> ";

  W->getOperand(1).print(OS);
  OS << ";\n";
}

static void printBitCat(raw_ostream &OS, ArrayRef<VASTUse> Ops) {
  OS << '{';
  printSimpleOp(OS, Ops, " , ");
  OS << '}';
}

static void printBitRepeat(raw_ostream &OS, ArrayRef<VASTUse> Ops) {
  OS << '{';
  Ops[2].print(OS);
  OS << '{';
  Ops[1].print(OS);
  OS << "}}";
}

void VASTWire::print(raw_ostream &OS) const {
  // Skip unknown datapath, it should printed to the datapath buffer of the
  // module
  if (Opc == dpUnknown) return;

  // SRA need special printing method.
  if (Opc == dpSRA) {
    printSRAOp(OS, this);
    return;
  }

  printAssign(OS, this);

  switch (Opc) {
  case dpNot: printUnaryOp(OS, getOperand(0), " ~ ");  break;
  case dpAnd: printSimpleOp(OS, Operands, " & "); break;
  case dpOr:  printSimpleOp(OS, Operands, " | "); break;
  case dpXor: printSimpleOp(OS, Operands, " ^ "); break;

  case dpRAnd:  printUnaryOp(OS, getOperand(0), "&");  break;
  case dpROr:   printUnaryOp(OS, getOperand(0), "|");  break;
  case dpRXor:  printUnaryOp(OS, getOperand(0), "^");  break;

  case dpAdd: printSimpleOp(OS, Operands, " + "); break;
  case dpMul: printSimpleOp(OS, Operands, " * "); break;
  case dpShl: printSimpleOp(OS, Operands, " << ");break;
  case dpSRL: printSimpleOp(OS, Operands, " >> ");break;

  case dpAssign: getOperand(0).print(OS);     break;

  case dpBitCat:    printBitCat(OS, Operands);    break;
  case dpBitRepeat: printBitRepeat(OS, Operands); break;
  default: llvm_unreachable("Unknown datapath opcode!"); break;
  }

  OS << ";\n";
}
