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

VASTCnd VASTCnd::Create(VASTModule *M, ucOperand &Op) {
  VASTCnd Cnd(M->getVASTValue(Op.getReg()), Op.isPredicateInverted());
  return Cnd;
}

void VASTSymbol::print(raw_ostream &OS) const {

}

void VASTCnd::print(raw_ostream &OS) const {
  OS << '(';
  if (isInverted()) OS << '~';
  if (VASTValue *V = getCndVal()) OS << V->getName();
  else                            OS << "1'b1";
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
  OS << "wire " << getName() << "Active = " << getName() << "Ready & "
     << getName() << ";\n";
}

void VASTSlot::printCtrl(vlang_raw_ostream &CtrlS, const VASTModule &Mod) const{
  CtrlS.if_begin(getName());
  std::string SlotReady = getName() + "Ready";
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
           E = AliasSlot->enable_end(); I != E; ++I)
        AliasEnables.insert(I->first);

      ReadyPresented  |= !AliasSlot->readyEmpty();
    }

    CtrlS << '\n';
  } // SS flushes automatically here.

  // Enable next slot only when resources are ready.
  if (ReadyPresented)
    CtrlS.if_begin(SlotReady);

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

  SmallPtrSet<const VASTValue*, 4> EnabledPorts;
  CtrlS << "// Enable the active FUs.\n";
  for (VASTSlot::const_fu_ctrl_it I = enable_begin(), E = enable_end();
       I != E; ++I) {
    // We may try to enable and disable the same port at the same slot.
    CtrlS << "if (";
    I->second.print(CtrlS);
    CtrlS << ") " << I->first->getName() << " <= 1'b1;\n";
  }

  if (ReadyPresented) CtrlS.exit_block("// End resource ready.\n");

  if (!disableEmpty()) {
    CtrlS << "// Disable the resources when the condition is true.\n";
    for (VASTSlot::const_fu_ctrl_it I = disable_begin(), E = disable_end();
         I != E; ++I) {
      // Look at the current enable set and alias enables set;
      bool Enabled = isEnabled(I->first) || AliasEnables.count(I->first);
      if (Enabled) {
        assert(ReadyPresented && "Port conflict cannot be resolved!");
        CtrlS.if_begin("~" + SlotReady, "// Resolve the conflict\n");
      }

      CtrlS << "if (";
      I->second.print(CtrlS);
      CtrlS << ") "  << I->first->getName() << " <= 1'b0;\n";

      if (Enabled) CtrlS.exit_block();
    }
  }
  CtrlS.exit_block("\n\n");
}

VASTModule::~VASTModule() {
  // Release all ports.
  Ports.clear();
  Signals.clear();
  Slots.clear();
  Allocator.Reset();
  SymbolTable.clear();

  delete &(StateDecl.str());
  delete &(DataPath.str());
  delete &(ControlBlock.str());
}

void VASTModule::clear() {
  // Clear buffers
  StateDecl.str().clear();
  DataPath.str().clear();
  ControlBlock.str().clear();
}

void VASTModule::printDatapath(raw_ostream &OS) const{
  for (std::vector<VASTDatapath *>::const_iterator I = Datapaths.begin(),
       E = Datapaths.end(); I != E; ++I) {
   (*I)->print(OS);
  }
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
  
  // And Slots
  for (SlotVecTy::const_iterator I = Slots.begin(), E = Slots.end();I != E;++I)
    if (VASTSlot *S = *I) {
      S->printDecl(OS);
      OS << "\n";
    }
}

void VASTModule::printRegisterReset(raw_ostream &OS) {
  // Reset output registers.
  for (PortVector::const_iterator I = Ports.begin(), E = Ports.end();
       I != E; ++I) {
    VASTPort *port = *I;
    if (port->isRegister()) {
      port->printReset(OS);
      OS << "\n";
    }
  }

  for (SignalVector::const_iterator I = Signals.begin(), E = Signals.end();
       I != E; ++I) {
    VASTSignal *signal = *I;
    if (signal->isRegister()) {
      signal->printReset(OS);
      OS << "\n";
    }
  }

  // And Slots
  for (SlotVecTy::const_iterator I = Slots.begin(), E = Slots.end();I != E;++I)
    if (VASTSlot *S = *I) {
      S->printReset(OS);
      OS << "\n";
    }
}

void VASTModule::print(raw_ostream &OS) const {
  // Print the verilog module?
}


VASTPort *VASTModule::addInputPort(const std::string &Name, unsigned BitWidth,
                                   PortTypes T /*= Others*/,
                                   const std::string &Comment /*= ""*/ ) {
  VASTPort *Port
    = new (Allocator.Allocate<VASTPort>()) VASTPort(Name, BitWidth, true, false,
                                                    Comment);
  insertVASTValue(Name, Port);
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
                                    PortTypes T /*= Others*/, bool isReg /*= true*/,
                                    const std::string &Comment /*= ""*/ ) {
  VASTPort *Port
    = new (Allocator.Allocate<VASTPort>()) VASTPort(Name, BitWidth, false, isReg,
                                                    Comment);
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
  insertVASTValue(Name, Port);
  return Port;
}

VASTSignal *VASTModule::addRegister(const std::string &Name, unsigned BitWidth,
                                    const std::string &Comment /*= ""*/ ) {
  VASTSignal *Reg
    = new (Allocator.Allocate<VASTSignal>()) VASTSignal(Name, BitWidth, true,
                                                        Comment);
  Signals.push_back(Reg);
  insertVASTValue(Name, Reg);
  return Reg;
}


void llvm::VASTModule::addVASTValue(unsigned RegNum, VASTValue *V) {
  bool Inserted = RegsMap.insert(std::make_pair(RegNum, V)).second;
  assert(Inserted && "ValueIndex already existed!");
  (void) Inserted;
}

VASTSignal *VASTModule::addWire(const std::string &Name, unsigned BitWidth,
                                const std::string &Comment /*= ""*/ ) {
  VASTSignal *Signal =
    new (Allocator.Allocate<VASTSignal>()) VASTSignal(Name, BitWidth, false,
                                                      Comment);
  Signals.push_back(Signal);
  insertVASTValue(Name, Signal);
  return Signal;
}

// Out of line virtual function to provide home for the class.
void VASTModule::anchor() {}

void VASTValue::printReset( raw_ostream &OS ) const {
  OS << getName()  << " <= " 
    << verilogConstToStr(InitVal, getBitWidth(), false) << ";";
}

// Out of line virtual function to provide home for the class.
void VASTValue::anchor() {}

void VASTPort::print(raw_ostream &OS) const {
  if (isInput())
    OS << "input ";
  else
    OS << "output ";

  if (isRegister())
    OS << "reg";
  else
    OS << "wire";
  
  if (getBitWidth() > 1)
    OS << "[" << (getBitWidth() - 1) << ":0]";

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

void VASTSignal::print(raw_ostream &OS) const {

}

void VASTSignal::printDecl(raw_ostream &OS) const {
  if (isRegister())
    OS << "reg";
  else
    OS << "wire";

  if (getBitWidth() > 1)
    OS << "[" << (getBitWidth() - 1) << ":0]";

  OS << ' ' << getName();

//  if (isRegister())
//    OS << " = " << verilogConstToStr(0, getBitWidth(), false);

  OS << ";";
}

void VASTDatapath::print(raw_ostream &OS) const{
  const_cast<VASTDatapath*>(this)->CodeStream.flush();
  OS << Code ;
}

// Out of line virtual function to provide home for the class.
void VASTSignal::anchor() {}
