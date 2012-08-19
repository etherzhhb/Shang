//===---- ControlLogicBuilder.cpp - Build the control logic  ----*- C++ -*-===//
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
// This file implement the functions related to the control path of the design.
//
//===----------------------------------------------------------------------===//
#include "VASTExprBuilder.h"

#include "vtm/VerilogAST.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/Support/CommandLine.h"
#define DEBUG_TYPE "vtm-ctrl-logic-builder"
#include "llvm/Support/Debug.h"

using namespace llvm;


static cl::opt<bool>
EnableBBProfile("vtm-enable-bb-profile",
                cl::desc("Generate counters to profile the design"),
                cl::init(false));

VASTValPtr VASTSlot::buildFUReadyExpr(VASTExprBuilder &Builder) {
  SmallVector<VASTValPtr, 4> Ops;

  for (VASTSlot::const_fu_rdy_it I = ready_begin(), E = ready_end();I != E; ++I) {
    // If the condition is true then the signal must be 1 to ready.
    VASTValPtr ReadyCnd = Builder.buildNotExpr(I->second->getAsInlineOperand());
    Ops.push_back(Builder.buildOrExpr(I->first, ReadyCnd, 1));
  }

  // No waiting signal means always ready.
  if (Ops.empty()) return Builder.getBoolImmediate(true);

  return Builder.buildAndExpr(Ops, 1);
}

void VASTSlot::buildReadyLogic(VASTModule &Mod, VASTExprBuilder &Builder) {
  SmallVector<VASTValPtr, 4> Ops;
  // FU ready for current slot.
  Ops.push_back(buildFUReadyExpr(Builder));

  if (hasAliasSlot()) {
    for (unsigned s = alias_start(), e = alias_end(), ii = alias_ii();
         s < e; s += ii) {
      if (s == SlotNum) continue;

      VASTSlot *AliasSlot = Mod.getSlot(s);

      if (AliasSlot->readyEmpty()) continue;

      // FU ready for alias slot, when alias slot register is 1, its waiting
      // signal must be 1.
      VASTValPtr AliasReady = AliasSlot->buildFUReadyExpr(Builder);
      VASTValPtr AliasDisactive = Builder.buildNotExpr(AliasSlot->getRegister());
      Ops.push_back(Builder.buildOrExpr(AliasDisactive, AliasReady, 1));
    }
  }

  // All signals should be 1 before the slot is ready.
  VASTValPtr ReadyExpr = Builder.buildAndExpr(Ops, 1);
  Mod.assign(cast<VASTWire>(getReady()), ReadyExpr);
  // The slot is activated when the slot is enable and all waiting signal is
  // ready.
  Mod.assign(cast<VASTWire>(getActive()),
             Builder.buildAndExpr(SlotReg, ReadyExpr, 1));
}

void VASTSlot::buildCtrlLogic(VASTModule &Mod, VASTExprBuilder &Builder) {
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

  assert(!NextSlots.empty() && "Expect at least 1 next slot!");
  CtrlS << "// Enable the successor slots.\n";
  for (VASTSlot::const_succ_cnd_iterator I = succ_cnd_begin(),E = succ_cnd_end();
        I != E; ++I) {
    hasSelfLoop |= I->first->SlotNum == SlotNum;
    VASTRegister *NextSlotReg = I->first->getRegister();
    Mod.addAssignment(NextSlotReg, *I->second, this, EmptySlotEnCnd);
  }

  assert(!(hasSelfLoop && PredAliasSlots)
         && "Unexpected have self loop and pred alias slot at the same time.");
  // Do not assign a value to the current slot enable twice.
  if (!hasSelfLoop) {
    // Only disable the current slot if there is no alias slot enable current
    // slot.
    if (PredAliasSlots)
      EmptySlotEnCnd.push_back(Builder.buildNotExpr(PredAliasSlots));

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
    VASTValPtr ReadyCnd
      = Builder.buildAndExpr(getReady()->getAsInlineOperand(false),
                             I->second->getAsInlineOperand(), 1);
    Mod.addAssignment(cast<VASTRegister>(I->first), ReadyCnd, this,
                      EmptySlotEnCnd, 0, false);
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
            DisableAndCnds.push_back(Builder.buildNotExpr(ASlot->getRegister()));
            continue;
          }
        }
      }

      DisableAndCnds.push_back(*I->second);

      VASTRegister *En = cast<VASTRegister>(I->first);
      Mod.addAssignment(En, Mod.getBoolImmediate(false), this,
                        DisableAndCnds, 0, false);
      DisableAndCnds.clear();
    }
  }
  CtrlS.exit_block("\n\n");
}

void VASTModule::buildSlotLogic(VASTExprBuilder &Builder) {
  bool IsFirstSlotInBB = false;
  for (SlotVecTy::const_iterator I = Slots.begin(), E = Slots.end();I != E;++I){
    if (VASTSlot *S = *I) {
      S->buildCtrlLogic(*this, Builder);

      // Create a profile counter for each BB.
      if (EnableBBProfile) writeProfileCounters(S, IsFirstSlotInBB);
      IsFirstSlotInBB = false;
      continue;
    }

    // We meet an end slot, The next slot is the first slot in new BB
    IsFirstSlotInBB = true;
  }
}
