//===----- VASTExprBuilder.h - Building Verilog AST Expressions -*- C++ -*-===//
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
// The Interface to build and optimze VASTExprs.
//
//===----------------------------------------------------------------------===//
#ifndef VTM_VASTEXPR_BUILDER_H
#define VTM_VASTEXPR_BUILDER_H

#include "vtm/VerilogAST.h"

#include "llvm/Support/Allocator.h"

namespace llvm {
struct VASTExprHelper {
  SmallVector<VASTValPtr, 4> Operands;
  VASTExpr::Opcode Opc;
  unsigned BitWidth;
  bool BuildNot;

  void init(VASTExpr::Opcode opc, unsigned bitWidth, bool buildNot = false) {
    Opc = opc;
    BitWidth = bitWidth;
    BuildNot = buildNot;
  }

  void addOperand(VASTValPtr V) {
    Operands.push_back(V.getAsInlineOperand());
  }
};

class VASTExprBuilderContext {
public:
  virtual ~VASTExprBuilderContext() {}

  virtual VASTValPtr nameExpr(VASTValPtr V) { return V; }
  virtual VASTValPtr stripName(VASTValPtr V) const { return V; }

  virtual bool shouldExprBeFlatten(VASTExpr *E) const {
    return E->isInlinable();
  }

  VASTValPtr stripZeroBasedBitSlize(VASTValPtr V) {
    VASTExprPtr Expr = dyn_cast<VASTExprPtr>(V);
    if (Expr.get() && Expr->isSubBitSlice() && Expr->LB == 0)
      return Expr.getOperand(0);

    return V;
  }

  // If V is an addition which can be flatten the addition that using its result
  // return the expression, or return null otherwise.
  virtual VASTExpr *getAddExprToFlatten(VASTValPtr V) {
    V = stripName(stripZeroBasedBitSlize(V));

    VASTExpr *Expr = dyn_cast<VASTExpr>(V);
    if (!Expr || Expr->getOpcode() != VASTExpr::dpAdd) return 0;

    // We only flatten the expression to make full use of the carry bit.
    // So check if there is only 2 operand and the second operand can be fitted
    // into the carry bit.
    if (Expr->NumOps != 2 || Expr->getOperand(1)->getBitWidth() != 1)
      return 0;

    return Expr;
  }

  virtual VASTImmediate *getOrCreateImmediate(uint64_t Value, int8_t BitWidth) {
    return 0;
  }

  virtual VASTValPtr createExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                                unsigned UB, unsigned LB) {
    return 0;
  }
};

template<VASTExpr::Opcode Opcode>
struct VASTExprOpInfo {
  VASTExprOpInfo() {}

  VASTValPtr analyzeOperand(VASTValPtr V) {
    // Do nothing by default.
    return V;
  }
};

class VASTExprBuilder {
  void operator=(const VASTExprBuilder &RHS); // DO NOT IMPLEMENT
  VASTExprBuilder(const VASTExprBuilder &RHS); // DO NOT IMPLEMENT

  VASTExprBuilderContext &Context;

  VASTValPtr foldBitSliceExpr(VASTValPtr U, uint8_t UB, uint8_t LB);

  template<VASTExpr::Opcode Opcode, typename visitor>
  void flattenExpr(VASTValPtr V, visitor F);
  template<VASTExpr::Opcode Opcode, typename iterator, typename visitor>
  void flattenExpr(iterator begin, iterator end, visitor F);

  static bool isAllZeros(VASTValPtr V) {
    if (VASTImmPtr Imm = dyn_cast<VASTImmPtr>(V))
      return Imm->isAllZeros();

    return false;
  }

  template<VASTExpr::Opcode Opcode, class _Container>
  struct op_filler_iterator : public std::iterator<std::output_iterator_tag,
                                                   void, void, void, void> {
    typedef op_filler_iterator<Opcode, _Container> Self;

    VASTExprBuilder &Builder;
    VASTExprOpInfo<Opcode> &OpInfo;
    _Container &C;
    explicit op_filler_iterator(_Container &C, VASTExprOpInfo<Opcode> &OpInfo,
                                VASTExprBuilder &Builder)
      : Builder(Builder), OpInfo(OpInfo), C(C) {}

    Self &operator=(VASTValPtr V) {
      if ((V = OpInfo.analyzeOperand(V)))
        C.push_back(V);

      return *this;
    }

    Self& operator*() {
      // pretend to return designated value
      return (*this);
    }

    Self& operator++() {
      // pretend to preincrement
      return (*this);
    }

    Self operator++(int) {
      // pretend to postincrement
      return (*this);
    }
  };

  template<VASTExpr::Opcode Opcode, class _Container>
  op_filler_iterator<Opcode, _Container> op_filler(_Container &C,
                                                   VASTExprOpInfo<Opcode> &Info)
  {
    return op_filler_iterator<Opcode, _Container>(C, Info, *this);
  }

  VASTValPtr padHigherBits(VASTValPtr V, unsigned BitWidth, bool ByOnes);
public:
  explicit VASTExprBuilder(VASTExprBuilderContext &Context)
    : Context(Context) {}
  

  // Bit mask analyzing, bitmask_collecting_iterator.
  void calculateBitMask(VASTValPtr V, uint64_t &KnownZeros,uint64_t &KnownOnes);
  void calculateBitCatBitMask(VASTExprPtr Expr, uint64_t &KnownZeros,
                              uint64_t &KnownOnes);

  VASTValPtr getBoolImmediate(bool Val) {
    return Context.getOrCreateImmediate(Val, 1);
  }

  VASTImmediate *getOrCreateImmediate(uint64_t Value, int8_t BitWidth) {
    return Context.getOrCreateImmediate(Value, BitWidth);
  }

  bool shouldExprBeFlatten(VASTExpr *E) const {
    return Context.shouldExprBeFlatten(E);
  }

  VASTValPtr getOrCreateCommutativeExpr(VASTExpr::Opcode Opc,
                                        SmallVectorImpl<VASTValPtr> &Ops,
                                        unsigned BitWidth);

  VASTValPtr buildExpr(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                       unsigned BitWidth);

  VASTValPtr buildExprByOpBitSlice(VASTExpr::Opcode Opc, ArrayRef<VASTValPtr> Ops,
                                   uint8_t UB, uint8_t LB) {
    SmallVector<VASTValPtr, 8> OpBitSlices;
    for (unsigned i = 0; i < Ops.size(); ++i)
      OpBitSlices.push_back(buildBitSliceExpr(Ops[i], UB, LB));

    return buildExpr(Opc, OpBitSlices, UB - LB);
  }

  VASTValPtr buildExpr(VASTExpr::Opcode Opc,VASTValPtr Op, unsigned BitWidth);
  VASTValPtr buildExpr(VASTExpr::Opcode Opc, VASTValPtr LHS, VASTValPtr RHS,
                       unsigned BitWidth);
  template<VASTExpr::Opcode Opc>
  static VASTValPtr buildExpr(VASTValPtr LHS, VASTValPtr RHS, unsigned BitWidth,
                              VASTExprBuilder *Builder) {
    return Builder->buildExpr(Opc, LHS, RHS, BitWidth);
  }

  VASTValPtr buildExpr(VASTExpr::Opcode Opc, VASTValPtr Op0, VASTValPtr Op1,
                       VASTValPtr Op2, unsigned BitWidth);
  VASTValPtr buildExpr(VASTExprHelper &Builder) {
    VASTValPtr V = buildExpr(Builder.Opc, Builder.Operands, Builder.BitWidth);

    // If opc is dpAnd and BuildNot is true. It mean Or in And Invert Graph.
    if (Builder.BuildNot) V = buildNotExpr(V);

    return V;
  }

  VASTValPtr buildBitSliceExpr(VASTValPtr U, uint8_t UB, uint8_t LB);
  VASTValPtr buildBitCatExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);
  VASTValPtr buildAndExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);
  VASTValPtr buildMulExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);
  VASTValPtr buildAddExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);

  VASTValPtr buildReduction(VASTExpr::Opcode Opc,VASTValPtr Op);

  VASTValPtr buildNotExpr(VASTValPtr U);

  static VASTValPtr buildOr(VASTValPtr LHS, VASTValPtr RHS, unsigned BitWidth,
                            VASTExprBuilder *Builder) {
    return Builder->buildOrExpr(LHS, RHS, BitWidth);
  }

  VASTValPtr buildOrExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);

  VASTValPtr buildOrExpr(VASTValPtr LHS, VASTValPtr RHS, unsigned BitWidth) {
    VASTValPtr Ops[] = { LHS, RHS };
    return buildOrExpr(Ops, BitWidth);
  }

  static VASTValPtr buildXor(VASTValPtr LHS, VASTValPtr RHS, unsigned BitWidth,
                             VASTExprBuilder *Builder) {
    VASTValPtr Ops[] = { LHS, RHS };
    return Builder->buildXorExpr(Ops, BitWidth);
  }

  VASTValPtr buildXorExpr(ArrayRef<VASTValPtr> Ops, unsigned BitWidth);
};
}

#endif
