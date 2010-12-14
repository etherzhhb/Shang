//===- LangStream.h - The raw_ostream for writing C/C++/Verilog -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file define the raw_ostream named lang_raw_ostream for writing
// C/C++/Verilog. The feature of lang_raw_ostream including:
//   Auto-indentation
//   ...
//
//===----------------------------------------------------------------------===//

#ifndef HAA_LANG_STREAM_H
#define HAA_LANG_STREAM_H

#include "llvm/Support/FormattedStream.h"

namespace llvm {
template<class LangTraits>
class lang_raw_ostream : public formatted_raw_ostream {
protected:
  // Current block indent.
  unsigned Indent;
  // We are start form a new line?
  bool newline;

  static bool isPrepProcChar(char C) {
    return C == LangTraits::PrepProcChar;
  }

  static bool isNewLine(const char *Ptr) {
    return *Ptr == '\n' || *Ptr == '\r';
  }

  // 
  size_t line_length(const char *Ptr, size_t Size) {
    size_t Scanned = 0;
    const char *End = Ptr + Size;
    for (; Ptr != End; ++Ptr) {
      ++Scanned;
      if (isNewLine(Ptr)) return Scanned;
    }

    return Size;
  }

  virtual void write_impl(const char *Ptr, size_t Size) {
    assert(Size > 0 && "Unexpected writing zero char!");

    //Do not indent the preprocessor directive.
    if (newline && !(isPrepProcChar(*Ptr)) && Indent)
      TheStream->indent(Indent);

    size_t line_len = line_length(Ptr, Size);
    const char *NewLineStart = Ptr + line_len;

    newline = isNewLine(NewLineStart - 1);

    // Write current line.
    TheStream->write(Ptr, line_len);

    // Write the rest lines.
    if (size_t SizeLeft = Size - line_len)
      write_impl(NewLineStart, SizeLeft);
    else // Compute the column for last line.
      ComputeColumn(Ptr, Size);
  }

public:
  explicit lang_raw_ostream(unsigned Ind = 0)
    : formatted_raw_ostream(), Indent(Ind), newline(true) {}

  explicit lang_raw_ostream(raw_ostream &Stream, bool Delete = false,
                            unsigned Ind = 0)
    : formatted_raw_ostream(Stream, Delete), Indent(Ind), newline(true) {}

  template<typename PosfixT>
  lang_raw_ostream &enter_block(PosfixT Posfix,
                                const char *Begin = LangTraits::getBlockBegin())
  {
    // flush the buffer.
    flush();
    // Increase the indent.
    Indent += 2;

    // Write the block begin character.
    operator<<(Begin);
    write(' ');
    operator<<(Posfix);
    
    return *this;
  }

  lang_raw_ostream &enter_block(const char *Posfix = "\n") {
    return enter_block<const char*>(Posfix);
  }

  template<typename PosfixT>
  lang_raw_ostream &exit_block(PosfixT Posfix,
                               const char *End = LangTraits::getBlockEnd()) {
    // flush the buffer.
    flush();

    assert(Indent >= 2 && "Unmatch block_begin and exit_block!");
    // Decrease the indent.
    Indent -= 2;

    // Write the block begin character.
    operator<<(End);
    write(' ');
    operator<<(Posfix);
    return *this;
  }

  lang_raw_ostream &exit_block(const char *Posfix = "\n") {
    return exit_block<const char*>(Posfix);
  }
};

struct CppTraits {
  static const char PrepProcChar = '#';

  static const char *getBlockBegin() {
    static const char *S = "{";
    return S;
  }

  static const char *getBlockEnd() {
    static const char *S = "}";
    return S;
  }
};

typedef lang_raw_ostream<CppTraits> clang_raw_ostream;

struct VerilogTraits {
  static const char PrepProcChar = '`';

  static const char *getBlockBegin() {
    static const char *S = "begin";
    return S;
  }

  static const char *getBlockEnd() {
    static const char *S = "end";
    return S;
  }
};

class vlang_raw_ostream : public lang_raw_ostream<VerilogTraits> {
public:
  explicit vlang_raw_ostream(unsigned Ind = 0)
    : lang_raw_ostream<VerilogTraits>(Ind) {}

  explicit vlang_raw_ostream(raw_ostream &Stream, bool Delete = false,
    unsigned Ind = 0)
    : lang_raw_ostream<VerilogTraits>(Stream, Delete) {}

  vlang_raw_ostream &always_ff_begin(const std::string &Clk = "clk",
                                     const std::string &ClkEdge = "posedge",
                                     const std::string &Rst = "rstN",
                                     const std::string &RstEdge = "negedge") {
    *this << "always @(" << ClkEdge << " "<< Clk <<", " 
                         << RstEdge << " " << Rst <<")";
    enter_block();
    *this << "if (";
    // negative edge reset?
    if (RstEdge == "negedge")
      *this  << "!";
    *this  << Rst << ")";
    enter_block("// reset registers\n");
    return *this;
  }

  vlang_raw_ostream &always_ff_end() {
    exit_block("//else reset\n");
    exit_block("//always @(..)\n\n");
    return *this;
  }

  template<typename CaseT>
  vlang_raw_ostream &switch_begin(CaseT Case) {
    *this << "case (" << Case << ") ";
    enter_block("\n", "");
    Indent -= 2;
    return *this;
  }

  template<typename CaseT>
  vlang_raw_ostream &match_case(CaseT Case) {
    *this << Case << ":";
    enter_block();
    return *this;
  }

  vlang_raw_ostream &switch_end() {
    // Flush the content before change the indent.
    flush();
    Indent += 2;
    exit_block("\n", "endcase");
    return *this;
  }

  template<typename CndT>
  vlang_raw_ostream &if_begin(CndT Cnd) {
    *this <<"if (" << Cnd << ") ";
    enter_block();
    return *this;
  }

  vlang_raw_ostream &else_begin() {
    exit_block("else ");
    enter_block();
    return *this;
  }

  vlang_raw_ostream &module_begin() {
    enter_block("\n", "");
    return *this;
  }

  vlang_raw_ostream &module_end() {
    exit_block("\n", "endmodule");
    return *this;
  }
};


}

#endif
