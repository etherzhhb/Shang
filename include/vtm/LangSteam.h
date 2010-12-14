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

  lang_raw_ostream &enter_block(bool newline = true) {
    // flush the buffer.
    flush();
    // Increase the indent.
    Indent += 2;

    // Write the block begin character.
    operator<<(LangTraits::getBlockBegin());
    if (newline) write('\n');
    
    return *this;
  }

  lang_raw_ostream &exit_block(bool newline = true) {
    // flush the buffer.
    flush();

    assert(Indent >= 2 && "Unmatch block_begin and exit_block!");
    // Decrease the indent.
    Indent -= 2;

    // Write the block begin character.
    operator<<(LangTraits::getBlockEnd());
    if (newline) write('\n');
    return *this;
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

typedef lang_raw_ostream<VerilogTraits> vlang_raw_ostream;

}

#endif
