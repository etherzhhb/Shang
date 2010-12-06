//===-- FileInfo.cpp - The FileInfo manage the output files of the backend -===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file inplement the FileInfo pass, which manage the various output files
// of the backend.
//
//===----------------------------------------------------------------------===//
#include "vtm/FileInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Support/Path.h"

using namespace llvm;

FileInfo::~FileInfo() {
  DeleteContainerPointers(OpenedFiles);
}

void FileInfo::setOutFilesDir(const std::string &Val) {
  OutFilesDir = Val;

  sys::Path OFDir(OutFilesDir);
  if (!OFDir.isEmpty() && !OFDir.isDirectory())
    report_fatal_error("Bad output file directory: " + OutFilesDir);
}

tool_output_file *FileInfo::getOutFile(const std::string &Suffix,
                                       unsigned Flags) {
  // Do not open stdout twice.
  if (writeAllToStdOut() && !OpenedFiles.empty()) {
    assert(OpenedFiles.size() == 1 && "Unexpected opened files number!");
    return OpenedFiles.front();
  }

  std::string FileName = getOutFilePath(HWSubSysName, Suffix);
  std::string error;

  tool_output_file *File = new tool_output_file(FileName.c_str(), error, Flags);
  if (!error.empty()) {
    report_fatal_error("Can not open file " + FileName + ": " + error);
    return 0;
  }

  OpenedFiles.push_back(File);
  return File;

}

std::string FileInfo::getOutFilePath(const std::string &Name,
                                     const std::string &Suffix) const {
  if (WriteAllToStdOut) return "-";

  sys::Path OFDir(OutFilesDir);
  OFDir.appendComponent(Name);
  OFDir.appendSuffix(Suffix);

  return std::string(OFDir.c_str());
}

FileInfo &llvm::vtmfiles() {
  static FileInfo Files;

  return Files;
}
