//===-==-- vtm/FileInfo.h - The FileInfo manage the output files ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file define the FileInfo pass, which manage the various output files
// of the backend.
//
//===----------------------------------------------------------------------===//

#ifndef VTM_FILE_INFO_H
#define VTM_FILE_INFO_H
#include "llvm/ADT/SmallVector.h"

#include <string>

namespace llvm {
class VTargetMachine;
class tool_output_file;

class FileInfo{
  // DO NOT IMPLEMENT
  FileInfo(const FileInfo&);
  // DO NOT IMPLEMENT
  const FileInfo &operator=(const FileInfo&);

  // Write all contents to stdout, for debug use.
  bool WriteAllToStdOut;

  // The directory for output files. 
  std::string OutFilesDir;

  // The The name of the hardware sub system. 
  std::string SystemName;

  SmallVector<tool_output_file*, 2> OpenedFiles;

  // Configuration accessor.
  std::string getOutFilePath(const std::string &Name,
                             const std::string &Suffix) const;

  tool_output_file *getOutFile(const std::string &Name,
                               const std::string &Suffix,
                               unsigned Flags = 0);

  bool writeAllToStdOut() const {
    return SystemName.empty() || WriteAllToStdOut;
  }

  void setOutFilesDir(const std::string &Val);

  const std::string &getOutFilesDir() const {
    return OutFilesDir;
  }

  friend struct ConstraintsParser;
public:
  FileInfo() : WriteAllToStdOut(false) {}
  ~FileInfo();

  const std::string &getSystemName() const {
    return SystemName;
  }

  std::string getHWSubSysName() const {
    return SystemName + "_RTL";
  }

  tool_output_file *getRTLOut() { return getOutFile("_RTL", "v"); }

  tool_output_file *getIFDvrOut() { return getOutFile("_IF", "cpp"); }

  tool_output_file *getSWOut() { return getOutFile("_SW", "ll"); }
};

FileInfo &vtmfiles();

}

#endif
