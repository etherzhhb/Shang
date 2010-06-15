/*
* Copyright: 2008 by Nadav Rotem. all rights reserved.
* IMPORTANT: This software is supplied to you by Nadav Rotem in consideration
* of your agreement to the following terms, and your use, installation, 
* modification or redistribution of this software constitutes acceptance
* of these terms.  If you do not agree with these terms, please do not use, 
* install, modify or redistribute this software. You may not redistribute, 
* install copy or modify this software without written permission from 
* Nadav Rotem. 
*/

#include "vbe/ResourceConfig.h"
#include "vbe/HWAtom.h"

#include "llvm/ADT/StringRef.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MemoryBuffer.h"

#define DEBUG_TYPE "vbe-res-config-file"
#include "llvm/Support/Debug.h"

#define RAPIDXML_NO_EXCEPTIONS
#include "rapidxml.hpp"
#include "rapidxml_iterators.hpp"
#include "rapidxml.hpp"

using namespace llvm;
using namespace esyn;
using namespace rapidxml;

//===----------------------------------------------------------------------===//
/// Xml stuff
static cl::opt<std::string>
ConfigFilename("vbe-res-config-file",
               cl::desc("vbe - The resource config file."));

void rapidxml::parse_error_handler(const char *what, void *where) {
  errs() << "Xml Pase error: " << what << '\n';
  report_fatal_error("Error parsing resoure config xml");
}

//===----------------------------------------------------------------------===//
/// Hardware resource.
void HWResource::print(raw_ostream &OS) const {
  OS << "Resource: " << Name << '\n';
  OS.indent(2) << "TotalNum: " << TotalRes << '\n';
  OS.indent(2) << "Latency: " << Latency << '\n';
  OS.indent(2) << "StartInterval: " << StartInt << '\n';
}

void HWResource::clear() {
  UsingAtoms.clear();
  CycMap.clear();
}

//===----------------------------------------------------------------------===//
/// Resource config implement
void ResourceConfig::initializePass() {
  ParseConfigFile(ConfigFilename);
  print(dbgs());
}

#define DefXmlNode(Parent, Name); \
  XmlNode *XmlNode##Name = ResNode->first_node(#Name); \
  if (!XmlNode##Name) report_fatal_error("Can not parse: "#Name); \
  StringRef Name##Val = XmlNode##Name->value();

#define GetAsInteger(Name, Result); \
  if (Name##Val.getAsInteger(0, Result)) \
    report_fatal_error("Not integer: "#Name);

void ResourceConfig::ParseConfigFile(const std::string &Filename) {
  std::string ErrMsg;
  MemoryBuffer *F = MemoryBuffer::getFile(Filename, &ErrMsg);
  if (!F)
    report_fatal_error(Filename + ": Can not open config file!");

  // Get target data
  memory_pool<> pool;
  // Create the string, do not forget the null terminator.
  char *context = pool.allocate_string(F->getBufferStart(),
                                       F->getBufferSize() + 1);
  DEBUG(dbgs() << context << '\n');
  xml_document<> xml;
  xml.parse<0>(context);

  typedef xml_node<> XmlNode;
  
  for (XmlNode *ResNode = xml.first_node("Resources");
      ResNode != 0; ResNode = ResNode->next_sibling("Resources")) {
    DefXmlNode(ResNode, Name);
    DefXmlNode(ResNode, TotalNum);
    DefXmlNode(ResNode, Latency);
    DefXmlNode(ResNode, StartInterval);
    
    //
    unsigned totalNum;
    GetAsInteger(TotalNum, totalNum);
    
    unsigned latency ;
    GetAsInteger(Latency, latency);

    unsigned startInterval;
    GetAsInteger(StartInterval, startInterval);

    HWResource *Res = new HWResource(NameVal, latency, startInterval, totalNum);
    ResTab.insert(std::pair<std::string, HWResource*> (NameVal.str(), Res));
  }
}

void ResourceConfig::print(raw_ostream &OS) const {
  OS << "-=========================Resource Config=========================-\n";
  for (ResTabTy::const_iterator I = ResTab.begin(), E = ResTab.end();
      I != E; ++I) {
    I->second->print(OS);
    OS << '\n';
  }
}

ResourceConfig::~ResourceConfig() {
  while (!ResTab.empty()) {
    delete ResTab.begin()->second;
    ResTab.erase(ResTab.begin());
  }
}

char ResourceConfig::ID = 0;

static RegisterPass<ResourceConfig>
X("vbe-resource-config", "vbe - resource config", false, true);
