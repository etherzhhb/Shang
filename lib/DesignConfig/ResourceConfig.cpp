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

//
typedef xml_node<> XmlNode;

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

unsigned HWResource::getLeastBusyInstance() const {
  // {Least using count, idx}
  std::pair<unsigned, unsigned> ret(0, 0);
  for (unsigned i = 0, e = UsingCount.size(); i != e; ++i) {
    if (UsingCount[i] == 0)
      return i + 1;
    if (UsingCount[i] < ret.first) {
      ret.first = UsingCount[i];
      ret.second = i;
    }
  }
  return ret.second + 1;
}

void HWResource::clear() {
  UsingAtoms.clear();
  for (unsigned i = 0, e = UsingCount.size(); i != e; ++i)
    UsingCount[i] = 0;
}

//===----------------------------------------------------------------------===//
// Resource parsing

static char *getSubNodeAsString(XmlNode *Node, std::string name) {
  assert(Node && "Node can not be null!");
  XmlNode *SubNode = Node->first_node(name.c_str());
  if (SubNode == 0)
    report_fatal_error("Can not parse " + name + "!\n");

  return SubNode->value();
}

static unsigned getSubNodeAsInteger(XmlNode *Node, std::string name) {
  unsigned ret;
  StringRef val = StringRef(getSubNodeAsString(Node, name));
  if (val.getAsInteger(0, ret))
    report_fatal_error("Not integer node: " + name + "!\n");

  return ret;
}

HWMemBus * HWMemBus::createFromXml(xml_node<> Node) {
  return 0;
}

//===----------------------------------------------------------------------===//
/// Resource config implement
void ResourceConfig::initializePass() {
  ParseConfigFile(ConfigFilename);
  DEBUG(print(dbgs()));
}


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

  
  for (XmlNode *ResNode = xml.first_node("Resources");
      ResNode != 0; ResNode = ResNode->next_sibling("Resources")) {

    HWResource *Res =
      new HWResource(LogicUnit,
                     getSubNodeAsString(ResNode, "Name"),
                     getSubNodeAsInteger(ResNode, "TotalNum"),
                     getSubNodeAsInteger(ResNode, "Latency"),
                     getSubNodeAsInteger(ResNode, "StartInterval"));
    ResTab.insert(std::pair<std::string, HWResource*> (Res->getName(), Res));
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
