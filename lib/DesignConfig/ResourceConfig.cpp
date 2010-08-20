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
typedef xml_attribute<> XmlAttr;

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
void HWResType::print(raw_ostream &OS) const {
  // OS << "Resource: " << Name << '\n';
  OS.indent(2) << "TotalNum: " << TotalRes << '\n';
  OS.indent(2) << "Latency: " << Latency << '\n';
  OS.indent(2) << "StartInterval: " << StartInt << '\n';
}

//===----------------------------------------------------------------------===//
// Resource parsing
static enum HWResType::Types getResourceType(XmlNode *Node) {
  XmlAttr *attr = Node->first_attribute("type");
  if (attr == 0)
    return HWResType::AddSub;
  unsigned ret;
  StringRef val = StringRef(attr->value());
  if (val.getAsInteger(0, ret) || 
      (ret > HWResType::LastResourceType || ret < HWResType::FirstResourceType))
    report_fatal_error("Bad resource type!\n");
  
  return (HWResType::Types)ret;
}

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

HWMemBus *HWMemBus::createFromXml(XmlNode *Node) {
  assert(Node && "Node can not be null!");
  if (getSubNodeAsString(Node, "Name") != HWMemBus::getTypeName())
    report_fatal_error("Bad Resource name, not match " +  getTypeName());
  return new HWMemBus(getSubNodeAsInteger(Node, "Latency"),
                      getSubNodeAsInteger(Node, "StartInterval"),
                      getSubNodeAsInteger(Node, "TotalNum"),
                      getSubNodeAsInteger(Node, "AddressWidth"),
                      getSubNodeAsInteger(Node, "DataWidth"));
}

template<class BinOpResType>
BinOpResType *HWBinOpResType::createFromXml(XmlNode *Node) {
  assert(Node && "Node can not be null!");
  if (getSubNodeAsString(Node, "Name") != BinOpResType::getTypeName())
    report_fatal_error("Bad Resource name, not match " + BinOpResType::getTypeName());
  
  return new BinOpResType(//,
    getSubNodeAsInteger(Node, "Latency"),
    getSubNodeAsInteger(Node, "StartInterval"),
    getSubNodeAsInteger(Node, "TotalNum"),
    getSubNodeAsInteger(Node, "MaxBitWidth"));
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

  XmlNode *ResConfNode = xml.first_node("ResourcesConfiguration");
  if (!ResConfNode)
    report_fatal_error("Can not resource configuration!");

  for (XmlNode *ResNode = ResConfNode->first_node("Resource");
      ResNode != 0; ResNode = ResNode->next_sibling("Resource")) {
    HWResType *Res = 0;
    switch (getResourceType(ResNode)) {
    case HWResType::MemoryBus:
      Res = HWMemBus::createFromXml(ResNode);
      break;
    case HWResType::AddSub:
      Res = HWBinOpResType::createFromXml<HWAddSub>(ResNode);
      break;
    case HWResType::Mult:
      Res = HWBinOpResType::createFromXml<HWMult>(ResNode);
      break;
    case HWResType::SHL:
      Res = HWBinOpResType::createFromXml<HWSHL>(ResNode);
      break;
    case HWResType::ASR:
      Res = HWBinOpResType::createFromXml<HWASR>(ResNode);
      break;
    case HWResType::LSR:
      Res = HWBinOpResType::createFromXml<HWLSR>(ResNode);
      break;
    default:
      report_fatal_error("Unknow resource type!");
      break;
    }

    if (Res != 0) { // Only setup the available resources.
      unsigned idx = (unsigned)Res->getType()
                      - (unsigned)HWResType::FirstResourceType;
      ResSet[idx] = Res;
    }
  }
}

HWFUnit *ResourceConfig::allocaBinOpFU(HWResType::Types T, unsigned BitWitdh,
                                       unsigned UnitID) {
  FoldingSetNodeID ID;
  ID.AddInteger(T);
  ID.AddInteger(BitWitdh);
  ID.AddInteger(UnitID);

  void *IP = 0;
  HWFUnit *FU = UniqiueHWFUs.FindNodeOrInsertPos(ID, IP);
  if (FU) return FU;
  // TODO: Assert bit width smaller than max bit width.
  unsigned short Inputs[] = { BitWitdh, BitWitdh };
  unsigned short Outputs[] = { BitWitdh };
  HWBinOpResType *HWTy = cast<HWBinOpResType>(getResType(T));
  FU = new (HWFUAllocator) HWFUnit(ID.Intern(HWFUAllocator), T,
                                   HWTy->getTotalRes(), HWTy->getLatency(),
                                   Inputs, Inputs + 2, Outputs, Outputs + 1);
  UniqiueHWFUs.InsertNode(FU, IP);
  return FU;
}


HWFUnit *ResourceConfig::allocaMemBusFU(unsigned UnitID) {
  FoldingSetNodeID ID;
  ID.AddInteger(HWResType::MemoryBus);
  ID.AddInteger(UnitID);

  void *IP = 0;

  HWFUnit *FU = UniqiueHWFUs.FindNodeOrInsertPos(ID, IP);
  if (FU) return FU;

  HWMemBus *HWTy = getResType<HWMemBus>();
  unsigned short Inputs[] = { HWTy->getDataWidth(), HWTy->getAddrWidth() };
  unsigned short Outputs[] = { HWTy->getDataWidth() };
  FU = new (HWFUAllocator) HWFUnit(ID.Intern(HWFUAllocator), HWResType::MemoryBus,
                                   1, HWTy->getLatency(),
                                   Inputs, Inputs + 2, Outputs, Outputs + 1);
  UniqiueHWFUs.InsertNode(FU, IP);
  return FU;
}


HWFUnit *ResourceConfig::allocaTrivialFU(unsigned latency) {
  FoldingSetNodeID ID;
  ID.AddInteger(HWResType::Trivial);
  ID.AddInteger(latency);

  void *IP = 0;

  HWFUnit *FU = UniqiueHWFUs.FindNodeOrInsertPos(ID, IP);
  if (FU) return FU;

  unsigned short *Null = 0;
  FU = new (HWFUAllocator) HWFUnit(ID.Intern(HWFUAllocator), HWResType::Trivial,
                                   ~0, latency, Null, Null, Null, Null);
  UniqiueHWFUs.InsertNode(FU, IP);
  return FU;
  return 0;
}

void ResourceConfig::print(raw_ostream &OS) const {
  OS << "-=========================Resource Config=========================-\n";
  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if (*I != 0) {
      (*I)->print(OS);
      OS << '\n';
    }
  }
}

ResourceConfig::~ResourceConfig() {
  for (iterator I = begin(), E = end(); I != E; ++I)
    delete *I;

  HWFUAllocator.Reset();
  UniqiueHWFUs.clear();
}

char ResourceConfig::ID = 0;

static RegisterPass<ResourceConfig>
X("vbe-resource-config", "vbe - resource config", false, true);
