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

#define RAPIDXML_NO_EXCEPTIONS
#include "rapidxml.hpp"

using namespace llvm;
using namespace esyn;
using namespace rapidxml;



void rapidxml::parse_error_handler(const char *what, void *where) {
  errs() << "Xml Pase error: " << what << '\n';
  llvm_unreachable("Error parsing resoure config xml");
}

void ResourceConfig::initializePass() {
  // Get target data

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
