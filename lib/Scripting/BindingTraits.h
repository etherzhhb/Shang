//====- BindingTraits.h - The Traits of Classes Bound to LUA ----*- C++ -*-===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file implement the traits for classes which are going to bind to lua.
//
//===----------------------------------------------------------------------===//
#ifndef VTM_BINDING_TRAITS_H
#define VTM_BINDING_TRAITS_H

#include "vtm/FUInfo.h"
#include "vtm/SystemInfo.h"
#include "vtm/VerilogAST.h"

#include "luabind/luabind.hpp"

namespace llvm {
template<class T> struct BindingTraits {
  // static inline luabind::class_<T> register_(const char *Name);

  // If anyone tries to use this class without having an appropriate
  // specialization, make an error.  If you get this error, it's because you
  // need to include the appropriate specialization of BindingTraits<> for your
  // class, or you need to define it for a new type that are going to bind to
  // luabind.
  //
  typedef typename T::UnknownLuaTypeError NodeType;
};

template<> struct BindingTraits<VASTPort> {
  static inline luabind::class_<VASTPort> register_(const char *Name) {
    luabind::class_<VASTPort> C(Name);
    C.def("getName", &VASTPort::getName);
    C.def("isInput", &VASTPort::isInput);
    C.def("getExternalDriverStr", &VASTPort::getExternalDriverStr);
    return C;
  }
};

template<> struct BindingTraits<VASTModule> {
  static inline luabind::class_<VASTModule> register_(const char *Name) {
    luabind::class_<VASTModule> C(Name);

    C.enum_("PortTypes")[
      luabind::value("Clk",               VASTModule::Clk),
        luabind::value("RST",               VASTModule::RST),
        luabind::value("Start",             VASTModule::Start),
        luabind::value("SpecialInPortEnd",  VASTModule::Clk),
        luabind::value("Finish",            VASTModule::Finish),
        luabind::value("SpecialOutPortEnd", VASTModule::SpecialOutPortEnd),
        luabind::value("NumSpecialPort",    VASTModule::NumSpecialPort),
        luabind::value("ArgPort",           VASTModule::ArgPort),
        luabind::value("Others",            VASTModule::Others),
        luabind::value("RetPort",           VASTModule::RetPort)
    ];

    // Bind functions.
    C.def("getName",                &VASTModule::getName);
    // All ports.
    C.def("getPort",                &VASTModule::getPort);
    C.def("getNumPorts",            &VASTModule::getNumPorts);

    C.def("getCommonPort",          &VASTModule::getCommonPort);
    C.def("getNumCommonPorts",      &VASTModule::getNumCommonPorts);

    C.def("getArgPort",             &VASTModule::getArgPort);
    C.def("getNumArgPorts",         &VASTModule::getNumArgPorts);

    C.def("getPortName",            &VASTModule::getPortName);

    return C;
  }
};
} // end namespace llvm

#endif
