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
#include "vtm/SynSettings.h"
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

template<> struct BindingTraits<VASTNode> {
  template<class CurClass>
  static inline luabind::class_<CurClass> register_(const char *Name) {
    luabind::class_<CurClass> C(Name);
    C.def("getName", &CurClass::getName);
    return C;
  }
};

template<> struct BindingTraits<VASTValue> {
  template<class CurClass>
  static inline luabind::class_<CurClass> register_(const char *Name) {
    luabind::class_<CurClass> C =
      BindingTraits<VASTNode>::register_<CurClass>(Name);
    C.def("getBitWidth", &CurClass::getBitWidth);
    C.def("isRegister", &CurClass::isRegister);
    return C;
  }
};

template<> struct BindingTraits<VASTPort> {
  template<class CurClass>
  static inline luabind::class_<CurClass> register_(const char *Name) {
    luabind::class_<CurClass> C =
      BindingTraits<VASTValue>::register_<CurClass>(Name);
    C.def("isInput", &CurClass::isInput);
    C.def("getExternalDriverStr", &CurClass::getExternalDriverStr);
    return C;
  }

  static inline luabind::class_<VASTPort> register_(const char *Name) {
    return register_<VASTPort>(Name);
  }
};

template<> struct BindingTraits<VASTModule> {
  template<class CurClass>
  static inline luabind::class_<CurClass> register_(const char *Name) {
    luabind::class_<CurClass> C =
      BindingTraits<VASTNode>::register_<CurClass>(Name);

    C.enum_("PortTypes")[
      luabind::value("Clk",               CurClass::Clk),
        luabind::value("RST",               CurClass::RST),
        luabind::value("Start",             CurClass::Start),
        luabind::value("SpecialInPortEnd",  CurClass::Clk),
        luabind::value("Finish",            CurClass::Finish),
        luabind::value("SpecialOutPortEnd", CurClass::SpecialOutPortEnd),
        luabind::value("NumSpecialPort",    CurClass::NumSpecialPort),
        luabind::value("ArgPort",           CurClass::ArgPort),
        luabind::value("Others",            CurClass::Others),
        luabind::value("RetPort",           CurClass::RetPort)
    ];
    // All ports.
    C.def("getPort",                &CurClass::getPort);
    C.def("getNumPorts",            &CurClass::getNumPorts);
    // Common ports
    C.def("getCommonPort",          &CurClass::getCommonPort);
    C.def("getNumCommonPorts",      &CurClass::getNumCommonPorts);
    // ArgPorts
    C.def("getArgPort",             &CurClass::getArgPort);
    C.def("getNumArgPorts",         &CurClass::getNumArgPorts);

    C.def("getPortName",            &CurClass::getPortName);

    return C;
  }

  static inline luabind::class_<VASTModule> register_(const char *Name) {
    return register_<VASTModule>(Name);
  }
};
} // end namespace llvm

#endif
