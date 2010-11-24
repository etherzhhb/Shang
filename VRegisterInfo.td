//===- CTVRegisterInfo.td - CTV Register defs ----------*- tblgen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the CTV register file
//===----------------------------------------------------------------------===//

class VTMReg<string n> : Register<n> {
  let Namespace = "VTM";
}

//===----------------------------------------------------------------------===//
//  Registers
//===----------------------------------------------------------------------===//

def D1 : VTMReg<"dr1">;
def D8 : VTMReg<"dr8">;
def D16 : VTMReg<"dr16">;
def D32 : VTMReg<"dr32">;
def D64 : VTMReg<"dr64">;
def PR : VTMReg<"pr">;

class DynRegClass<string namespace, list<ValueType> regTypes, int alignment,
                    list<Register> regList>
  : RegisterClass<namespace, regTypes, alignment, regList> {
  let MethodProtos = [{
    SmallVector<unsigned, 128> PhyRegs;
    
    iterator allocation_order_begin(const MachineFunction &MF) const {
      return PhyRegs.begin();
    }
    
    iterator allocation_order_end(const MachineFunction &MF) const {
      return PhyRegs.end();
    }
    
    iterator       begin() const { return PhyRegs.begin(); }
    iterator         end() const { return PhyRegs.end(); }
  
    bool count(unsigned RegNo) const {
      if(PhyRegs.empty())
       return false;
      
      if(PhyRegs.front() > RegNo || PhyRegs.back() < RegNo)
        return false;
        
      return true;
    }
  
    EVT getVT() const { return *vt_begin(); }
    
    void clear() { PhyRegs.clear(); }
    void createPhyRegs(unsigned &PhyRegBase, unsigned NumPhyRegs) {
      while(NumPhyRegs-- > 0) {
        RegSet.insert(PhyRegBase);
        PhyRegs.push_back(PhyRegBase++);
      }
    }
  }];  
}

def DR1   : DynRegClass<"VTM", [i1], 1, [D1]>;
def DR8   : DynRegClass<"VTM", [i8], 8, [D8]>;
def DR16  : DynRegClass<"VTM", [i16], 16, [D16]>;
def DR32  : DynRegClass<"VTM", [i32], 32, [D32]>;
def DR64  : DynRegClass<"VTM", [i64], 64, [D64]>;

// Pointer is default to i64 type.
def PR64 : RegisterClass<"VTM", [i64], 64, [PR]>;
