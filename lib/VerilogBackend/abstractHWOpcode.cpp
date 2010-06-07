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
#include "abstractHWOpcode.h"

/// assign part entry impl

namespace xVerilog{

    void assignPartEntry::setUnit(unsigned int unitid, const string& name,
            const string& stateName, unsigned int cycle) {
        m_unitId = unitid;
        m_unitName = name;
        stringstream sb;
        sb<<stateName<<cycle;
        m_state = sb.str();
    }

    string assignPartEntry::getUnitName() {
        stringstream sb;
        sb <<m_unitName<<m_unitId;
        return sb.str();
    }


    /// abstractHWOpcode impl

    void abstractHWOpcode::addBinaryInstruction(Instruction *inst, string op, unsigned int delay) { 
        // TODO: if operands of this bin command are not 32bit then we need
        // to cast them.               

        m_opcodeName = op;
        // get global register from global storage 
        GlobalValue* BinO = GVR->getGVByName(string("out_")+op,32);

        // create the assign part multiplexer for the the binary unit
        m_assignPart = new assignPartEntry(inst->getOperand(0), inst->getOperand(1));

        // load instruction from the value of the assign part
        LoadInst* ld = new LoadInst(BinO);

        // replace all uses of bin with the new load command insted
        // of the multiplication command, it will be done in the assign
        // part
        unsigned int bitWidth0 = cast<IntegerType>(ld->getType())->getBitWidth();
        unsigned int bitWidth1 = cast<IntegerType>(inst->getType())->getBitWidth();
        if (bitWidth0 == bitWidth1) {
            inst->replaceAllUsesWith(ld);
        } else if(bitWidth0 > bitWidth1) {
                        CastInst* t = new TruncInst(ld, IntegerType::get(inst->getContext(), bitWidth1),"trAssign");
                        inst->replaceAllUsesWith(t);
        } else if(bitWidth0 < bitWidth1) { 
                        CastInst* c = new SExtInst(ld, IntegerType::get(inst->getContext(), bitWidth1),"exAssign");
                        inst->replaceAllUsesWith(c);
        }
        

        InstructionCycle nop;
        InstructionCycle cycle0;
        InstructionCycle cycleN;
        cycleN.push_back(ld);
        this->appendInstructionCycle(nop, 1);
        this->appendInstructionCycle(cycle0, 0);

        // add the empty cycles, this is based on the delay from the
        // configuration of this execution unit
        for (unsigned int i=0; i<(delay-1); i++) {
            this->appendInstructionCycle(nop, 1);
            this->appendInstructionCycle(nop, 0);
        }
        this->appendInstructionCycle(nop, 0);
        this->appendInstructionCycle(cycleN, 1);

        // Append all unused instruction for dependecy calculation
        // This will be used when calculating the next available slots
        // for the next command.
        this->appendUsedInstruction(inst);
        this->appendUsedInstruction(ld);
        return;
    }

    const StructType * abstractHWOpcode::isPtrToStructType(const Value* Op) { //JAWAD
    const Type* baseType = Op->getType();
    if(isa<PointerType>(baseType)){
        const Type *elemT =      cast<PointerType>(baseType)->getElementType();
        if(isa<PointerType>(elemT)){//is there anther level of pointers ( **P)
           baseType=elemT;
           elemT= cast<PointerType>(elemT)->getElementType();
        }
        return dyn_cast<StructType>(elemT);
    }
    return NULL;
}

static string getVarName(Value* Op){

	if(GetElementPtrInst* gep = dyn_cast<GetElementPtrInst>(Op)){
		if(gep->getNumOperands() ==3){
                	Value* Op3 = gep->getOperand(2);
			const ConstantInt  *C = dyn_cast<ConstantInt>(Op3);
		
			if( abstractHWOpcode::isPtrToStructType(Op)){
				//std::cout <<"isPtrToStructType\n"; ; std::cout.flush() ;
				return "field_"+C->getZExtValue();
			}
		}
	}
        return "VAR";	
}
    abstractHWOpcode::abstractHWOpcode(Instruction* inst, string stateName,GVRegistry *gvr,  unsigned int streamNum,
      TargetData* TD): 
        TD(TD),GVR(gvr), m_empty(false),m_place(0),m_stateName(stateName),m_iv(streamNum),m_mustBeLast(false) {
              
         
            // this may not be used
            m_assignPart = NULL;

            if (abstractHWOpcode::isInstructionOnlyWires(inst)) {
                m_opcodeName = "other";
                //empty instruction;
                InstructionCycle cycle;
                cycle.push_back(inst);
                appendInstructionCycle(cycle, 0);
                m_empty = true;
                return;
            }

            if (isa<PHINode>(inst)) {
                m_opcodeName = "other";
                //empty instruction;
                InstructionCycle cycle;
                cycle.push_back(inst);
                appendInstructionCycle(cycle, 0);
                m_empty = true;
                return;
            }


            if (LoadInst* ld = dyn_cast<LoadInst>(inst)) {
                ArrayInfo inf = getVariableNameFromMemoryCommand(ld);//JAWAD
                std::string arrName = inf.first;
                m_opcodeName = "mem_" + arrName;

                // store the mode
                StoreInst* l1 = new StoreInst(GVR->getZero(1), 
                        GVR->getGVByName("mem_"+arrName+"_mode",1,false)); 
                // store address first
                const Type* addr_type = inst->getOperand(0)->getType(); //JAWAD
		std::cout <<"ADDR_TYPE1 = : " << addr_type->getDescription() << "   arrName:  " << arrName <<"\n";
		std::cout <<"ADDR_TYPE2 = : " << addr_type->getDescription() << "   arrName:  " << arrName <<"\n";
                Value* port = GVR->getGVByName("mem_"+arrName+"_addr",addr_type);
                StoreInst* l2 = new StoreInst(inst->getOperand(0) ,port);
                // read answer after
                GVR->trashWhenDone(l1);
                GVR->trashWhenDone(l2);
        
                inst->setOperand(0,GVR->getGVByName("mem_"+arrName+"_out", inst->getOperand(0)->getType())); 
                InstructionCycle cycle0;
                InstructionCycle cycle1;
                cycle0.push_back(l1);
                cycle0.push_back(l2);
                cycle1.push_back(inst);
                InstructionCycle nop;

                this->appendInstructionCycle(nop, 1);
                this->appendInstructionCycle(cycle0, 0);

                // add memport delay
                for (unsigned int i=0, e = ResourceConfig::getResConfig("delay_memport") - 1;
                    i< e; i++) {
                  this->appendInstructionCycle(nop, 1);
                  this->appendInstructionCycle(cycle0, 0);
                }

                this->appendInstructionCycle(nop, 0);
                this->appendInstructionCycle(cycle1, 1);
                return;
            } else if (StoreInst *st = dyn_cast<StoreInst>(inst)) {
                ArrayInfo inf = getVariableNameFromMemoryCommand(st); //JAWAD
                std::string arrName = inf.first; 
                m_opcodeName = "mem_" + arrName;
                // store the data

    		const Type* typ0 =  inst->getOperand(0)->getType(); //JAWAD
		GlobalVariable*  v1;
                	v1 = GVR->getGVByName("mem_"+arrName+"_in", typ0);
		StoreInst* s1;
		//check if there is IntToPtrInst ...
		if (IntToPtrInst* i2p = dyn_cast<IntToPtrInst>(inst->getOperand(0))) {
                	s1 = new StoreInst(i2p->getOperand(0),v1); 

		}else{
                	s1 = new StoreInst(inst->getOperand(0),v1); 
		}
                // store the data write mode 
                StoreInst* s2 = new StoreInst(GVR->getOne(1), 
                        GVR->getGVByName("mem_"+arrName+"_mode",1));
                // store address
		GlobalVariable*  v2;

		getVarName( inst->getOperand(1));
      
                const Type* addr_type = inst->getOperand(1)->getType();
		v2 = GVR->getGVByName("mem_"+arrName+"_addr", addr_type);	
		StoreInst* s3 ;
                
		if (BitCastInst *BCI = dyn_cast<BitCastInst>((inst->getOperand(1)))){
                	s3 = new StoreInst(BCI->getOperand(0),v2); 
        	}else{ 
   
                	s3 = new StoreInst(inst->getOperand(1),v2); 
		}
                // store the data write mode 
                StoreInst* s4 = new StoreInst(GVR->getZero(1), 
                        GVR->getGVByName("mem_"+arrName+"_mode",1));

                InstructionCycle cycle0;
                InstructionCycle cycle1;

                GVR->trashWhenDone(s1);
                GVR->trashWhenDone(s2);
                GVR->trashWhenDone(s3);
                GVR->trashWhenDone(s4);

                cycle0.push_back(s1);
                cycle0.push_back(s2);
                cycle0.push_back(s3);
                cycle1.push_back(s4);
                 
                InstructionCycle nop;

                this->appendInstructionCycle(cycle0, 0);
                this->appendInstructionCycle(nop, 1);
                // add memport delay
                for (unsigned int i=0, e = ResourceConfig::getResConfig("delay_memport") - 1;
                    i < e; i++) {
                    this->appendInstructionCycle(cycle0, 1);
                    this->appendInstructionCycle(nop, 0);
                }
                this->appendInstructionCycle(cycle1, 1);
                this->appendInstructionCycle(nop, 0);
                return;
            } else if (isa<BinaryOperator>(inst)) {
                BinaryOperator* bin = (BinaryOperator*) inst;
                // in here, add a switch where 'add' and 'neg' will turn into verilog
                // key words. However, << and mod, div will turn into scheduling blocks,
                // similar to the 'load' instruction. 

                if ((bin->getOpcode()) == Instruction::Mul) {
                    addBinaryInstruction(bin, "mul", ResourceConfig::getResConfig("delay_mul"));
                    return;
                }
                if ((bin->getOpcode()) == Instruction::SDiv) {
                    addBinaryInstruction(bin, "div", ResourceConfig::getResConfig("delay_div"));
                    return;
                }
                if ((bin->getOpcode()) == Instruction::Shl) {
                    // do not create an assign part if this shift
                    // is by a constant  example: (a<<2)
                    if (!dyn_cast<Constant>(bin->getOperand(1))) {
                        addBinaryInstruction(bin, "shl", ResourceConfig::getResConfig("delay_shl"));
                        return;
                    }
                }

                m_opcodeName = "other";
            } // end of (is binOperator)

            m_opcodeName = "other";
            InstructionCycle cycle;
            cycle.push_back(inst);
            this->appendInstructionCycle(cycle, 0);
            return;
        } // Instruction c'tor



      InstructionCycle abstractHWOpcode::getInstructionForCycle(int cycleNum) {
        InstructionCycle cycle;
        for (vector<InstructionSequence>::iterator it = m_iv.begin(); it != m_iv.end(); ++it) {
            cycle.insert(cycle.begin(), (*it)[cycleNum].begin(), (*it)[cycleNum].end());
        }
        return cycle;
      }


    void abstractHWOpcode::addDependency(abstractHWOpcode* dep) {
        m_dependencies.insert(dep);
    }

    bool abstractHWOpcode::isDepends(abstractHWOpcode* dep) {
        return m_dependencies.find(dep)!=m_dependencies.end();
    }  

    unsigned int abstractHWOpcode::getLength() {
        if (m_empty) return 0;
        unsigned int max = 0;
        for (unsigned int i = 0; i<m_iv.size(); i++) {
            max = std::max(max, getLength(i));
        }
        return max;
    }

    unsigned int abstractHWOpcode::getFirstSchedulableSlot() {
        unsigned int slot = 0;
        for (set<abstractHWOpcode*>::iterator I = m_dependencies.begin();
                I != m_dependencies.end(); ++I) {
            slot = std::max((*I)->getFirstAfter() ,slot) ;
        }
        return slot;
    }  

    Instruction* abstractHWOpcode::hasInstruction(std::vector<Instruction *> &insts) {
        for (std::vector<Instruction*>::iterator it = insts.begin(); it != insts.end(); ++it) {
            if (Instruction* in = hasInstruction(*it)) { 
                // Found dep
                return in;
            }
        }
        return NULL;
    }

    Instruction* abstractHWOpcode::hasInstruction(Instruction *inst) {
        InstructionSequence::iterator cyc_it;
        InstructionCycle::iterator inst_it;

        // search this instruction in the list of instructions which
        // do not map to uOps (but participate in the dep calculation)
        if (m_usedInst.find(inst)!=m_usedInst.end()) return inst;

        for (unsigned i=0; i<m_iv.size(); i++) {
            // and then search in the instructions which make this command
            for (cyc_it = m_iv[i].begin(); cyc_it!=m_iv[i].end(); ++cyc_it) {
                for (inst_it = cyc_it->begin(); inst_it != cyc_it->end(); ++inst_it ) {
                    if (inst == *inst_it) return inst;
                }
            }
        }
        return NULL;
    }



    string abstractHWOpcode::toString() {
        std::stringstream sb;
        for (unsigned i=0; i<m_iv.size(); i++) {
            sb<<getName()<<"_"<<i<<"\t";
            for (unsigned int j=0; j<m_iv[i].size();j++) {
                sb<<m_iv[i][j].size()<<":";
            }
            sb<<"\n";
        }
        return sb.str();
    }

    void abstractHWOpcode::addDependencyIfDepends(abstractHWOpcode *hwop) {
        for (unsigned i=0; i<m_iv.size(); i++) {
            addDependencyIfDepends(hwop, m_iv[i]);
        }
    }

    void abstractHWOpcode::addDependencyIfDepends(abstractHWOpcode *hwop, InstructionSequence &iv) {
        // TODO: refactor
        InstructionSequence::iterator cyc_it;
        InstructionCycle::iterator inst_it; 
        if (this==hwop) return;

        // for each cycle in this opcode 
        for (cyc_it = iv.begin(); cyc_it!=iv.end(); ++cyc_it) {
            // for each instruction in this cycle
            for (inst_it = cyc_it->begin(); inst_it != cyc_it->end(); ++inst_it ) {
                /* 
                 * Note: 
                 * branch instructions must come last (because of the jump). 
                 * They do not simply depend on everything else because that
                 *  we we may loose a cycle if the condition may be calculated
                 *  way before the end of the BB and the branch can be merged
                 *  with the last instruction. However, in the last instrtuction 
                 *  we also have the coding of the PHI nodes which set some 
                 *  values for phi variavles of loops. In here we depend only
                 *  on opcodes which the next PHINode depends on.
                 */
                if (BranchInst* br = dyn_cast<BranchInst>(*inst_it)) {
                    // This opcode must come last
                    m_mustBeLast = true;
            
                    // We will check to see if our target BB PHINodes are dependent
                    // on this instruction. If we are dependent on them then we need 
                    // to be at least one cycle after them.
                    std::vector<Instruction*> v = getAllIncomingValuesFromBranch(br);
                    if (hwop->hasInstruction(v)) { 
                        addDependency(hwop);
                    }

                    // Now, we will continue and check if the condition depends 
                    // on any other opcodes.
                }

                /*
                 * The return instruction must come last. Also, a return is usually
                 * not inside a loop so we do not care about loosing a cycle or two. 
                 */
                if (isa<ReturnInst>(*inst_it)) this->addDependency(hwop);

                // for each dependency of each instruction in our opcode
                for (User::op_iterator dep_iter = (*inst_it)->op_begin();
                        dep_iter != (*inst_it)->op_end(); ++dep_iter){

                    // turn dependency to instruction
                    if (Instruction *inst_dep = dyn_cast<Instruction>(*dep_iter)){
                        // if op holds one of our dependencies, we depend on it
                        if (hwop->hasInstruction(inst_dep)) this->addDependency(hwop);
                    }
                }// each dep
            }//each inst
        }//each cycle

        // for each one of the non participating inst    
        for (set<Instruction*>::iterator inst_it = m_usedInst.begin(); 
                inst_it!=m_usedInst.end(); ++inst_it) {
            // for each dependency of our passive instructions
            for (User::op_iterator dep_iter = (*inst_it)->op_begin();
                    dep_iter != (*inst_it)->op_end(); ++dep_iter){
                // turn dependency to instruction
                if (Instruction *inst_dep = dyn_cast<Instruction>(*dep_iter)){
                    // if op holds one of our dependencies, we depend on it
                    if (hwop->hasInstruction(inst_dep)) this->addDependency(hwop);
                }
            }// each dep
        }
    }



    void abstractHWOpcode::place(unsigned int cycle,unsigned int unitid) {
        m_place = cycle;
        if (m_assignPart) m_assignPart->setUnit(unitid, getName(),m_stateName, cycle);
    }


    bool abstractHWOpcode::isInstructionOnlyWires(Instruction* inst) {
        if (isa<TruncInst>(inst)) return true;
        if (isa<IntToPtrInst>(inst)) return true;
        if (isa<PtrToIntInst>(inst)) return true;
        if (isa<ZExtInst>(inst)) return true;
        if (isa<SExtInst>(inst)) return true;
        if (isa<TruncInst>(inst)) return true;
        if (isa<CallInst>(inst)) return true;
        // FIXME: this is not true!
        if (isa<GetElementPtrInst>(inst)) return true;

        
        if (inst->hasOneUse()) {
            // Inline expressions in return values. We can't handle PHINodes, so 
            // don't inline them. 
            // Do 
            if ( (dyn_cast<ReturnInst>(inst->use_begin()) && 
                  dyn_cast<LoadInst>(inst->use_begin()) && 
                  dyn_cast<StoreInst>(inst->use_begin())) 
                    //Don't
                    && !dyn_cast<PHINode>(inst) 
                    && !dyn_cast<LoadInst>(inst)) {
                return true;
            }            
        }


        if (BinaryOperator *calc = dyn_cast<BinaryOperator>(inst)) {
            // small logic gates are also count as wires ...
            // Do not sample and turn into clocks any operations
            // which are simply a few gates (we are going to get killed by
            // the MUX any ways)
            unsigned int bitWidth = cast<IntegerType>(calc->getType())->getBitWidth();
            if (bitWidth <= ResourceConfig::getResConfig("inline_op_to_wire")) {
                //errs()<<"Treating register as wire: "<<*calc;
                return true;
            }

            // if second parameter is a constant (shift by constant is wiring)
            Value* param1 = calc->getOperand(1);
            if (isa<ConstantInt>(param1)) {
                if (calc->getOpcode() == Instruction::Shl) return true;
                if (calc->getOpcode() == Instruction::LShr || 
                        calc->getOpcode() == Instruction::AShr) return true; 


            } 
            // if constant is truncated ...
            if (TruncInst* tr = dyn_cast<TruncInst>(param1)) {
                if (isa<ConstantInt>(tr->getOperand(0))) {
                    if (calc->getOpcode() == Instruction::Shl) return true;
                    if (calc->getOpcode() == Instruction::LShr || 
                            calc->getOpcode() == Instruction::AShr) return true; 

                }
            }   

        }
        return false;
    }

    std::vector<Instruction*> abstractHWOpcode::getAllIncomingValuesFromPHIs(BasicBlock* bb) {
        std::vector<Instruction*> incomings;

        // For each instruction
        for (BasicBlock::iterator it = bb->begin(); it!= bb->end(); ++it) {
            // If it is a PHINode
            if (PHINode* phi = dyn_cast<PHINode>(it)) {
                //For each one of the Incoming
                int in = phi->getNumIncomingValues();
                for(int i=0; i< in; i++) {  
                    // If the incoming value is an instruction (and not a constant, etc)
                    if (Instruction* inst = dyn_cast<Instruction>(phi->getIncomingValue(i))) {
                        incomings.push_back(inst);
                    }
                }
            }
        }
        return incomings;
    }

    std::vector<Instruction*> abstractHWOpcode::getAllIncomingValuesFromBranch(BranchInst* br) {
        std::vector<Instruction*> incomings;

        assert(br && "BranchInst cannot be NULL");

        //For each one of the outgoing branches
        int out = br->getNumSuccessors();
        for(int i=0; i< out; i++) {  
            BasicBlock* bb = br->getSuccessor(i);
            std::vector<Instruction*> ins = getAllIncomingValuesFromPHIs(bb);
            incomings.insert(incomings.end(), ins.begin(), ins.end());
        }

        return incomings;
    }
    
    //TODO Jawad -  fix this function ... need to go up on the graph and see which function arrgument is the predecessor of this inst  
    static string get_array_port_name(Instruction* inst){ //JAWAD
 	Function* F = inst->getParent()->getParent();
      	for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end(); I != E; ++I) {
		std::string name = I->getName();
		const Type* baseType = I->getType();
    		if(isa<PointerType>(baseType)){
			return name;
		}
	}
	return "NULL";
     }
		 
        ArrayInfo abstractHWOpcode::getVariableNameFromMemoryCommand(Instruction* inst) {
        string suffix =""; //JAWAD
        Value* array = NULL;

        Value* param = NULL; 
        // If we have a Store/Load parameter then the array can be direct
        // or it can be via a GetElementPtrInst.
        if (StoreInst *st = dyn_cast<StoreInst>(inst)) {
            param = st->getOperand(1); // Store X to 'param'
        } else if (LoadInst *ld = dyn_cast<LoadInst>(inst)) {
            param = ld->getOperand(0); // Load 'param'
        }
        assert(param && "Memory access is not via Load or Store.");

        if (GetElementPtrInst* getptr = dyn_cast<GetElementPtrInst>(param)){
            // Our array is the first parameter of the GetElementPtrInst.
	    if( (getptr->getNumOperands() ==3) &&  abstractHWOpcode::isPtrToStructType(getptr->getPointerOperand())) { //JAWAD
                Value* Op2 = getptr->getOperand(2);
		const ConstantInt  *C = dyn_cast<ConstantInt>(Op2);
		stringstream idx_str;
		idx_str << C->getZExtValue();
		suffix = string("_field_")+idx_str.str();
	    };
            array = getptr->getOperand(0);
        } else {
  		 if (BitCastInst *BCI = dyn_cast<BitCastInst>(param)){ //JAWAD
            		// Our Load/Store reference the array directly
			array = BCI->getOperand(0);
        	} else{
		       if (IntToPtrInst* i2p = dyn_cast<IntToPtrInst>(param)) {
			    array = i2p->getOperand(0);  
		       }else{
            		    // Our Load/Store reference the array directly
            		    array = param;
		       }
		}
	}

        //errs()<<"Param:" << *param<<"\n"; 
        // If the param is from a PHINode, never mind. Pick any
        // hope that the first iteration reads the same value as the
        // body of the loop.
        // TODO: BUG in case of split PHI node of two basic blocks. 
        if (PHINode* phi = dyn_cast<PHINode>(array)){
            if (isa<Argument>(phi->getOperand(0))) {
                array = phi->getOperand(0);
            } else if (isa<Argument>(phi->getOperand(2))) {
                array = phi->getOperand(2);
            }
            assert(array && "Unable to find the array argument from the PHI node");
        } 

        assert(array && "Unable to find the name of the array we are accessing. \n"
                         "Unable to synthesize a memory port for it");

	unsigned NumBits = TD->getTypeSizeInBits(array->getType()); //JAWAD

        ArrayInfo p;
	string array_name = get_array_port_name(inst) + suffix; //JAWAD
	array_name = ResourceConfig::chrsubst(array_name,'.','_');
        p.first = array_name;
        p.second = NumBits;
        return p;
    }
     //JAWAD (remove abstractHWOpcode::getBitWidthFromType)
} // namespace
