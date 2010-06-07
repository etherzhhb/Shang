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
#include "listScheduler.h"
#include "instPriority.h"

using namespace xVerilog;

    resourceUnit::resourceUnit(string name, unsigned int id, unsigned int streamNum):
        m_name(name),m_id(id),m_seq(streamNum) {
            this->enlargeResourceTable(40);
        }

    void resourceUnit::enlargeResourceTable(unsigned int newSize) {
        assert(newSize < 1000000 && "Piecetable should not exceed 1M items");
        for (unsigned int j=0; j<m_seq.size(); j++) { // for each stream
            for (unsigned int i=m_seq[j].size(); i<newSize+1;i++) {  // insert blank cycles
                m_seq[j].push_back(InstructionCycle());
            }
        }
    }

    unsigned int resourceUnit::length() {
        unsigned int max = 0;
        for (unsigned int i=0; i<m_seq.size(); i++) {
            // the length of the max stream
            max = std::max(max, length(i));
        }
        return max;
    }
    unsigned int resourceUnit::length(unsigned int streamID) {
        unsigned int len = 0;
        // for both streams, look for the last opcode
        for (unsigned int i=0; i<m_seq[streamID].size();i++) {
            if (0 != m_seq[streamID][i].size()) {
                // opcode in here!
                len = i +1;  //size if last opcode index plus one
            }
        }
        return len; 
    }


    bool resourceUnit::hasInstruction(Instruction* inst) {

        for (unsigned int sq=0; sq<m_seq.size();sq++) { // stream
            for (unsigned int i=0; i<m_seq[sq].size();i++) { // cycle
                for (unsigned int j=0; j<m_seq[sq][i].size(); j++) { // instruction in cycle
                    if (inst == m_seq[sq][i][j]) return true;
                }
            }
        }
        return false; 
    }

    InstructionCycle resourceUnit::getInstructionForCycle(unsigned int cycleNum) {
        InstructionCycle cycle;
        for (unsigned int sq=0; sq<m_seq.size();sq++) { // stream
            this->enlargeResourceTable(cycleNum);
            assert(m_seq[sq].size() > cycleNum && "Cycle num is to large");
            cycle.insert(cycle.begin(), m_seq[sq][cycleNum].begin(), m_seq[sq][cycleNum].end());
        }
        return cycle;
    }

    bool resourceUnit::emptyAt(unsigned int streamID, unsigned int cycle) {
        this->enlargeResourceTable(cycle);
        assert(cycle < m_seq[streamID].size() && "placing instruction out of piecetable");
        return (0 == m_seq[streamID][cycle].size());
    }

    unsigned int resourceUnit::getBestSchedulingCycle(abstractHWOpcode* op) {

        unsigned int start = op->getFirstSchedulableSlot();
        // "other units" may schedule on any available slot
        if (this->getName() == "other") return start;


        // search for an available slot:
        // move starting slot one forward and try to schedule slot
        while (true) {
            bool fit = true;
            for (unsigned int strm=0; strm < m_seq.size(); strm++) {
                for(unsigned int i=0; i<op->getLength(); i++) {
                    // if opcode cycle is not empty and also resource is not empty at this point
                    if (!op->emptyAt(strm, i) && !emptyAt(strm, i+start)) {
                        // then this place is not available
                        fit = false;
                    }
                }
            }//stream
            if (fit) {
                return start;
            }
            ++start;
        }
    }

    void resourceUnit::place(abstractHWOpcode* op, unsigned int place) {

        this->enlargeResourceTable(place);
        assert(place < m_seq[0].size() && "placing instruction out of piecetable");

        op->place(place, getId());
        // for each stream in the opcode
        for (unsigned int strm=0; strm < m_seq.size(); strm++) {
            // for each cycle in the hardware opcode
            for(unsigned int i=0; i<op->getLength(); i++) {
                // for each operation
                InstructionCycle cycle = op->cycleAt(strm,i);
                for (InstructionCycle::iterator it = cycle.begin(); it!=cycle.end();it++) {
                    // add the operation to the local sequence of operations
                    m_seq[strm][place+i].push_back(*it);
                }
            } 
        }
    }

    string resourceUnit::toString() {
        std::stringstream sb;
        for (unsigned strm=0; strm<m_seq.size(); strm++) {

            // check if this instruction unit is empty
            bool empty_unit = true;
            for (unsigned int j=0; j<m_seq[strm].size();j++) {
                if (0!=m_seq[strm][j].size()) empty_unit = false;
            }
            if (empty_unit) continue; 

            sb<<getName()<<"_"<<strm<<"    \t";
            for (unsigned int j=0; j<30;j++) {
                unsigned int sz = m_seq[strm][j].size();
                if (0==sz) {
                    sb<<"."; 
                } else if (sz<10) {
                    sb<<m_seq[strm][j].size();
                } else {
                    sb<<"*"; 
                }
            }
            sb<<"\n";
        }
        return sb.str();
    }

    unsigned int resourceUnit::takenSlots() {
        unsigned int slots = 0;
        for (unsigned j=0; j<m_seq.size(); j++) {
            // for all streams, look at all opcodes
            for (unsigned int i=0; i<m_seq[j].size();i++) {
                if (0 != m_seq[j][i].size()) {
                    // opcode in here!
                    slots++;
                }
            }
        }
        return slots;
    }


    resourceUnit* resourceUnit::getLeastBusyResource(vector<resourceUnit*> availableResources) {
        resourceUnit* best = NULL;
        unsigned int less = 10000;
        for (vector<resourceUnit*>::iterator un = availableResources.begin(); 
                un!=availableResources.end();++un) {
            if ((*un)->takenSlots() < less) {
                less = (*un)->takenSlots();
                best = *un;
            }
        }
        return best;
    }


    /// list scheduler below

listScheduler::listScheduler(BasicBlock* BB,
                             TargetData* TD, GVRegistry *GVR)
                             :  TD(TD), m_bb(BB) { //JAWAD
  Function *F = BB->getParent();
  for (Function::const_arg_iterator I = F->arg_begin(), E = F->arg_end();
      I != E; ++I) {
    // integers:
    const Type *ArgTy = I->getType();
    if (ArgTy->isPointerTy())
      addResource("mem_" + I->getNameStr(),
                  ResourceConfig::getResConfig("memport"));
  }

  addResource("mul", ResourceConfig::getResConfig("mul"));
  addResource("div", ResourceConfig::getResConfig("div"));
  addResource("shl", ResourceConfig::getResConfig("shl"));
  addResource("other",1);

  scheduleBasicBlock(BB, GVR);
}

    vector<Instruction*> listScheduler::getInstructionForCycle(unsigned int cycleNum) {
        vector<Instruction*> ret;
        // for each unit in the processor
        for (vector<resourceUnit*>::iterator it = m_units.begin(); it!=m_units.end(); ++it) {
            // for each of the collection of cycles from the different units
            InstructionCycle cycle = (*it)->getInstructionForCycle(cycleNum);
            for (InstructionCycle::iterator ins = cycle.begin(); ins!=cycle.end(); ++ins) {
                ret.push_back(*ins);
            }
        }
        return ret; 
    }




    vector<assignPartEntry*> listScheduler::getAssignParts() {
        vector<assignPartEntry*> ret;
        // for each scheduled abstractHWOpcode
        for (vector<abstractHWOpcode*>::iterator it=m_ops.begin(); it!=m_ops.end();++it) {
            if ((*it)->getAssignPart()) {
                ret.push_back((*it)->getAssignPart());
            }
        }
        return ret; 
    }



    unsigned int listScheduler::length() {
        unsigned int len = 0;
        for(vector<resourceUnit*>::iterator it = m_units.begin(); it!=m_units.end(); ++it) {
            len = std::max(len, (*it)->length());
        }
        return len;
    }



    unsigned int listScheduler::getResourceIdForInstruction(Instruction* inst) {
        for(vector<resourceUnit*>::iterator it = m_units.begin(); it!=m_units.end(); ++it) {
            if ((*it)->hasInstruction(inst)) return (*it)->getId();
        }
        errs()<<"unable to find the resource unit for instruction "<<inst<<"\n";
        abort();
        return 0;
    }


    void listScheduler::addResource(string name, unsigned int count) {
        for (unsigned int i=0; i<count;i++)
            m_units.push_back(new resourceUnit(name, i, 2));
    }




    /// This is the heart of the list scheduler, the "do-it-all" algorithem

    void listScheduler::scheduleBasicBlock(BasicBlock* BB, GVRegistry *GVR) {
        // create the "abstract Hardware Opcodes"

        instructionPriority prioritizer(BB);
        InstructionVector order = prioritizer.getOrderedInstructions();

        for (InstructionVector::iterator I = order.begin(), E = order.end(); I != E; ++I) {
            abstractHWOpcode *op = new abstractHWOpcode(*I, toPrintable(BB->getName()),GVR, 2,TD); //JAWAD
            m_ops.push_back(op);
            // establish dependencies with previously generated opcodes
            for (vector<abstractHWOpcode*>::iterator depop = m_ops.begin(); depop!= m_ops.end(); ++depop) {
                op->addDependencyIfDepends(*depop);
            }
        }


        // populate the opcoded in the scheduling table
        // This is the actual scheduling of the abstract opcodes
        // For each vector, place it in best location
        for (vector<abstractHWOpcode*>::iterator depop = m_ops.begin(); depop!= m_ops.end(); ++depop) {
            vector<resourceUnit*> availableUnits;
            resourceUnit* best_unit = NULL;
            unsigned int best_loc = 10000;
            // for each resource unit 
            for (vector<resourceUnit*>::iterator un = m_units.begin(); un!=m_units.end();++un) {
                // if this is the correct type of unit
                if ((*un)->isSameUnitType(*depop)) {
                    unsigned int res = (*un)->getBestSchedulingCycle(*depop);
                    if (res < best_loc) {
                        best_loc = res;
                        availableUnits.clear(); 
                        availableUnits.push_back(*un);
                    } else if (res == best_loc) {
                        availableUnits.push_back(*un);
                    }
                } // if matches
            }// foreach unit

            // find the resourceUnit which has the least scheduled instructions
            // we do this to create muxes which are balanced.
            best_unit = resourceUnit::getLeastBusyResource(availableUnits);

            // After finding the best resource unit, schedule this opcode there
            assert(best_unit && "Unable to find best unit to schedule the opcode");
            if ((*depop)->isMustBeLastOpcode()) {
                //errs()<<"placing at "<<length()-1<<" or "<<best_loc<<" inside:\n "<<best_unit->toString()<<"\n";
                unsigned int cur_max_len = ((length()>0) ? length()-1 : 0) ;
                best_unit->place(*depop, std::max(cur_max_len, best_loc));
            } else {
                // place this abstractHWOpcode in the right cycles
                best_unit->place(*depop, best_loc);
            }
        }// for each opcode 

        //print list Scheduler
        // debug
        errs()<<"---=="<<this->getBB()->getName()<<"["<<this->length()<<"]==---\n";
        for (vector<resourceUnit*>::iterator un = m_units.begin(); un!=m_units.end();++un) {
            errs()<<(*un)->toString();
        }

        errs()<<"resource usage: "
            <<"memport:"<<getMaxResourceUsage("mem")<<" "
            <<"mul:"<<getMaxResourceUsage("mul")<<" "
            <<"div:"<<getMaxResourceUsage("div")<<" "
            <<"shl:"<<getMaxResourceUsage("shl")<<"\n";

    }//method


    unsigned int listScheduler::getMaxResourceUsage(std::string resourceName) {

        // holds number of the correct unit type
        unsigned int units=0; 

        // find which units are of the correct type 
        // and what is the unit with the lowest cycle count;
        for (vector<resourceUnit*>::iterator it = m_units.begin(); it != m_units.end(); ++it) {
            if ((*it)->getName().find(resourceName) != std::string::npos) {
                if ((*it)->takenSlots() >0) {
                    units++;
                }
            }
        }
        return units;
    }
