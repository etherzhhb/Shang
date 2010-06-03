#define DEBUG_TYPE "test"
#include "llvm/Function.h"
#include <sstream>
#include "llvm/Transforms/Scalar.h"
#include "llvm/Function.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Analysis/LoopInfo.h"




using namespace llvm;

STATISTIC(TestCounter, "Counts number of functions greeted");

namespace {
 
  struct Test : public FunctionPass{
    static char ID; 
    DenseMap<const Value*,unsigned> Anonnamenumber;
    unsigned Nextanonnamenumber;
    raw_ostream & Out;
    int level;

	explicit Test()
		:FunctionPass(&ID),Out(errs()),Nextanonnamenumber(0) {}

	virtual bool runOnFunction(Function &F) {
	  TestCounter++;
	
      Out<<testBechHeader(F,level);
	
	  return false;
	}
  
  virtual void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequiredID<VBEID>();
  }
  
	std::string testBechHeader(Function&F,int i){
		std::stringstream ss;
		int k=0;
		int x=0;
		int y=0;
		ss<<"module tb;\n";
    const Type *RetTy = F.getReturnType();
    assert((RetTy->isIntegerTy() || RetTy->isVoidTy()) && "Unsupport type.");
    ss<<" import \"DPI-C\" pure function int " << F.getNameStr() << "(";
		if (!F.arg_empty()) {
		  for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
		    k++;
		    ss<<"int ";
		    ss<<GetValueName(I);
		    if (k!=F.arg_size())
		      ss<<",";
		  }
		}
		ss<<");\n";
		ss<<"reg clk = 1'b0,\n";
		ss<<" rstN = 1'b0,\n";
		ss<<"start = 1'b0;\n";
		ss<<"wire [31:0] r1;\n";
		ss<<"reg[31:0]  r0 = 32'b0,";
		for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
		x++;
		ss<<GetValueName(I);
		ss<<"=";
		const Type*t=(*I).getType();
		int bit=cast<IntegerType>(*t).getBitWidth();
		ss<<bit<<"'b0";
		if (x!=F.arg_size())
		ss<<",";
		}
		ss<<";\n";
		ss<<"always\n";
		ss<<" #5ns clk = ~clk;\n";
		ss<<"initial begin\n";
		ss<<" #6ns rstN = 1'b1;\n";
		ss<<"end\n";
		ss<<" always begin\n";
		ss<<" @(negedge clk) begin\n";
		for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
		ss<<GetValueName(I);
		ss<<"<= $random();\n";

		}
		ss<<"end\n";
		for(int j=i;j!=1;j--){
		ss<<"@(negedge clk)\n";
		ss<<";\n";
		}
		ss<<"@(negedge clk)\n";
		ss<<"r0 <= add(";
		for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
		y++;
		ss<<GetValueName(I);
		if (y!=F.arg_size())
		ss<<",";
		}
		ss<<");;\n";
		ss<<"end\n";
		ss<<"add dut(";
		for (Function::const_arg_iterator I = F.arg_begin(), E = F.arg_end();I != E; ++I) {
		ss<<".";
		ss<<GetValueName(I)<<"(";
		ss<<GetValueName(I)<<"),";
		}
		ss<<".clk(clk),.reset(rstN),.Ret_add(r1));\n";
		ss<<"endmodule\n";
		return ss.str();
		}


 };
}


char Test::ID = 0;


static RegisterPass<Test> X("test", "test the module");

FunctionPass *llvm::createTestPass() {
	return new Test();
}
