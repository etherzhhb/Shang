SCIFGScript = [=[
// Include the headers.
$('#')include "systemc.h"
$('#')include "verilated.h"
$('#')include <iostream>
$('#')include <fstream>
using namespace std;

// GlobalVariables
$('#')ifdef __cplusplus
extern "C" {
$('#')endif

#for k,v in pairs(GlobalVariables) do
void *vlt_$(escapeNumber(k))() {
  $(if v.isLocal == 1 then _put('static') else _put('extern')
    end) $(getType(v.ElemSize)) $(k)$(if v.NumElems > 1 then  _put('[' .. v.NumElems .. ']') end) $(if v.Initializer ~= nil then
    _put(' = {')
    for i,n in ipairs(v.Initializer) do
      if i ~= 1 then _put(', ') end
      _put(n)
      if v.ElemSize == 32 then _put('u')
      elseif v.ElemSize > 32 then _put('ull')
      end
    end
    _put('}')
  end);
  return (void *)$(if v.NumElems == 1 then  _put('&') end)$(k);
}

#end

// Wrapper functions.
void *verilator_memset(void *src, int v, long long num) {
  return memset(src, v, num);
}

void *verilator_memcpy(void *dst, void *src, long long num) {
  return memcpy(dst, src, num);
}

void *verilator_memmove(void *dst, void *src, long long num) {
  return memmove(dst, src, num);
}

// Dirty Hack: Only C function is supported now.
$('#')ifdef __cplusplus
}
$('#')endif

]=]

SCIFFScript = [=[
#if Functions[FuncInfo.Name] ~= nil then
#local RTLModuleName = Functions[FuncInfo.Name].ModName
// And the header file of the generated module.
$('#')include "V$(RTLModuleName).h"
static int cnt = 0;
static int memcnt = 0;

//Including the functions which used in the tb
$('#')ifdef __cplusplus
extern"C"{
$('#')endif
$(getType(FuncInfo.ReturnSize)) $(FuncInfo.Name)_if($(
  for i,v in ipairs(FuncInfo.Args) do
    if i ~= 1 then _put(', ') end
    _put(getType(v.Size) .. ' '.. v.Name)
  end
 ));
#if FuncInfo.Name ~= "main" then
  int sw_main();
#end

$('#')ifdef __cplusplus
}
$('#')endif

//Top module here
SC_MODULE(V$(RTLModuleName)_tb){
  public:
    sc_in_clk clk;
    sc_signal<bool> fin;
    sc_signal<bool> mem0en;
    sc_signal<uint32_t> mem0cmd;
    sc_signal<uint32_t> mem0be;
    $(GetRetPort(FuncInfo.ReturnSize));
    sc_signal<uint$(FUs.MemoryBus.AddressWidth)_t> mem0addr;
    sc_signal<uint64_t> mem0out;
    sc_signal<bool> rstN;
    sc_signal<bool> start;
    sc_signal<bool> mem0rdy;

    sc_signal<bool> mem0waitrequest;

    sc_signal<uint64_t>mem0in;
#for i,v in ipairs(FuncInfo.Args) do
    sc_signal<$(SetBitWids(v.Size))>$(v.Name);
#end

    V$(RTLModuleName) DUT;
    
    void sw_main_entry(){
      V$(RTLModuleName)_tb *tb_ptr = V$(RTLModuleName)_tb::Instance();
      wait(); tb_ptr->rstN = 0;
      wait(10); tb_ptr->rstN = 1;
      wait();
#if FuncInfo.Name ~= "main"	then
			sw_main();
#else
			$(getType(FuncInfo.ReturnSize)) RetVle = $(FuncInfo.Name)_if($(
				for i,v in ipairs(FuncInfo.Args) do
					if i ~= 1 then _put(', ') end
					_put(v.Name)
				end
			 ));
		  assert(RetVle == 0 && "The main function don't have a return value!");
#end
      ofstream outfile;
      outfile.open ("$(CounterFile)"); 
      outfile <<"$(RTLModuleName) hardware run cycles " << cnt << " wait cycles " << memcnt <<endl;
      outfile.close();
      outfile.open ("$(BenchmarkCycles)", ios_base::app); 
      outfile <<",\n{\"name\":\"$(RTLModuleName)\", \"total\":" << cnt << ", \"wait\":" << memcnt << '}' <<endl;
      outfile.close();
      sc_stop();
    }
    //Memory bus function
    void bus_transation(){
      mem0waitrequest = 0;
      while (true){
        if(mem0en){
          unsigned CyclesToWait = 0;
		  unsigned char cur_be = mem0be.read(), addrmask = 0;
		  long long cur_addr = mem0addr.read();

          if(mem0cmd){
            CyclesToWait = 2;
            switch (cur_be){
            case 1:  *((unsigned char *)(cur_addr)) = ((unsigned char ) (mem0out.read()));   addrmask = 0; break;
            case 3:  *((unsigned short *)(cur_addr)) = ((unsigned short ) (mem0out.read())); addrmask = 1; break;
            case 15: *((unsigned int *)(cur_addr)) = ((unsigned int ) (mem0out.read()));     addrmask = 3; break;
            case 255: *((unsigned long long *)(cur_addr)) = ((unsigned long long ) (mem0out.read())); addrmask = 7; break;
            default: assert(0 && "Unsupported size!"); break;
            }
          }else {
            CyclesToWait = 1;
            switch (cur_be){
            case 1:  (mem0in) = *((unsigned char *)(cur_addr));  addrmask = 0; break;
            case 3:  (mem0in) = *((unsigned short *)(cur_addr)); addrmask = 1; break;
            case 15: (mem0in) = *((unsigned int *)(cur_addr));   addrmask = 3; break;
            case 255: (mem0in)= *((unsigned long long *)(cur_addr)); addrmask = 7; break;
            default: assert(0 && "Unsupported size!"); break;
            }

            assert((cur_addr & addrmask) == 0 && "Unexpected unalign access!");
          }

          for (unsigned i = 0; i < CyclesToWait; ++i) {
            mem0waitrequest = 1;
            ++memcnt;
            wait();
            assert(!mem0en && "Please disable memory while waiting it ready!");
          }

          mem0waitrequest = 0;
          wait();
        } else {
          mem0in = 0xcdcdcdcdcdcdcdcd;
          // Wait for next cycle.
          wait();
        }
      }
    }

    void memrdyLogic() {
      mem0rdy = !(mem0en || mem0waitrequest);
    }

    static V$(RTLModuleName)_tb* Instance() { static V$(RTLModuleName)_tb _instance("top") ; return &_instance ; }
    protected:
    //Include the DUT in the top module
      SC_CTOR(V$(RTLModuleName)_tb): DUT("DUT"){
        DUT.clk(clk);
#for i,v in ipairs(FuncInfo.Args) do
        DUT.$(v.Name)($(v.Name));
#end        
        DUT.fin(fin);
        //whether there is a return value
#  if FuncInfo.ReturnSize~=0 then
        DUT.return_value(return_value);  
#else
        
#  end
        DUT.start(start);
        DUT.rstN(rstN);
        DUT.mem0en(mem0en);
        DUT.mem0cmd(mem0cmd);
        DUT.mem0rdy(mem0rdy);
        DUT.mem0in(mem0in);
        DUT.mem0out(mem0out);
        DUT.mem0be(mem0be);
        DUT.mem0addr(mem0addr);
        SC_CTHREAD(sw_main_entry,clk.pos());
        SC_CTHREAD(bus_transation,clk.pos());
        SC_METHOD(memrdyLogic);
        sensitive << mem0en << mem0waitrequest;
      }
    private:
      V$(RTLModuleName)_tb(const V$(RTLModuleName)_tb&) ;
      V$(RTLModuleName)_tb& operator=(const V$(RTLModuleName)_tb&) ;
    };  

  $(getType(FuncInfo.ReturnSize)) $(FuncInfo.Name)_if($(
    for i,v in ipairs(FuncInfo.Args) do
      if i ~= 1 then _put(', ') end
      _put(getType(v.Size) .. ' '.. v.Name)
    end
  )){
    V$(RTLModuleName)_tb *tb_ptr = V$(RTLModuleName)_tb::Instance();
#for i,v in ipairs(FuncInfo.Args) do
    tb_ptr->$(v.Name)=$(v.Name);
#end       
    tb_ptr->start=1;
    wait(); tb_ptr->start=0;
    while(!(tb_ptr->fin)){
      wait();
      ++cnt;
    }
    
    //printf("$(RTLModuleName) finish\n");
#  if FuncInfo.ReturnSize~=0 then
    return ($(getType(FuncInfo.ReturnSize))) tb_ptr->return_value;      
#    else 
    return;
#  end
    }    
//The main function called sc_main in SystemC  
  int sc_main(int argc, char **argv) {
    //Close the information which provided by SystemC
    sc_report_handler::set_actions("/IEEE_Std_1666/deprecated", SC_DO_NOTHING);
    
    Verilated::commandArgs(argc,argv);
    sc_clock clk ("clk",10, 0.5, 3, true);
    V$(RTLModuleName)_tb *top = V$(RTLModuleName)_tb::Instance();
    //Link the stimulate to the clk
    top->clk(clk);
    //Start the test
    sc_start();

    return 0;
  }  
#end  
]=]

Passes.SCIFCodegen = { FunctionScript = [=[
local IfFile = assert(io.open (IFFileName, "a+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=SCIFFScript, output=IfFile}
if message ~= nil then print(message) end
IfFile:close()
]=], GlobalScript =[=[
local IfFile = assert(io.open (IFFileName, "w"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=SCIFGScript, output=IfFile}
if message ~= nil then print(message) end
IfFile:close()
]=]}
