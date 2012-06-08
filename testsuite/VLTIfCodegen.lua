VLTIFGScript = [=[
// Include the verilator header.
$('#')include "verilated.h"

// Current simulation time
static long sim_time = 0;

// Called by $time in Verilog
double sc_time_stamp () {
  return sim_time;
}

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

VLTIFFScript = [=[
#if Functions[FuncInfo.Name] ~= nil then
#local RTLModuleName = Functions[FuncInfo.Name].ModName
// And the header file of the generated module.
$('#')include "V$(RTLModuleName).h"


// Instantiation of module
#local RTLModuleNameInst = 'V' .. RTLModuleName .. '_Inst'
static V$(RTLModuleName) $(RTLModuleNameInst)("$(FuncInfo.Name)");

$('#')ifdef __cplusplus
extern "C" {
$('#')endif
$(getType(FuncInfo.ReturnSize)) $(FuncInfo.Name)_if($(
  for i,v in ipairs(FuncInfo.Args) do
    if i ~= 1 then _put(', ') end
    _put(getType(v.Size) .. ' '.. v.Name)
  end
 )) {
  $(RTLModuleNameInst).rstN = (1);
  // Reset the module if we first time invoke the module.
  if (sim_time == 0) {
    //Increase clk by half cycle.
    $(RTLModuleNameInst).clk = (sim_time++ & 0x1);
    // Evaluate model.
    $(RTLModuleNameInst).eval();
    $(RTLModuleNameInst).rstN = (0);
  }

  //Increase clk by half cycle.
  $(RTLModuleNameInst).clk = (sim_time++ & 0x1);
  // Evaluate model.
  $(RTLModuleNameInst).eval();
    // Deassert reset
  $(RTLModuleNameInst).rstN = (1);
  //Increase clk by half cycle.
  $(RTLModuleNameInst).clk = (sim_time++ & 0x1);
  // Evaluate model.
  $(RTLModuleNameInst).eval();

  //Increase clk by half cycle.
  $(RTLModuleNameInst).clk = (sim_time++ & 0x1);
  // Evaluate model.
  $(RTLModuleNameInst).eval();
  assert(!($(RTLModuleNameInst).fin)&& "Module finished before start!");

  // Setup the parameters.
#for i,v in ipairs(FuncInfo.Args) do
  $(RTLModuleNameInst).$(v.Name) = $(v.Name);
#end

  // Start the module.
  $(RTLModuleNameInst).start = (1);
  // Remember the start time.
  long start_time = sim_time;
  long ready_time = sim_time;
  // Commit the signals.
  //Increase clk by half cycle.
  $(RTLModuleNameInst).clk = (sim_time++ & 0x1);
  // Evaluate model.
  $(RTLModuleNameInst).eval();

  // The main evaluation loop.
  do{
    // Check for if the clk rising
    if (( (~sim_time) & 0x1) == 1) {
      $(RTLModuleNameInst).mem0rdy = ((sim_time >= ready_time));
      if (($(RTLModuleNameInst).mem0en)){ // If membus0 active
$('#')ifdef __DEBUG_IF
        unsigned long long Addr = ($(RTLModuleNameInst).mem0addr);
        printf("Bus active with address %x\n", Addr);
$('#')endif
        if (($(RTLModuleNameInst).mem0we)){ // This is a write
          switch (($(RTLModuleNameInst).mem0be) & 0xff){
            case 1: *((unsigned char *)($(RTLModuleNameInst).mem0addr)) = ((unsigned char ) ($(RTLModuleNameInst).mem0out)); break;
            case 3: *((unsigned short *)($(RTLModuleNameInst).mem0addr)) = ((unsigned short ) ($(RTLModuleNameInst).mem0out)); break;
            case 15: *((unsigned int *)($(RTLModuleNameInst).mem0addr)) = ((unsigned int ) ($(RTLModuleNameInst).mem0out)); break;
            case 255: *((unsigned long long *)($(RTLModuleNameInst).mem0addr)) = ((unsigned long long ) ($(RTLModuleNameInst).mem0out)); break;
            default: assert(0 && "Unsupported size!"); break;
          }
        } else { // This is a read
          switch (($(RTLModuleNameInst).mem0be) & 0xff){
            case 1: ($(RTLModuleNameInst).mem0in) = *((unsigned char *)($(RTLModuleNameInst).mem0addr)); break;
            case 3: ($(RTLModuleNameInst).mem0in) = *((unsigned short *)($(RTLModuleNameInst).mem0addr)); break;
            case 15: ($(RTLModuleNameInst).mem0in) = *((unsigned int *)($(RTLModuleNameInst).mem0addr)); break;
            case 255: ($(RTLModuleNameInst).mem0in) = *((unsigned long long *)($(RTLModuleNameInst).mem0addr)); break;
            default: assert(0 && "Unsupported size!"); break;
          }
        } // end read/write
      } // end membus0
      // Simulate the ready port of the membus.
      if (($(RTLModuleNameInst).mem0rdy) && ($(RTLModuleNameInst).mem0en)){ // If membus0 ready for next transaction
        // Update the ready time of membus.
        ready_time = sim_time + 2 + 5;
$('#')ifdef __DEBUG_IF
        printf("Memory Active at %x going to ready at %x\n", sim_time, ready_time);
$('#')endif
      } // end next transaction membus0
    } // end clk rising

    // Check if the module finish its job at last.
    if (($(RTLModuleNameInst).fin)) {
      return $(if FuncInfo.ReturnSize ~= 0 then _put('(' .. getType(FuncInfo.ReturnSize) .. ')' .. RTLModuleNameInst .. '.return_value') end);
    }

    //Increase clk by half cycle.
    $(RTLModuleNameInst).clk = (sim_time++ & 0x1);
    // Evaluate model.
    $(RTLModuleNameInst).eval();
    $(RTLModuleNameInst).start = (0);
  } while(!Verilated::gotFinish() && (sim_time - start_time) <  16000000);

  assert(0 && "Something went wrong during the simulation!");
  return $(if FuncInfo.ReturnSize ~= 0 then _put('0') end);
}
// Dirty Hack: Only C function is supported now.
$('#')ifdef __cplusplus
}
$('#')endif
#end
]=]

Passes.VLTIFCodegen = { FunctionScript = [=[
local IfFile = assert(io.open (IFFileName, "a+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=VLTIFFScript, output=IfFile}
if message ~= nil then print(message) end
IfFile:close()
]=], GlobalScript =[=[
local IfFile = assert(io.open (IFFileName, "w"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=VLTIFGScript, output=IfFile}
if message ~= nil then print(message) end
IfFile:close()
]=]}

