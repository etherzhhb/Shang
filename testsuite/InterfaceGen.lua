function getGVBit(Num)
  if Num <= 2 then        return 1
  elseif Num <= 4 then    return 2
  elseif Num <= 8 then    return 3
  elseif Num <= 16 then    return 4
  elseif Num <= 32 then    return 5
  elseif Num <= 64 then    return 6
  elseif Num <= 128 then    return 7
  elseif Num <= 256 then    return 8
  elseif Num <= 512 then    return 9
  elseif Num <= 1024 then    return 10
  elseif Num <= 2048 then    return 11
  elseif Num <= 4096 then    return 12
  elseif Num <= 8192 then    return 13
  elseif Num <= 16384 then    return 14
  else                    return 15
  end
end
InterfaceGen =[=[
#local table_size_tmp = # LineTotal
#local Num64GV = LineTotal[table_size_tmp] + 1
///////////////////////////////////////////////////////////////////////
//------------------------Experiment module--------------------------//
///////////////////////////////////////////////////////////////////////
module DUT_TOP(
	input clk,
	input rstN,
	input start,
	output [7:0] LED7
);

wire   					    fin;
wire  [31:0] 			  return_value;
wire   					    mem0en;
wire  [3:0] 			  mem0cmd;
wire  [31:0] 			  mem0addr;
wire   					    mem0rdy;
wire  [7:0]			    mem0be;
wire  [$(getGVBit(Num64GV)-1):0] 			  addr2R;   //////////////////////////////
wire  [7:0] 			  byteenable;
wire  [63:0] 			  data2R;
wire  [63:0] 			  q_i;
wire  [63:0] 			  mem0in;
wire   					    wren;
wire	[63:0]        mem0out;
wire					      start_N =~start;

$(RTLModuleName) $(RTLModuleName)_inst(
    .clk(clk),
    .rstN(rstN),
    .start(start_N),
    .fin(fin),
    .return_value(return_value),
    .mem0en(mem0en),
    .mem0cmd(mem0cmd),
    .mem0addr(mem0addr),
    .mem0in(mem0in),
    .mem0out(mem0out),
    .mem0be(mem0be),
    .mem0rdy(mem0rdy)
);
	 
Main2Bram i1(
	.addr2R(addr2R),
	.clk(clk),
	.mem0addr(mem0addr),
	.mem0cmd(mem0cmd),
	.mem0en(mem0en),
	.mem0rdy(mem0rdy),
	.mem0be(mem0be),
	.mem0out(mem0out),
	.mem0in(mem0in),
	.q_i(q_i),
	.data2R(data2R),
	.byen2R(byteenable),
	.rstN(rstN),
	.wren(wren),
	.fin(fin),
	.return_value(return_value),
	.LED7(LED7)
);

BRAM i2( 
	.waddr(addr2R),
	.raddr(addr2R),
	.be(byteenable),
	.wdata(data2R), 
	.we(wren), 
	.clk(clk),
	.q(q_i)
);

endmodule

//-_-------------------------Interface module for Bram-----------------------------_-// 
//-_-------------------------Interface module for Bram-----------------------------_-// 
//-_-------------------------Interface module for Bram-----------------------------_-// 
module Main2Bram(
  //_---------Signal from IP----------------------//
	input 									clk,
	input 									rstN,
	input										fin,
	input 									mem0en,
	input 		  [3:0] 			mem0cmd,
	input				[7:0]				mem0be,
	input 		  [31:0] 			mem0addr,
	input				[63:0]   	 	mem0out,//
	input       [31:0]			return_value,
  //--------Signal from Bram----------------------//
	input				[63:0]    	q_i,
	//_-------Linking LED to show the correct-------// 
	output reg	[7:0]				LED7,	
  //--------Signal to IP--------------------------//
	output            			mem0rdy,
	output    	[63:0]    	mem0in,
  //--------Signal to Bram------------------------//
	output      [7:0]				byen2R,
	output 	 	  [$(getGVBit(Num64GV)-1):0] 			addr2R,  ///////////////////////////////////
	output      [63:0]   	  data2R,
	output     							wren
  );
//-=======================================================================================
//Some declarn
//-=======================================================================================
//The process of Reading 
//-=======================================================================================
parameter 				S0 = 1'b0,
									S_wait = 1'b1;			 
//-=======================================================================================
reg       				state;
reg [31:0] 				addr2R_read;
reg      					readrdy;
reg [7:0] 			 	readbyte_en;
reg               rden;
//-=======================================================================================
//-Active signal start the read process
//-=======================================================================================
wire  	 					readactive = mem0en && (mem0cmd==0)? 1:0;
wire		[7:0]			mem0be_wire = mem0be << mem0addr[2:0];
wire    [63:0]    q;
//-=======================================================================================
assign          q[7:0] = (rden&&byen2R[0])? q_i[7:0]:0;
assign          q[15:8] = (rden&&byen2R[1])? q_i[15:8]:0;
assign          q[23:16] = (rden&&byen2R[2])? q_i[23:16]:0;
assign          q[31:24] = (rden&&byen2R[3])? q_i[31:24]:0;
assign          q[39:32] = (rden&&byen2R[4])? q_i[39:32]:0;
assign          q[47:40] = (rden&&byen2R[5])? q_i[47:40]:0;
assign          q[55:48] = (rden&&byen2R[6])? q_i[55:48]:0;
assign          q[63:56] = (rden&&byen2R[7])? q_i[63:56]:0;            
//-=======================================================================================
assign          mem0in = q >> {addr2R_read[2:0],3'b0};
 
always@(posedge clk, negedge rstN)begin
	if(!rstN)begin
		state <= S0;
		addr2R_read <= 0;
		readrdy <= 0;
		readbyte_en <= 8'b1111_1111;
		rden <= 0;
	end else begin
		case(state)
			S0 :begin//Get the read or write data when mem0en turns to high
				if(readactive)begin
					addr2R_read <= mem0addr;
					state <= S_wait;
					readbyte_en <= mem0be_wire;
					rden <= 1;
				end else begin
					addr2R_read <= 0;
					state <= S0;
					readbyte_en <= 8'b1111_1111;
					readrdy <= 0;
					rden <= 0;
				end
			end
			S_wait :begin

				state <= S0;//Write process is less a cycle to Read process
				readrdy <= 1;
			end
			default : state <= S0;			
		endcase
	end
end

//-=======================================================================================
//The process of Writing 
//
//-Active the wren signal 
//-=======================================================================================
wire 						writeactive = (mem0en&&mem0cmd)? 1:0;
wire            writerdy = writeactive ? 1 : 0;
wire	 [7:0]		writebyte_en = writeactive? (mem0be<<mem0addr[2:0]):8'b1111_1111;
assign     		  data2R = writeactive? (mem0out<<{mem0addr[2:0],3'b0}):0;
assign          wren = writeactive? 1:0;
assign 					mem0rdy = ((readrdy)||(writerdy))? 1:0;
assign 					addr2R = (wren)? mem0addr[$(getGVBit(Num64GV)+2):3]:addr2R_read[$(getGVBit(Num64GV)+2):3];/////////////////////////////////
assign					byen2R = (wren)? writebyte_en:readbyte_en;
//-=======================================================================================
//-=======================================================================================
//Return the value
//-=======================================================================================
always@(posedge clk,negedge rstN)begin
	if(!rstN)begin
		LED7 <= 8'b10101010;
	end else begin
		if(fin)begin
			if(return_value==0)begin
				LED7 <= 8'b11111111;
			end else begin
				LED7 <= 8'b00000000;
			end	
		end else begin
			LED7 <= LED7;
		end
	end
end

endmodule

]=]

Passes.InterfaceGen = { FunctionScript = [=[
if Functions[FuncInfo.Name] ~= nil then
end
]=], GlobalScript =[=[
table_name = {}
table_num = {}
LineTotal = {}
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=BlockRAMInitFileGenScript}
local IntfFile = assert(io.open (INTFFILE, "a+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=InterfaceGen, output=IntfFile}
if message ~= nil then print(message) end
IntfFile:close()
]=]}

BRAMGen = [=[
#local table_size_tmp = # LineTotal
#local Num64GV = LineTotal[table_size_tmp] + 1
module BRAM
	$('#')(parameter int
		ADDR_WIDTH = $(getGVBit(Num64GV)),                       ///////////////////////////
		BYTE_WIDTH = 8,
		BYTES = 8,
		WIDTH = BYTES * BYTE_WIDTH
)
( 
	input [ADDR_WIDTH-1:0] waddr,
	input [ADDR_WIDTH-1:0] raddr,
	input [BYTES-1:0] be,
	input [WIDTH-1:0] wdata, 
	input we, clk,
	output reg [WIDTH - 1:0] q
);
	localparam int WORDS = 1 << ADDR_WIDTH ;

	// use a multi-dimensional packed array to model individual bytes within the word
	logic [BYTES-1:0][BYTE_WIDTH-1:0] ram[0:WORDS-1];
  
  // Add the initial file in ram
  initial	begin
  $('$')readmemb("$(RTLModuleName)_BramInit.txt",ram);
  end	

	always_ff@(posedge clk)
	begin
		if(we) begin
		// edit this code if using other than four bytes per word
			if(be[0]) ram[waddr][0] <= wdata[7:0];
			if(be[1]) ram[waddr][1] <= wdata[15:8];
			if(be[2]) ram[waddr][2] <= wdata[23:16];
			if(be[3]) ram[waddr][3] <= wdata[31:24];
      if(be[4]) ram[waddr][4] <= wdata[39:32];
			if(be[5]) ram[waddr][5] <= wdata[47:40];
			if(be[6]) ram[waddr][6] <= wdata[55:48];
			if(be[7]) ram[waddr][7] <= wdata[63:56];
	end
		q <= ram[raddr];
	end
endmodule : BRAM

]=]

Passes.BRAMGen = { FunctionScript = [=[
if Functions[FuncInfo.Name] ~= nil then
end
]=], GlobalScript =[=[
table_name = {}
table_num = {}
LineTotal = {}
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=BlockRAMInitFileGenScript}
local BramFile = assert(io.open (BRAMFILE, "a+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=BRAMGen, output=BramFile}
if message ~= nil then print(message) end
BramFile:close()
]=]}

DUTtbGen = [=[
`timescale 1 ps/ 1 ps
module DUT_TOP_tb();
reg clk;
reg rstN;
reg start;
wire [7:0] LED7;
reg startcnt;

DUT_TOP i1 (
	.clk(clk),
	.rstN(rstN),
  .start(start),
	.LED7(LED7)
);
  integer wfile,wferror,wtmpfile;
  initial wfile = $('$')fopen("$(CounterFile)");
  initial wtmpfile = $('$')fopen("$(BenchmarkCycles)","a");

initial
begin
clk = 0;rstN = 1; start = 1; startcnt = 0;
$('#')6 rstN = 0;
$('#')10 rstN = 1;start = 0;
$('#')10 start = 1;startcnt= 1;
end

always
$('#')5 clk = ~clk;

reg [31:0] cnt = 0;
always@(posedge clk)begin
  if(startcnt)begin
    if(i1.i1.fin)begin
      $('$')fwrite (wfile,"$(RTLModuleName) hardware run cycles %0d\n",cnt);
      $('$')fwrite (wtmpfile,",\n{\"name\":\"$(RTLModuleName)\", \"total\": %0d, \"wait\": 1}",cnt);
      if(i1.i1.return_value == 0)begin
        $('$')display ("The result is correct~");
      end else begin
        $('$')display ("The result is wrong!!!");
        wferror = $('$')fopen("$(INTFFILE)","a");
        $('$')fwrite (wferror,"there is a error");
        $('$')fclose(wferror);
      end
      $('$')fclose(wfile);
      $('$')fclose(wtmpfile);
      $('#')1000 $('$')stop;
    end else begin
      cnt <= cnt + 1;
    end
  end
end

endmodule
]=]

Passes.DUTtbGen = { FunctionScript = [=[
if Functions[FuncInfo.Name] ~= nil then
end
]=], GlobalScript =[=[
local tbFile = assert(io.open (TBFILE, "a+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=DUTtbGen, output=tbFile}
if message ~= nil then print(message) end
tbFile:close()
]=]}

ModelsimScGen = [=[
$('#')!/bin/sh
PATH=~/altera/10.1/modelsim_ase/bin/:~/altera/modelsim_ase/bin/:$PATH
export DISPLAY=:0
vlib work
vlog +define+quartus_synthesis $(RTLModuleName).v
vlog INTF_$(RTLModuleName).v
vlog DUT_TOP_tb.v
vlog BRAM.sv
vsim DUT_TOP_tb -novopt -do "run -all;quit -f"

]=]

Passes.ModelsimScGen = { FunctionScript = [=[
if Functions[FuncInfo.Name] ~= nil then
end
]=], GlobalScript =[=[
local ModelDoFile = assert(io.open (MODELDOFILE, "a+"))
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=ModelsimScGen, output=ModelDoFile}
if message ~= nil then print(message) end
ModelDoFile:close()
]=]}
