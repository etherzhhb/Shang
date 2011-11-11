local  AvalonTemplate = [=[
module  Avalon_user_logic(
  // -- ADD USER PORTS BELOW THIS LINE ---------------
  clk,                                   // Bus to IP clock
  reset_n,                               // Bus to IP reset_n
  avs_Address_Bus2IP,                    // Bus to IP address
  avs_Byteenable_Bus2IP,                 // Bus to IP byteenable
  avs_Write_Bus2IP,                      // Bus to IP write
  avs_Writedata_Bus2IP,                  // Bus to IP writedata
  avs_Read_Bus2IP,                       // Bus to IP read
  avs_Readdata_IP2Bus,                   // IP  to Bus readdate
  avs_Waitrequest_IP2Bus,                // IP  to Bus waitrequest

  avm_Readdata_Bus2IP,                   // Bus to IP master readdata
  avm_Waitrequest_Bus2IP,                // Bus to IP master waitrequest
  avm_Address_IP2Bus,                    // IP  to Bus master address
  avm_Byteenable_IP2Bus,                 // IP  to Bus master byteenable
  avm_Read_IP2Bus,                       // IP  to Bus master read
  avm_Write_IP2Bus,                      // IP  to Bus master write
  avm_Writedata_IP2Bus                   // IP  to Bus master writedata
); // user_logic

  // -- ADD USER PARAMETERS BELOW THIS LINE ------------
  parameter C_SLV_AWIDTH = /*dirty hack*/ 19;
  parameter C_SLV_DWIDTH = 32;
  parameter C_MST_AWIDTH = 32;
  parameter C_MST_DWIDTH = 32;

  // -- Bus protocol ports, do not add to or delete
  input                                clk;
  input                                reset_n;
  input      [C_SLV_AWIDTH-1:0]        avs_Address_Bus2IP;
  input      [C_SLV_DWIDTH/8-1:0]      avs_Byteenable_Bus2IP;
  input                                avs_Write_Bus2IP;
  input      [C_SLV_DWIDTH-1:0]        avs_Writedata_Bus2IP;
  input                                avs_Read_Bus2IP;
  output     [C_SLV_DWIDTH-1:0]        avs_Readdata_IP2Bus;
  output                               avs_Waitrequest_IP2Bus;

  input      [C_MST_DWIDTH-1:0]        avm_Readdata_Bus2IP;
  input                                avm_Waitrequest_Bus2IP;
  output     [C_MST_AWIDTH-1:0]        avm_Address_IP2Bus;
  output     [C_MST_DWIDTH/8-1:0]      avm_Byteenable_IP2Bus;
  output                               avm_Read_IP2Bus;
  output                               avm_Write_IP2Bus;
  output     [C_MST_DWIDTH-1:0]        avm_Writedata_IP2Bus;

//Implementation
//----------------------------------------------------------------------------
  //Slave output register
  wire        [C_SLV_DWIDTH-1:0]       avs_Readdata_IP2Bus;
  wire                                 avs_Waitrequest_IP2Bus;
  //Master output register
  reg        [C_MST_AWIDTH-1:0]        avm_Address_IP2Bus;
  reg        [C_MST_DWIDTH/8-1:0]      avm_Byteenable_IP2Bus;
  reg                                  avm_Read_IP2Bus;
  reg                                  avm_Write_IP2Bus;
  reg        [C_MST_DWIDTH-1:0]        avm_Writedata_IP2Bus;
  //IP_slave internal register
  reg                                  begin_wr;          //begin to write, pull down slave_waitrequest
  reg                                  begin_rd;          //begin to read, pull down slave_waitrequest
  reg        [C_SLV_DWIDTH-1:0]        slave_reg_0_start; //slave_reg_0, write a number(except 0) to start user logic
#-----------------------------------------------------------------------------------------------------
#local NumCommonPorts = CurModule:getNumCommonPorts()
#local NumArgPorts = CurModule:getNumArgPorts()
#for i = 0, NumArgPorts - 1 do
#local port = CurModule:getArgPort(i)
#local BitWidth = port:getBitWidth()
  reg        [$(BitWidth-1):0]                    $(port:getName());
#end
#-----------------------------------------------------------------------------------------------------
  //user logic net and register
  wire                                 start;
  wire                                 fin;
//---------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------
#for i = NumArgPorts , NumCommonPorts -1 do
#  local port = CurModule:getCommonPort(i)
#  local BitWidth = port:getBitWidth()
#  if (BitWidth == 1) then
  wire                                 $(port:getName());
#  else
  wire       [$(BitWidth-1):0]                    $(port:getName());
#  end
#end

//instantiate the user logic
//================================================================
  $(CurModule:getName()) dut_$(CurModule:getName())(
        .clk(clk),
        .rstN(reset_n),
        .start(start),
        .fin(fin),
#for i = 0, NumCommonPorts - 1 do
#  local port = CurModule:getCommonPort(i):getName()
        .$(port)($(port))$(if i ~= NumCommonPorts - 1 then _put(',') end)
#end
  );
//==================================================================
	//Block RAM initial below
  reg	  [9:0]   address_a;
	reg	  [9:0]   address_b;
	reg	  [3:0]   byteena_a;
	reg	  [3:0]   byteena_b;
	reg	  [31:0]  data_a;
	reg	  [31:0]  data_b;
	reg	          wren_a;
	reg	          wren_b;
	wire	[31:0]  q_a;
	wire	[31:0]  q_b;
  wire          ram_enable;

	altsyncram	altsyncram_component (
				.byteena_a (byteena_a),
				.clock0 (clk),
				.wren_a (wren_a),
				.address_b (address_b),
				.byteena_b (byteena_b),
        .clocken0 (ram_enable),
				.data_b (data_b),
				.wren_b (wren_b),
				.address_a (address_a),
				.data_a (data_a),
				.q_a (q_a),
				.q_b (q_b),
				.aclr0 (1'b0),
				.aclr1 (1'b0),
				.addressstall_a (1'b0),
				.addressstall_b (1'b0),
				.clock1 (1'b1),
				.clocken1 (1'b1),
				.clocken2 (1'b1),
				.clocken3 (1'b1),
				.eccstatus (),
				.rden_a (1'b1),
				.rden_b (1'b1));
	defparam
		altsyncram_component.address_reg_b = "CLOCK0",
		altsyncram_component.byteena_reg_b = "CLOCK0",
		altsyncram_component.byte_size = 8,
		altsyncram_component.clock_enable_input_a = "NORMAL",
		altsyncram_component.clock_enable_input_b = "NORMAL",
		altsyncram_component.clock_enable_output_a = "BYPASS",
		altsyncram_component.clock_enable_output_b = "BYPASS",
		altsyncram_component.indata_reg_b =  "CLOCK0",
    altsyncram_component.init_file = "Avalon.mif",
		altsyncram_component.intended_device_family = "Cyclone II",
		altsyncram_component.lpm_type = "altsyncram",
		altsyncram_component.numwords_a = 1024,
		altsyncram_component.numwords_b = 1024,
		altsyncram_component.operation_mode = "BIDIR_DUAL_PORT",
		altsyncram_component.outdata_aclr_a = "NONE",
		altsyncram_component.outdata_aclr_b = "NONE",
		altsyncram_component.outdata_reg_a = "UNREGISTERED",
		altsyncram_component.outdata_reg_b = "UNREGISTERED",
		altsyncram_component.power_up_uninitialized = "FALSE",
		altsyncram_component.read_during_write_mode_mixed_ports = "DONT_CARE",
		altsyncram_component.widthad_a = 10,
		altsyncram_component.widthad_b = 10,
		altsyncram_component.width_a = 32,
		altsyncram_component.width_b = 32,
		altsyncram_component.width_byteena_a = 4,
		altsyncram_component.width_byteena_b = 4,
		altsyncram_component.wrcontrol_wraddress_reg_b = "CLOCK0";
//==================================================================

  //latency counter
  reg [63:0] shang_count;
  reg shang_count_en;

  always @* begin
    if(start) shang_count_en = 1;
    if(fin) shang_count_en = 0;
  end

  always @(posedge clk) begin
    if (~reset_n) begin
      shang_count <= 0;
    end else begin
      if(shang_count_en)
        shang_count <= shang_count + 1;
      if(fin)
        $display("shang_count = %d", shang_count);
    end
  end

  //User IP access the Local BRam
  parameter LocalBRam_Idle = 3'b001,
            read_bram      = 3'b010,
            write_bram     = 3'b100;
  parameter BlockRamInterBase = 3'h1,
            BlockRamBase      = 16'h1201;
  reg [2:0] LocalBRamState;
  reg       BRam2Mem0rdy;
  reg       LocalBRam_en;
  always @(posedge clk) begin
    if (~reset_n) begin
      wren_b          <= 0;
      byteena_b       <= 0;
      address_b       <= 0;
      data_b          <= 32'hffffffff;
      LocalBRamState  <= LocalBRam_Idle;
      BRam2Mem0rdy    <= 0;
      LocalBRam_en    <= 0;
    end else begin
      case (LocalBRamState)
        LocalBRam_Idle: begin
          LocalBRam_en <= 0;
          if (mem0en && (mem0addr[31:16] == BlockRamBase)) begin
            address_b    <= mem0addr[9:0] >> 2;
            case(mem0cmd)
              4'b0000 : begin
                LocalBRamState <= read_bram;
                LocalBRam_en   <= 1;
              end
              4'b0001 : LocalBRamState <= write_bram;
              default : ;
            endcase
          end else begin
            BRam2Mem0rdy <= 0;
            wren_b       <= 0;
            byteena_b    <= 0;
          end
        end//end BramIdle

        read_bram: begin
          wren_b         <= 0;
          byteena_b      <= mem0be;
          BRam2Mem0rdy   <= 1;
          LocalBRamState <= LocalBRam_Idle;
          LocalBRam_en   <= 0;
        end

        write_bram: begin
          wren_b         <= 1;
          byteena_b      <= mem0be;
          data_b         <= mem0out;
          BRam2Mem0rdy   <= 1;
          LocalBRamState <= LocalBRam_Idle;
          LocalBRam_en   <= 1;
        end

        default: LocalBRamState <= LocalBRam_Idle;
      endcase
    end // end reset else
  end //end always

//==================================================================
  //Avalon Bus access the Local BRam
  parameter Bus_LocalBRam_Idle = 3'b001,
            Bus_read_bram      = 3'b010,
            Bus_write_bram     = 3'b100;
  reg [2:0] Bus_LocalBRamState;
  reg       Bus2Bram_rd;
  reg       Bus2Bram_en;
  always @(posedge clk) begin
    if (~reset_n) begin
      wren_a             <= 0;
      byteena_a          <= 0;
      address_a          <= 0;
      data_a             <= 32'hffffffff;
      Bus_LocalBRamState <= Bus_LocalBRam_Idle;
      Bus2Bram_rd        <= 0;
      Bus2Bram_en        <= 0;
    end else begin
      case (Bus_LocalBRamState)
        Bus_LocalBRam_Idle: begin
          Bus2Bram_en <= 0;
          if (~(avs_Address_Bus2IP[18:16] < BlockRamInterBase)) begin
            if(avs_Read_Bus2IP & avs_Waitrequest_IP2Bus) begin
              Bus2Bram_en        <= 1;
              Bus2Bram_rd        <= 1;
              Bus_LocalBRamState <= Bus_read_bram;
              address_a          <= avs_Address_Bus2IP[9:0];
            end
            if(avs_Write_Bus2IP) begin
              Bus_LocalBRamState <= Bus_write_bram;
              address_a          <= avs_Address_Bus2IP[9:0];
            end
          end else begin
            wren_a    <= 0;
            byteena_a <= 0;
          end
        end//end BramIdle

        Bus_read_bram: begin
          wren_a             <= 0;
          byteena_a          <= avs_Byteenable_Bus2IP;
          Bus2Bram_rd        <= 0;
          Bus_LocalBRamState <= Bus_LocalBRam_Idle;
          Bus2Bram_en        <= 0;
        end

        Bus_write_bram: begin
          wren_a             <= 1;
          Bus2Bram_en        <= 1;
          byteena_a          <= avs_Byteenable_Bus2IP;
          data_a             <= avs_Writedata_Bus2IP;
          Bus_LocalBRamState <= Bus_LocalBRam_Idle;
        end

        default: Bus_LocalBRamState <= Bus_LocalBRam_Idle;
      endcase
    end // end reset else
  end //end always

  //generating ram_enable signal to start BlockRam
  assign ram_enable = (LocalBRam_en | Bus2Bram_en);
//==================================================================
  //signal IP_slave waitrequest
  assign avs_Waitrequest_IP2Bus = ~((begin_wr | begin_rd) & (avs_Write_Bus2IP | avs_Read_Bus2IP) & (~Bus2Bram_rd));

  //IP_slave write process
  always@(posedge  clk) begin
    if(~reset_n)  begin
      begin_wr          <=  0;
      slave_reg_0_start <=  0;
#for i = 0, NumArgPorts - 1 do
#  local port = CurModule:getArgPort(i)
      $(port:getName()) <=  0;
#end
    end else begin
      begin_wr <= avs_Write_Bus2IP;
      if(avs_Write_Bus2IP && (avs_Address_Bus2IP[18:16] < BlockRamInterBase)) begin
        case(avs_Address_Bus2IP[3:0])
          4'd0 : begin
            if(avs_Byteenable_Bus2IP[0]) slave_reg_0_start[0] <= avs_Writedata_Bus2IP[0];
          end
#local num = 0;
#for i = 0, NumArgPorts - 1 do
#  local port = CurModule:getArgPort(i)
#  local BitWidth = port:getBitWidth()
          4'd$(num+1) : begin
#if(BitWidth == 8) then
            if(avs_Byteenable_Bus2IP[0]) $(port:getName())[7:0]    <= avs_Writedata_Bus2IP[7:0];
#end
#if(BitWidth == 16) then
            if(avs_Byteenable_Bus2IP[0]) $(port:getName())[7:0]    <= avs_Writedata_Bus2IP[7:0];
            if(avs_Byteenable_Bus2IP[1]) $(port:getName())[15:8]   <= avs_Writedata_Bus2IP[15:8];
#end
#if(BitWidth >= 32) then
            if(avs_Byteenable_Bus2IP[0]) $(port:getName())[7:0]    <= avs_Writedata_Bus2IP[7:0];
            if(avs_Byteenable_Bus2IP[1]) $(port:getName())[15:8]   <= avs_Writedata_Bus2IP[15:8];
            if(avs_Byteenable_Bus2IP[2]) $(port:getName())[23:16]  <= avs_Writedata_Bus2IP[23:16];
            if(avs_Byteenable_Bus2IP[3]) $(port:getName())[31:24]  <= avs_Writedata_Bus2IP[31:24];
#end            
          end
#num = num + 1
#if(BitWidth == 64) then
          4'd$(num+1) : begin
            if(avs_Byteenable_Bus2IP[0]) $(port:getName())[39:32]  <= avs_Writedata_Bus2IP[7:0];
            if(avs_Byteenable_Bus2IP[1]) $(port:getName())[47:40]  <= avs_Writedata_Bus2IP[15:8];
            if(avs_Byteenable_Bus2IP[2]) $(port:getName())[55:48]  <= avs_Writedata_Bus2IP[23:16];
            if(avs_Byteenable_Bus2IP[3]) $(port:getName())[63:56]  <= avs_Writedata_Bus2IP[31:24];
          end
#num = num + 1
#end
#end
          default: ;
        endcase
      end else begin
        slave_reg_0_start[0] <=  32'h0;
      end
      if(fin) begin
        slave_reg_0_start[1] <= 1;
      end else begin
        if(start) slave_reg_0_start[1] <= 0;
      end
    end
  end

  //IP_slave read process
  reg [31:0] avs_Readdata;
  always@(posedge  clk) begin
    if(~reset_n)  begin
      begin_rd     <= 0;
      avs_Readdata <= 0;
    end else begin
      begin_rd <= avs_Read_Bus2IP;
      if((avs_Read_Bus2IP && (avs_Address_Bus2IP[18:16] < BlockRamInterBase))) begin
        case(avs_Address_Bus2IP[3:0])
          4'd0 : begin
            if(avs_Byteenable_Bus2IP[0]) avs_Readdata[7:0]   <= slave_reg_0_start[7:0];
            if(avs_Byteenable_Bus2IP[1]) avs_Readdata[15:8]  <= slave_reg_0_start[15:8];
            if(avs_Byteenable_Bus2IP[2]) avs_Readdata[23:16] <= slave_reg_0_start[23:16];
            if(avs_Byteenable_Bus2IP[3]) avs_Readdata[31:24] <= slave_reg_0_start[31:24];
          end
    //==============================================================================
#local num = 0;
#for i = 0, NumArgPorts - 1 do
#  local port = CurModule:getArgPort(i)
#  local BitWidth = port:getBitWidth()
          4'd$(num+1) : begin
#if(BitWidth == 8) then
            if(avs_Byteenable_Bus2IP[0]) avs_Readdata[7:0]   <= $(port:getName())[7:0];
#end
#if(BitWidth == 16) then
            if(avs_Byteenable_Bus2IP[0]) avs_Readdata[7:0]   <= $(port:getName())[7:0];
            if(avs_Byteenable_Bus2IP[1]) avs_Readdata[15:8]  <= $(port:getName())[15:8];
#end
#if(BitWidth >= 32) then
            if(avs_Byteenable_Bus2IP[0]) avs_Readdata[7:0]   <= $(port:getName())[7:0];
            if(avs_Byteenable_Bus2IP[1]) avs_Readdata[15:8]  <= $(port:getName())[15:8];
            if(avs_Byteenable_Bus2IP[2]) avs_Readdata[23:16] <= $(port:getName())[23:16];
            if(avs_Byteenable_Bus2IP[3]) avs_Readdata[31:24] <= $(port:getName())[31:24];
#end
          end
#num = num + 1
#if(BitWidth == 64) then
          4'd$(num+1) : begin
            if(avs_Byteenable_Bus2IP[0]) avs_Readdata[7:0]   <= $(port:getName())[39:32];
            if(avs_Byteenable_Bus2IP[1]) avs_Readdata[15:8]  <= $(port:getName())[47:40];
            if(avs_Byteenable_Bus2IP[2]) avs_Readdata[23:16] <= $(port:getName())[55:48];
            if(avs_Byteenable_Bus2IP[3]) avs_Readdata[31:24] <= $(port:getName())[63:56];
          end
#num = num + 1
#end
#end
    //==============================================================================
#if(CurModule:getRetPortIdx()) then
#  local port = CurModule:getRetPort()
#  local BitWidth = port:getBitWidth()
          4'd$(num+1) : begin
#if(BitWidth == 8) then
            if(avs_Byteenable_Bus2IP[0]) avs_Readdata[7:0]   <= return_value[7:0];
#end
#if(BitWidth == 16) then
            if(avs_Byteenable_Bus2IP[0]) avs_Readdata[7:0]   <= return_value[7:0];
            if(avs_Byteenable_Bus2IP[1]) avs_Readdata[15:8]  <= return_value[15:8];
#end
#if(BitWidth >= 32) then
            if(avs_Byteenable_Bus2IP[0]) avs_Readdata[7:0]   <= return_value[7:0];
            if(avs_Byteenable_Bus2IP[1]) avs_Readdata[15:8]  <= return_value[15:8];
            if(avs_Byteenable_Bus2IP[2]) avs_Readdata[23:16] <= return_value[23:16];
            if(avs_Byteenable_Bus2IP[3]) avs_Readdata[31:24] <= return_value[31:24];
#end
          end
#if(BitWidth == 64) then
          4'd$(num+2) : begin
            if(avs_Byteenable_Bus2IP[0]) avs_Readdata[7:0]   <= return_value[39:32];
            if(avs_Byteenable_Bus2IP[1]) avs_Readdata[15:8]  <= return_value[47:40];
            if(avs_Byteenable_Bus2IP[2]) avs_Readdata[23:16] <= return_value[55:48];
            if(avs_Byteenable_Bus2IP[3]) avs_Readdata[31:24] <= return_value[63:56];
          end
#end

#end
          default: ;
        endcase
      end else begin
        avs_Readdata <= 0;
      end
    end
  end

  //generating signal start to start user logic
  assign start = slave_reg_0_start[0];
  // assign IP readdata to Avalon Bus.
  assign avs_Readdata_IP2Bus = (avs_Address_Bus2IP[18:16] < BlockRamInterBase)? avs_Readdata : q_a;

  //---------------------------------------------------------------------------------------------------------------------------------
  // master below
  //---------------------------------------------------------------------------------------------------------------------------------
  //IP_master FSM declaration
  parameter m_start   = 3'b001;
  parameter m_read_0  = 3'b010;
  parameter m_write_0 = 3'b100;
  reg [2:0] mst_state;
  reg       Mst2Mem0rdy;
  reg       Master2Mem0in;
  //master block to get the data according to the address sent by lookup_rtl
  always@(posedge  clk) begin
    if(~reset_n) begin
      mst_state             <= m_start;
      avm_Address_IP2Bus    <= 0;
      avm_Byteenable_IP2Bus <= 4'b1111;
      avm_Read_IP2Bus       <= 0;
      avm_Write_IP2Bus      <= 0;
      avm_Writedata_IP2Bus  <= 32'hffffffff;
      Mst2Mem0rdy           <= 0;
      Master2Mem0in         <= 0;
    end else begin
      case(mst_state)
        m_start: begin
          Mst2Mem0rdy <= 0;
          if(mem0en & (mem0addr[31:16] != BlockRamBase)) begin  //if signal mem0en is asserted ,this block choose to read or write by checking signal mem0cmd
            case(mem0cmd)
              4'b0000 : mst_state <= m_read_0;
              4'b0001 : mst_state <= m_write_0;
              default : ;
            endcase
          end else begin
            mst_state       <= m_start;
            avm_Read_IP2Bus <= 0;
          end
        end
        m_read_0 : begin
          avm_Address_IP2Bus    <= {mem0addr[31:2], 2'b0};
          avm_Byteenable_IP2Bus <= (mem0be >> (mem0addr[1:0]));
          avm_Read_IP2Bus       <= 1;
          if(avm_Read_IP2Bus & (~avm_Waitrequest_Bus2IP)) begin
            avm_Read_IP2Bus       <= 0;
            Mst2Mem0rdy           <= 1;
            Master2Mem0in         <= (avm_Readdata_Bus2IP >> {mem0addr[1:0], 3'b0});
            avm_Address_IP2Bus    <= 32'b0;
            avm_Byteenable_IP2Bus <= 4'b0;
            mst_state             <= m_start;
          end else begin
            mst_state   <= m_read_0;
            Mst2Mem0rdy <= 0;
          end
        end
        m_write_0 : begin
          avm_Address_IP2Bus    <= {mem0addr[31:2], 2'b0};
          avm_Byteenable_IP2Bus <= (mem0be >> (mem0addr[1:0]));
          avm_Write_IP2Bus      <= 1;
          avm_Writedata_IP2Bus  <= (mem0out << {mem0addr[1:0], 3'b0});
          if(avm_Write_IP2Bus&(~avm_Waitrequest_Bus2IP)) begin
            avm_Write_IP2Bus      <= 0;
            Mst2Mem0rdy           <= 1;
            avm_Address_IP2Bus    <= 32'b0;
            avm_Byteenable_IP2Bus <= 4'b0;
            mst_state             <= m_start;
          end else begin
            mst_state   <= m_write_0;
            Mst2Mem0rdy <= 0;
          end
        end
        default: mst_state <= m_start;
      endcase
    end
  end

  // assign the ready signal to the IP.
  assign mem0rdy = BRam2Mem0rdy | Mst2Mem0rdy;
  // assign the mem0in data to the IP.
  assign mem0in = (mem0addr[31:16] == BlockRamBase)? q_b : Master2Mem0in;

endmodule
]=]
local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=AvalonTemplate, output={[[D:\cygwin\home\Government\test_llvm\]]..'Avalon_' .. CurModule:getName() .. '.v'}}
print(message)
