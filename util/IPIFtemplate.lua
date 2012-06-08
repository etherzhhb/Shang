local IPIFTemplate = [=[
module user_logic
(
  // -- ADD USER PORTS BELOW THIS LINE ---------------
  // --USER ports added here 
  // -- ADD USER PORTS ABOVE THIS LINE ---------------

  // -- DO NOT EDIT BELOW THIS LINE ------------------
  // -- Bus protocol ports, do not add to or delete 
  Bus2IP_Clk,                     // Bus to IP clock
  Bus2IP_Reset,                   // Bus to IP reset
  Bus2IP_Addr,                    // Bus to IP address bus
  Bus2IP_CS,                      // Bus to IP chip select for user logic memory selection
  Bus2IP_RNW,                     // Bus to IP read/not write
  Bus2IP_Data,                    // Bus to IP data bus
  Bus2IP_BE,                      // Bus to IP byte enables
  Bus2IP_RdCE,                    // Bus to IP read chip enable
  Bus2IP_WrCE,                    // Bus to IP write chip enable
  IP2Bus_Data,                    // IP to Bus data bus
  IP2Bus_RdAck,                   // IP to Bus read transfer acknowledgement
  IP2Bus_WrAck,                   // IP to Bus write transfer acknowledgement
  IP2Bus_Error,                   // IP to Bus error response
  IP2Bus_MstRd_Req,               // IP to Bus master read request
  IP2Bus_MstWr_Req,               // IP to Bus master write request
  IP2Bus_Mst_Addr,                // IP to Bus master address bus
  IP2Bus_Mst_BE,                  // IP to Bus master byte enables
  IP2Bus_Mst_Lock,                // IP to Bus master lock
  IP2Bus_Mst_Reset,               // IP to Bus master reset
  Bus2IP_Mst_CmdAck,              // Bus to IP master command acknowledgement
  Bus2IP_Mst_Cmplt,               // Bus to IP master transfer completion
  Bus2IP_Mst_Error,               // Bus to IP master error response
  Bus2IP_Mst_Rearbitrate,         // Bus to IP master re-arbitrate
  Bus2IP_Mst_Cmd_Timeout,         // Bus to IP master command timeout
  Bus2IP_MstRd_d,                 // Bus to IP master read data bus
  Bus2IP_MstRd_src_rdy_n,         // Bus to IP master read source ready
  IP2Bus_MstWr_d,                 // IP to Bus master write data bus
  Bus2IP_MstWr_dst_rdy_n          // Bus to IP master write destination ready
  // -- DO NOT EDIT ABOVE THIS LINE ------------------
); // user_logic

// -- ADD USER PARAMETERS BELOW THIS LINE ------------
// --USER parameters added here 
// -- ADD USER PARAMETERS ABOVE THIS LINE ------------

// -- DO NOT EDIT BELOW THIS LINE --------------------
// -- Bus protocol parameters, do not add to or delete
parameter C_SLV_AWIDTH                   = 32;
parameter C_SLV_DWIDTH                   = 32;
parameter C_MST_AWIDTH                   = 32;
parameter C_MST_DWIDTH                   = 32;
parameter C_NUM_REG                      = $(CurModule:getNumArgPorts()+7);
parameter C_NUM_MEM                      = 1;
// -- DO NOT EDIT ABOVE THIS LINE --------------------

// -- ADD USER PORTS BELOW THIS LINE -----------------
// --USER ports added here 
// -- ADD USER PORTS ABOVE THIS LINE -----------------

// -- DO NOT EDIT BELOW THIS LINE --------------------
// -- Bus protocol ports, do not add to or delete
input                                     Bus2IP_Clk;
input                                     Bus2IP_Reset;
input      [0 : C_SLV_AWIDTH-1]           Bus2IP_Addr;
input      [0 : C_NUM_MEM-1]              Bus2IP_CS;
input                                     Bus2IP_RNW;
input      [0 : C_SLV_DWIDTH-1]           Bus2IP_Data;
input      [0 : C_SLV_DWIDTH/8-1]         Bus2IP_BE;
input      [0 : C_NUM_REG-1]              Bus2IP_RdCE;
input      [0 : C_NUM_REG-1]              Bus2IP_WrCE;
output     [0 : C_SLV_DWIDTH-1]           IP2Bus_Data;
output                                    IP2Bus_RdAck;
output                                    IP2Bus_WrAck;
output                                    IP2Bus_Error;
output                                    IP2Bus_MstRd_Req;
output                                    IP2Bus_MstWr_Req;
output     [0 : C_MST_AWIDTH-1]           IP2Bus_Mst_Addr;
output     [0 : C_MST_DWIDTH/8-1]         IP2Bus_Mst_BE;
output                                    IP2Bus_Mst_Lock;
output                                    IP2Bus_Mst_Reset;
input                                     Bus2IP_Mst_CmdAck;
input                                     Bus2IP_Mst_Cmplt;
input                                     Bus2IP_Mst_Error;
input                                     Bus2IP_Mst_Rearbitrate;
input                                     Bus2IP_Mst_Cmd_Timeout;
input      [0 : C_MST_DWIDTH-1]           Bus2IP_MstRd_d;
input                                     Bus2IP_MstRd_src_rdy_n;
output     [0 : C_MST_DWIDTH-1]           IP2Bus_MstWr_d;
input                                     Bus2IP_MstWr_dst_rdy_n;
// -- DO NOT EDIT ABOVE THIS LINE --------------------

//----------------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------------

  // --USER nets declarations added here, as needed for user logic

  // Nets for user logic slave model s/w accessible register example
  reg        [0 : C_SLV_DWIDTH-1]           ip_start;
  reg        [0 : C_SLV_DWIDTH-1]           ip_finish;
  reg        [0 : C_SLV_DWIDTH-1]           ip_return_value;
#-----------------------------------------------------------------------------
#--instantiate the arguement into our slave register, and then go on instantiate
#--the default slave register
#-----------------------------------------------------------------------------
#local NumCommonPorts = CurModule:getNumCommonPorts() 
#local NumArgPorts = CurModule:getNumArgPorts()
#for i = 0, NumArgPorts - 1 do
#  local port = CurModule:getArgPort(i)
  reg        [0 : C_SLV_DWIDTH-1]           $(port:getName());
#end
#--for i = NumArgPorts + 3, 29 do
#--  reg        [0 : C_SLV_DWIDTH-1]           slv_reg$(i);
#--end
#-----------------------------------------------------------------------------
  wire       [0 : 29]                       slv_reg_write_sel;
  wire       [0 : 29]                       slv_reg_read_sel;
  reg        [0 : C_SLV_DWIDTH-1]           slv_ip2bus_data;
  wire                                      slv_read_ack;
  wire                                      slv_write_ack;
  integer                                   byte_index, bit_index;
  reg                                       IP2Bus_MstRd_Req;
  reg                                       IP2Bus_MstWr_Req;
  reg        [0 : C_MST_AWIDTH-1]           IP2Bus_Mst_Addr;
  reg 		   [0 : C_MST_DWIDTH/8-1]         IP2Bus_Mst_BE;
  reg 		   [0 : C_MST_DWIDTH-1]           IP2Bus_MstWr_d;
  // IP & master control register
  reg                                       IP2MST_start;
  //master register
  reg        [0 : C_MST_DWIDTH-1]           MST_read_data;
  reg        [0 : C_MST_DWIDTH-1]           MST_write_data;
  reg        [0 : C_SLV_DWIDTH/8-1]         MST_byte_enable;
  
  // --USER logic implementation added here
  
  // declare the connection to the BRam.
	wire          [3:0]                       wea;
	reg           [3:0]                       web;
	wire          [5:0]                       addra;
	reg           [5:0]                       addrb;
	reg                                       enb;
	reg           [0:31]                      dib;
	wire          [0:31]                      dob;
	wire          [0:31]                      doa;
  
  // declare the connection to the UserIP.
  wire                                      start;
  wire                                      finish;
#-------------------------------------------------------------------------------
#--declare wire or reg which will be used to instantiate our generated IP
#-------------------------------------------------------------------------------
#for i = NumArgPorts , NumCommonPorts -1 do
#  local port = CurModule:getCommonPort(i)
#  local BitWidth = port:getBitWidth()
#  if (port:isInput()) then
#    if (BitWidth == 1) then
  wire                                      $(port:getName());
#    else
  wire       [0 : $(BitWidth-1)]                       $(port:getName());
#    end
#  else
#    if (BitWidth == 1) then
  wire                                      $(port:getName());
#    else
  wire       [0 : $(BitWidth-1)]                       $(port:getName());
#    end
#  end
#end
#---------------------------------------------------------------------------------
  //initialize the local bram
  bram32 dut_bram32 (
    .clka(Bus2IP_Clk),
    .ena(Bus2IP_CS),
    .wea(wea),
    .addra(addra),
    .dina(Bus2IP_Data),
    .douta(doa),
    .clkb(Bus2IP_Clk),
    .enb(enb),
    .web(web),
    .addrb(addrb),
    .dinb(dib),
    .doutb(dob));
    
  //initialize the UserIP 
  $(CurModule:getName()) dut_$(CurModule:getName())(
      .clk(Bus2IP_Clk),
      .reset(Bus2IP_Reset),
      .start(start),
      .fin(finish),
#for i = 0, NumCommonPorts - 1 do
#  local port = CurModule:getCommonPort(i):getName()
      .$(port)($(port))$(if i ~= NumCommonPorts - 1 then _put(',') end)
#end
      );

  parameter GlobalBRam_Idle    = 2'b10,
            GlobalRead         = 2'b01;
  reg 															 ram_rd_ack;
  reg      [0:1]                     GlobalBRamState;
  // Implement the Global BRam FSM
  always @( posedge Bus2IP_Clk ) begin: Global_BRam_Operate
    if ( Bus2IP_Reset ) begin
      ram_rd_ack <= 0;
      GlobalBRamState <= GlobalBRam_Idle;
    end else begin
      case ( GlobalBRamState )
        GlobalBRam_Idle: begin
          if ( Bus2IP_CS && Bus2IP_RNW ) begin
            ram_rd_ack <= 1;
            GlobalBRamState <= GlobalRead;
          end else begin
            ram_rd_ack <= 0;
            GlobalBRamState <= GlobalBRam_Idle;
          end
        end
        
        GlobalRead:begin
          ram_rd_ack <= 0;
          GlobalBRamState <= GlobalBRam_Idle;
        end
        
        default: GlobalBRamState <= GlobalBRam_Idle;
      endcase
    end
  end
  
  parameter LocalBRam_Idle     = 2'b10,
            read_bram          = 2'b01;
  parameter BlockRamBase = 8'h85;
  reg      [0:1]                     LocalBRamState;
  reg                                BRam2Mem0rdy;
  // Implement the Local BRam Operation
  always @( posedge Bus2IP_Clk ) begin: Local_BRam_Operate
    if ( Bus2IP_Reset ) begin
      enb <= 0;
      web <= 0;
      addrb <= 0;
      LocalBRamState <= LocalBRam_Idle;
      BRam2Mem0rdy <= 0;
    end else begin
      case (LocalBRamState)
        LocalBRam_Idle: begin 
          if ( mem0en && ( mem0addr[0:7] == BlockRamBase ) ) begin
            enb <= 1;
            addrb <= mem0addr[27:31];
            if ( mem0we ) begin 
              web <= mem0be;
              dib <= mem0out;
              BRam2Mem0rdy <= 1;
              LocalBRamState <= LocalBRam_Idle;
            end else begin
              web <= ~ mem0be;
              LocalBRamState <= read_bram;
            end
          end else begin//end IP2MST_start & mem0addr judgement
            BRam2Mem0rdy <= 0;
            enb <= 0;
            web <= 0;
            addrb <= 0;
          end
        end//end BramIdle
            
        read_bram: begin
          enb <= 0;
          BRam2Mem0rdy <= 1;
          LocalBRamState <= LocalBRam_Idle;
        end

        default: LocalBRamState <= LocalBRam_Idle;
      endcase
    end // end reset else
  end //end always
  
  parameter MasterIdle          = 7'b1000000,
            request_read        = 7'b0100000,
            read_data           = 7'b0010000,
            read_data_finish    = 7'b0001000,
            request_write       = 7'b0000100,
            write_data		      = 7'b0000010,
            write_data_finish   = 7'b0000001;
            
  reg 		[0:6] 								    MasterState;
  reg     [0 : C_MST_DWIDTH-1]      Master2Mem0in;
  reg                               Master2Mem0rdy;

  // Implement the IPIF Master FSM
  always @( posedge Bus2IP_Clk ) begin: Master_section
    if ( Bus2IP_Reset ) begin
      MasterState <= MasterIdle;
      IP2Bus_MstWr_d <= 0;
      IP2Bus_MstRd_Req <= 0;
      IP2Bus_MstWr_Req <= 0;
      IP2Bus_Mst_Addr <= 0;
      IP2Bus_Mst_BE <= 0;
      MST_read_data <= 0;
      MST_write_data <= 0;
      Master2Mem0in <= 0;
      Master2Mem0rdy <= 0;
    end else begin
      case( MasterState )
        MasterIdle:	begin
          if ( mem0en && ( mem0addr[0:7] != BlockRamBase ) ) begin
              MST_byte_enable <= ( mem0be >> ( mem0addr[30:31] ) );
              if (mem0we) begin
                MasterState <= request_write;
                MST_write_data <= ( mem0out >> { mem0addr[30:31], 3'b0 } );
              end else 
                MasterState <= request_read;
          end else begin
            MasterState <= MasterIdle;
            Master2Mem0rdy <= 0;
          end
        end
        
        request_read: begin
          if ( Bus2IP_Mst_CmdAck ) begin
            IP2Bus_MstRd_Req <= 0;
            MasterState <= read_data;
          end else begin
            MasterState<=request_read;
            IP2Bus_MstRd_Req <= 1;
            IP2Bus_Mst_Addr <= { mem0addr[0:29], 2'b0 };
            IP2Bus_Mst_BE <= ( mem0be >> ( mem0addr[30:31] ) ); 
          end
        end
          
        read_data: begin
          IP2Bus_MstRd_Req <= 0;
          IP2Bus_Mst_BE <= 0;
          if (Bus2IP_Mst_Cmplt) begin                
            if ( MST_byte_enable[0] ) MST_read_data[0:7]   <= Bus2IP_MstRd_d[0:7];   else MST_read_data[0:7]   <= 0;
            if ( MST_byte_enable[1] ) MST_read_data[8:15]  <= Bus2IP_MstRd_d[8:15];  else MST_read_data[8:15]  <= 0;
            if ( MST_byte_enable[2] ) MST_read_data[16:23] <= Bus2IP_MstRd_d[16:23]; else MST_read_data[16:23] <= 0;
            if ( MST_byte_enable[3] ) MST_read_data[24:31] <= Bus2IP_MstRd_d[24:31]; else MST_read_data[24:31] <= 0;
            MasterState <= read_data_finish;
          end else MasterState <= read_data;
        end
        
        read_data_finish: begin
          Master2Mem0in <= ( MST_read_data << { mem0addr[30:31], 3'b0 } );
          Master2Mem0rdy <= 1;
          MasterState <= MasterIdle;
        end  
        
        request_write: begin
          if ( Bus2IP_Mst_CmdAck ) begin
            if ( Bus2IP_Mst_Cmplt )
              MasterState <= write_data_finish;
            else
              MasterState <= write_data;
            IP2MST_start <= 0;
          end else begin
            MasterState <= request_write;
            IP2Bus_MstWr_Req <= 1;
            IP2Bus_Mst_Addr <= { mem0addr[0:29], 2'b0 };
            IP2Bus_Mst_BE <= ( mem0be >> ( mem0addr[30:31] ) );
            if ( MST_byte_enable[0] ) IP2Bus_MstWr_d[0:7]   <= MST_write_data[0:7];   else IP2Bus_MstWr_d[0:7]   <= 0;
            if ( MST_byte_enable[1] ) IP2Bus_MstWr_d[8:15]  <= MST_write_data[8:15];  else IP2Bus_MstWr_d[8:15]  <= 0;
            if ( MST_byte_enable[2] ) IP2Bus_MstWr_d[16:23] <= MST_write_data[16:23]; else IP2Bus_MstWr_d[16:23] <= 0;
            if ( MST_byte_enable[3] ) IP2Bus_MstWr_d[24:31] <= MST_write_data[24:31]; else IP2Bus_MstWr_d[24:31] <= 0;
          end
        end
        
        write_data : begin
          IP2Bus_MstWr_Req <= 0;
          IP2Bus_Mst_Addr <= 0;
          IP2Bus_Mst_BE <= 0;
          if ( Bus2IP_Mst_Cmplt ) begin
            MasterState <= write_data_finish;
          end else begin
            MasterState <= write_data;
          end
        end
        
        write_data_finish: begin
          IP2Bus_MstWr_Req <= 0;
          IP2Bus_Mst_Addr <= 0;
          IP2Bus_Mst_BE <= 0;
          Master2Mem0rdy <= 1;
          IP2Bus_MstWr_d <= 0;
          MasterState <= MasterIdle;
        end

        default: MasterState<=MasterIdle;
      endcase
    end // end else
  end //end always

  // ------------------------------------------------------
  // Example code to read/write user logic slave model s/w accessible registers
  // 
  // Note:
  // The example code presented here is to show you one way of reading/writing
  // software accessible registers implemented in the user logic slave model.
  // Each bit of the Bus2IP_WrCE/Bus2IP_RdCE signals is configured to correspond
  // to one software accessible register by the top level template. For example,
  // if you have four 32 bit software accessible registers in the user logic,
  // you are basically operating on the following memory mapped registers:
  // 
  //    Bus2IP_WrCE/Bus2IP_RdCE   Memory Mapped Register
  //                     "1000"   C_BASEADDR + 0x0
  //                     "0100"   C_BASEADDR + 0x4
  //                     "0010"   C_BASEADDR + 0x8
  //                     "0001"   C_BASEADDR + 0xC
  //  
  // ------------------------------------------------------
  
  // assign the data to the IP.
  assign mem0in = mem0addr[0:7] == BlockRamBase ? dob : Master2Mem0in;
  // assign the ready signal to the IP.
  assign mem0rdy = mem0addr[0:7] == BlockRamBase ? BRam2Mem0rdy : Master2Mem0rdy;
  // assign the write enable to the bram from the bus transation.
	assign wea = Bus2IP_RNW ? 4'b0000 : Bus2IP_BE; 
  // assign the address to the bram from the bus transtion.
	assign addra = Bus2IP_Addr[27:31]; 
  //assign the start signal the launch this peripheral.
# local ip_start_t_sel = {1,0,0}
#  for m = 0, NumArgPorts - 1 do 
#    table.insert(ip_start_t_sel,3,0)
#  end
  assign start =  slv_reg_write_sel == $(NumArgPorts+3)'b$(table.concat(ip_start_t_sel)) ?   Bus2IP_Data[0] : 0;
  assign
    slv_reg_write_sel = Bus2IP_WrCE[0:$(NumArgPorts+2)],
    slv_reg_read_sel  = Bus2IP_RdCE[0:$(NumArgPorts+2)],
    // when the bram or the slave is written, they will give a acknowledge signal to the bus.
    slv_write_ack     = ( Bus2IP_CS && ( ~Bus2IP_RNW ) ) || ( |Bus2IP_WrCE ), 
    // when the bram or the slave is read, they will give a acknowledge signal to the bus.
    slv_read_ack      = ram_rd_ack || ( |Bus2IP_RdCE );
    
  // implement slave model register(s)
  always @( posedge Bus2IP_Clk )
    begin: SLAVE_REG_WRITE_PROC

      if ( Bus2IP_Reset ) begin
          ip_start <= 0;
          ip_finish <= 0;
          ip_return_value <= 0;
#------------------------------------------------------------------------
#--reset arguement and slave register
#------------------------------------------------------------------------
#for i = 0, NumArgPorts - 1 do
#  local port = CurModule:getArgPort(i)
          $(port:getName())<= 0;
#end
#--for i = NumArgPorts + 3, 29 do
#--          slv_reg$(i)<= 0;
#--end
#------------------------------------------------------------------------
      end else begin
        ip_finish[0] <= finish;
        ip_return_value <= return_value;
        case ( slv_reg_write_sel )
#local ip_start_t = {1,0,0," :"}
#  for m = 0, NumArgPorts - 1 do 
#    table.insert(ip_start_t,4,0)
#  end
          $(NumArgPorts+3)'b$(table.concat(ip_start_t))
//            for ( byte_index = 0; byte_index <= (C_SLV_DWIDTH/8)-1; byte_index = byte_index+1 )
//              if ( Bus2IP_BE[byte_index] == 1 )
//                for ( bit_index = byte_index*8; bit_index <= byte_index*8+7; bit_index = bit_index+1 )
//                  ip_start[bit_index] <= Bus2IP_Data[bit_index];
            begin
              ip_start[1:31] <=  Bus2IP_Data[1:31];
              ip_start[0] <=  ~Bus2IP_Data[0];
            end
#local ip_finish_t = {0,1,0," :"}
#  for m = 0, NumArgPorts - 1 do 
#    table.insert(ip_finish_t,4,0)
#  end
          $(NumArgPorts+3)'b$(table.concat(ip_finish_t))
            for ( byte_index = 0; byte_index <= (C_SLV_DWIDTH/8)-1; byte_index = byte_index+1 )
              if ( Bus2IP_BE[byte_index] == 1 )
                for ( bit_index = byte_index*8; bit_index <= byte_index*8+7; bit_index = bit_index+1 )
                  ip_finish[bit_index] <= Bus2IP_Data[bit_index];
#local ip_return_value_t = {0,0,1," :"}
#  for m = 0, NumArgPorts - 1 do 
#    table.insert(ip_return_value_t,4,0)
#  end
          $(NumArgPorts+3)'b$(table.concat(ip_return_value_t))
            for ( byte_index = 0; byte_index <= (C_SLV_DWIDTH/8)-1; byte_index = byte_index+1 )
              if ( Bus2IP_BE[byte_index] == 1 )
                for ( bit_index = byte_index*8; bit_index <= byte_index*8+7; bit_index = bit_index+1 )
                  ip_return_value[bit_index] <= Bus2IP_Data[bit_index];
#----------------------------------------------------------------------------------------------------------------
#--arguement register  assignment
#----------------------------------------------------------------------------------------------------------------
#for i = 0, NumArgPorts - 1 do
#  local numcount = i + 4
#  local t = {0,0,0," :"}
#--insert 0 to the table
#  for m = 0, NumArgPorts - 2 do 
#    table.insert(t,1,0)
#  end
#  table.insert(t,numcount,1)
          $(NumArgPorts+3)'b$(table.concat(t))
            for ( byte_index = 0; byte_index <= (C_SLV_DWIDTH/8)-1; byte_index = byte_index+1 )
              if ( Bus2IP_BE[byte_index] == 1 )
                for ( bit_index = byte_index*8; bit_index <= byte_index*8+7; bit_index = bit_index+1 )
#  local port = CurModule:getArgPort(i)
                  $(port:getName())[bit_index] <= Bus2IP_Data[bit_index];
#end

#--for i = NumArgPorts+3, 29 do
#--  local numcount = i + 2
#--  local t = {"30'b",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0," :"}
#--  table.insert(t,numcount,1)
#--          $(table.concat(t))
#--            for ( byte_index = 0; byte_index <= (C_SLV_DWIDTH/8)-1; byte_index = byte_index+1 )
#--              if ( Bus2IP_BE[byte_index] == 1 )
#--                for ( bit_index = byte_index*8; bit_index <= byte_index*8+7; bit_index = bit_index+1 )
#--                  slv_reg$(i)[bit_index] <= Bus2IP_Data[bit_index];
#--end
#----------------------------------------------------------------------------------------------------------------
          default : ;
        endcase
      end
    end // SLAVE_REG_WRITE_PROC
    
// implement slave model register read mux
  always @*
    begin: SLAVE_REG_READ_PROC
    
      case ( slv_reg_read_sel )
#local ip_start_t = {1,0,0," :"}
#  for m = 0, NumArgPorts - 1 do 
#    table.insert(ip_start_t,4,0)
#  end
        $(NumArgPorts+3)'b$(table.concat(ip_start_t)) slv_ip2bus_data <= ip_start;
#local ip_finish_t = {0,1,0," :"}
#  for m = 0, NumArgPorts - 1 do 
#    table.insert(ip_finish_t,4,0)
#  end
        $(NumArgPorts+3)'b$(table.concat(ip_finish_t)) slv_ip2bus_data <= ip_finish;
#local ip_return_value_t = {0,0,1," :"}
#  for m = 0, NumArgPorts - 1 do 
#    table.insert(ip_return_value_t,4,0)
#  end
        $(NumArgPorts+3)'b$(table.concat(ip_return_value_t)) slv_ip2bus_data <= ip_return_value;
#for i = 0, NumArgPorts - 1 do
#local port = CurModule:getArgPort(i)
#  local numcount = i + 4
#  local t = {0,0,0," :"}
#--insert 0 to the table
#  for m = 0, NumArgPorts - 2 do 
#    table.insert(t,1,0)
#  end
#  table.insert(t,numcount,1)
        $(NumArgPorts+3)'b$(table.concat(t)) slv_ip2bus_data <= $(port:getName());
#end
#--for i = NumArgPorts+3, 29 do
#--  local numcount = i+2
#--  local t = {"30'b",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0," :"}
#--  table.insert(t,numcount,1) 
#--        $(table.concat(t)) slv_ip2bus_data <= slv_reg$(i);
#--end                
        default : slv_ip2bus_data <= 0;
      endcase

    end // SLAVE_REG_READ_PROC

  // ------------------------------------------------------------
  // Example code to drive IP to Bus signals
  // ------------------------------------------------------------

  assign IP2Bus_Data    = Bus2IP_Addr[0:7] == BlockRamBase ? doa : slv_ip2bus_data;
  assign IP2Bus_WrAck   = slv_write_ack;
  assign IP2Bus_RdAck   = slv_read_ack;
  assign IP2Bus_Error   = 0;

endmodule

]=]

local preprocess = require "luapp" . preprocess
local _, message = preprocess {input=IPIFTemplate, output={[[D:\cygwin\home\tsingray\work\lua\fubram\]]..'IPIF_' .. CurModule:getName() .. '_user_logic' .. '.v'}}
print(message)

  
  
  
  
  
  
  
  

