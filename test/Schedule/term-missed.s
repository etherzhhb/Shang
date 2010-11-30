module f(
    input wire[31:0] a,
    input wire[31:0] b,
    output reg[31:0] return_value,
    input wire clk,
    input wire rstN,
    input wire start,
    output reg fin);


  // States
  parameter state_idle = 1'h0;
  // State for entryBB0
  parameter entryBB0 = 1'h1;


  // Reg and wire decl
  reg NextFSMState = 1'h0;
  reg[31:0] reg2 = 32'h0;
  reg[31:0] reg3 = 32'h0;
  reg cur_entryBB0_enable = 1'h0;
  wire[30:0] entryBB0_wire1;
  wire[31:0] entryBB0_wire2;
  wire[31:0] entryBB0_wire3;
  wire entryBB0_wire5;
  wire[31:0] entryBB0_wire4;


  // Datapath
  assign entryBB0_wire1 = reg3[31:1];
  assign entryBB0_wire2 = {1'h0,entryBB0_wire1};
  assign entryBB0_wire3 = entryBB0_wire2 & 32'h3;
  assign {entryBB0_wire5, entryBB0_wire4} = entryBB0_wire3 + reg2[31:0] + 1'h0;


  // Always Block
  always @(posedge clk, negedge rstN) begin
    if (!rstN) begin
      // reset registers
      return_value <= 32'h0;
      fin <= 1'h0;
      NextFSMState <= 1'h0;
      reg2 <= 32'h0;
      reg3 <= 32'h0;
      cur_entryBB0_enable <= 1'h0;
    end else begin
      // SeqCompute:
      // FSM
      case (NextFSMState)
      state_idle: begin
        if (start) begin
          reg2[31:0] <= a;
          reg3[31:0] <= b;
          fin <= 1'b0;
          NextFSMState <= entryBB0;
          cur_entryBB0_enable <= 1'b1;
        end else begin
          NextFSMState <= state_idle;
          fin <= 1'b0;
        end
      end
      // entryBB0 Total Slot: 2 II: 0
      entryBB0: begin
        cur_entryBB0_enable <= 1'b0;
      end
      default:  NextFSMState <= state_idle;
      endcase
    end //else reset
  end //always @(..)

endmodule

