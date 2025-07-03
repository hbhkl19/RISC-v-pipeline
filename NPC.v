`include "ctrl_encode_def.v"

module NPC(PC, NPCOp, BranchTaken,JALR_Addr,Branch_Addr,JAL_Addr,IF_ID_PC,NPC,stall);  // next pc module
    
   input  [31:0] PC,IF_ID_PC;        // pc
   input  [2:0]  NPCOp;     // next pc operation
   input         BranchTaken;
   input  [31:0] JALR_Addr;
   input  [31:0] Branch_Addr;
   input  [31:0] JAL_Addr;
   inout stall,
   output reg [31:0] NPC;   // next pc
   
   wire [31:0] PCPLUS4;
   
   assign PCPLUS4 = PC + 4; // pc + 4
   
   always @(*) begin
      if(stall) begin
         NPC = PC;  // 停止时保持当前 PC
      end else begin
      case (NPCOp)
          `NPC_PLUS4:  NPC = PCPLUS4;
          `NPC_BRANCH: NPC = BranchTaken ? Branch_Addr : PCPLUS4;
          `NPC_JUMP:   NPC = JAL_Addr;
		  `NPC_JALR:   NPC = JALR_Addr;
          default:     NPC = PCPLUS4;
      endcase end
   end // end always
   
endmodule
