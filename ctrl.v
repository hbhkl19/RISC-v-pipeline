`include "ctrl_encode_def.v"

module ctrl(Op, Funct7, Funct3, 
            RegWrite, MemWrite,
            EXTOp, ALUOp, NPCOp, 
            ALUSrc, GPRSel, WDSel,DMType,MemRead
            );
            
   input  [6:0] Op;       // opcode
   input  [6:0] Funct7;    // funct7
   input  [2:0] Funct3;    // funct3;
   output       RegWrite; // control signal for register write
   output       MemWrite; // control signal for memory write
   output [5:0] EXTOp;    // control signal to signed extension
   output [4:0] ALUOp;    // ALU opertion
   output [2:0] NPCOp;    // next pc operation
   output       ALUSrc;   // ALU source for A
   output [2:0] DMType;
   output       MemRead;
   output [1:0] GPRSel;   // general purpose register selection
   output [1:0] WDSel;    // (register) write data selection
   
    reg [4:0] myALUOp;
   // ALU operation codes
    wire auipc  = (Op == 7'b0010111);  // AUIPC
    wire rtype  = (Op == 7'b0110011);
    wire itype  = (Op == 7'b0010011);
    wire stype  = (Op == 7'b0100011);
    wire btype  = (Op == 7'b1100011);
    wire utype  = (Op == 7'b0110111);  // LUI
    wire jal    = (Op == 7'b1101111);
    wire jalr   = (Op == 7'b1100111);
    wire load   = (Op == 7'b0000011);
    
    // R-type 
    wire i_add = rtype & (Funct3 == 3'b000) & (Funct7 == 7'b0000000);
    wire i_sub = rtype & (Funct3 == 3'b000) & (Funct7 == 7'b0100000);
    wire i_sll = rtype & (Funct3 == 3'b001) & (Funct7 == 7'b0000000);
    wire i_slt = rtype & (Funct3 == 3'b010) & (Funct7 == 7'b0000000);
    wire i_sltu= rtype & (Funct3 == 3'b011) & (Funct7 == 7'b0000000);
    wire i_xor = rtype & (Funct3 == 3'b100) & (Funct7 == 7'b0000000);
    wire i_srl = rtype & (Funct3 == 3'b101) & (Funct7 == 7'b0000000);
    wire i_sra = rtype & (Funct3 == 3'b101) & (Funct7 == 7'b0100000);
    wire i_or  = rtype & (Funct3 == 3'b110) & (Funct7 == 7'b0000000);
    wire i_and = rtype & (Funct3 == 3'b111) & (Funct7 == 7'b0000000);
  // I-type 
    wire i_addi = itype & (Funct3 == 3'b000);
    wire i_slti = itype & (Funct3 == 3'b010);
    wire i_sltiu= itype & (Funct3 == 3'b011);
    wire i_xori = itype & (Funct3 == 3'b100);
    wire i_ori  = itype & (Funct3 == 3'b110);
    wire i_andi = itype & (Funct3 == 3'b111);
    wire i_slli = itype & (Funct3 == 3'b001) & (Funct7 == 7'b0000000);
    wire i_srli = itype & (Funct3 == 3'b101) & (Funct7 == 7'b0000000);
    wire i_srai = itype & (Funct3 == 3'b101) & (Funct7 == 7'b0100000);
    // B-type 
    wire i_beq  = btype & (Funct3 == 3'b000);
    wire i_bne  = btype & (Funct3 == 3'b001);
    wire i_blt  = btype & (Funct3 == 3'b100);
    wire i_bge  = btype & (Funct3 == 3'b101);
    wire i_bltu = btype & (Funct3 == 3'b110);
    wire i_bgeu = btype & (Funct3 == 3'b111);
    // Load/Store 
    wire i_lb = load & (Funct3 == 3'b000);
    wire i_lh = load & (Funct3 == 3'b001);
    wire i_lw = load & (Funct3 == 3'b010);
    wire i_lbu = load & (Funct3 == 3'b100);
    wire i_lhu = load & (Funct3 == 3'b101);
    wire i_sb = stype & (Funct3 == 3'b000);
    wire i_sh = stype & (Funct3 == 3'b001);
    wire i_sw = stype & (Funct3 == 3'b010);
   
    assign RegWrite = rtype | itype | utype | auipc | jal | jalr | load;
    assign MemWrite = stype;
    assign ALUSrc   = itype | stype | utype | auipc | jal | jalr | load;
    
    
    assign MemRead  = load; // load instructions read from memory
    
    // signed extension
  // EXT_CTRL_ITYPE_SHAMT 6'b100000
  // EXT_CTRL_ITYPE	      6'b010000
  // EXT_CTRL_STYPE	      6'b001000
  // EXT_CTRL_BTYPE	      6'b000100
  // EXT_CTRL_UTYPE	      6'b000010
  // EXT_CTRL_JTYPE	      6'b000001
    


    assign EXTOp[5] = i_slli | i_srli | i_srai;  // I-type shift amount
    assign EXTOp[4] = load || (itype & ~(i_slli | i_srli | i_srai)); // I-type
    assign EXTOp[3] = stype;    // S-type
    assign EXTOp[2] = btype;   // B-type
    assign EXTOp[1] = utype | auipc; // U-type
    assign EXTOp[0] = jal;     // J-type

  // WDSel_FromALU 2'b00
  // WDSel_FromMEM 2'b01
  // WDSel_FromPC  2'b10 
    assign WDSel = (load) ? `WDSel_FromMEM : 
                   (jal | jalr) ? `WDSel_FromPC : 
                   `WDSel_FromALU;
    
   
    assign GPRSel = (jal) ? `GPRSel_31 : `GPRSel_RD;
    
    // NPC operation codes
    assign NPCOp = (jalr) ? `NPC_JALR :
                   (jal)  ? `NPC_JUMP :
                   (btype) ? `NPC_BRANCH : `NPC_PLUS4;
   // ALU
    assign ALUOp=(utype)? `ALUOp_lui :
            (auipc)? `ALUOp_auipc:
            i_sub?`ALUOp_sub:
            i_slt?`ALUOp_slt:
            i_sltu?`ALUOp_sltu:
            i_xor?`ALUOp_xor:
            i_or?`ALUOp_or:
            i_and?`ALUOp_and:
            i_sll?`ALUOp_sll:
            i_srl?`ALUOp_srl:
            i_sra?`ALUOp_sra:
            i_addi?`ALUOp_add:
            i_slti?`ALUOp_slt:
            i_sltiu?`ALUOp_sltu:
            i_xori?`ALUOp_xor:
            i_ori?`ALUOp_or:
            i_andi?`ALUOp_and:
            i_slli?`ALUOp_sll:
            i_srli?`ALUOp_srl:
            i_srai?`ALUOp_sra:
            i_beq? `ALUOp_beq:
            i_bne?`ALUOp_bne:
            i_blt?`ALUOp_blt:
            i_bge?`ALUOp_bge:
            i_bltu?`ALUOp_bltu:
            i_bgeu?`ALUOp_bgeu:
            i_lb?`ALUOp_add:
            i_lh?`ALUOp_add:
            i_lw?`ALUOp_add:
            i_lbu?`ALUOp_add:
            i_lhu?`ALUOp_add:
            i_sb?`ALUOp_add:
            i_sh?`ALUOp_add:
            i_sw?`ALUOp_add:`ALUOp_add;
  
  // DMType assignments
  assign DMType =(Funct3 == 3'b000) ? `dm_byte : 
                 (Funct3 == 3'b001) ? `dm_halfword :
                 (Funct3 == 3'b010) ? `dm_word : 
                 (Funct3 == 3'b100) ? `dm_byte_unsigned : 
                 (Funct3 == 3'b101) ? `dm_halfword_unsigned : `dm_word ;

endmodule
