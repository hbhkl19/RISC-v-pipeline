`include "ctrl_encode_def.v"
module SCPU(
    input      clk,            // clock
    input      reset,          // reset
    input [31:0]  inst_in,     // instruction
    input [31:0]  Data_in,     // data from data memory
   
    output    MemWrite,          // output: memory write signal
    output [31:0] PC_out,     // PC address
      // memory write
    output [31:0] Addr_out,   // ALU output
    output [31:0] Data_out,// data to data memory

    input  [4:0] reg_sel,    // register selection (for debug use)
    output [31:0] reg_data,  // selected register data (for debug use)
    output [2:0] DMType
);
    
    //Normal Single
    wire        RegWrite;    // control signal to register write
    wire [5:0]  EXTOp;       // control signal to signed extension
    wire [4:0]  ALUOp;       // ALU opertion
    wire [2:0]  NPCOp;       // next PC operation
    wire [1:0]  WDSel;       // (register) write data selection
    wire [1:0]  GPRSel;      // general purpose register selection
    wire        ALUSrc;      // ALU source for A
    wire        Zero;        // ALU ouput zero
    wire [31:0] NPC;         // next PC
    wire [11:0] Imm12;       // 12-bit immediate
    wire [31:0] Imm32;       // 32-bit immediate
    wire [19:0] IMM;         // 20-bit immediate (address)
    wire [4:0]  A3;          // register address for write
    reg [31:0] WD;          // register write data
    wire [31:0] RD1,RD2;         // register data specified by rs
    wire [31:0] alu_A, alu_B;
	wire [4:0] iimm_shamt;
	wire [11:0] iimm,simm,bimm;
	wire [19:0] uimm,jimm;
	wire [31:0] immout;
    wire[31:0] aluout;
    wire [2:0] DMType_ctrl;
    wire MemWrite_ctrl;
    wire MemRead_crtl;
	// Pipeline registers
    //IF/ID
    reg [31:0] IF_ID_inst, IF_ID_PC;
    reg IF_ID_valid;
    //ID/EX
    reg [31:0] ID_EX_PC, ID_EX_A, ID_EX_B, ID_EX_imm;
    reg [4:0] ID_EX_rd, ID_EX_rs1, ID_EX_rs2;
    reg ID_EX_RegWrite, ID_EX_MemWrite, ID_EX_ALUSrc;
    reg [1:0] ID_EX_WDSel, ID_EX_GPRSel;
    reg [4:0] ID_EX_ALUOp;
    reg [31:0] ID_EX_inst;
    reg [2:0] ID_EX_DMType;
    reg ID_EX_MemRead;
    //EX/MEM
    reg [31:0] EX_MEM_alu_out, EX_MEM_B,EX_MEM_PC;

    reg [4:0] EX_MEM_rs2;

    reg [4:0] EX_MEM_rd;
    reg [31:0] EX_MEM_inst;//test
    reg EX_MEM_RegWrite, EX_MEM_MemWrite;
    reg [1:0] EX_MEM_WDSel;
    reg [2:0] EX_MEM_DMType;
    reg  EX_MEM_MemRead;
    //MEM/WB
    reg [31:0] MEM_WB_alu_out, MEM_WB_dmem_out,MEM_WB_PC;
    reg [4:0] MEM_WB_rd;
    reg MEM_WB_RegWrite;
    reg [1:0] MEM_WB_WDSel;
    reg [31:0] MEM_WB_inst;
	
	assign iimm_shamt=IF_ID_inst[24:20];
	assign iimm=IF_ID_inst[31:20];
	assign simm={IF_ID_inst[31:25],IF_ID_inst[11:7]};
	assign bimm={IF_ID_inst[31],IF_ID_inst[7],IF_ID_inst[30:25],IF_ID_inst[11:8]};
	assign uimm=IF_ID_inst[31:12];
	assign jimm={IF_ID_inst[31],IF_ID_inst[19:12],IF_ID_inst[20],IF_ID_inst[30:21]};
   
     // Instruction fields
    wire [6:0] Op = IF_ID_inst[6:0];
    wire [6:0] Funct7 = IF_ID_inst[31:25];
    wire [2:0] Funct3 = IF_ID_inst[14:12];
    wire [4:0] rs1 = IF_ID_inst[19:15];
    wire [4:0] rs2 = IF_ID_inst[24:20];
    wire [4:0] rd = IF_ID_inst[11:7];
    assign Imm12 = IF_ID_inst[31:20];// 12-bit immediate
    assign IMM = IF_ID_inst[31:12];  // 20-bit immediate
    
    // instantiation of pc unit
	PC U_PC(.clk(clk), .rst(reset), .NPC(NPC), .PC(PC_out), .stall(stall)); // PC module
	 
     
    // NPC
    wire [31:0] JALR_Addr, Branch_Addr, JAL_Addr;
    wire Flush;
    assign JALR_Addr = (branch_operand_A + immout) & ~32'd1;
    assign Branch_Addr = IF_ID_PC + immout;
    assign JAL_Addr = IF_ID_PC + immout;
    //



    reg [31:0] branch_operand_A;
reg [31:0] branch_operand_B;

// 为操作数A (rs1) 创建Mux
always @(*) begin
    case (ForwardID_A)
        `Fwd_EX:   branch_operand_A = EX_MEM_alu_out; // 从MEM阶段前递
        `Fwd_WB:   branch_operand_A = WD;            // 从WB阶段前递
        default:   branch_operand_A = RD1;           // 默认从RF读取
    endcase
end

// 为操作数B (rs2) 创建Mux
always @(*) begin
    case (ForwardID_B)
        `Fwd_EX:   branch_operand_B = EX_MEM_alu_out;
        `Fwd_WB:   branch_operand_B = WD;
        default:   branch_operand_B = RD2;
    endcase
end

    BranchUnit U_BranchUnit(
        .Funct3(Funct3),
        .rs1_val(branch_operand_A),
        .rs2_val(branch_operand_B),
        .BranchTaken(BranchTaken)
    );
     // Flush
    assign Flush = (!stall) && ((Op == 7'b1100111) ||   // JALR
                   (Op == 7'b1101111) ||   // JAL
                   (Op == 7'b1100011 && BranchTaken));  // Branch taken

    
    wire is_Branch;
    assign is_Branch = (Op == 7'b1101111) ||  // JAL 
                   (Op == 7'b1100111) ||   // JALR
                   (Op == 7'b1100011);   // Branch
    
    
    //NPC
    NPC U_NPC(
        .PC(PC_out),
        .IF_ID_PC(IF_ID_PC),
        .NPCOp(NPCOp),
        .BranchTaken(BranchTaken),
        .JALR_Addr(JALR_Addr),
        .Branch_Addr(Branch_Addr),
        .JAL_Addr(JAL_Addr),
        .NPC(NPC)
    );
    // instantiation of control unit
	ctrl U_ctrl(
		.Op(Op), .Funct7(Funct7), .Funct3(Funct3), .MemRead(MemRead_crtl),
		.RegWrite(RegWrite), .MemWrite(MemWrite_ctrl),
		.EXTOp(EXTOp), .ALUOp(ALUOp), .NPCOp(NPCOp), 
		.ALUSrc(ALUSrc), .GPRSel(GPRSel), .WDSel(WDSel), .DMType(DMType_ctrl)
	);
	EXT U_EXT(
		.iimm_shamt(iimm_shamt), .iimm(iimm), .simm(simm), .bimm(bimm),
		.uimm(uimm), .jimm(jimm),
		.EXTOp(EXTOp), .immout(immout)
	);
	RF U_RF(
		.clk(clk), .rst(reset),
		.RFWr(MEM_WB_RegWrite), 
		.A1(rs1), .A2(rs2), .A3(MEM_WB_rd), 
        .reg_sel(reg_sel),
        .reg_data(reg_data),
		.WD(WD), 
		.RD1(RD1), .RD2(RD2)
		//.reg_sel(reg_sel),
		//.reg_data(reg_data)
	);
    // instantiation of alu unit
	alu U_alu(.A(alu_A), .B(alu_B), .ALUOp(ID_EX_ALUOp), .C(aluout), .Zero(Zero), .PC(ID_EX_PC));
  always @(*)
  begin
	case(MEM_WB_WDSel)
		`WDSel_FromALU: WD<=MEM_WB_alu_out;
		`WDSel_FromMEM: WD<=MEM_WB_dmem_out;
		`WDSel_FromPC: WD<=MEM_WB_PC +4;
		 default:      WD = MEM_WB_alu_out;
	endcase
  end
  
  reg [31:0] alu_in1;  
    reg [31:0] alu_in2;
    always@(*) begin
        case(ForwardA)
        `Forward_None: alu_in1 = ID_EX_A;
        `Forward_EX: alu_in1 = EX_MEM_alu_out;
        `Forward_MEM: alu_in1 = WD;
        endcase
        case(ForwardB)
        `Forward_None: alu_in2 = ID_EX_B;
        `Forward_EX: alu_in2 = EX_MEM_alu_out;
        `Forward_MEM: alu_in2 = WD;
        endcase
    end

    
    assign alu_A = alu_in1;
                   
    assign alu_B = (ID_EX_ALUSrc) ? ID_EX_imm : alu_in2;

    wire [1:0] ForwardA, ForwardB;
    wire [1:0] ForwardID_A, ForwardID_B; // For Branch comparison

    Forwarding U_Forwarding(
        .IF_ID_is_Branch(is_Branch),
        .EX_MEM_RegWrite(EX_MEM_RegWrite),
        .MEM_WB_RegWrite(MEM_WB_RegWrite),
        .EX_MEM_rd(EX_MEM_rd),
        .MEM_WB_rd(MEM_WB_rd),
        .ID_EX_rs1(ID_EX_rs1),
        .ID_EX_rs2(ID_EX_rs2),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB),
        .IF_ID_rs1(rs1),      // ID阶段的rs1地址
        .IF_ID_rs2(rs2),      // ID阶段的rs2地址
        .ForwardID_A(ForwardID_A), // 控制送往分支比较器的操作数A
        .ForwardID_B(ForwardID_B)  // 控制送往分支比较器的操作数B
    );

    wire stall;
    // Hazard detection unit
    Hazard_Detect U_Hazard_Detect(
        .IF_ID_is_Branch(is_Branch),
        .IF_ID_rs1(rs1),
        .IF_ID_rs2(rs2),
        .ID_EX_rd(ID_EX_rd),
        .ID_EX_MemRead(ID_EX_MemRead), // load-use hazard
        .ID_EX_RegWrite(ID_EX_RegWrite),
        .EX_MEM_RegWrite(EX_MEM_RegWrite),
        .EX_MEM_MemRead(EX_MEM_MemRead), // load-use hazard
        .EX_MEM_rd(EX_MEM_rd),
        .stall(stall)
    );





wire forward_WB_to_MEM; // Mux的选择信号

assign forward_WB_to_MEM = (EX_MEM_MemWrite) && 
                           (MEM_WB_RegWrite) && 
                           (MEM_WB_rd != 0) && 
                           (MEM_WB_rd == EX_MEM_rs2);


wire [31:0] final_data_to_memory; // Mux的输出

assign final_data_to_memory = forward_WB_to_MEM ? MEM_WB_alu_out : EX_MEM_B;



    
    // Outputs
    assign Addr_out = EX_MEM_alu_out;
    assign Data_out = final_data_to_memory;
    assign MemWrite = EX_MEM_MemWrite;
    assign DMType = EX_MEM_DMType;
    
    // Debug register access
    assign reg_data = (reg_sel != 0) ? U_RF.rf[reg_sel] : 0;
    
    // Pipeline stages
     always @(posedge clk) begin
        if (Flush) begin
            IF_ID_inst <= 0;
            IF_ID_PC <= 0;
        end
    end

    wire final_ID_EX_RegWrite, final_ID_EX_MemWrite, final_ID_EX_MemRead;
    assign final_ID_EX_RegWrite = stall ? 1'b0 : RegWrite;
    assign final_ID_EX_MemWrite = stall ? 1'b0 : MemWrite_ctrl;
    assign final_ID_EX_MemRead  = stall ? 1'b0 : MemRead_crtl;


    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset pipeline registers
            IF_ID_inst <= 0;IF_ID_PC <= 0;
            ID_EX_PC <= 0; ID_EX_A <= 0; ID_EX_B <= 0; ID_EX_imm <= 0;
            ID_EX_rd <= 0; ID_EX_rs1 <= 0; ID_EX_rs2 <= 0;ID_EX_inst <= 0;
            ID_EX_RegWrite <= 0; ID_EX_MemWrite <= 0;  ID_EX_MemRead<=0 ;ID_EX_ALUSrc <= 0;
            ID_EX_WDSel <= 0; ID_EX_GPRSel <= 0; ID_EX_ALUOp <= 0;ID_EX_DMType <= 0;EX_MEM_rs2 <= 0;
            EX_MEM_alu_out <= 0; EX_MEM_B <= 0; EX_MEM_rd <= 0;EX_MEM_PC<=0;
            EX_MEM_RegWrite <= 0; EX_MEM_MemWrite <= 0; EX_MEM_WDSel <= 0;
            EX_MEM_inst<=0;EX_MEM_DMType <= 0;EX_MEM_MemRead <= 0;
            MEM_WB_alu_out <= 0; MEM_WB_dmem_out <= 0; MEM_WB_rd <= 0;
            MEM_WB_RegWrite <= 0; MEM_WB_WDSel <= 0;MEM_WB_PC<=0;MEM_WB_inst<=0; 
        end
        else begin

            // IF/ID Pipeline Register
            if(!stall) begin
            IF_ID_PC <= PC_out;
            IF_ID_inst <= inst_in;
            IF_ID_valid <= 1;
            end
            // ID/EX Pipeline Register
            
            ID_EX_PC <= IF_ID_PC;
            ID_EX_inst <= IF_ID_inst;
            ID_EX_A <= RD1;
            ID_EX_B <= RD2;
            ID_EX_imm <= immout;
            ID_EX_rd <= rd;
            ID_EX_rs1 <= rs1;
            ID_EX_rs2 <= rs2;
            ID_EX_RegWrite <= final_ID_EX_RegWrite;
            ID_EX_MemWrite <= final_ID_EX_MemWrite;
            ID_EX_MemRead <= final_ID_EX_MemRead; // Add MemRead control signal
            ID_EX_ALUSrc <= ALUSrc;
            ID_EX_WDSel <= WDSel;
            ID_EX_GPRSel <= GPRSel;
            ID_EX_ALUOp <= ALUOp;
            ID_EX_DMType <= DMType_ctrl;
            
            // EX/MEM Pipeline Register  
            EX_MEM_rs2 <= ID_EX_rs2; // Store rs2 for forwarding       
            EX_MEM_PC <= ID_EX_PC;
            EX_MEM_inst <= ID_EX_inst;
            EX_MEM_alu_out <= aluout;
            EX_MEM_B <= alu_in2; // alu_B
            EX_MEM_rd <= ID_EX_rd;
            EX_MEM_RegWrite <= ID_EX_RegWrite;
            EX_MEM_MemWrite <= ID_EX_MemWrite;
            EX_MEM_MemRead <= ID_EX_MemRead; // Add MemRead control signal
            EX_MEM_WDSel <= ID_EX_WDSel;
            EX_MEM_DMType <= ID_EX_DMType;
            // MEM/WB Pipeline Register
            MEM_WB_PC <= EX_MEM_PC;
            MEM_WB_inst <= EX_MEM_inst;
            MEM_WB_alu_out <= EX_MEM_alu_out;
            MEM_WB_dmem_out <= Data_in;
            MEM_WB_rd <= EX_MEM_rd;
            MEM_WB_RegWrite <= EX_MEM_RegWrite;
            MEM_WB_WDSel <= EX_MEM_WDSel;
        end
    end
endmodule

module BranchUnit(
    input [2:0] Funct3,
    input [31:0] rs1_val,
    input [31:0] rs2_val,
    output reg BranchTaken
);
    always @(*) begin
        case (Funct3)
            3'b000: BranchTaken = (rs1_val == rs2_val);   // BEQ
            3'b001: BranchTaken = (rs1_val != rs2_val);   // BNE
            3'b100: BranchTaken = ($signed(rs1_val) < $signed(rs2_val)); // BLT
            3'b101: BranchTaken = ($signed(rs1_val) >= $signed(rs2_val)); // BGE
            3'b110: BranchTaken = (rs1_val < rs2_val);    // BLTU
            3'b111: BranchTaken = (rs1_val >= rs2_val);   // BGEU
            default: BranchTaken = 0;
        endcase
    end
endmodule