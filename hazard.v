// PreciseDataHazardUnit_AlwaysBlock.v
// (Procedural Style using always @(*))
`include "ctrl_encode_def.v"

module Hazard_Detect (
    // ---- Inputs from IF/ID Pipeline Register ----
    input wire       IF_ID_is_Branch,
    input wire [4:0] IF_ID_rs1,
    input wire [4:0] IF_ID_rs2,

    // ---- Inputs from ID/EX Pipeline Register ----
    input wire       ID_EX_RegWrite,
    input wire       ID_EX_MemRead,
    input wire [4:0] ID_EX_rd,

    // ---- Inputs from EX/MEM Pipeline Register ----
    input wire       EX_MEM_RegWrite,
    input wire       EX_MEM_MemRead, 
    input wire [4:0] EX_MEM_rd,
    
    // ---- Output Signal ----
    // 因为在always块中赋值，所以必须声明为reg类型
    output reg       stall
);
    always @(*) begin
        // --- Case 1: lw -> ALU ---
        // 条件：EX阶段是lw，ID阶段是ALU指令，且有依赖
        if ((ID_EX_MemRead && !IF_ID_is_Branch) &&
            ((ID_EX_rd != 0) && ((ID_EX_rd == IF_ID_rs1) || (ID_EX_rd == IF_ID_rs2)))) begin
            stall = 1'b1;
        
        // --- Case 2: ALU -> Branch ---
        // 条件：EX阶段是ALU指令，ID阶段是分支指令，且有依赖
        end else if ((ID_EX_RegWrite && !ID_EX_MemRead && IF_ID_is_Branch) &&
                     ((ID_EX_rd != 0) && ((ID_EX_rd == IF_ID_rs1) || (ID_EX_rd == IF_ID_rs2)))) begin
            stall = 1'b1;

        // --- Case 3: lw -> Branch (Cycle 1) ---
        // 条件：EX阶段是lw，ID阶段是分支，且有依赖
        end else if ((ID_EX_MemRead && IF_ID_is_Branch) &&
                     ((ID_EX_rd != 0) && ((ID_EX_rd == IF_ID_rs1) || (ID_EX_rd == IF_ID_rs2)))) begin
            stall = 1'b1;

        // --- Case 3: lw -> Branch (Cycle 2) ---
        // 条件：MEM阶段是lw，ID阶段是分支，且有依赖
        end else if ((EX_MEM_MemRead && IF_ID_is_Branch) &&
                     ((EX_MEM_rd != 0) && ((EX_MEM_rd == IF_ID_rs1) || (EX_MEM_rd == IF_ID_rs2)))) begin
            stall = 1'b1;
        
        // --- Default Case ---
        // 如果以上所有条件都不满足，则不产生停顿信号
        end else begin
            stall = 1'b0;
        end
    end

endmodule


module Forwarding(
    input wire       IF_ID_is_Branch,
    input EX_MEM_RegWrite,
    input MEM_WB_RegWrite,
    // input EX_MEM_MemWrite,
    input [4:0] EX_MEM_rd,
    input [4:0] MEM_WB_rd,
    input [4:0] ID_EX_rs1,
    input [4:0] ID_EX_rs2,


    input wire [4:0] IF_ID_rs1,      // ID阶段的rs1地址
    input wire [4:0] IF_ID_rs2,      // ID阶段的rs2地址


    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB,


    // ---- NEW: Outputs for ID Stage (Branch) ----
    output reg [1:0] ForwardID_A, // 控制送往分支比较器的操作数A
    output reg [1:0] ForwardID_B  // 控制送往分支比较器的操作数B
);

    // ForwardA
    always @(*) begin
        ForwardA = `Forward_None;    // default

        // Top priority: EX/MEM(latest) EX/M -> EX
        if (EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs1)) begin
            ForwardA = `Forward_EX;   // choose EX/MEM as the source
        end
        // Second priority: MEM/WB(earlier) MEM/W -> EX
        else if (MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs1) && (!(EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs1)))) begin
            ForwardA = `Forward_MEM;   // choose MEM/WB as the source
        end 
    end

    // ForwardB
    always @(*) begin
        ForwardB = `Forward_None;    // default

        if(EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs2)) begin
            ForwardB = `Forward_EX;   // choose EX/MEM as the source
        end
        else if(MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs2)  && (!(EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs2)))) begin
            ForwardB = `Forward_MEM;   // choose MEM/WB as the source
        end
    end


    // ... Part 2 ...
always @(*) begin
    // --- ForwardID_A Logic (for rs1) ---
    // 加上 IF_ID_is_Branch 条件，确保只为分支指令服务
    if (IF_ID_is_Branch && EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == IF_ID_rs1)) begin
        ForwardID_A = `Fwd_EX;
    end else if (IF_ID_is_Branch && MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == IF_ID_rs1)) begin
        ForwardID_A = `Fwd_WB;
    end else begin
        ForwardID_A = `Fwd_None;
    end

    // --- ForwardID_B Logic (for rs2) ---
    if (IF_ID_is_Branch && EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == IF_ID_rs2)) begin
        ForwardID_B = `Fwd_EX;
    end else if (IF_ID_is_Branch && MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == IF_ID_rs2)) begin
        ForwardID_B = `Fwd_WB;
    end else begin
        ForwardID_B = `Fwd_None;
    end
end

endmodule