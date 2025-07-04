module PC( clk, rst, NPC, PC ,stall);

  input              clk;
  input              rst;
  input       [31:0] NPC;
  input             stall; // 停止信号
  output reg  [31:0] PC;

  always @(posedge clk, posedge rst)
    if (rst) 
      PC <= 32'h0000_0000;
//      PC <= 32'h0000_3000;
    else if (!stall) // 如果没有停止信号，则更新 PC
      PC <= NPC;
      
endmodule

