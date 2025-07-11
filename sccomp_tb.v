// testbench for simulation
module sccomp_tb();
   reg  clk, rstn;
   reg  [4:0] reg_sel;
   wire [31:0] reg_data;
    
// instantiation of sccomp    
   sccomp U_SCCOMP(
      .clk(clk), .rstn(rstn), .reg_sel(reg_sel), .reg_data(reg_data) 
   );

   integer foutput;
   integer counter = 0;
   
   initial begin
      $readmemh("fib.dat", U_SCCOMP.U_IM.ROM, 0, 18); // load instructions into instruction memory
      foutput = $fopen("results.txt");
      $dumpfile("fib.vcd");
      $dumpvars(0, sccomp_tb);
      clk = 1;
      rstn = 1;
      #5 ;
      rstn = 0;
      #20 ;
      rstn = 1;
      #1000 ;
      reg_sel = 7;
   end
   
//     always begin
//       #(50) clk = ~clk;
      
//       if (clk == 1'b1) begin
//         if ((counter == 1000) || (U_SCCOMP.U_SCPU.PC_out === 32'hxxxxxxxx)) begin
//           $fclose(foutput);
//           $stop;
//         end
//         else begin
//           if ((U_SCCOMP.PC == 32'h00000060) && (counter > 40) ) begin
//             counter = counter + 1;
//             $fdisplay(foutput, "pc:\t %h", U_SCCOMP.PC);
//             $fdisplay(foutput, "instr:\t\t %h", U_SCCOMP.instr);
//             $fdisplay(foutput, "rf00-03:\t %h %h %h %h", 0, U_SCCOMP.U_SCPU.U_RF.rf[1], U_SCCOMP.U_SCPU.U_RF.rf[2], U_SCCOMP.U_SCPU.U_RF.rf[3]);
//             $fdisplay(foutput, "rf04-07:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[4], U_SCCOMP.U_SCPU.U_RF.rf[5], U_SCCOMP.U_SCPU.U_RF.rf[6], U_SCCOMP.U_SCPU.U_RF.rf[7]);
//             $fdisplay(foutput, "rf08-11:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[8], U_SCCOMP.U_SCPU.U_RF.rf[9], U_SCCOMP.U_SCPU.U_RF.rf[10], U_SCCOMP.U_SCPU.U_RF.rf[11]);
//             $fdisplay(foutput, "rf12-15:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[12], U_SCCOMP.U_SCPU.U_RF.rf[13], U_SCCOMP.U_SCPU.U_RF.rf[14], U_SCCOMP.U_SCPU.U_RF.rf[15]);
//             $fdisplay(foutput, "rf16-19:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[16], U_SCCOMP.U_SCPU.U_RF.rf[17], U_SCCOMP.U_SCPU.U_RF.rf[18], U_SCCOMP.U_SCPU.U_RF.rf[19]);
//             $fdisplay(foutput, "rf20-23:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[20], U_SCCOMP.U_SCPU.U_RF.rf[21], U_SCCOMP.U_SCPU.U_RF.rf[22], U_SCCOMP.U_SCPU.U_RF.rf[23]);
//             $fdisplay(foutput, "rf24-27:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[24], U_SCCOMP.U_SCPU.U_RF.rf[25], U_SCCOMP.U_SCPU.U_RF.rf[26], U_SCCOMP.U_SCPU.U_RF.rf[27]);
//             $fdisplay(foutput, "rf28-31:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[28], U_SCCOMP.U_SCPU.U_RF.rf[29], U_SCCOMP.U_SCPU.U_RF.rf[30], U_SCCOMP.U_SCPU.U_RF.rf[31]);
//             //$fdisplay(foutput, "hi lo:\t %h %h", U_SCCOMP.U_SCPU.U_RF.rf.hi, U_SCCOMP.U_SCPU.U_RF.rf.lo);
//             $fclose(foutput);
//             $stop;
//           end
//           else begin
//             counter = counter + 1;
// //            $display("pc: %h", U_SCCOMP.U_SCPU.PC);
// //            $display("instr: %h", U_SCCOMP.U_SCPU.instr);
//           end
//         end
//       end
//     end //end always


  // sccomp_tb.v (Corrected Version)

always begin
    #(50) clk = ~clk;
    
    if (clk == 1'b1) begin
        // 我们现在使用 counter 作为主要的停止和报告条件
        // 运行100个周期，足以让这个短程序完全执行并排空流水线
        if ((counter == 20000) || (U_SCCOMP.U_SCPU.PC_out === 32'hxxxxxxxx)) begin
            // 当我们确定程序已经结束后，再把所有寄存器的最终状态写入文件
            $fdisplay(foutput, "Simulation stopped at counter = %d", counter);
            $fdisplay(foutput, "Final PC:\t %h", U_SCCOMP.PC);
            $fdisplay(foutput, "rf00-03:\t %h %h %h %h", 0, U_SCCOMP.U_SCPU.U_RF.rf[1], U_SCCOMP.U_SCPU.U_RF.rf[2], U_SCCOMP.U_SCPU.U_RF.rf[3]);
            $fdisplay(foutput, "rf04-07:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[4], U_SCCOMP.U_SCPU.U_RF.rf[5], U_SCCOMP.U_SCPU.U_RF.rf[6], U_SCCOMP.U_SCPU.U_RF.rf[7]);
            $fdisplay(foutput, "rf08-11:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[8], U_SCCOMP.U_SCPU.U_RF.rf[9], U_SCCOMP.U_SCPU.U_RF.rf[10], U_SCCOMP.U_SCPU.U_RF.rf[11]);
            $fdisplay(foutput, "rf12-15:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[12], U_SCCOMP.U_SCPU.U_RF.rf[13], U_SCCOMP.U_SCPU.U_RF.rf[14], U_SCCOMP.U_SCPU.U_RF.rf[15]);
            $fdisplay(foutput, "rf16-19:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[16], U_SCCOMP.U_SCPU.U_RF.rf[17], U_SCCOMP.U_SCPU.U_RF.rf[18], U_SCCOMP.U_SCPU.U_RF.rf[19]);
            $fdisplay(foutput, "rf20-23:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[20], U_SCCOMP.U_SCPU.U_RF.rf[21], U_SCCOMP.U_SCPU.U_RF.rf[22], U_SCCOMP.U_SCPU.U_RF.rf[23]);
            $fdisplay(foutput, "rf24-27:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[24], U_SCCOMP.U_SCPU.U_RF.rf[25], U_SCCOMP.U_SCPU.U_RF.rf[26], U_SCCOMP.U_SCPU.U_RF.rf[27]);
            $fdisplay(foutput, "rf28-31:\t %h %h %h %h", U_SCCOMP.U_SCPU.U_RF.rf[28], U_SCCOMP.U_SCPU.U_RF.rf[29], U_SCCOMP.U_SCPU.U_RF.rf[30], U_SCCOMP.U_SCPU.U_RF.rf[31]);
            
            $fclose(foutput);
            $stop;
        end
        else begin
            counter = counter + 1;
        end
    end
end
   
endmodule
