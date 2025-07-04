module RF(     input         clk, 
               input         rst,
               input         RFWr, 
               input  [4:0]  A1, A2, A3, 
               input  [31:0] WD, 
               output [31:0] RD1, RD2);

  reg [31:0] rf[31:0];

  integer i;

  always @(posedge clk, posedge rst)begin
    if (rst) begin    //  reset
      for (i=1; i<32; i=i+1)
        rf[i] <= 0; //  i;
    end
    else 
      if (RFWr) begin
        if(A3!=0)
        rf[A3] <= WD;
      end
  end

  // Debug output
  always @(posedge clk) begin
    if (RFWr) begin
        if(A3!=0||WD==0)
        $display("x%0d = 0x%8X", A3, WD);
    end
  end
  
  wire [31:0] rf_read_data1 = (A1 != 0) ? rf[A1] : 32'b0;
  wire [31:0] rf_read_data2 = (A2 != 0) ? rf[A2] : 32'b0;


  wire bypass_for_rd1 = RFWr && (A3 != 0) && (A3 == A1);
    // 检查写操作是否与读操作2冲突
  wire bypass_for_rd2 = RFWr && (A3 != 0) && (A3 == A2);

  assign RD1 = bypass_for_rd1 ? WD : rf_read_data1;
  assign RD2 = bypass_for_rd2 ? WD : rf_read_data2;

  // assign RD1 = (A1 != 0) ? rf[A1] : 0;
  // assign RD2 = (A2 != 0) ? rf[A2] : 0;
  //assign reg_data = (reg_sel != 0) ? rf[reg_sel] : 0; 

endmodule 
