
// instruction memory
module im(input  [9:2]  addr,
            output [31:0] dout );

  reg  [31:0] ROM[255:0];

  assign dout = ROM[addr]; // word aligned
endmodule  
