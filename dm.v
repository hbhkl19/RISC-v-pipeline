`include "ctrl_encode_def.v"
// data memory
module dm(clk, DMWr, addr, din, DMType,dout);
   input          clk;
   input          DMWr;
   input  [8:0]   addr;
   input  [31:0]  din;
   input [2:0] DMType;
   output reg [31:0]  dout;
  
  wire [8:2] Num;
  reg [31:0] dmem[255:0];
  assign Num=addr[8:2];
  always @(posedge clk) begin
    if (DMWr) begin
      case (DMType)
        `dm_byte: 
                case(addr[1:0])
                2'b00:dmem[Num][7:0] <= din[7:0];
                2'b01:dmem[Num][15:8] <= din[7:0];
                2'b10:dmem[Num][23:16] <= din[7:0];
                2'b11:dmem[Num][31:24] <= din[7:0];
                endcase
        `dm_byte_unsigned: 
                case(addr[1:0])
                2'b00:dmem[Num][7:0] <= din[7:0];
                2'b01:dmem[Num][15:8] <= din[7:0];
                2'b10:dmem[Num][23:16] <= din[7:0];
                2'b11:dmem[Num][31:24] <= din[7:0];
                endcase
        `dm_halfword: 
                case(addr[1:0])
                2'b00:dmem[Num][15:0] <= din[15:0];
                2'b01:dmem[Num][23:8] <= din[15:0];
                2'b10:dmem[Num][31:16] <= din[15:0];
                2'b11: begin
                    dmem[Num][31:24] <= din[7:0];
                    dmem[Num+1][7:0] <= din[15:8];
                    end
                endcase
        `dm_halfword_unsigned:
                case(addr[1:0])
                2'b00:dmem[Num][15:0] <= din[15:0];
                2'b01:dmem[Num][23:8] <= din[15:0];
                2'b10:dmem[Num][31:16] <= din[15:0];
                2'b11: begin
                    dmem[Num][31:24] <= din[7:0];
                    dmem[Num+1][7:0] <= din[15:8];
                    end
                endcase
        `dm_word: begin
          dmem[Num]<=din[31:0];
        end
      endcase
      //$display("dmem[0x%8X] = 0x%8X,", addr>>2, dmem[Num]);
    end
  end
   // read
always @(*) begin
      case (DMType)
        `dm_byte: 
                case(addr[1:0])
                2'b00:dout <= {{24{dmem[Num][7]}},dmem[Num][7:0]};
                2'b01:dout <= {{24{dmem[Num][15]}},dmem[Num][15:8]};
                2'b10:dout <= {{24{dmem[Num][23]}},dmem[Num][23:16]};
                2'b11:dout <= {{24{dmem[Num][31]}},dmem[Num][31:24]};
                endcase
        `dm_byte_unsigned: 
                case(addr[1:0])
                2'b00:dout <= {{24'b0},dmem[Num][7:0]};
                2'b01:dout <= {{24'b0},dmem[Num][15:8]};
                2'b10:dout <= {{24'b0},dmem[Num][23:16]};
                2'b11:dout <= {{24'b0},dmem[Num][31:24]};
                endcase
        `dm_halfword: 
                case(addr[1:0])
                2'b00:dout<={{16{dmem[Num][15]}},dmem[Num][15:0]};
                2'b01:dout <= {{16{dmem[Num][23]}},dmem[Num][23:8]};
                2'b10:dout <= {{16{dmem[Num][31]}},dmem[Num][31:16]};
                2'b11: dout <= {{16{dmem[Num+1][7]}},dmem[Num+1][7:0],dmem[Num][31:24]};
                endcase
        `dm_halfword_unsigned:
                case(addr[1:0])
                2'b00:dout<={16'b0,dmem[Num][15:0]};
                2'b01:dout <= {16'b0,dmem[Num][23:8]};
                2'b10:dout <= {16'b0,dmem[Num][31:16]};
                2'b11: dout <= {16'b0,dmem[Num+1][7:0],dmem[Num][31:24]};
                endcase
        `dm_word: dout <= dmem[Num];
      endcase
  end
endmodule    
