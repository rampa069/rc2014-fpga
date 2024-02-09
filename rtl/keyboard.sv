module keyboard
(
   input         clk,
	input         reset,
	input         key_strobe,
	input         key_pressed,
	input         key_extended,
	input   [7:0] key_code,
   input   [3:0] kb_row,
   output  [7:0] kb_data
);

wire [3:0] row;
logic [7:0] row_state [16] = '{default:8'hFF};
logic [8:0] key_decode;
logic [7:0] pos;

assign key_decode = {key_extended,key_code};
assign row = map[7:4];
assign pos = 8'b1 << map[3:0];
assign kb_data = row_state[kb_row];


reg change;
reg down;

always @(negedge clk) begin
 logic old_strobe;
   change     <= 1'b0;
   old_strobe <= key_strobe;
   if (old_strobe != key_strobe)
	begin
      down       <= key_pressed;
      change     <= 1'b1;
   end
end


always @(posedge clk) begin
   if (reset) begin
	  row_state <= '{default:8'hFF};
	end
	else begin
     if (change) row_state[row] <= key_pressed ?  row_state[row] & ~pos : row_state[row] | pos;
   end	  
end

wire [7:0] map;
rom_kbd rom_kbd(
   .clock(clk),
   .address(key_decode),
   .q(map),
);

endmodule