/*
 * Verilog MSX Keyboard Module
 * Copyright (c) rampa 2024
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

module keyboard (
    input          clk,            // Clock input
    input          reset,          // Reset input
    input          key_strobe,     // Signal indicating a key strobe
    input          key_pressed,    // Signal indicating a key press
    input          key_extended,   // Signal indicating an extended key
    input   [7:0]  key_code,       // 8-bit input for the key code
    input   [3:0]  kb_row,         // 4-bit input for the keyboard row
    output  [7:0]  kb_data         // 8-bit output for the keyboard data
);

    logic [3:0] row;                    // 4-bit wire for the keyboard row
    logic [7:0] row_state [16] = '{default:8'hFF}; // 8-bit array representing the state of each row
    logic [8:0] key_decode;             // 9-bit signal formed by concatenating key_extended and key_code
    logic [7:0] pos;                    // 8-bit signal with a single '1' at the position specified by map

    // Assignments
    assign key_decode = {key_extended, key_code};
    assign row = map[7:4];
    assign pos = 8'b1 << map[3:0];
    assign kb_data = row_state[kb_row]; // Update kb_data 
 
    // Registers for change detection and key press
    logic change;
    logic down;
	 
    // Clocked process for change detection and key press
    always @(posedge clk) begin
		  // Registers for change detection and key press
        logic change;
        logic down;
  		  logic [10:0] old_signals;
        logic [10:0] key_signals;

  		  // Reset initialization
        if (reset) begin
            change <= 1'b0;
            down <= 1'b0;
            old_signals <= 11'h00;
				key_signals <= 11'h00;
				row_state <= '{default:8'hFF};
        end
        else begin
            // Update change, down, and old_row based on key signals
            change <= 1'b0;
            old_signals <= key_signals;
				key_signals <= {key_strobe,key_pressed,key_extended,key_code};

            // Detect change in key_strobe
            if (old_signals != key_signals) begin
                down <= key_pressed;
                change <= 1'b1;
            end
				if (change) row_state[row] <= key_pressed ? row_state[row] & ~pos : row_state[row] | pos;
       end
    end

    // ROM Instance (rom_kbd)
    wire [7:0] map;
    rom_kbd rom_kbd(
        .clock(clk),
        .address(key_decode),
        .q(map),
    );

endmodule
