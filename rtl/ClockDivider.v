module ClockDivider (
  input wire clk,          // Tu reloj original a 58.9820 MHz
  output reg clk_enable    // Tu "clock enable" ajustado a 22.1184 MHz
);

  reg [31:0] counter;
  parameter DIV_FACTOR = 0.3753 * 2**32;  // Calcula el factor de división y conviértelo a formato de 32 bits

  always @(posedge clk) begin
    counter <= counter + 1;
    if (counter == DIV_FACTOR - 1) begin
      counter <= 0;
      clk_enable <= ~clk_enable;  // Genera un pulso cada vez que el contador alcanza el valor deseado
    end
  end

endmodule
