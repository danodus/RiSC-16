module top(
    input  wire logic       CLK,
    input  wire logic       BTN_N,
    // Display
    output      logic       LEDR_N,
    output      logic       LEDG_N,
    );

    // reset
    logic auto_reset;
    logic [5:0] auto_reset_counter = 0;
    logic reset;

    logic [15:0] display;
    assign {LEDR_N, LEDG_N} = ~display[1:0];

    toplevel soc(
        .clk(CLK),
        .pc(),
        .display(display)
    );

    // reset
    assign auto_reset = auto_reset_counter < 5'b11111;
    assign reset = auto_reset || !BTN_N;

	always_ff @(posedge CLK) begin
        auto_reset_counter <= auto_reset_counter + auto_reset;
	end

endmodule