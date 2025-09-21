/* Colour Cycling
RED (1,0,0)
YELLOW (1,1,0)
GREEN (0,1,0)
CYAN (0,1,1)
BLUE (0,0,1)
MAGENTA (1,0,1)
set the LED to the bit
*/

module fsm #(
    parameter STATE_INTERVAL = 2000000,     // CLK freq is 12MHz, so 6,000,000 cycles is 0.5s. 12 000 000 cycles is 1s. 2 000 000 cycles per colour.
)(
    input logic     clk, 
    output logic    red, // led
    output logic    green, 
    output logic    blue
);

    // Define state variable values
    localparam RED = 3'b100; // the various states represented by bits
    localparam YELLOW = 3'b110;
    localparam GREEN = 3'b010;
    localparam CYAN = 3'b011;
    localparam BLUE = 3'b001;
    localparam MAGENTA = 3'b101;

    // Declare state variables
    logic [2:0] current_state = RED;
    logic [2:0] next_state;

    // Declare next output variables
    logic next_red, next_green, next_blue;

    // Declare counter variables for when to change states
    logic [$clog2(STATE_INTERVAL) - 1:0] count = 0;
    logic state_done;

    // Register the next state of the FSM
    always_ff @(posedge clk)
        current_state <= next_state;

    // Compute the next state of the FSM
    always_comb begin
        next_state = 3'bxxx;
        case (current_state)
            RED:
                if (state_done == 1'b1)
                    next_state = YELLOW;
                else
                    next_state = RED;
            YELLOW:
                if (state_done == 1'b1)
                    next_state = GREEN;
                else
                    next_state = YELLOW;
            GREEN:
                if (state_done == 1'b1)
                    next_state = CYAN;
                else
                    next_state = GREEN;
            CYAN:
                if (state_done == 1'b1)
                    next_state = BLUE;
                else
                    next_state = CYAN;
            BLUE:
                if (state_done == 1'b1)
                    next_state = MAGENTA;
                else
                    next_state = BLUE;
            MAGENTA:
                if (state_done == 1'b1)
                    next_state = RED;
                else
                    next_state = MAGENTA;
        endcase
    end

    // Register the FSM outputs
    always_ff @(posedge clk) begin
        red <= next_red;
        green <= next_green;
        blue <= next_blue;
    end

    // Compute next output values
    always_comb begin
        next_red = next_state[0];
        next_green = next_state[1];
        next_blue = next_state[2];
    end

    always_ff @(posedge clk) begin
            if (count == STATE_INTERVAL - 1) begin
                count <= 0;
            end
            else begin
                count <= count + 1;
            end
        end

    assign state_done = (count == STATE_INTERVAL - 1) ? 1'b1 : 1'b0;

endmodule
