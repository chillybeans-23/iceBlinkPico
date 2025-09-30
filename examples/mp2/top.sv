`include "rgb_cycle.sv"
`include "pwm.sv"

// Fade top level module

module top #(
    parameter PWM_INTERVAL = 1200       // CLK frequency is 12MHz, so 1,200 cycles is 100us
    
)(
    input logic     clk, 
    output logic    RGB_R,
    output logic    RGB_G,
    output logic    RGB_B
);

    localparam PWM_INC = 3'b000;
    localparam PWM_DEC = 3'b001;
    localparam PWM_LOW1 = 3'b010;
    localparam PWM_HIGH1 = 3'b011;
    localparam PWM_LOW2 = 3'b100;
    localparam PWM_HIGH2 = 3'b101;

    logic [$clog2(PWM_INTERVAL) - 1:0] pwm_value_r;
    logic pwm_out_r;
    logic [2:0] start_state_r = PWM_HIGH2;
    logic [$clog2(PWM_INTERVAL) - 1:0] pwm_value_g;
    logic pwm_out_g;
    logic [2:0] start_state_g = PWM_INC;
    logic [$clog2(PWM_INTERVAL) - 1:0] pwm_value_b;
    logic pwm_out_b;
    logic [2:0] start_state_b = PWM_LOW1;


    rgb_cycle #(
        .PWM_INTERVAL   (PWM_INTERVAL),
        .START_STATE    (3'b101)
    ) u1 (
        .clk            (clk), 
        .pwm_value      (pwm_value_r)
    );

    pwm #(
        .PWM_INTERVAL   (PWM_INTERVAL)
    ) u2 (
        .clk            (clk), 
        .pwm_value      (pwm_value_r), 
        .pwm_out        (pwm_out_r)
    );


    rgb_cycle #(
        .PWM_INTERVAL   (PWM_INTERVAL),
        .START_STATE    (3'b000)
    ) u3 (
        .clk            (clk), 
        .pwm_value      (pwm_value_g)
    );

    pwm #(
        .PWM_INTERVAL   (PWM_INTERVAL)
    ) u4 (
        .clk            (clk), 
        .pwm_value      (pwm_value_g), 
        .pwm_out        (pwm_out_g)
    );


    rgb_cycle #(
        .PWM_INTERVAL   (PWM_INTERVAL),
        .START_STATE    (3'b010)
    ) u5 (
        .clk            (clk), 
        .pwm_value      (pwm_value_b)
    );

    pwm #(
        .PWM_INTERVAL   (PWM_INTERVAL)
    ) u6 (
        .clk            (clk), 
        .pwm_value      (pwm_value_b), 
        .pwm_out        (pwm_out_b)
    );

    assign RGB_R = ~pwm_out_r;
    assign RGB_G = ~pwm_out_g;
    assign RGB_B = ~pwm_out_b;

endmodule
