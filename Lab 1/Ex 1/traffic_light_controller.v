`timescale 1ns / 1ps

module traffic_light_controller(
    input clk,
    input rst,
    input manual_auto_switch,
    input change_color_switch,

    output reg [2:0] traffic_light_1,
    output reg [2:0] traffic_light_2,

    output reg [3:0] bcd1_tens,
    output reg [3:0] bcd1_ones,
    output reg [3:0] bcd2_tens,
    output reg [3:0] bcd2_ones
    );

    parameter RED = 3'b100;
    parameter YELLOW = 3'b010;
    parameter GREEN = 3'b001;

    parameter S0 = 2'b00;
    parameter S1 = 2'b01;
    parameter S2 = 2'b10;
    parameter S3 = 2'b11;

    parameter GREEN_TIME = 7;
    parameter YELLOW_TIME = 5;
    parameter RED_TIME = GREEN_TIME + YELLOW_TIME;

    reg [1:0] state;
    reg [5:0] timer1;
    reg [5:0] timer2;

    wire one_second_tick;

    clk_divider divider (
        .clk(clk),
        .rst(rst),
        .tick(one_second_tick)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S0;
            timer1 <= GREEN_TIME;
            timer2 <= RED_TIME;
        end else if (!manual_auto_switch) begin
            if (one_second_tick) begin
                timer1 <= timer1 - 1;
                timer2 <= timer2 - 1;

                case(state)
                    S0: begin
                        if (timer1 == 1) begin
                            state <= S1;
                            timer1 <= YELLOW_TIME;
                        end
                    end
                    S1: begin
                        if (timer1 == 1) begin
                            state <= S2;
                            timer1 <= RED_TIME;
                            timer2 <= GREEN_TIME;
                        end
                    end
                    S2: begin
                        if (timer2 == 1) begin
                            state <= S3;
                            timer2 <= YELLOW_TIME;
                        end
                    end
                    S3: begin
                        if (timer2 == 1) begin
                            state <= S0;
                            timer1 <= GREEN_TIME;
                            timer2 <= RED_TIME;
                        end
                    end
                endcase
            end
        end
    end

    always @(*) begin
        if (manual_auto_switch) begin
            if (change_color_switch) begin
                {traffic_light_1, traffic_light_2} = {RED, GREEN};
            end else begin
                {traffic_light_1, traffic_light_2} = {GREEN, RED};
            end
        end else begin
            case(state)
                S0: {traffic_light_1, traffic_light_2} = {GREEN, RED};
                S1: {traffic_light_1, traffic_light_2} = {YELLOW, RED};
                S2: {traffic_light_1, traffic_light_2} = {RED, GREEN};
                S3: {traffic_light_1, traffic_light_2} = {RED, YELLOW};
                default: {traffic_light_1, traffic_light_2} = {RED, RED};
            endcase
        end
    end

    always @(*) begin
        if (manual_auto_switch) begin
            {bcd1_tens, bcd1_ones, bcd2_tens, bcd2_ones} = 16'd0;
        end else begin
            bcd1_tens = timer1 / 10;
            bcd1_ones = timer1 % 10;
            bcd2_tens = timer2 / 10;
            bcd2_ones = timer2 % 10;
        end
    end

endmodule