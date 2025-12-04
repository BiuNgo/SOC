`timescale 1ns / 1ps

module traffic_light_controller_tb;

    reg clk;
    reg rst;
    reg manual_auto_switch;
    reg change_color_switch;

    wire [2:0] traffic_light_1;
    wire [2:0] traffic_light_2;
    wire [3:0] bcd1_tens;
    wire [3:0] bcd1_ones;
    wire [3:0] bcd2_tens;
    wire [3:0] bcd2_ones;
    
    defparam uut.divider.COUNT_MAX = 1;

    traffic_light_controller uut (
        .clk(clk),
        .rst(rst),
        .manual_auto_switch(manual_auto_switch),
        .change_color_switch(change_color_switch),
        .traffic_light_1(traffic_light_1),
        .traffic_light_2(traffic_light_2),
        .bcd1_tens(bcd1_tens),
        .bcd1_ones(bcd1_ones),
        .bcd2_tens(bcd2_tens),
        .bcd2_ones(bcd2_ones)
    );

    initial begin
        clk = 0;
        forever #4 clk = ~clk;
    end

    initial begin
        rst = 1;
        manual_auto_switch = 0;
        change_color_switch = 0;
        #10;
        rst = 0;

        #300;

        manual_auto_switch = 1;
        #20;

        change_color_switch = 0;
        #50;

        change_color_switch = 1;
        #50;

        manual_auto_switch = 0;

        #300;

        $finish;
    end

endmodule