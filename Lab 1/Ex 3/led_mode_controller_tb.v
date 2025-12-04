`timescale 1ns / 1ps

module led_mode_controller_tb;

    reg i_clk;
    reg i_rst;
    reg [3:0] i_btn;

    wire [3:0] o_led;
    
    defparam uut.divider.COUNT_MAX = 1;

    led_mode_controller uut (
        .i_clk(i_clk), 
        .i_rst(i_rst), 
        .i_btn(i_btn), 
        .o_led(o_led)
    );

    initial begin
        i_clk = 0;
        forever #4 i_clk = ~i_clk; 
    end

    initial begin
        i_rst = 1;
        i_btn = 4'b0000;
        #10;
        i_rst = 0;
        #10;

        i_btn = 4'b0010;
        #10;
        i_btn = 4'b0000;
        
        #300;

        i_btn = 4'b1000;
        #10;
        i_btn = 4'b0000;
        
        #300; 

        i_btn = 4'b0100;
        #10;
        i_btn = 4'b0000;
        
        #200; 

        i_btn = 4'b0001;
        #10;
        i_btn = 4'b0000;
        #10;
        
        #100;
        $stop;
    end
    
    initial begin
        $monitor("T=%0t | rst: %b, btn: %b, led: %b",
                 $time, i_rst, i_btn, o_led);
    end

endmodule