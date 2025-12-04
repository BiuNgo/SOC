`timescale 1ns / 1ps

module led_decorator_tb;

    reg clk;
    reg rst;
    reg sw0;

    wire [1:0] led;
    wire [3:0] bcd3, bcd2, bcd1, bcd0;
    
    defparam uut.divider.COUNT_MAX = 1;

    led_decorator uut (
        .clk(clk),
        .rst(rst),
        .sw0(sw0),
        .led(led),
        .bcd3(bcd3),
        .bcd2(bcd2),
        .bcd1(bcd1),
        .bcd0(bcd0)
    );

    initial begin
        clk = 0;
        forever #4 clk = ~clk;
    end

    initial begin
        rst = 1;
        sw0 = 0;
        #20;
        rst = 0;
        
        #100;

        sw0 = 1;
        
        #200;
        
        rst = 1;
        #20;
        rst = 0;

        #100;
        
        sw0 = 0;
        #200;

        $finish;
    end

endmodule