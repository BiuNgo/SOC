`timescale 1ns / 1ps

module clk_divider(
    input clk,
    input rst,
    output reg tick
);
    parameter COUNT_MAX = 125000000;
    
    reg [26:0] counter = 0;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            counter <= 0;
            tick <= 0;
        end
        else begin
            if (counter == COUNT_MAX - 1) begin
                counter <= 0;
                tick <= 1;
            end else begin
                counter <= counter + 1;
                tick <= 0;
            end
        end
    end

endmodule