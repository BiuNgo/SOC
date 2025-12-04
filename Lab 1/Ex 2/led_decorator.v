`timescale 1ns / 1ps

module led_decorator(
    input clk,
    input rst,
    input sw0,
    output reg [1:0] led,
    
    output reg [3:0] bcd3,
    output reg [3:0] bcd2,
    output reg [3:0] bcd1,
    output reg [3:0] bcd0
);

    localparam CHAR_C     = 4'h2;
    localparam CHAR_E     = 4'h5;
    localparam CHAR_BLANK = 4'hF;
    
    localparam PATTERN_STAGES = 6;
    localparam WINDOW_SIZE    = 4;
    localparam MAX_POSITION   = PATTERN_STAGES - WINDOW_SIZE;
    
    reg [3:0] pattern [0:PATTERN_STAGES-1];
    initial begin
        pattern[0] = CHAR_BLANK;
        pattern[1] = CHAR_BLANK;
        pattern[2] = CHAR_C;
        pattern[3] = CHAR_E;
        pattern[4] = CHAR_BLANK;
        pattern[5] = CHAR_BLANK;
    end

    reg [$clog2(PATTERN_STAGES)-1:0] position = 0;
    reg direction = 0;

    wire tick;
    clk_divider divider (
        .clk(clk),
        .rst(rst),
        .tick(tick)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            position <= 0;
            direction <= 0;
        end else if (tick) begin
            if (sw0 == 0) begin
                position <= (position == PATTERN_STAGES - 1) ? 0 : position + 1;
                direction <= 0;
            end else begin
                if (direction == 0) begin
                    if (position == MAX_POSITION) begin
                        direction <= 1;
                        position <= position - 1;
                    end else begin
                        position <= position + 1;
                    end
                end else begin
                    if (position == 0) begin 
                        direction <= 0;
                        position <= position + 1;
                    end else begin
                        position <= position - 1;
                    end
                end
            end
        end
    end
    
    always @(*) begin
        bcd3 = pattern[(position + 0) % PATTERN_STAGES];
        bcd2 = pattern[(position + 1) % PATTERN_STAGES];
        bcd1 = pattern[(position + 2) % PATTERN_STAGES];
        bcd0 = pattern[(position + 3) % PATTERN_STAGES];
        
        led[0] = ~sw0;
        led[1] = sw0;
    end

endmodule