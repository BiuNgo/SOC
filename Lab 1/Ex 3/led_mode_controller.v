`timescale 1ns / 1ps

module led_mode_controller(
    input  i_clk,
    input  i_rst,
    input  [3:0] i_btn,
    output [3:0] o_led
    );

    parameter S_RESET       = 2'b00;
    parameter S_SHIFT_LEFT  = 2'b01;
    parameter S_SHIFT_RIGHT = 2'b10;
    parameter S_PAUSE       = 2'b11;
    
    reg [1:0] r_main_sm = S_RESET;

    reg  [3:0] r_led_data = 4'b0011;
    wire       w_tick;

    clk_divider divider (
        .clk(i_clk),
        .rst(i_rst),
        .tick(w_tick)
    );

    always @(posedge i_clk or posedge i_rst) begin
        if (i_rst) begin
            r_main_sm <= S_RESET;
            r_led_data <= 4'b0011;
        end
        else begin
            if (i_btn[0]) begin
                r_main_sm <= S_RESET;
                r_led_data <= 4'b0011;
            end 
            else begin
                case (r_main_sm)
                    S_RESET: begin
                        r_led_data <= 4'b0011;
                        if (i_btn[1])       r_main_sm <= S_SHIFT_LEFT;
                        else if (i_btn[2])  r_main_sm <= S_SHIFT_RIGHT;
                    end

                    S_SHIFT_LEFT: begin
                        if (w_tick) begin
                            r_led_data <= {r_led_data[2:0], r_led_data[3]};
                        end
                        
                        if (i_btn[2])       r_main_sm <= S_SHIFT_RIGHT;
                        else if (i_btn[3])  r_main_sm <= S_PAUSE;
                    end

                    S_SHIFT_RIGHT: begin
                        if (w_tick) begin
                            r_led_data <= {r_led_data[0], r_led_data[3:1]};
                        end
                        
                        if (i_btn[1])       r_main_sm <= S_SHIFT_LEFT;
                        else if (i_btn[3])  r_main_sm <= S_PAUSE;
                    end
                    
                    S_PAUSE: begin
                        if (i_btn[1])       r_main_sm <= S_SHIFT_LEFT;
                        else if (i_btn[2])  r_main_sm <= S_SHIFT_RIGHT;
                    end

                    default: begin
                        r_main_sm <= S_RESET;
                    end
                endcase
            end
        end
    end
    
    assign o_led = r_led_data;

endmodule