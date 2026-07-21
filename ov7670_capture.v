/*
> Decimation (Reducing a large portion):
Instead of relying on the camera's internal (which has bug) scaler to shrink a 640x480 VGA image down to 320x240 QVGA.
We take directly VGA data from camera module and the FPGA mathematically removes half the pixels. It is 'Downsampling' 
from 2:1.

> RGB565 to RGB444:
Byte1: R4 R3 R2 R1 R0 G5 G4 G3 ; Byte2: G2 G1 G0 B4 B3 B2 B1 B0
RGB444 = {byte1[7:4], byte1[2:0], byte2[7], byte2[4:1]}
*/

`timescale 1ns / 1ps

module ov7670_capture #(
    parameter integer H_ACT = 320,
    parameter integer V_ACT = 240
    //Target Resolution (H_ACT x V_ACT) needed from capture module.
)(
    input  wire pclk, //Pixel clock from camera
    input  wire rst,
    //'vsync' will be kept high in vertical blanking period of camera & kept low during active frame.
    //'href' will be kept high during active row & it goes low in horizontal blanking period & when it gets high again means new row will occur.
    input  wire vsync, //Vsync from camera.
    input  wire href,
    input  wire [7:0] d,//Incoming single byte data.
    //A single RGB565 pixel is 16 bits (2 bytes) wide, it takes two clock cycles for the camera to send one full pixel.
    output reg [11:0] pixel, //RGB444
    output reg pixel_valid
);
    localparam integer H_IN = 2*H_ACT;
    localparam integer V_IN = 2*V_ACT;
    //Camera send VGA signal, so we need to read upto that.
    
    reg byte_sel; //Denotes which byte we are recieving.
    reg [7:0] byte1; //Temporarily holds the first byte while second byte to arrive on the next clock cycle.
    //To determine location on the screen.
    reg [10:0] col;
    reg [10:0] row;
    reg href_d; //1-bit delay register.

    wire take = (~col[0] & ~row[0]);
    /*
    In binary, if the LSB is '0', the number is even & if LSB is '1', the number is odd.
    ~col[0] & ~row[0] means "Only evaluate to True if we are on an Even Row AND an Even Column." 
    This forces the FPGA to drop 3 out of every 4 pixels (0,0 is kept. 0,1 dropped. 1,0 dropped. 1,1 dropped).
    For column counter: 0 -> ...000 (LSB is 0, Even)
                        1 -> ...001 (LSB is 1, Odd)
                        2 -> ...010 (LSB is 0, Even)
                        3 -> ...011 (LSB is 1, Odd)
    */
    wire in_win = (col < H_IN[10:0]) && (row < V_IN[10:0]);
    //'href' ignores the blanking periods. Because cameras are analog-digital. Sometimes, the camera might accidentally hold href HIGH for 642 clocks     instead of 640, or send 481 rows instead of 480. To avoid that 'in_win' is bounded.

    always @(posedge pclk) begin
        if (rst) begin
            byte_sel<=1'b0; 
            byte1<=8'd0; 
            pixel<=12'd0; 
            pixel_valid<=1'b0;
            col<=11'd0; 
            row<=11'd0; 
            href_d<=1'b0;
        end
        else begin
            pixel_valid <= 1'b0;
            href_d <= href; //Creating 1 clock cycle delay.

            if (vsync) begin //New Frame will start.
                byte_sel<=1'b0; 
                col<=11'd0; 
                row<=11'd0; 
                href_d<=1'b0;
            end
            else if (href_d && !href) begin //1 row finished. href_d is '1' but href is '0'.
                col<=11'd0; 
                byte_sel<=1'b0; //Reset column to 0.
                if (row < V_IN[10:0]) row <= row + 1'b1; //Update the row if it didn't not hit the limit.
            end
            else if (href) begin //Camera is producing data.                 
                if (byte_sel == 1'b0) begin //If it's 1st byte.
                    byte1<=d; 
                    byte_sel<=1'b1;
                end
                else begin
                    byte_sel<=1'b0;
                    if (take && in_win) begin //Checks the even cordinates & target window.
                        pixel <= {byte1[7:4], byte1[2:0], d[7], d[4:1]};
                        pixel_valid <= 1'b1; //'pixel_valid' defaults to 0 every single cycle. It becomes 1 when a pixel is completely finished.
                    end
                    col <= col + 1'b1;
                end
            end
            else begin
                byte_sel<=1'b0;
                /*
                When camera is transmitting Pixel 1. It sends A1, and our FPGA catches it (byte_sel becomes 1).
                But suddenly, before A2 can be sent, a static shock hits the board. The href wire drops to LOW prematurely. 
                If we did not have that safety else block, byte_sel would stay stuck at 1 & functioning will be flipped.
                */
            end
        end
    end
endmodule
