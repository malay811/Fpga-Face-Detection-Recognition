//============================================================================
// osd_overlay.v  (two-identity version)
//----------------------------------------------------------------------------
// Paints the detection result onto the live 640x480 RGB444 stream.
//   - Green box outline (0x0F0) around the detected face.
//   - White name (0xFFF) inside the box top-left, ONLY when recognised
//     (match=1): "MALAY" (match_id=0) or "NILESH" (match_id=1).
//   - Detected-but-not-recognised (match=0) -> box only, no name.
//   - Everything else -> camera pixel passthrough.
//
// Text region widened to fit "NILESH" (6 glyphs) instead of the old fixed
// "MALAY" (5 glyphs): 6*16 = 96px wide (was 80px). MALAY's 6th slot is
// blanked via a dedicated all-zero glyph so both names share one region
// size without changing box geometry.
//
// Font: 9 unique 8x8 glyphs (M,A,L,Y,N,I,E,S,H) + 1 blank, in a distributed
// ROM. Rendered x2 -> each glyph 16x16, string up to 96x16 wide, placed at
// (X0+4, Y0+4).
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module osd_overlay #(
    parameter [11:0] COL_BOX  = 12'h0F0,   // green
    parameter [11:0] COL_TEXT = 12'hFFF,   // white
    parameter integer THICK   = 2          // border thickness (px)
)(
    input  wire        clk,
    input  wire        rst,
    // display coords (signed) + active-region pixel
    input  wire signed [10:0] sx,
    input  wire signed [10:0] sy,
    input  wire        de,                 // display enable (active pixel)
    input  wire [11:0] pix_in,             // camera RGB444
    // detection result (pixel-domain, stable)
    input  wire        box_valid,
    input  wire        match,              // recognised as someone
    input  wire        match_id,           // 0=MALAY, 1=NILESH
    input  wire [8:0]  box_x,              // detection space 0..256
    input  wire [7:0]  box_y,              // 0..176
    output reg  [11:0] pix_out
);

    // ---- box in display space ----
    wire [10:0] X0 = {1'b0, box_x, 1'b0};       // box_x*2
    wire [10:0] Y0 = {2'b0, box_y, 1'b0};       // box_y*2
    wire [10:0] X1 = X0 + 11'd127;
    wire [10:0] Y1 = Y0 + 11'd127;

    // active-region unsigned coords
    wire in_active = de && !sx[10] && !sy[10];
    wire [10:0] ux = sx[10:0];
    wire [10:0] uy = sy[10:0];

    wire in_x = (ux >= X0) && (ux <= X1);
    wire in_y = (uy >= Y0) && (uy <= Y1);

    wire on_lr = in_x && in_y && ((ux < X0 + THICK) || (ux > X1 - THICK));
    wire on_tb = in_x && in_y && ((uy < Y0 + THICK) || (uy > Y1 - THICK));
    wire on_border = box_valid && in_active && (on_lr || on_tb);

    // ---- name text region: 96x16 at (X0+4, Y0+4), 6 glyph slots ----
    localparam [10:0] TX = 11'd4, TY = 11'd4;
    wire [10:0] txs = X0 + TX;
    wire [10:0] tys = Y0 + TY;
    wire in_text_box = in_active && match && box_valid
                       && (ux >= txs) && (ux < txs + 11'd96)
                       && (uy >= tys) && (uy < tys + 11'd16);

    wire [10:0] rel_x = ux - txs;                  // 0..95
    wire [10:0] rel_y = uy - tys;                  // 0..15
    wire [2:0]  char_i = rel_x[6:4];               // which glyph slot 0..5 (16 px each)
    wire [2:0]  gcol   = rel_x[3:1];               // column in 8x8 (x2 scale)
    wire [2:0]  grow   = rel_y[3:1];               // row in 8x8

    // glyph ids: M=0 A=1 L=2 Y=3 N=4 I=5 E=6 S=7 H=8 BLANK=9
    reg [3:0] glyph_id;
    always @(*) begin
        if (!match_id) begin
            // MALAY (5 letters + 1 blank slot)
            case (char_i)
                3'd0: glyph_id = 4'd0; // M
                3'd1: glyph_id = 4'd1; // A
                3'd2: glyph_id = 4'd2; // L
                3'd3: glyph_id = 4'd1; // A
                3'd4: glyph_id = 4'd3; // Y
                default: glyph_id = 4'd9; // blank
            endcase
        end else begin
            // NILESH (6 letters)
            case (char_i)
                3'd0: glyph_id = 4'd4; // N
                3'd1: glyph_id = 4'd5; // I
                3'd2: glyph_id = 4'd2; // L
                3'd3: glyph_id = 4'd6; // E
                3'd4: glyph_id = 4'd7; // S
                default: glyph_id = 4'd8; // H
            endcase
        end
    end

    // ---- 8x8 font ROM: 10 glyphs x 8 rows (distributed) ----
    (* rom_style = "distributed" *) reg [7:0] font [0:79];
    initial begin
        // M (0)
        font[ 0]=8'h81; font[ 1]=8'hC3; font[ 2]=8'hA5; font[ 3]=8'h99;
        font[ 4]=8'h81; font[ 5]=8'h81; font[ 6]=8'h81; font[ 7]=8'h00;
        // A (1)
        font[ 8]=8'h3C; font[ 9]=8'h66; font[10]=8'hC3; font[11]=8'hC3;
        font[12]=8'hFF; font[13]=8'hC3; font[14]=8'hC3; font[15]=8'h00;
        // L (2)
        font[16]=8'hC0; font[17]=8'hC0; font[18]=8'hC0; font[19]=8'hC0;
        font[20]=8'hC0; font[21]=8'hC0; font[22]=8'hFF; font[23]=8'h00;
        // Y (3)
        font[24]=8'hC3; font[25]=8'h66; font[26]=8'h3C; font[27]=8'h18;
        font[28]=8'h18; font[29]=8'h18; font[30]=8'h18; font[31]=8'h00;
        // N (4)
        font[32]=8'hC3; font[33]=8'hE3; font[34]=8'hF3; font[35]=8'hDB;
        font[36]=8'hCF; font[37]=8'hC7; font[38]=8'hC3; font[39]=8'h00;
        // I (5)
        font[40]=8'h3C; font[41]=8'h18; font[42]=8'h18; font[43]=8'h18;
        font[44]=8'h18; font[45]=8'h18; font[46]=8'h3C; font[47]=8'h00;
        // E (6)
        font[48]=8'hFF; font[49]=8'hC0; font[50]=8'hC0; font[51]=8'hFC;
        font[52]=8'hC0; font[53]=8'hC0; font[54]=8'hFF; font[55]=8'h00;
        // S (7)
        font[56]=8'h7E; font[57]=8'hC0; font[58]=8'hC0; font[59]=8'h7C;
        font[60]=8'h03; font[61]=8'h03; font[62]=8'hFC; font[63]=8'h00;
        // H (8)
        font[64]=8'hC3; font[65]=8'hC3; font[66]=8'hC3; font[67]=8'hFF;
        font[68]=8'hC3; font[69]=8'hC3; font[70]=8'hC3; font[71]=8'h00;
        // BLANK (9)
        font[72]=8'h00; font[73]=8'h00; font[74]=8'h00; font[75]=8'h00;
        font[76]=8'h00; font[77]=8'h00; font[78]=8'h00; font[79]=8'h00;
    end

    wire [7:0] frow = font[{glyph_id, grow}];
    wire       text_on = in_text_box && frow[3'd7 - gcol];  // MSB = leftmost col

    // ---- compose (text > border > passthrough) ----
    always @(posedge clk) begin
        if (rst)
            pix_out <= 12'd0;
        else if (!in_active)
            pix_out <= 12'd0;
        else if (text_on)
            pix_out <= COL_TEXT;
        else if (on_border)
            pix_out <= COL_BOX;
        else
            pix_out <= pix_in;
    end

endmodule