//============================================================================
// hog_gradient.v
//----------------------------------------------------------------------------
// Per-pixel gradient -> (magnitude, orientation bin) for HOG.
//   gx = p12 - p10   (right - left)
//   gy = p21 - p01   (bottom - top)
//   mag = |gx| + |gy|   (L1, zero-multiplier magnitude)
//   bin = 9-bin orientation via scaled-tangent comparison, folded to 0..180
//
// SATURATION-AWARE GRADIENT SUPPRESSION (new):
//   A blown-out light source is a region of SATURATED pixels -- clipped at
//   the sensor ceiling, so the *true* scene gradient there is destroyed, not
//   merely large. Highlight-detection literature (e.g. face-relighting work)
//   treats saturated pixels as carrying no reliable gradient information.
//   We therefore ZERO the magnitude wherever any of the 4 gradient-relevant
//   neighbours is saturated. This collapses a light source's internal HOG
//   structure (its bright core produces no gradients -> flat B-HOG block ->
//   no face signature) WITHOUT touching a real face, which is not saturated
//   (confirmed on hardware: face sat in proper midtones, only background
//   blew out). Orientation bin is unaffected -- only magnitude is gated, so
//   a suppressed pixel contributes 0 to every cell histogram bin.
//
//   IMPORTANT: this exact gate is mirrored in hog_extract.py (SAT_THRESH),
//   and the SVM is retrained with it, so hardware features == training
//   features bit-for-bit. Changing SAT_THRESH here REQUIRES retraining.
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module hog_gradient #(
    // Effective saturation level AFTER grayscale. Pure sensor clip is 255,
    // but RGB444 truncation + gain push "effectively clipped" lower. Grayscale
    // Y = 0.257R+0.504G+0.098B+16 maxes near 251; treat >= SAT_THRESH as
    // saturated. Calibrate against hardware (see debug_gray_tap output).
    parameter [7:0] SAT_THRESH = 8'd210
)(
    input  wire        clk,
    input  wire        rst,
    // 3x3 window (row0 = current); centre = p11
    input  wire [7:0]  p01,        // top-centre
    input  wire [7:0]  p10,        // mid-left
    input  wire [7:0]  p12,        // mid-right
    input  wire [7:0]  p21,        // bottom-centre
    input  wire        window_valid,
    input  wire [9:0]  win_col_in,
    input  wire [9:0]  win_row_in,
    output reg  [10:0] mag,        // |gx|+|gy|, max 510 -> 9 bits, use 11 safe
    output reg  [3:0]  bin,        // 0..8
    output reg         out_valid,
    output reg  [9:0]  win_col,
    output reg  [9:0]  win_row
);

    // ---- gradients (signed) ----
    wire signed [8:0] gx = $signed({1'b0,p12}) - $signed({1'b0,p10});
    wire signed [8:0] gy = $signed({1'b0,p21}) - $signed({1'b0,p01});

    // ---- fold to upper half plane (gy>=0) ----
    // If gy<0 (or gy==0 & gx<0), negate both: same orientation line, angle 0..180.
    wire flip = gy[8] | (~(|gy) & gx[8]);   // gy<0, or gy==0 and gx<0
    wire signed [8:0] fgx = flip ? -gx : gx;
    wire signed [8:0] fgy = flip ? -gy : gy;   // now fgy >= 0

    // absolute gx, and magnitudes
    wire [8:0] agx = fgx[8] ? (~fgx + 1'b1) : fgx;   // |fgx|
    wire [8:0] agy = fgy;                            // fgy>=0 already
    wire [8:0] axgx_forabs = gx[8] ? (~gx + 1'b1) : gx; // |gx| for mag
    wire [8:0] aygy_forabs = gy[8] ? (~gy + 1'b1) : gy; // |gy| for mag

    wire [10:0] mag_raw = axgx_forabs + aygy_forabs;   // |gx|+|gy|

    // ---- saturation gate ----
    // If ANY of the 4 gradient-relevant neighbours is at/above the sat level,
    // this pixel's gradient is unreliable -> force magnitude to 0.
    wire sat = (p01 >= SAT_THRESH) | (p10 >= SAT_THRESH) |
               (p12 >= SAT_THRESH) | (p21 >= SAT_THRESH);
    wire [10:0] mag_c = sat ? 11'd0 : mag_raw;

    // ---- scaled tangents (x1024) of 20,40,60,80 deg ----
    localparam [22:0] T20 = 23'd373;
    localparam [22:0] T40 = 23'd859;
    localparam [22:0] T60 = 23'd1774;
    localparam [22:0] T80 = 23'd5807;

    // cross-multiply terms: gy*1024  vs  |gx|*tan_edge
    wire [22:0] gy1024 = {agy, 10'd0};      // agy * 1024
    wire [22:0] gxT20  = agx * T20[13:0];
    wire [22:0] gxT40  = agx * T40[13:0];
    wire [22:0] gxT60  = agx * T60[13:0];
    wire [22:0] gxT80  = agx * T80[13:0];

    reg [3:0] bin_c;
    always @* begin
        if (agy == 9'd0) begin
            bin_c = 4'd0;                 // angle 0
        end else if (agx == 9'd0) begin
            bin_c = 4'd4;                 // angle 90
        end else if (~fgx[8]) begin
            // fgx > 0 : angle 0..90
            if      (gy1024 <  gxT20) bin_c = 4'd0;
            else if (gy1024 <  gxT40) bin_c = 4'd1;
            else if (gy1024 <  gxT60) bin_c = 4'd2;
            else if (gy1024 <  gxT80) bin_c = 4'd3;
            else                      bin_c = 4'd4;
        end else begin
            // fgx < 0 : angle 90..180 (mirrored)
            if      (gy1024 >  gxT80) bin_c = 4'd4;
            else if (gy1024 >  gxT60) bin_c = 4'd5;
            else if (gy1024 >  gxT40) bin_c = 4'd6;
            else if (gy1024 >  gxT20) bin_c = 4'd7;
            else                      bin_c = 4'd8;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            mag <= 11'd0; bin <= 4'd0; out_valid <= 1'b0;
            win_col <= 10'd0; win_row <= 10'd0;
        end else begin
            mag       <= mag_c;           // saturation-gated magnitude
            bin       <= bin_c;
            out_valid <= window_valid;
            win_col   <= win_col_in;
            win_row   <= win_row_in;
        end
    end

endmodule