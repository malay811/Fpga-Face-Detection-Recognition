//============================================================================
// grayscale.v
//----------------------------------------------------------------------------
// RGB444 -> 8-bit luminance (Y). Integer fixed-point form of the XAPP637
// coefficients used during SVM/LBPH training:
//
//   float:   Y = 0.257*R + 0.504*G + 0.098*B + 16     (R,G,B are 8-bit)
//   integer: Y = ((66*R + 129*G + 25*B) >> 8) + 16     (Q8 coeffs)
//   (66/256=0.2578, 129/256=0.5039, 25/256=0.0977 - matches within 1 LSB)
//
// The camera delivers RGB444 (4-bit channels = the high nibble of the original
// 8-bit sensor value). Training used full 8-bit pixels, so to align, each
// 4-bit channel is expanded back to 8-bit by replication: v8 = {v4,v4} = v4*17
// (0->0, 15->255). This bit-replication is the standard 4->8 bit expansion and
// keeps hardware Y within 1 LSB of the trained float Y.
//
// One pipeline register on the output. gray_valid follows pixel_valid delayed
// by 1 cycle to stay aligned with the registered gray output.
//============================================================================
`timescale 1ns / 1ps

module grayscale (
    input  wire        clk,
    input  wire        rst,
    input  wire [11:0] pixel_in,     // RGB444: {R[3:0], G[3:0], B[3:0]}
    input  wire        pixel_valid,
    output reg  [7:0]  gray,
    output reg         gray_valid
);

    // ---- split RGB444 channels ----
    wire [3:0] r4 = pixel_in[11:8];
    wire [3:0] g4 = pixel_in[7:4];
    wire [3:0] b4 = pixel_in[3:0];

    // ---- expand 4-bit -> 8-bit by replication (v8 = v4*17) ----
    wire [7:0] r8 = {r4, r4};
    wire [7:0] g8 = {g4, g4};
    wire [7:0] b8 = {b4, b4};

    // ---- weighted sum in Q8 (coeffs 66,129,25) ----
    // Max: (66+129+25)*255 = 56100 -> fits 16 bits. Add rounding half (128).
    wire [15:0] acc = 16'd128
                    + (r8 * 8'd66)
                    + (g8 * 8'd129)
                    + (b8 * 8'd25);

    // Y = (acc >> 8) + 16, clamped to 255.
    wire [8:0] y_pre = acc[15:8] + 9'd16;      // add offset
    wire [7:0] y_sat = (y_pre > 9'd255) ? 8'd255 : y_pre[7:0];

    always @(posedge clk) begin
        if (rst) begin
            gray       <= 8'd0;
            gray_valid <= 1'b0;
        end else begin
            gray       <= y_sat;
            gray_valid <= pixel_valid;
        end
    end

endmodule