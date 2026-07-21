//============================================================================
// display_480p.v
//----------------------------------------------------------------------------
// VGA 640x480 @ 60 Hz timing generator (VESA), driven at 25.175 MHz
// (we use 25.2 MHz). Produces sync pulses, data-enable, and signed pixel
// coordinates for the display pipeline.
//
// VESA 640x480@60 timing (in pixel clocks):
//   Horizontal: 640 active + 16 front porch + 96 sync + 48 back porch = 800
//   Vertical:   480 active + 10 front porch +  2 sync + 33 back porch = 525
//   Frame = 800 * 525 = 420000 clocks; at 25.2 MHz => 60 Hz.
//
// Counter origin: h=0 / v=0 is the START of the sync pulse (not active video).
// Signed coordinates (sx, sy) place origin (0,0) at the first ACTIVE pixel,
// so sync + back porch produce negative sx/sy. This lets downstream logic use
// simple sx>=0 && sx<H_RES range checks.
//
// hsync/vsync are REGISTERED (1-cycle latency). The linebuffer read output is
// also registered (1-cycle), so sync and pixel data stay aligned at the pins.
//============================================================================
`timescale 1ns / 1ps

module display_480p #(
    parameter signed H_RES  = 640,
    parameter signed V_RES  = 480,
    parameter signed H_FP   = 16,
    parameter signed H_SYNC = 96,
    parameter signed H_BP   = 48,
    parameter signed V_FP   = 10,
    parameter signed V_SYNC = 2,
    parameter signed V_BP   = 33
)(
    input  wire               clk_pix,
    input  wire               rst_pix,
    output reg                hsync,
    output reg                vsync,
    output wire               de,     // high during active video
    output wire               frame,  // 1-cycle pulse at frame start
    output wire               line,   // 1-cycle pulse at active line start
    output wire signed [10:0] sx,     // signed pixel X (origin at first active)
    output wire signed [10:0] sy      // signed pixel Y
);

    localparam signed H_TOTAL = H_RES + H_FP + H_SYNC + H_BP; // 800
    localparam signed V_TOTAL = V_RES + V_FP + V_SYNC + V_BP; // 525

    reg [9:0] h;   // 0..799
    reg [9:0] v;   // 0..524

    // ---- raster counters ----
    always @(posedge clk_pix) begin
        if (rst_pix) begin
            h <= 10'd0;
            v <= 10'd0;
        end else begin
            if (h == H_TOTAL - 1) begin
                h <= 10'd0;
                v <= (v == V_TOTAL - 1) ? 10'd0 : v + 1'b1;
            end else begin
                h <= h + 1'b1;
            end
        end
    end

    // ---- registered sync (active low) ----
    // hsync low during h = 0..H_SYNC-1 ; vsync low during v = 0..V_SYNC-1.
    always @(posedge clk_pix) begin
        hsync <= ~(h < H_SYNC);
        vsync <= ~(v < V_SYNC);
    end

    // ---- signed coordinates: origin at first active pixel ----
    // sx = h - (H_SYNC + H_BP) ; = 0 when h reaches sync+backporch (144).
    assign sx = $signed({1'b0, h}) - $signed(H_SYNC + H_BP);
    assign sy = $signed({1'b0, v}) - $signed(V_SYNC + V_BP);

    // ---- data enable: inside the active 640x480 window ----
    assign de = (sx >= 0) && (sx < H_RES) && (sy >= 0) && (sy < V_RES);

    // ---- frame start pulse: h=0 at the line just after last active line ----
    assign frame = (h == 0) && (v == V_SYNC + V_BP + V_RES);

    // ---- line start pulse: first active pixel of an active line ----
    assign line = (sx == 0) && (sy >= 0) && (sy < V_RES);

endmodule