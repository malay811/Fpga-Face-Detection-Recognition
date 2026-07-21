//============================================================================
// xd.v - toggle-based single-cycle-pulse CDC synchronizer
//----------------------------------------------------------------------------
// Safely carries a 1-cycle pulse from clk_src into clk_dst. A raw pulse can be
// missed across clock domains (its width may be shorter than the destination
// period); a level TOGGLE cannot be missed. This module toggles a level on
// each source pulse, 3-stage-synchronizes it into the destination, and edge-
// detects to regenerate a 1-cycle destination pulse.
//
// Standard Cummings-style toggle synchronizer. The (* ASYNC_REG *) attribute
// on the first two sync flops tells Vivado to place them close for MTBF.
//============================================================================
`timescale 1ns / 1ps

module xd (
    input  wire clk_src,
    input  wire rst_src,
    input  wire pulse_src,   // 1-cycle pulse in source domain
    input  wire clk_dst,
    input  wire rst_dst,
    output wire pulse_dst    // 1-cycle pulse in destination domain
);

    // Source-domain toggle: flips level on each source pulse.
    reg tgl_src;
    always @(posedge clk_src) begin
        if (rst_src)        tgl_src <= 1'b0;
        else if (pulse_src) tgl_src <= ~tgl_src;
    end

    // Destination 3-stage synchronizer + edge detect.
    (* ASYNC_REG = "TRUE" *) reg s0, s1;
    reg s2;
    always @(posedge clk_dst) begin
        if (rst_dst) begin
            s0 <= 1'b0; s1 <= 1'b0; s2 <= 1'b0;
        end else begin
            s0 <= tgl_src;  // may go metastable; resolved by s1
            s1 <= s0;
            s2 <= s1;
        end
    end

    // Any level change (rising or falling) = one original pulse.
    assign pulse_dst = s1 ^ s2;

endmodule