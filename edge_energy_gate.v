//============================================================================
// edge_energy_gate.v
//----------------------------------------------------------------------------
// Reference: Image_Processing_FPGA_Book, Sec 15.6 (Lim 2012) -- real face
// detectors combine multiple independent feature maps (skin/edge/brightness)
// rather than a single classifier score, precisely to reject false positives
// that satisfy one signal by coincidence. This module adds a second,
// independent signal alongside the SVM score: total internal edge energy
// over the 64x64 window.
//
// WHY THIS TARGETS LIGHT SOURCES: a ceiling light / window / reflection is,
// internally, a FLAT SATURATED region -- high average brightness but LOW
// internal gradient energy (little texture). A real face has substantial
// internal edge energy everywhere (eye sockets, nose bridge, mouth, brow,
// hairline) even under harsh lighting. This is measured pre-binarization,
// so it is genuinely independent of what the SVM's binarized features see.
//
// THIS VERSION: INSTRUMENTATION ONLY. Does not gate the face decision yet.
// Feeds scan_controller's debug accumulators (dbg_edge_min/dbg_edge_max)
// so edge_sum can be observed on real hardware (face vs light-source
// windows) before ENERGY_MIN is calibrated and wired into the decision.
//
// Self-contained pixel counter (not desc_done from cell_hist_bin) --
// avoids the same off-by-one timing hazard found in svm_addertree.
// Finalizes on the SAME cycle as the last pixel's magnitude arrives, by
// comparing (edge_sum + mag_in) combinationally rather than waiting one
// extra cycle for edge_sum's registered update.
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module edge_energy_gate #(
    parameter integer WIN_PIXELS = 4096   // 64*64 pixels per window
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        win_start,     // new window -> clear accumulator
    input  wire        pix_valid,     // one pixel's magnitude ready (= hog_v)
    input  wire [10:0] mag_in,        // |gx|+|gy| for this pixel (hog_gradient's mag, 11b)
    output reg  [21:0] edge_sum_o,    // total edge energy this window (debug/calibration)
    output reg         gate_valid     // 1-cycle pulse when edge_sum_o is final for this window
);
    reg [12:0] cnt;         // 0..4095, 13 bits covers WIN_PIXELS
    reg [21:0] edge_sum;    // accumulator; see width note in comments below
    wire [21:0] sum_next = edge_sum + {11'd0, mag_in};

    always @(posedge clk) begin
        if (rst) begin
            cnt        <= 13'd0;
            edge_sum   <= 22'd0;
            edge_sum_o <= 22'd0;
            gate_valid <= 1'b0;
        end else begin
            gate_valid <= 1'b0;

            if (win_start) begin
                cnt      <= 13'd0;
                edge_sum <= 22'd0;
            end else if (pix_valid) begin
                if (cnt == WIN_PIXELS-1) begin
                    edge_sum_o <= sum_next;   // include THIS pixel, same cycle
                    gate_valid <= 1'b1;
                    cnt        <= 13'd0;
                    edge_sum   <= 22'd0;
                end else begin
                    edge_sum <= sum_next;
                    cnt      <= cnt + 13'd1;
                end
            end
        end
    end

endmodule