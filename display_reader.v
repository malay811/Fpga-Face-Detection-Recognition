//============================================================================
// display_reader.v
//----------------------------------------------------------------------------
// Drives the display side of the ping-pong line buffer for 2x upscaling
// 320x240 -> 640x480. Two responsibilities:
//
//  1) Read address (pixel domain, 25.2 MHz): maps VGA (sx,sy) to a line-buffer
//     read address. disp_half = sy[1] (toggles every 2 lines = 2x vertical
//     scale); col = sx[9:1] (2x horizontal scale, clamped during blanking).
//
//  2) Fill FSM (pixel domain) + CDC handshake to sram_ctrl (50 MHz):
//     PRIME0/PRIME1 preload rows 0 and 1 into halves 0/1 at frame start; RUN
//     prefetches the next source row into the OFF half on each even line.
//     req_pix (pulse) crosses to fill_req via xd; fill_done crosses back via xd.
//     fill_row / fill_half are STABLE LEVELS (set before the pulse), sampled
//     by sram_ctrl after the synchronized pulse arrives - stable-before-sample
//     CDC (valid because they never transition while being sampled).
//============================================================================
`timescale 1ns / 1ps

module display_reader #(
    parameter integer V_ACT = 240
)(
    // ---- pixel domain (25.2 MHz) ----
    input  wire               clk_pix,
    input  wire               rst_pix,
    input  wire signed [10:0] sx,
    input  wire signed [10:0] sy,
    input  wire               frame,
    input  wire               line,
    output wire [9:0]         lb_raddr,

    // ---- system domain (50 MHz): fill handshake to sram_ctrl ----
    input  wire               clk_sys,
    input  wire               rst_sys,
    output wire               fill_req,
    output wire               fill_half,
    output wire [8:0]         fill_row,
    input  wire               fill_done
);

    // ---- read address (pixel domain) ----
    wire        disp_half = sy[1];                    // 2x vertical scale
    wire [8:0]  col       = sx[10] ? 9'd0 : sx[9:1];  // 2x horizontal, clamp blank
    assign      lb_raddr  = {disp_half, col};

    // ---- fill FSM (pixel domain) ----
    localparam [2:0] PRIME0   = 3'd0,
                     PRIME0_W = 3'd1,
                     PRIME1   = 3'd2,
                     PRIME1_W = 3'd3,
                     RUN      = 3'd4;
    reg [2:0] fst;

    reg       req_pix;
    reg       fill_half_d;
    reg [8:0] fill_row_d;
    wire      fill_done_pix;   // fill_done crossed into pixel domain

    wire [8:0] src_row  = sy[9:1];
    wire [8:0] next_row = src_row + 9'd1;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            fst         <= PRIME0;
            req_pix     <= 1'b0;
            fill_half_d <= 1'b0;
            fill_row_d  <= 9'd0;
        end else begin
            req_pix <= 1'b0;   // default: single-cycle pulse only

            if (frame) begin
                fst <= PRIME0;    // restart priming each frame
            end else begin
                case (fst)
                PRIME0: begin
                    fill_row_d  <= 9'd0;
                    fill_half_d <= 1'b0;
                    req_pix     <= 1'b1;
                    fst         <= PRIME0_W;
                end
                PRIME0_W: if (fill_done_pix) fst <= PRIME1;
                PRIME1: begin
                    fill_row_d  <= 9'd1;
                    fill_half_d <= 1'b1;
                    req_pix     <= 1'b1;
                    fst         <= PRIME1_W;
                end
                PRIME1_W: if (fill_done_pix) fst <= RUN;
                RUN: begin
                    // prefetch next source row into the OFF half at each even
                    // active line >= 2 (skips the doubled rows 0 and 1).
                    if (line && (sy[0]==1'b0) && (sy >= 11'sd2) &&
                        (next_row < V_ACT[8:0])) begin
                        fill_row_d  <= next_row;
                        fill_half_d <= next_row[0];
                        req_pix     <= 1'b1;
                    end
                end
                default: fst <= PRIME0;
                endcase
            end
        end
    end

    // stable levels (stable-before-sample CDC)
    assign fill_half = fill_half_d;
    assign fill_row  = fill_row_d;

    // pulse CDC: req_pix (pix) -> fill_req (sys); fill_done (sys) -> pix
    xd xd_req (
        .clk_src(clk_pix), .rst_src(rst_pix), .pulse_src(req_pix),
        .clk_dst(clk_sys), .rst_dst(rst_sys), .pulse_dst(fill_req)
    );
    xd xd_done (
        .clk_src(clk_sys), .rst_src(rst_sys), .pulse_src(fill_done),
        .clk_dst(clk_pix), .rst_dst(rst_pix), .pulse_dst(fill_done_pix)
    );

endmodule