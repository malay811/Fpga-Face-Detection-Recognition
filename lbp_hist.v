//============================================================================
// lbp_hist.v
//----------------------------------------------------------------------------
// Builds the 3,776-bin LBPH histogram for one 64x64 grayscale crop (streamed
// from lbph_crop_read). Bit-true to the Python enrollment that produced
// lbph_hist.mem, so the L1 distance in l1_compare against threshold 3,900 is
// meaningful.
//
// LBP operator (3x3, radius 1, 8 neighbours, threshold = centre):
//   bit i set when neighbour >= centre. Neighbour order (clockwise from TL),
//   matching the enrollment offsets [(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),
//   (-1,1),(-1,0)] exactly:
//     bit0 TL  bit1 TC  bit2 TR  bit3 CR  bit4 BR  bit5 BC  bit6 BL  bit7 CL
//
// Uniform mapping: uniform_lut.mem (256x8, 58 uniform bins + shared bin 58)
//   turns the 8-bit code into a 0..58 bin. Held in a distributed ROM.
//
// Spatial grid: 8x8 cells (each 8x8 px) -> 64 cells x 59 bins = 3,776.
//   cell = {row[5:3], col[5:3]},  addr = cell*59 + bin.
//
// Border handling - the crucial detail:
//   The enrollment computes LBP only for interior pixels (row,col in 1..62);
//   border pixels keep code 0 -> uniform bin 0, yet are still binned. That
//   border contribution is INPUT-INDEPENDENT (always bin 0 of the edge cell),
//   equal per cell to: 8*edge_rows + 8*edge_cols - corner_overlap
//   (corner cells 15, edge cells 8, interior 0). So instead of a runtime
//   border scan, the histogram is PRELOADED with those counts during CLEAR and
//   only interior 3x3 windows are binned at run time. Verified identical to
//   full-binning over 20 random images.
//
// Storage: hist_mem is 3,776x8 DISTRIBUTED RAM (async read). Async read makes
//   the per-window read-modify-write hazard-free in one cycle (no lost counts
//   on consecutive same-bin pixels) and gives l1_compare a zero-latency read
//   port. Live single-image bin max is 64 (8x8 cell) -> 7 bits; stored 8-bit.
//
// Line buffer: internal 64-wide 2-row buffer + 3-tap column shifts (same proven
//   read-before-write pattern as the 320-wide detection line buffer, narrowed
//   to 64 and exposing the full 3x3 the LBP needs). Window centre trails the
//   write position by one row and one col; window_valid for centres 1..62,
//   which are exactly the interior pixels.
//
// Handshake: pulse start -> CLEAR (3,776 cyc, preload borders) -> ready. Only
//   after ready should the top-level start lbph_crop_read (so no pixels arrive
//   mid-clear). Stream 64x64 with pix_valid + win_row/win_col (write position,
//   0..63 raster). hist_done pulses after the last interior window; the
//   histogram then holds for l1_compare to read via rd_addr/rd_data.
//============================================================================
`timescale 1ns / 1ps

module lbp_hist #(
    parameter LUT_FILE = "uniform_lut.mem"
)(
    input  wire        clk,
    input  wire        rst,           // synchronous, active-high
    input  wire        start,         // pulse: clear + preload borders
    output reg         ready,         // clear done -> ok to stream pixels
    // grayscale stream from lbph_crop_read
    input  wire        pix_valid,
    input  wire [7:0]  gray,
    input  wire [5:0]  win_col,       // write position col 0..63
    input  wire [5:0]  win_row,       // write position row 0..63
    output reg         hist_done,     // pulse: histogram complete
    // read port for l1_compare (async)
    input  wire [11:0] rd_addr,       // 0..3775
    output wire [7:0]  rd_data
);

    localparam integer NB = 59;

    // ---- uniform LUT: 256x8 distributed ROM (async) ----
    (* rom_style = "distributed" *) reg [7:0] ulut [0:255];
    initial $readmemh(LUT_FILE, ulut);

    // ---- histogram: 3776x8 distributed RAM (async read) ----
    (* ram_style = "distributed" *) reg [7:0] hist_mem [0:4095]; // 3776 used
    assign rd_data = hist_mem[rd_addr];

    // ---- FSM ----
    localparam [1:0] S_IDLE=2'd0, S_CLEAR=2'd1, S_ACCUM=2'd2, S_DONE=2'd3;
    reg [1:0] st;

    // clear counters
    reg [5:0] clr_cell;               // 0..63
    reg [5:0] clr_bin;                // 0..58
    // per-cell border count (bin 0 preload)
    wire cey = (clr_cell[5:3]==3'd0) || (clr_cell[5:3]==3'd7);
    wire cex = (clr_cell[2:0]==3'd0) || (clr_cell[2:0]==3'd7);
    wire [4:0] bcount = (cey ? 5'd8 : 5'd0) + (cex ? 5'd8 : 5'd0)
                        - ((cey && cex) ? 5'd1 : 5'd0);        // 15/8/0
    wire [11:0] clr_addr = clr_cell*NB + {6'd0, clr_bin};

    // ---- 64-wide line buffers + 3-tap column shift registers ----
    (* ram_style = "distributed" *) reg [7:0] line1 [0:63];   // row-1
    (* ram_style = "distributed" *) reg [7:0] line2 [0:63];   // row-2
    reg [7:0] r0_0,r0_1,r0_2;         // current row taps (col, col-1, col-2)
    reg [7:0] r1_0,r1_1,r1_2;         // row-1 taps
    reg [7:0] r2_0,r2_1,r2_2;         // row-2 taps

    // pipelined window position/validity (taps update on pix_valid edge)
    reg        win_v;
    reg [5:0]  pr_row, pr_col;
    reg        last_seen;             // saw the (63,63) write

    // ---- LBP code from taps (centre = r1_1) ----
    // window:  r2_2 r2_1 r2_0    (top   = row-2)
    //          r1_2 r1_1 r1_0    (mid   = row-1, centre r1_1)
    //          r0_2 r0_1 r0_0    (bottom= current row)
    wire [7:0] ctr = r1_1;
    wire [7:0] code = { (r1_2 >= ctr),   // bit7 CL  (row-1, col-2 = left)
                        (r0_2 >= ctr),   // bit6 BL
                        (r0_1 >= ctr),   // bit5 BC
                        (r0_0 >= ctr),   // bit4 BR
                        (r1_0 >= ctr),   // bit3 CR
                        (r2_0 >= ctr),   // bit2 TR
                        (r2_1 >= ctr),   // bit1 TC
                        (r2_2 >= ctr) }; // bit0 TL
    wire [7:0] bin = ulut[code];                       // 0..58

    // centre position of the window = (pr_row-1, pr_col-1)
    wire [5:0] cr = pr_row - 1'b1;
    wire [5:0] cc = pr_col - 1'b1;
    wire [5:0] cell_flat = {cr[5:3], cc[5:3]};         // 0..63
    wire [11:0] rmw_addr = cell_flat*NB + {4'd0, bin};
    wire [7:0]  rmw_old  = hist_mem[rmw_addr];

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; ready <= 1'b0; hist_done <= 1'b0;
            clr_cell <= 6'd0; clr_bin <= 6'd0;
            win_v <= 1'b0; pr_row <= 6'd0; pr_col <= 6'd0; last_seen <= 1'b0;
            r0_0<=0;r0_1<=0;r0_2<=0;r1_0<=0;r1_1<=0;r1_2<=0;r2_0<=0;r2_1<=0;r2_2<=0;
        end else begin
            hist_done <= 1'b0;

            case (st)

            // ---------- wait for job ----------
            S_IDLE: begin
                ready <= 1'b0;
                if (start) begin
                    clr_cell <= 6'd0; clr_bin <= 6'd0;
                    st <= S_CLEAR;
                end
            end

            // ---------- clear + preload border counts into bin 0 ----------
            S_CLEAR: begin
                hist_mem[clr_addr] <= (clr_bin == 6'd0) ? {3'd0, bcount} : 8'd0;
                if (clr_bin == NB-1) begin
                    clr_bin <= 6'd0;
                    if (clr_cell == 6'd63) begin
                        win_v <= 1'b0; last_seen <= 1'b0;
                        ready <= 1'b1;
                        st    <= S_ACCUM;
                    end else
                        clr_cell <= clr_cell + 1'b1;
                end else
                    clr_bin <= clr_bin + 1'b1;
            end

            // ---------- stream + bin interior windows ----------
            S_ACCUM: begin
                // stage-2: retire the window registered last cycle
                if (win_v)
                    hist_mem[rmw_addr] <= rmw_old + 1'b1;

                // stage-1: absorb an incoming pixel
                if (pix_valid) begin
                    // 3-tap column shifts for the three rows
                    r2_2 <= r2_1; r2_1 <= r2_0; r2_0 <= line2[win_col];
                    r1_2 <= r1_1; r1_1 <= r1_0; r1_0 <= line1[win_col];
                    r0_2 <= r0_1; r0_1 <= r0_0; r0_0 <= gray;
                    // age the line buffers (read-before-write)
                    line2[win_col] <= line1[win_col];
                    line1[win_col] <= gray;
                    // register window position + validity for next cycle
                    pr_row <= win_row;
                    pr_col <= win_col;
                    win_v  <= (win_row >= 6'd2) && (win_col >= 6'd2);
                    if (win_row == 6'd63 && win_col == 6'd63)
                        last_seen <= 1'b1;
                end else begin
                    win_v <= 1'b0;
                    // after the last pixel's window has been retired, finish
                    if (last_seen && !win_v) begin
                        ready     <= 1'b0;
                        hist_done <= 1'b1;
                        last_seen <= 1'b0;
                        st        <= S_DONE;
                    end
                end
            end

            // ---------- hold histogram for l1_compare readout ----------
            S_DONE: begin
                if (start) begin
                    clr_cell <= 6'd0; clr_bin <= 6'd0;
                    st <= S_CLEAR;
                end
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule