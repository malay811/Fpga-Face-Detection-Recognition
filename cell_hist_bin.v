//============================================================================
// cell_hist_bin.v
//----------------------------------------------------------------------------
// Builds the 1,764-bit B-HOG descriptor for one 64x64 detection window.
//
//   1) ACCUM: each (mag,bin) vote adds mag into cell_hist[cell][bin].
//      Cells are 8x8 px -> 8x8 = 64 cells, 9 bins. cell = {py[5:3], px[5:3]}.
//   2) GATHER/BINZ: for each of 49 blocks (2x2 cells, stride 1 cell), read the
//      4 cells' 9 bins (36 values), sum them, binarize each value against the
//      block mean WITHOUT division:  bit = (val*36 > sum)  ==  (val > sum/36).
//      Strict '>' matches the Python training extractor exactly (ties -> 0).
//
// Storage: 9 bin-banks of 64x17 DISTRIBUTED RAM (async read), not one
// 576-entry array and not BRAM, because:
//   - ACCUM does 1-cycle read-modify-write; back-to-back votes can hit the
//     same (cell,bin). BRAM's synchronous read would return stale data for
//     consecutive same-address RMW (lost votes) unless bypassed. Distributed
//     RAM async read makes RMW hazard-free by construction.
//   - GATHER reads all 9 bins of one cell in parallel (one bank address),
//     so a block needs only 4 gather cycles, with tiny mux cost.
//   Cost: ~306 LUTs of LUTRAM, 0 BRAM, 0 DSP (x36 is shift-add).
//
// Handshake contract:
//   win_start (1-cycle pulse) starts a window. The module then spends 64
//   cycles clearing the banks; pix_ready rises when it can accept pixels.
//   Drive pix_valid ONLY while pix_ready is high (scanning controller gates
//   on it). Exactly 4096 pixels per window, raster order.
//
// Streaming out: block_valid pulses 49 times with block_bits[35:0] and
// block_index (0..48, row-major by*7+bx). Bit i of block_bits corresponds to
// value i in the order [cell(by,bx) bins0-8, cell(by,bx+1) bins0-8,
// cell(by+1,bx) bins0-8, cell(by+1,bx+1) bins0-8] -- the SVM adder tree must
// index svm_weights.mem as weight[block_index*36 + i]. desc_done pulses one
// cycle after the 49th block.
//
// Per-window timing: 64 (clear) + 4096 (accum) + 49*5 (emit) + 1 = 4406 clks.
//============================================================================
`timescale 1ns / 1ps

module cell_hist_bin (
    input  wire        clk,
    input  wire        rst,          // synchronous, active-high
    input  wire        win_start,    // pulse: begin a new 64x64 window
    input  wire        pix_valid,    // one vote (mag,bin); only when pix_ready
    input  wire [10:0] mag,          // |gx|+|gy|, max 510
    input  wire [3:0]  bin,          // orientation bin 0..8
    output wire        pix_ready,    // high while module accepts pixels
    output reg  [35:0] block_bits,   // binarized block descriptor
    output reg  [5:0]  block_index,  // 0..48
    output reg         block_valid,  // 1-cycle pulse per block
    output reg         desc_done     // 1-cycle pulse after 49th block
);

    localparam integer NBINS = 9;

    // ---- FSM ----
    localparam [2:0]
        S_IDLE  = 3'd0,
        S_CLEAR = 3'd1,   // zero all banks, 64 cycles
        S_ACCUM = 3'd2,   // 4096 pixel votes
        S_GATH  = 3'd3,   // 4 cycles: read 2x2 cells, 9 bins each
        S_BINZ  = 3'd4,   // binarize 36 values, emit block
        S_FINI  = 3'd5;   // pulse desc_done
    reg [2:0] st;

    // ---- position / bookkeeping ----
    reg [5:0]  clr_idx;              // 0..63 clear address
    reg [5:0]  px, py;               // pixel position in window, 0..63
    reg [11:0] pix_count;            // 0..4095
    reg [2:0]  by, bx;               // block row/col, 0..6
    reg [5:0]  blk_i;                // 0..48
    reg [1:0]  gph;                  // gather phase: which of the 4 cells

    assign pix_ready = (st == S_ACCUM);

    // ---- bank addressing ----
    // cell index = {cell_y, cell_x} = {py[5:3], px[5:3]}  (0..63)
    wire [5:0] cell_wr = {py[5:3], px[5:3]};
    // gather order: gph=0:(by,bx) 1:(by,bx+1) 2:(by+1,bx) 3:(by+1,bx+1)
    wire [2:0] g_row = by + {2'b00, gph[1]};
    wire [2:0] g_col = bx + {2'b00, gph[0]};
    wire [5:0] cell_rd = (st == S_ACCUM) ? cell_wr : {g_row, g_col};

    // ---- 9 bin-banks: 64 x 17-bit distributed RAM each ----
    // Max cell/bin value: 64 px * 510 = 32,640 -> 16 bits; 17 used for margin.
    wire [NBINS*17-1:0] rd_bus;
    genvar b;
    generate
        for (b = 0; b < NBINS; b = b + 1) begin : g_bank
            (* ram_style = "distributed" *) reg [16:0] mem [0:63];
            assign rd_bus[b*17 +: 17] = mem[cell_rd];
            always @(posedge clk) begin
                if (st == S_CLEAR)
                    mem[clr_idx] <= 17'd0;
                else if (st == S_ACCUM && pix_valid && (bin == b[3:0]))
                    mem[cell_wr] <= rd_bus[b*17 +: 17] + {6'd0, mag};
            end
        end
    endgenerate

    // sum of the 9 bins currently addressed (one cell): max 9*32640 -> 19 bits
    wire [21:0] sum9 =
          rd_bus[ 0*17 +: 17] + rd_bus[ 1*17 +: 17] + rd_bus[ 2*17 +: 17]
        + rd_bus[ 3*17 +: 17] + rd_bus[ 4*17 +: 17] + rd_bus[ 5*17 +: 17]
        + rd_bus[ 6*17 +: 17] + rd_bus[ 7*17 +: 17] + rd_bus[ 8*17 +: 17];

    // ---- block assembly ----
    reg [16:0] bvals [0:35];         // 36 gathered values
    reg [21:0] bsum;                 // block sum: max 36*32640 = 1,175,040 (21b)

    // val*36 = (val<<5) + (val<<2): inlined in S_BINZ, 22-bit, no DSP.
    // Max val*36 = 32,640*36 = 1,175,040 (21 bits) -- no overflow in 22.

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            st          <= S_IDLE;
            block_valid <= 1'b0;
            desc_done   <= 1'b0;
            block_bits  <= 36'd0;
            block_index <= 6'd0;
            clr_idx     <= 6'd0;
            px <= 6'd0; py <= 6'd0; pix_count <= 12'd0;
            by <= 3'd0; bx <= 3'd0; blk_i <= 6'd0; gph <= 2'd0;
            bsum <= 22'd0;
        end else begin
            block_valid <= 1'b0;
            desc_done   <= 1'b0;

            case (st)

            // ---------- wait for a window ----------
            S_IDLE: begin
                if (win_start) begin
                    clr_idx <= 6'd0;
                    st      <= S_CLEAR;
                end
            end

            // ---------- clear all 9 banks, one address per cycle ----------
            S_CLEAR: begin
                if (clr_idx == 6'd63) begin
                    px <= 6'd0; py <= 6'd0; pix_count <= 12'd0;
                    st <= S_ACCUM;
                end
                clr_idx <= clr_idx + 1'b1;
            end

            // ---------- accumulate 4096 votes ----------
            S_ACCUM: begin
                if (pix_valid) begin
                    // bank write happens in the generate block above
                    if (px == 6'd63) begin
                        px <= 6'd0;
                        py <= py + 1'b1;
                    end else
                        px <= px + 1'b1;

                    if (pix_count == 12'd4095) begin
                        by <= 3'd0; bx <= 3'd0; blk_i <= 6'd0;
                        gph <= 2'd0; bsum <= 22'd0;
                        st  <= S_GATH;
                    end else
                        pix_count <= pix_count + 1'b1;
                end
            end

            // ---------- gather: 4 cycles, one 2x2 cell per cycle ----------
            S_GATH: begin
                for (i = 0; i < NBINS; i = i + 1)
                    bvals[gph*NBINS + i] <= rd_bus[i*17 +: 17];
                bsum <= bsum + sum9;
                if (gph == 2'd3)
                    st <= S_BINZ;
                gph <= gph + 1'b1;
            end

            // ---------- binarize 36 values against block mean ----------
            S_BINZ: begin
                for (i = 0; i < 36; i = i + 1)
                    block_bits[i] <= (({bvals[i], 5'd0} + {3'd0, bvals[i], 2'd0}) > bsum);
                block_index <= blk_i;
                block_valid <= 1'b1;
                bsum <= 22'd0;
                gph  <= 2'd0;
                if (blk_i == 6'd48) begin
                    st <= S_FINI;
                end else begin
                    blk_i <= blk_i + 1'b1;
                    if (bx == 3'd6) begin
                        bx <= 3'd0;
                        by <= by + 1'b1;
                    end else
                        bx <= bx + 1'b1;
                    st <= S_GATH;
                end
            end

            // ---------- descriptor complete ----------
            S_FINI: begin
                desc_done <= 1'b1;
                st        <= S_IDLE;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule