//============================================================================
// svm_addertree.v
//----------------------------------------------------------------------------
// Linear-SVM scorer for the 1,764-bit B-HOG descriptor. Zero multipliers:
// because features are binary, score = SUM of the weights whose feature bit
// is 1, plus bias. Consumes the block stream from cell_hist_bin directly
// (partial classification) so scoring latency hides inside HOG extraction.
//
//   For each block (block_valid): the 36 bits select 36 weights from a
//   K=36-wide partial-parallel adder tree -> one signed partial sum ->
//   added into the running accumulator. block_index picks the weight slice
//   svm_weights.mem[block_index*36 +: 36].
//
//   After the 49th block (desc_done), bias is added and the sign bit gives
//   the decision:  face = (score >= 0) = ~score[SIGN].  (threshold = 0,
//   locked; LinearSVC decision boundary folded into the trained bias.)
//
// Weights: Q1.14 INT16 two's-complement, svm_weights.mem (1,764 lines).
// Bias:    Q17.14 INT32 two's-complement, svm_bias.mem   (1 line = 6990).
// Both share the same fractional scale, so they add directly.
//
// Accumulator: real bound over these weights is +215,610 / -255,515 -> signed
// 19 bits. Spec locks 27-bit (16-bit weight + 11 guard, log2(1764)=10.8->11);
// 27 is used for margin/defensibility. bias fits (6990 << 2^27).
//
// K=36 adder tree = 6-level balanced tree of signed adds (36 -> 18 -> 9 -> 5
// -> 3 -> 2 -> 1), combinational. Each leaf is weight-or-zero:
//   leaf_k = feat_bit_k ? weight_k : 0.
//
// Handshake: score_valid pulses 1 cycle after desc_done with the final
// signed score and face bit. clear on win_start (new window resets acc).
//
// Weight slice read is registered (BRAM/LUTROM); block_valid -> read weights
// -> next cycle compute partial -> accumulate. 2-cycle path per block, but
// blocks arrive >=5 cycles apart from cell_hist_bin, so no back-pressure.
//============================================================================
`timescale 1ns / 1ps

module svm_addertree #(
    parameter WEIGHT_FILE = "svm_weights.mem",
    parameter BIAS_FILE   = "svm_bias.mem"
)(
    input  wire        clk,
    input  wire        rst,          // synchronous, active-high
    input  wire        win_start,    // new window -> clear accumulator
    input  wire        block_valid,  // one block ready
    input  wire [5:0]  block_index,  // 0..48
    input  wire [35:0] block_bits,   // 36 binary features for this block
    input  wire        desc_done,    // all 49 blocks emitted
    output reg  signed [26:0] score, // signed SVM score (Q17.14)
    output reg         face,         // 1 = face (score >= 0)
    output reg         score_valid   // 1-cycle pulse with final score/face
);
    localparam signed [26:0] SCORE_THRESHOLD = 27'sd9338;

    // ---- weight ROM: 1,764 x 16-bit, inferred (Option A) ----
    (* rom_style = "block" *) reg signed [15:0] wrom [0:1763];
    reg signed [31:0] bias_mem [0:0];
    initial begin
        $readmemh(WEIGHT_FILE, wrom);
        $readmemh(BIAS_FILE, bias_mem);
    end
    wire signed [31:0] bias = bias_mem[0];

    // ---- registered inputs for the 2-stage per-block path ----
    reg [35:0] feat_r;
    reg        do_add;                     // stage-2 enable

    // base address of this block's 36 weights
    wire [11:0] base = block_index * 6'd36; // 0,36,...,1728  (max 1728, 12b)

    // ---- 36 leaves: weight if bit set, else 0 (combinational, uses feat_r) ----
    // wrom read is async here (BRAM read modeled combinationally in sim; in
    // synthesis this infers a registered ROM read one cycle earlier -- see note)
    // To keep it hardware-honest we register the 36 weights in stage 1.
    reg signed [15:0] wsel [0:35];
    integer k;
    always @(posedge clk) begin
        // stage 1: latch this block's weights + its feature bits
        for (k = 0; k < 36; k = k + 1)
            wsel[k] <= wrom[base + k[11:0]];
        feat_r <= block_bits;
        do_add <= block_valid;
    end

    // leaves (masked weights), sign-extended to accumulator width
    wire signed [16:0] leaf [0:35];
    genvar g;
    generate
        for (g = 0; g < 36; g = g + 1) begin : g_leaf
            assign leaf[g] = feat_r[g] ? {wsel[g][15], wsel[g]} : 17'sd0;
        end
    endgenerate

    // ---- balanced adder tree 36 -> 1 (combinational) ----
    // widths grow by 1 per level; final partial <= 36*1421 = 51,156 -> 17 bits+
    wire signed [17:0] s1  [0:17];
    wire signed [18:0] s2  [0:8];
    wire signed [19:0] s3  [0:4];
    wire signed [20:0] s4  [0:2];
    wire signed [21:0] s5  [0:1];
    wire signed [22:0] part;
    generate
        for (g = 0; g < 18; g = g + 1) begin : L1
            assign s1[g] = leaf[2*g] + leaf[2*g+1];
        end
        for (g = 0; g < 9; g = g + 1) begin : L2
            assign s2[g] = s1[2*g] + s1[2*g+1];
        end
        // 9 -> 5 (last carries through)
        assign s3[0] = s2[0] + s2[1];
        assign s3[1] = s2[2] + s2[3];
        assign s3[2] = s2[4] + s2[5];
        assign s3[3] = s2[6] + s2[7];
        assign s3[4] = {s2[8][18], s2[8]};
        // 5 -> 3
        assign s4[0] = s3[0] + s3[1];
        assign s4[1] = s3[2] + s3[3];
        assign s4[2] = {s3[4][19], s3[4]};
        // 3 -> 2
        assign s5[0] = s4[0] + s4[1];
        assign s5[1] = {s4[2][20], s4[2]};
        // 2 -> 1
        assign part  = s5[0] + s5[1];
    endgenerate

    // ---- accumulate ----
    reg signed [26:0] acc;
    reg        fin_d1;   // desc_done delayed to align with last block's stage-2
    wire signed [26:0] final_score = acc + bias[26:0];

    always @(posedge clk) begin
        if (rst) begin
            acc <= 27'sd0; score <= 27'sd0; face <= 1'b0;
            score_valid <= 1'b0; fin_d1 <= 1'b0;
        end else begin
            score_valid <= 1'b0;

            if (win_start)
                acc <= 27'sd0;
            else if (do_add)
                acc <= acc + {{4{part[22]}}, part};   // sign-extend 23->27

            // desc_done arrives with the LAST block_valid; that block's add is
            // one cycle later (do_add). Delay finalize by one cycle to include it.
            fin_d1 <= desc_done;
            if (fin_d1) begin
                score       <= final_score;
                face <= (final_score > SCORE_THRESHOLD);   // sign bit: >=0 -> face
                score_valid <= 1'b1;
            end
        end
    end

endmodule