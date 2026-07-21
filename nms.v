//============================================================================
// nms.v
//----------------------------------------------------------------------------
// Greedy non-maximum suppression for the face detector. Single-scale, so every
// candidate box is 64x64 -> the IoU test collapses to integer compares with a
// constant and needs NO division:
//
//   two 64-boxes at (x1,y1),(x2,y2):
//     iw = (|dx|<64) ? 64-|dx| : 0 ,  ih = (|dy|<64) ? 64-|dy| : 0
//     inter = iw*ih ,  union = 2*64*64 - inter = 8192 - inter
//     IoU > 0.5  <=>  2*inter > union  <=>  3*inter > 8192      (locked rule)
//
// Two phases:
//   COLLECT (st=IDLE): every cand_valid appends (x,y,score) to a 64-deep
//     buffer. Overflow past 64 is dropped (a face cluster is ~5-9 boxes; 64 is
//     generous). Runs concurrently with detection across the whole frame.
//   RUN: on nms_start, greedy-select up to 8 boxes:
//     SCAN   - argmax score over non-suppressed entries (sequential)
//     LATCH  - read winner's x,y
//     EMIT   - pulse out_valid, mark winner suppressed
//     SUPPR  - walk all entries, suppress those overlapping the winner
//     repeat until 8 emitted or none remain -> nms_done.
//
// Buffer in distributed RAM (async read, single-cycle scan). suppressed is a
// 64-bit reg for parallel clear + single-bit set. One 7x7 multiply (iw*ih)
// runs once per suppress compare -- infers as fabric LUTs (too small for DSP),
// consistent with the zero-DSP datapath goal; it is not in any per-pixel path.
//
// Coordinates are DETECTION space (320x240): x in 0..256, y in 0..176.
// Downstream (osd_overlay, lbph_crop_read) scale x2 to display space.
//
// Timing per frame: <= 8*(64 scan + 1 latch + 1 emit + 64 suppress) ~ 1040
// clks @ 50 MHz, trivial vs ~800k clks/frame.
//============================================================================
`timescale 1ns / 1ps

module nms #(
    parameter integer MAXCAND = 64,
    parameter integer MAXOUT  = 8
)(
    input  wire        clk,
    input  wire        rst,          // synchronous, active-high
    // candidate stream (a scan position whose SVM score said face=1)
    input  wire        cand_valid,
    input  wire [8:0]  cand_x,       // 0..256
    input  wire [7:0]  cand_y,       // 0..176
    input  wire signed [26:0] cand_score,
    // control
    input  wire        nms_start,    // pulse: all candidates for the frame are in
    // kept-box output
    output reg  [8:0]  out_x,
    output reg  [7:0]  out_y,
    output reg  [3:0]  out_index,    // 0..7
    output reg         out_valid,    // 1-cycle pulse per kept box
    output reg         nms_done      // 1-cycle pulse when selection finished
);

    localparam integer AW = 6;       // log2(64)

    // ---- candidate buffer (distributed RAM, async read) ----
    (* ram_style = "distributed" *) reg [8:0]  bx_mem [0:MAXCAND-1];
    (* ram_style = "distributed" *) reg [7:0]  by_mem [0:MAXCAND-1];
    (* ram_style = "distributed" *) reg signed [26:0] bs_mem [0:MAXCAND-1];

    reg [AW:0] count;                // 0..64
    reg [MAXCAND-1:0] suppressed;

    // ---- FSM ----
    localparam [2:0]
        S_IDLE  = 3'd0,   // collecting
        S_SCAN  = 3'd1,   // argmax
        S_LATCH = 3'd2,   // read winner coords
        S_EMIT  = 3'd3,   // output winner
        S_SUPPR = 3'd4,   // suppress overlaps
        S_DONE  = 3'd5;
    reg [2:0] st;

    reg [AW:0]  si;                  // scan index
    reg [AW:0]  ji;                  // suppress index
    reg [3:0]   out_cnt;            // 0..8
    reg [AW-1:0] best_idx;
    reg signed [26:0] best_score;
    reg         best_found;
    reg [8:0]   win_x;              // winner coords latched
    reg [7:0]   win_y;

    // ---- overlap test against winner (combinational, uses by/ji read) ----
    wire [8:0] jx = bx_mem[ji[AW-1:0]];
    wire [7:0] jy = by_mem[ji[AW-1:0]];
    wire [8:0] dx = (win_x > jx) ? (win_x - jx) : (jx - win_x);
    wire [7:0] dy = (win_y > jy) ? (win_y - jy) : (jy - win_y);
    wire [6:0] iw = (dx < 9'd64) ? (7'd64 - dx[6:0]) : 7'd0;
    wire [6:0] ih = (dy < 8'd64) ? (7'd64 - dy[6:0]) : 7'd0;
    wire [13:0] inter  = iw * ih;                       // 0..4096
    wire [15:0] inter3 = {2'b0, inter} + {1'b0, inter, 1'b0}; // inter + 2*inter
    wire        overlap = inter3 > 16'd8192;            // IoU>0.5

    // scan read
    wire signed [26:0] s_score = bs_mem[si[AW-1:0]];
    wire        s_sup   = suppressed[si[AW-1:0]];

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; count <= 0; suppressed <= 0;
            out_valid <= 1'b0; nms_done <= 1'b0;
            out_x <= 9'd0; out_y <= 8'd0; out_index <= 4'd0;
            si <= 0; ji <= 0; out_cnt <= 0;
            best_idx <= 0; best_score <= 0; best_found <= 1'b0;
            win_x <= 9'd0; win_y <= 8'd0;
        end else begin
            out_valid <= 1'b0;
            nms_done  <= 1'b0;

            case (st)

            // ---------- collect candidates ----------
            S_IDLE: begin
                if (cand_valid && count < MAXCAND) begin
                    bx_mem[count[AW-1:0]] <= cand_x;
                    by_mem[count[AW-1:0]] <= cand_y;
                    bs_mem[count[AW-1:0]] <= cand_score;
                    count <= count + 1'b1;
                end
                if (nms_start) begin
                    out_cnt   <= 0;
                    si        <= 0;
                    best_found<= 1'b0;
                    if (count == 0)
                        st <= S_DONE;
                    else
                        st <= S_SCAN;
                end
            end

            // ---------- argmax over non-suppressed ----------
            S_SCAN: begin
                if (!s_sup && (!best_found || s_score > best_score)) begin
                    best_score <= s_score;
                    best_idx   <= si[AW-1:0];
                    best_found <= 1'b1;
                end
                if (si == count - 1'b1) begin
                    if (!best_found && s_sup)  // last entry suppressed, none found earlier
                        st <= S_DONE;
                    else
                        st <= S_LATCH;
                end else
                    si <= si + 1'b1;
            end

            // ---------- read winner coords ----------
            S_LATCH: begin
                if (!best_found) begin
                    st <= S_DONE;              // nothing left to emit
                end else begin
                    win_x <= bx_mem[best_idx];
                    win_y <= by_mem[best_idx];
                    st    <= S_EMIT;
                end
            end

            // ---------- emit winner, mark it suppressed ----------
            S_EMIT: begin
                out_x     <= bx_mem[best_idx];
                out_y     <= by_mem[best_idx];
                out_index <= out_cnt;
                out_valid <= 1'b1;
                suppressed[best_idx] <= 1'b1;
                out_cnt   <= out_cnt + 1'b1;
                ji        <= 0;
                st        <= S_SUPPR;
            end

            // ---------- suppress overlaps with winner ----------
            S_SUPPR: begin
                if (!suppressed[ji[AW-1:0]] && overlap)
                    suppressed[ji[AW-1:0]] <= 1'b1;
                if (ji == count - 1'b1) begin
                    // another round?
                    if (out_cnt == MAXOUT[3:0])
                        st <= S_DONE;
                    else begin
                        si         <= 0;
                        best_found <= 1'b0;
                        st         <= S_SCAN;
                    end
                end else
                    ji <= ji + 1'b1;
            end

            // ---------- finished this frame ----------
            S_DONE: begin
                nms_done   <= 1'b1;
                count      <= 0;       // ready for next frame
                suppressed <= 0;
                st         <= S_IDLE;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule