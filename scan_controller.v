//============================================================================
// scan_controller.v  (two-identity recognition version)
//----------------------------------------------------------------------------
// Orchestrates Phase-3 detection + recognition for one camera frame, then
// hands (box, match, match_id) to cdc_coords.
//
//   LOAD  : read completed SRAM frame via 4th client, grayscale, store in
//           gray_frame_buf. SRAM byte addr = read_base + (pixel_index<<1)
//           (sram_ctrl stores 2 bytes/pixel).
//   SCAN  : for each of NUM_X*NUM_Y window positions, stream a 66x66 region
//           (window edges REPLICATED via local clamp to [0,63], reproducing
//           HOG_Extract.py border diff) through line_buffer_3x3(66) ->
//           hog_gradient -> cell_hist_bin -> svm_addertree; face windows -> nms.
//   NMS   : greedy suppression -> up to 8 boxes.
//   RECOG : per box, stream 64x64 gray crop -> lbp_hist -> l1_compare
//           (TWO-IDENTITY: compares against Malay AND Nilesh in parallel,
//           returns best_dist + which identity it was closer to) -> match.
//   RESULT: load matched box (else top box, else none) + matched identity
//           into cdc_coords.
//
// gray_frame_buf read = 1-cycle latency; every feed registers a 1-cycle valid
// alongside the address. line_buffer_3x3 is reset (win_rst) before each window.
//============================================================================
`timescale 1ns / 1ps

module scan_controller #(
    parameter integer FRAME_W = 320,
    parameter integer FRAME_H = 240,
    parameter integer STRIDE  = 8,
    parameter integer NUM_X   = (320-64)/8 + 1,   // 33
    parameter integer NUM_Y   = (240-64)/8 + 1,   // 23
    parameter integer NPIX    = 320*240,          // 76800
    parameter integer AW      = 17,
    parameter         LUT_FILE     = "uniform_lut.mem",
    parameter         WEIGHT_FILE  = "svm_weights.mem",
    parameter         BIAS_FILE    = "svm_bias.mem",
    parameter         MALAY_FILE   = "lbph_hist_Malay.mem",
    parameter         NILESH_FILE  = "lbph_hist_Nilesh.mem"
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    output reg         busy,
    // SRAM 4th client (frame load only)
    output reg         lbph_req,
    output reg  [18:0] lbph_addr,
    input  wire [11:0] lbph_pixel,
    input  wire        lbph_valid,
    input  wire [18:0] read_base,
    // result to cdc_coords
    output reg         result_load,
    output reg  [8:0]  res_box_x,
    output reg  [7:0]  res_box_y,
    output reg         res_valid,
    output reg         res_match,
    output reg         res_match_id,    // 0=MALAY, 1=NILESH (valid if res_match=1)
    // ---- debug readout (updated once per completed pass) ----
    output reg  [15:0] dbg_ncand,    // # windows that scored face=1 this pass
    output reg  [15:0] dbg_minl1     // smallest LBPH L1 distance this pass (FFFF = none)
);

    // ---------- gray frame buffer ----------
    reg           gfb_we; reg [AW-1:0] gfb_waddr; reg [7:0] gfb_wdata;
    reg [AW-1:0]  gfb_raddr; wire [7:0] gfb_rdata;
    gray_frame_buf #(.NPIX(NPIX), .AW(AW)) u_gfb (
        .clk(clk), .we(gfb_we), .waddr(gfb_waddr), .wdata(gfb_wdata),
        .raddr(gfb_raddr), .rdata(gfb_rdata));

    // ---------- load-path grayscale ----------
    wire [7:0] gl_gray; wire gl_valid;
    grayscale u_gl (.clk(clk), .rst(rst),
        .pixel_in(lbph_pixel), .pixel_valid(lbph_valid),
        .gray(gl_gray), .gray_valid(gl_valid));

    // ---------- detection chain ----------
    reg        win_rst, win_start; wire scan_fv;
    wire       lb_rst = rst | win_rst;
    wire [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22;
    wire       lb_wv; wire [9:0] lb_col, lb_row;
    line_buffer_3x3 #(.WIDTH(66), .HEIGHT(66)) u_lb (
        .clk(clk), .rst(lb_rst), .pix_in(gfb_rdata), .pix_valid(scan_fv),
        .p00(p00),.p01(p01),.p02(p02),.p10(p10),.p11(p11),.p12(p12),
        .p20(p20),.p21(p21),.p22(p22),
        .window_valid(lb_wv), .win_col(lb_col), .win_row(lb_row));
    wire [10:0] mag; wire [3:0] bin; wire hog_v; wire [9:0] hcol,hrow;
    hog_gradient u_hog (.clk(clk), .rst(lb_rst),
        .p01(p01),.p10(p10),.p12(p12),.p21(p21),
        .window_valid(lb_wv), .win_col_in(lb_col), .win_row_in(lb_row),
        .mag(mag), .bin(bin), .out_valid(hog_v), .win_col(hcol), .win_row(hrow));
    wire [35:0] blk_bits; wire [5:0] blk_idx; wire blk_v, desc_done, chb_ready;
    cell_hist_bin u_chb (.clk(clk), .rst(rst), .win_start(win_start),
        .pix_valid(hog_v), .mag(mag), .bin(bin), .pix_ready(chb_ready),
        .block_bits(blk_bits), .block_index(blk_idx),
        .block_valid(blk_v), .desc_done(desc_done));
    wire signed [26:0] svm_score; wire svm_face, svm_v;
    svm_addertree #(.WEIGHT_FILE(WEIGHT_FILE), .BIAS_FILE(BIAS_FILE)) u_svm (
        .clk(clk), .rst(rst), .win_start(win_start),
        .block_valid(blk_v), .block_index(blk_idx), .block_bits(blk_bits),
        .desc_done(desc_done), .score(svm_score), .face(svm_face),
        .score_valid(svm_v));

    // ---------- NMS ----------
    reg cand_valid; reg [8:0] cand_x; reg [7:0] cand_y;
    reg signed [26:0] cand_score; reg nms_start;
    wire [8:0] nms_ox; wire [7:0] nms_oy; wire [3:0] nms_oi; wire nms_ov, nms_done;
    nms u_nms (.clk(clk), .rst(rst),
        .cand_valid(cand_valid), .cand_x(cand_x), .cand_y(cand_y),
        .cand_score(cand_score), .nms_start(nms_start),
        .out_x(nms_ox), .out_y(nms_oy), .out_index(nms_oi),
        .out_valid(nms_ov), .nms_done(nms_done));
    reg [8:0] box_x [0:7]; reg [7:0] box_y [0:7]; reg [3:0] n_boxes;

    // ---------- LBPH recognition (two-identity) ----------
    reg lbp_start; wire recog_fv; wire [5:0] recog_c, recog_r;
    wire lbp_ready, hist_done; wire [11:0] lbp_rd_addr; wire [7:0] lbp_rd_data;
    lbp_hist #(.LUT_FILE(LUT_FILE)) u_lbp (.clk(clk), .rst(rst),
        .start(lbp_start), .ready(lbp_ready),
        .pix_valid(recog_fv), .gray(gfb_rdata),
        .win_col(recog_c), .win_row(recog_r),
        .hist_done(hist_done), .rd_addr(lbp_rd_addr), .rd_data(lbp_rd_data));
    reg l1_start; wire [11:0] l1_addr; wire [15:0] l1_dist;
    wire l1_match, l1_matchid, l1_done;
    l1_compare #(.THRESHOLD(3793),
                 .MALAY_FILE(MALAY_FILE), .NILESH_FILE(NILESH_FILE)) u_l1 (
        .clk(clk), .rst(rst),
        .start(l1_start), .live_addr(l1_addr), .live_data(lbp_rd_data),
        .dist(l1_dist), .match(l1_match), .match_id(l1_matchid), .done(l1_done));
    assign lbp_rd_addr = l1_addr;

    // ---------- FSM ----------
    localparam [4:0]
        S_IDLE=0, S_LOAD=1, S_LOAD_DRAIN=2,
        S_SCAN_NEW=3, S_SCAN_FEED=4, S_SCAN_WAIT=5,
        S_NMS=6, S_NMS_COLLECT=7,
        S_REC_NEW=8, S_REC_CLR=9, S_REC_RDY=14, S_REC_FEED=10, S_REC_HIST=11, S_REC_CMP=12,
        S_RESULT=13;
    reg [4:0] st;

    reg [AW-1:0] load_ptr, load_wptr;
    reg [5:0]  wx, wy; reg [8:0] bx; reg [7:0] by;
    reg [6:0]  fr, fc; reg feed_run;
    reg        sfv_d1, sfv_d2;                      // scan valid pipeline (2 stg)
    reg [6:0]  rr, rc; reg rfeed_run;
    reg        rfv_d1, rfv_d2;                      // recog valid pipeline
    reg [5:0]  rc_d1, rc_d2, rr_d1, rr_d2;          // recog position pipeline
    reg [3:0]  rbox; reg any_match; reg [3:0] match_box; reg matched_id;
    // debug accumulators (per pass)
    reg [15:0] cand_cnt, min_l1;

    assign scan_fv  = sfv_d2;
    assign recog_fv = rfv_d2;
    assign recog_c  = rc_d2;
    assign recog_r  = rr_d2;

    // clamped local coords -> replicate window edge
    wire [6:0] frm1 = fr - 7'd1, fcm1 = fc - 7'd1;
    wire [5:0] clr = (fr==7'd0)?6'd0:(fr>=7'd65)?6'd63:frm1[5:0];
    wire [5:0] clc = (fc==7'd0)?6'd0:(fc>=7'd65)?6'd63:fcm1[5:0];
    wire [8:0] f_row = by + {3'd0,clr};
    wire [8:0] f_col = bx + {3'd0,clc};
    wire [AW-1:0] scan_addr = f_row*FRAME_W + f_col;

    wire [8:0] r_row = box_y[rbox] + {2'd0,rr};
    wire [8:0] r_col = box_x[rbox] + {2'd0,rc};
    wire [AW-1:0] rec_addr = r_row*FRAME_W + r_col;

    always @(posedge clk) begin
        if (rst) begin
            st<=S_IDLE; busy<=0; result_load<=0; lbph_req<=0; lbph_addr<=0;
            gfb_we<=0; gfb_waddr<=0; gfb_wdata<=0; gfb_raddr<=0;
            win_rst<=0; win_start<=0; sfv_d1<=0; sfv_d2<=0;
            cand_valid<=0; nms_start<=0; n_boxes<=0;
            lbp_start<=0; rfv_d1<=0; rfv_d2<=0; l1_start<=0;
            rc_d1<=0; rc_d2<=0; rr_d1<=0; rr_d2<=0;
            load_ptr<=0; load_wptr<=0; wx<=0; wy<=0; bx<=0; by<=0;
            fr<=0; fc<=0; feed_run<=0;
            rr<=0; rc<=0; rfeed_run<=0;
            rbox<=0; any_match<=0; match_box<=0; matched_id<=0;
            res_valid<=0; res_match<=0; res_match_id<=0; res_box_x<=0; res_box_y<=0;
            dbg_ncand<=0; dbg_minl1<=16'hFFFF;
            cand_cnt<=0;  min_l1<=16'hFFFF;
        end else begin
            win_rst<=0; win_start<=0; cand_valid<=0; nms_start<=0;
            lbp_start<=0; l1_start<=0; result_load<=0; gfb_we<=0;

            case (st)

            S_IDLE: begin
                busy<=0;
                if (start) begin
                    busy<=1; load_ptr<=0; load_wptr<=0;
                    lbph_addr<=read_base; lbph_req<=1;
                    cand_cnt<=0; min_l1<=16'hFFFF;   // reset debug accumulators
                    st<=S_LOAD;
                end
            end

            // ---- load frame (grayscale into gray_frame_buf) ----
            S_LOAD: begin
                lbph_req<=1;
                lbph_addr<=read_base + {load_ptr,1'b0};   // pixel_index<<1
                if (gl_valid) begin
                    gfb_we<=1; gfb_waddr<=load_wptr; gfb_wdata<=gl_gray;
                    load_wptr<=load_wptr+1'b1;
                end
                if (lbph_valid) begin
                    if (load_ptr==NPIX-1) begin lbph_req<=0; st<=S_LOAD_DRAIN; end
                    else load_ptr<=load_ptr+1'b1;
                end
            end
            S_LOAD_DRAIN: begin
                if (gl_valid) begin
                    gfb_we<=1; gfb_waddr<=load_wptr; gfb_wdata<=gl_gray;
                    load_wptr<=load_wptr+1'b1;
                    wx<=0; wy<=0; bx<=0; by<=0; n_boxes<=0;
                    st<=S_SCAN_NEW;
                end
            end

            // ---- scan: start window ----
            S_SCAN_NEW: begin
                win_rst<=1; win_start<=1;
                fr<=0; fc<=0; feed_run<=1; sfv_d1<=0; sfv_d2<=0;
                st<=S_SCAN_FEED;
            end

            // ---- scan: stream 66x66; valid delayed 2 cyc to match gfb read ----
            S_SCAN_FEED: begin
                sfv_d1 <= feed_run;
                sfv_d2 <= sfv_d1;
                if (feed_run) begin
                    gfb_raddr <= scan_addr;
                    if (fc==7'd65) begin
                        fc<=0;
                        if (fr==7'd65) feed_run<=0;
                        else fr<=fr+1'b1;
                    end else fc<=fc+1'b1;
                end
                if (!feed_run && !sfv_d1 && !sfv_d2)
                    st<=S_SCAN_WAIT;
            end

            // ---- scan: capture verdict, next window ----
            S_SCAN_WAIT: begin
                if (svm_v) begin
                    if (svm_face) begin
                        cand_valid<=1; cand_x<=bx; cand_y<=by; cand_score<=svm_score;
                        cand_cnt <= cand_cnt + 1'b1;      // debug: count positives
                    end
                    if (wx==NUM_X-1) begin
                        wx<=0; bx<=0;
                        if (wy==NUM_Y-1) st<=S_NMS;
                        else begin wy<=wy+1'b1; by<=by+STRIDE[7:0]; st<=S_SCAN_NEW; end
                    end else begin
                        wx<=wx+1'b1; bx<=bx+STRIDE[8:0]; st<=S_SCAN_NEW;
                    end
                end
            end

            // ---- NMS ----
            S_NMS: begin nms_start<=1; n_boxes<=0; st<=S_NMS_COLLECT; end
            S_NMS_COLLECT: begin
                if (nms_ov) begin
                    box_x[n_boxes[2:0]]<=nms_ox; box_y[n_boxes[2:0]]<=nms_oy;
                    n_boxes<=n_boxes+1'b1;
                end
                if (nms_done) begin
                    rbox<=0; any_match<=0; match_box<=0; matched_id<=0; st<=S_REC_NEW;
                end
            end

            // ---- recognition ----
            S_REC_NEW: st <= (n_boxes==0) ? S_RESULT : S_REC_CLR;
            S_REC_CLR: begin
                lbp_start<=1; rr<=0; rc<=0; rfeed_run<=0;
                rfv_d1<=0; rfv_d2<=0;
                st<=S_REC_RDY;
            end
            S_REC_RDY: begin
                if (lbp_ready) begin rfeed_run<=1; st<=S_REC_FEED; end
            end
            S_REC_FEED: begin
                rfv_d1 <= rfeed_run; rfv_d2 <= rfv_d1;
                rc_d1  <= rc[5:0];   rc_d2  <= rc_d1;
                rr_d1  <= rr[5:0];   rr_d2  <= rr_d1;
                if (rfeed_run) begin
                    gfb_raddr <= rec_addr;
                    if (rc==7'd63) begin
                        rc<=0;
                        if (rr==7'd63) rfeed_run<=0;
                        else rr<=rr+1'b1;
                    end else rc<=rc+1'b1;
                end
                if (!rfeed_run && !rfv_d1 && !rfv_d2)
                    st<=S_REC_HIST;
            end
            S_REC_HIST: if (hist_done) begin l1_start<=1; st<=S_REC_CMP; end
            S_REC_CMP: begin
                if (l1_done) begin
                    if (l1_dist < min_l1) min_l1 <= l1_dist;   // debug: best L1
                    if (l1_match && !any_match) begin
                        any_match<=1; match_box<=rbox; matched_id<=l1_matchid;
                    end
                    if (rbox==n_boxes-1) st<=S_RESULT;
                    else begin rbox<=rbox+1'b1; st<=S_REC_CLR; end
                end
            end

            // ---- result ----
            S_RESULT: begin
                if (n_boxes==0) begin
                    res_valid<=0; res_match<=0; res_match_id<=0;
                    res_box_x<=0; res_box_y<=0;
                end else if (any_match) begin
                    res_valid<=1; res_match<=1; res_match_id<=matched_id;
                    res_box_x<=box_x[match_box]; res_box_y<=box_y[match_box];
                end else begin
                    res_valid<=1; res_match<=0; res_match_id<=0;
                    res_box_x<=box_x[0]; res_box_y<=box_y[0];
                end
                result_load<=1;
                dbg_ncand <= cand_cnt;
                dbg_minl1 <= min_l1;
                st<=S_IDLE;
            end

            default: st<=S_IDLE;
            endcase
        end
    end

endmodule