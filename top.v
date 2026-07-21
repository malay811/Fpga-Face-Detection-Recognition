//============================================================================
// top.v  - Real-time face detection + TWO-IDENTITY recognition (Malay/Nilesh)
//----------------------------------------------------------------------------
// Three clock domains from clk_wiz_0 (50 MHz board oscillator in):
//   clk_pix  25.2 MHz  VGA pixel / display / overlay
//   clk_sys  50   MHz  SRAM, detection, recognition
//   clk_cam  24   MHz  OV7670 XCLK (forwarded to camera)
//
// Dataflow:
//   OV7670 -> ov7670_capture(pclk) -> FIFO(pclk->sys) -> sram_ctrl -> SRAM
//   SRAM   -> sram_ctrl fill -> linebuffer(sys->pix) -> osd_overlay -> VGA
//   SRAM   -> scan_controller (4th client, one load/pass) -> gray_frame_buf
//            -> HOG/SVM scan -> NMS -> LBPH (dual-identity L1 compare)
//            -> (box, match, match_id)
//   (box,match,match_id) -> cdc_coords(sys->pix) -> osd_overlay
//
// Detection trigger: on cam_vsync rising edge in clk_sys, start a pass IF not
// already busy. Video stays 60 Hz; box refreshes ~13/s.
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module top (
    input  wire        clk_50,        // board oscillator

    // OV7670 camera
    input  wire        cam_pclk,
    input  wire        cam_vsync,
    input  wire        cam_href,
    input  wire [7:0]  cam_data,
    output wire        cam_xclk,      // 24 MHz to camera
    output wire        cam_sioc,      // SCCB clock
    inout  wire        cam_siod,      // SCCB data
    output wire        cam_reset_n,   // camera reset - released when MMCM locks
    output wire        cam_pwdn,      // power down (tie low)

    // external async SRAM (IS61WV5128BLL-10)
    output wire [18:0] sram_addr,
    inout  wire [7:0]  sram_dq,
    output wire        sram_ce_n,
    output wire        sram_oe_n,
    output wire        sram_we_n,

    // VGA (RGB444)
    output wire        vga_hsync,
    output wire        vga_vsync,
    output wire [3:0]  vga_r,
    output wire [3:0]  vga_g,
    output wire [3:0]  vga_b
);

    assign cam_pwdn = 1'b0;   // camera powered on

    // ==================== clocking ====================
    wire clk_pix, clk_sys, clk_cam, clk_locked;
    clk_wiz_0 u_clk (
        .clk_in1(clk_50),
        .clk_out1(clk_pix),      // 25.2 MHz
        .clk_out2(clk_sys),      // 50 MHz
        .clk_out3(clk_cam),      // 24 MHz
        .reset(1'b0),
        .locked(clk_locked)
    );

    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("ASYNC")) u_xclk (
        .Q(cam_xclk), .C(clk_cam), .CE(1'b1),
        .D1(1'b1), .D2(1'b0), .R(1'b0), .S(1'b0));

    wire pclk_g;
    BUFG u_bufg_pclk (.I(cam_pclk), .O(pclk_g));

    // ==================== reset synchronizers ====================
    reg [1:0] r_sys, r_pix, r_pclk;
    wire rst_sys  = r_sys[1];
    wire rst_pix  = r_pix[1];
    wire rst_cam  = r_pclk[1];
    always @(posedge clk_sys) r_sys  <= {r_sys[0],  ~clk_locked};
    always @(posedge clk_pix) r_pix  <= {r_pix[0],  ~clk_locked};
    always @(posedge pclk_g)  r_pclk <= {r_pclk[0], ~clk_locked};

    assign cam_reset_n = clk_locked;

    // ==================== SCCB camera config ====================
    wire cam_config_done;
    sccb_master #(.CLK_FREQ(50_000_000)) u_sccb (
        .clk(clk_sys), .rst(rst_sys),
        .sccb_scl(cam_sioc), .sccb_sda(cam_siod),
        .config_done(cam_config_done));

    // ==================== camera capture (pclk domain) ====================
    wire        cap_valid;
    wire [11:0] cap_pixel;
    ov7670_capture #(.H_ACT(320), .V_ACT(240)) u_cap (
        .pclk(pclk_g), .rst(rst_cam),
        .vsync(cam_vsync), .href(cam_href), .d(cam_data),
        .pixel(cap_pixel), .pixel_valid(cap_valid));

    // ==================== FIFO (pclk -> sys) ====================
    wire [11:0] fifo_dout;
    wire        fifo_empty, fifo_full, fifo_rd_en;
    fifo_generator_0 u_fifo (
        .rst(rst_sys),
        .wr_clk(pclk_g), .rd_clk(clk_sys),
        .din(cap_pixel), .wr_en(cap_valid & ~fifo_full),
        .rd_en(fifo_rd_en), .dout(fifo_dout),
        .full(fifo_full), .empty(fifo_empty));

    // ==================== cam_vsync into clk_sys ====================
    reg vs1, vs2, vs3;
    always @(posedge clk_sys) begin vs1<=cam_vsync; vs2<=vs1; vs3<=vs2; end
    wire vsync_rise = vs2 & ~vs3;

    // ==================== SRAM controller ====================
    wire        fill_req, fill_half, fill_done;
    wire [8:0]  fill_row;
    wire        lb_we;
    wire [9:0]  lb_waddr;
    wire [11:0] lb_wdata;
    wire        lbph_req;
    wire [18:0] lbph_addr;
    wire [11:0] lbph_pixel;
    wire        lbph_valid;
    wire [18:0] read_base;

    sram_ctrl #(.H_ACT(320), .V_ACT(240)) u_sram (
        .clk(clk_sys), .rst(rst_sys),
        .fifo_dout(fifo_dout), .fifo_empty(fifo_empty), .fifo_rd_en(fifo_rd_en),
        .cam_vsync(cam_vsync),
        .fill_req(fill_req), .fill_half(fill_half), .fill_row(fill_row),
        .fill_done(fill_done),
        .lb_we(lb_we), .lb_waddr(lb_waddr), .lb_wdata(lb_wdata),
        .lbph_req(lbph_req), .lbph_addr(lbph_addr),
        .lbph_pixel(lbph_pixel), .lbph_valid(lbph_valid),
        .read_base_o(read_base),
        .sram_addr(sram_addr), .sram_dq(sram_dq),
        .sram_ce_n(sram_ce_n), .sram_oe_n(sram_oe_n), .sram_we_n(sram_we_n));

    // ==================== display line buffer ====================
    wire [9:0]  lb_raddr;
    wire [11:0] lb_rdata;
    linebuffer u_lb_disp (
        .clk_w(clk_sys), .we(lb_we), .waddr(lb_waddr), .wdata(lb_wdata),
        .clk_r(clk_pix), .raddr(lb_raddr), .rdata(lb_rdata));

    // ==================== display timing + reader ====================
    wire        de, frame, line;
    wire signed [10:0] sx, sy;
    display_480p u_disp (
        .clk_pix(clk_pix), .rst_pix(rst_pix),
        .hsync(vga_hsync), .vsync(vga_vsync),
        .de(de), .frame(frame), .line(line), .sx(sx), .sy(sy));

    display_reader #(.V_ACT(240)) u_rdr (
        .clk_pix(clk_pix), .rst_pix(rst_pix),
        .sx(sx), .sy(sy), .frame(frame), .line(line), .lb_raddr(lb_raddr),
        .clk_sys(clk_sys), .rst_sys(rst_sys),
        .fill_req(fill_req), .fill_half(fill_half), .fill_row(fill_row),
        .fill_done(fill_done));

    // ==================== detection + recognition (two-identity) ====================
    wire        scan_busy, result_load, res_valid, res_match, res_match_id;
    wire [8:0]  res_box_x;
    wire [7:0]  res_box_y;
    wire [15:0] dbg_ncand, dbg_minl1;  // kept as internal wires, not driven to pins
    reg  scan_start;
    always @(posedge clk_sys)
        scan_start <= rst_sys ? 1'b0 : (vsync_rise & ~scan_busy);

    scan_controller #(
        .FRAME_W(320), .FRAME_H(240), .STRIDE(8),
        .NUM_X(33), .NUM_Y(23), .NPIX(76800), .AW(17)
    ) u_scan (
        .clk(clk_sys), .rst(rst_sys), .start(scan_start), .busy(scan_busy),
        .lbph_req(lbph_req), .lbph_addr(lbph_addr),
        .lbph_pixel(lbph_pixel), .lbph_valid(lbph_valid), .read_base(read_base),
        .result_load(result_load),
        .res_box_x(res_box_x), .res_box_y(res_box_y),
        .res_valid(res_valid), .res_match(res_match), .res_match_id(res_match_id),
        .dbg_ncand(dbg_ncand), .dbg_minl1(dbg_minl1));

    // ==================== coords CDC (sys -> pix), two-identity ====================
    wire [8:0]  osd_box_x;
    wire [7:0]  osd_box_y;
    wire        osd_valid, osd_match, osd_match_id;
    cdc_coords u_cdc (
        .src_clk(clk_sys), .src_rst(rst_sys),
        .load(result_load), .box_x_i(res_box_x), .box_y_i(res_box_y),
        .valid_i(res_valid), .match_i(res_match), .match_id_i(res_match_id),
        .src_ready(),
        .dst_clk(clk_pix), .dst_rst(rst_pix),
        .box_x_o(osd_box_x), .box_y_o(osd_box_y),
        .valid_o(osd_valid), .match_o(osd_match), .match_id_o(osd_match_id),
        .dst_update());

    // ==================== overlay -> VGA ====================
    wire [11:0] vga_pix;
    osd_overlay u_osd (
        .clk(clk_pix), .rst(rst_pix),
        .sx(sx), .sy(sy), .de(de), .pix_in(lb_rdata),
        .box_valid(osd_valid), .match(osd_match), .match_id(osd_match_id),
        .box_x(osd_box_x), .box_y(osd_box_y),
        .pix_out(vga_pix));

    reg de_d;
    always @(posedge clk_pix) de_d <= de;

    assign vga_r = de_d ? vga_pix[11:8] : 4'd0;
    assign vga_g = de_d ? vga_pix[7:4]  : 4'd0;
    assign vga_b = de_d ? vga_pix[3:0]  : 4'd0;

endmodule

