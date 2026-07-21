//============================================================================
// sram_ctrl.v  (FIXED)
//----------------------------------------------------------------------------
// External-SRAM controller with 4-client arbitration for the IS61WV5128BLL-10
// (512K x 8, 10 ns access). Single async SRAM => one access at a time; the FSM
// returns to ARB after every pixel so clients interleave fairly.
//
// Clients, in priority order (checked each ARB cycle):
//   0. Pending buffer swap (highest) - only committed once old frame's FIFO
//      backlog is fully drained, so no stale pixel writes land in new buffer.
//   1. Camera write              - FIFO -> SRAM; camera stream cannot stall.
//   2. Display read (active)     - continue an in-progress line fill for VGA.
//   3. Display read (pending)    - start a newly requested line fill.
//   4. LBPH read      (lowest)   - recognition reads a 64x64 face region.
//
// Frame buffers ping-pong on camera vsync: camera writes one buffer while
// display + LBPH read the other (read_base). Each 12-bit RGB444 pixel occupies
// two consecutive SRAM bytes (low byte at even addr, high nibble at odd addr).
//
// FIX (vs original):
//   - vsync no longer swaps write_buf immediately. It only sets
//     swap_pending; the actual swap (write_buf toggle + wr_idx reset) is
//     deferred to the ARB state and only commits once fifo_empty is true,
//     guaranteeing all leftover camera-write-FIFO pixels from the OLD frame
//     finish writing into the OLD buffer before the pointer moves.
//   - wr_byte0 (pixel write address) is now latched into wr_addr_lat at
//     WR0 entry and reused in WR1, instead of being recomputed
//     combinationally from write_buf/wr_idx. This prevents a swap that
//     occurs between WR0 and WR1 of the same pixel from splitting that
//     pixel's two bytes across two different buffers.
//
// Timing (all met at 50 MHz / 20 ns cycle):
//   - Write pulse (we_n low) held 1 cycle = 20 ns  > tWP 8 ns.
//   - Read access: address driven, sampled next cycle = 20 ns > tAA 10 ns.
//   - Each pixel = 1 ARB + 4 op cycles = 5 cycles.
//============================================================================
`timescale 1ns / 1ps

module sram_ctrl #(
    parameter integer H_ACT = 320,
    parameter integer V_ACT = 240,
    parameter integer BUF_A = 0,
    parameter integer BUF_B = 320*240*2   // 153600
)(
    input  wire        clk,        // 50 MHz system clock
    input  wire        rst,        // synchronous, active-high

    // ---- Client 1: camera write (from FIFO) ----
    input  wire [11:0] fifo_dout,
    input  wire        fifo_empty,
    output reg         fifo_rd_en,
    input  wire        cam_vsync,  // frame sync for ping-pong swap

    // ---- Client 2/3: display line fill (to linebuffer) ----
    input  wire        fill_req,
    input  wire        fill_half,
    input  wire [8:0]  fill_row,
    output reg         fill_done,
    output reg         lb_we,
    output reg  [9:0]  lb_waddr,
    output reg  [11:0] lb_wdata,

    // ---- Client 4: LBPH read (recognition) ----
    input  wire        lbph_req,    // pulse: read a pixel at lbph_addr
    input  wire [18:0] lbph_addr,   // SRAM byte address (low byte) to read
    output reg  [11:0] lbph_pixel,  // assembled RGB444 pixel
    output reg         lbph_valid,  // 1-cycle strobe when lbph_pixel ready
    output wire [18:0] read_base_o, // exposed: base of the display/read buffer

    // ---- Physical SRAM pins ----
    output reg  [18:0] sram_addr,
    inout  wire [7:0]  sram_dq,
    output reg         sram_ce_n,
    output reg         sram_oe_n,
    output reg         sram_we_n
);

    // ---- Bidirectional data bus ----
    reg  [7:0] dq_out;
    reg        dq_oe;
    assign sram_dq = dq_oe ? dq_out : 8'bz;
    wire [7:0] dq_in = sram_dq;

    // ---- vsync synchronizer + frame-start edge detect ----
    reg vs0, vs1, vs2;
    always @(posedge clk) begin
        if (rst) {vs0,vs1,vs2} <= 3'b000;
        else     {vs0,vs1,vs2} <= {cam_vsync, vs0, vs1};
    end
    wire cam_frame = (vs1 & ~vs2);   // rising edge of vsync = start of blanking

    // ---- Ping-pong frame buffers ----
    reg         write_buf;          // 0: cam->A, disp->B ; 1: cam->B, disp->A
    reg         swap_pending;       // FIX: latched on vsync, committed in ARB
    reg  [16:0] wr_idx;             // camera pixel write index (0..76799)
    wire [18:0] write_base = write_buf ? BUF_B[18:0] : BUF_A[18:0];
    wire [18:0] read_base  = write_buf ? BUF_A[18:0] : BUF_B[18:0];
    wire [18:0] wr_byte0   = write_base + {wr_idx,1'b0};   // pixel_index*2
    assign read_base_o = read_base;   // LBPH uses this to compute addresses

    reg [18:0] wr_addr_lat;         // FIX: latched write address for this pixel

    // ---- Display fill state ----
    reg        fill_pending, fill_active;
    reg  [8:0] req_row, f_row;
    reg        req_half, f_half;
    reg  [8:0] f_col;
    reg  [7:0] rd_low;
    wire [18:0] rd_byte0 = read_base + (((f_row*H_ACT) + f_col) << 1);

    // ---- LBPH read state ----
    reg        lbph_pending;        // latched request awaiting service
    reg [18:0] lbph_byte0;          // captured target address
    reg  [7:0] lbph_low;            // low byte held between the two reads

    // ---- FSM states ----
    localparam [3:0]
        ARB  = 4'd0,
        WR0  = 4'd1, WR0b = 4'd2, WR1  = 4'd3, WR1b = 4'd4,
        RD0  = 4'd5, RD0b = 4'd6, RD1  = 4'd7, RD1b = 4'd8,
        LRD0 = 4'd9, LRD0b= 4'd10, LRD1= 4'd11, LRD1b=4'd12;

    reg [3:0]  st;
    reg [11:0] wpix;

    always @(posedge clk) begin
        if (rst) begin
            st           <= ARB;
            write_buf    <= 1'b0;
            swap_pending <= 1'b0;
            wr_idx       <= 17'd0;
            wr_addr_lat  <= 19'd0;
            fill_pending <= 1'b0;
            fill_active  <= 1'b0;
            f_col        <= 9'd0;
            f_row        <= 9'd0;
            f_half       <= 1'b0;
            req_row      <= 9'd0;
            req_half     <= 1'b0;
            lbph_pending <= 1'b0;
            lbph_byte0   <= 19'd0;
            lbph_valid   <= 1'b0;
            lbph_pixel   <= 12'd0;
            fifo_rd_en   <= 1'b0;
            fill_done    <= 1'b0;
            lb_we        <= 1'b0;
            dq_oe        <= 1'b0;
            dq_out       <= 8'd0;
            sram_ce_n    <= 1'b1;
            sram_oe_n    <= 1'b1;
            sram_we_n    <= 1'b1;
            sram_addr    <= 19'd0;
        end else begin
            // default one-cycle strobes
            fifo_rd_en <= 1'b0;
            fill_done  <= 1'b0;
            lb_we      <= 1'b0;
            lbph_valid <= 1'b0;

            // latch display fill request (any state)
            if (fill_req) begin
                fill_pending <= 1'b1;
                req_row      <= fill_row;
                req_half     <= fill_half;
            end

            // latch LBPH read request (any state)
            if (lbph_req) begin
                lbph_pending <= 1'b1;
                lbph_byte0   <= lbph_addr;
            end

            // FIX: only *latch* the swap request on vsync. Do NOT touch
            // write_buf/wr_idx here anymore - that happens in ARB, gated
            // on fifo_empty, so in-flight/backlogged camera pixels from
            // the old frame finish writing to the old buffer first.
            if (cam_frame) begin
                swap_pending <= 1'b1;
            end

            case (st)

            // ---------------- ARBITRATION ----------------
            ARB: begin
                sram_ce_n <= 1'b1;
                sram_oe_n <= 1'b1;
                sram_we_n <= 1'b1;
                dq_oe     <= 1'b0;
                if (swap_pending && fifo_empty) begin
                    // FIX: commit the deferred ping-pong swap now that the
                    // old frame's FIFO backlog is fully drained.
                    write_buf    <= ~write_buf;
                    wr_idx       <= 17'd0;
                    swap_pending <= 1'b0;
                    st           <= ARB;   // re-arbitrate next cycle
                end else if (!fifo_empty) begin
                    // Client 1: camera write (highest priority)
                    wpix       <= fifo_dout;
                    fifo_rd_en <= 1'b1;
                    st         <= WR0;
                end else if (fill_active) begin
                    // Client 2: continue active display fill
                    st <= RD0;
                end else if (fill_pending) begin
                    // Client 3: start new display fill
                    fill_active <= 1'b1;
                    f_row       <= req_row;
                    f_half      <= req_half;
                    f_col       <= 9'd0;
                    if (!fill_req) fill_pending <= 1'b0;
                    st <= RD0;
                end else if (lbph_pending) begin
                    // Client 4: LBPH read (lowest priority)
                    if (!lbph_req) lbph_pending <= 1'b0;
                    st <= LRD0;
                end
            end

            // ---------------- CAMERA WRITE ----------------
            WR0: begin
                wr_addr_lat <= wr_byte0;      // FIX: latch base addr for this pixel
                sram_addr   <= wr_byte0;
                dq_out      <= wpix[7:0];
                dq_oe       <= 1'b1;
                sram_ce_n   <= 1'b0;
                sram_oe_n   <= 1'b1;
                sram_we_n   <= 1'b0;    // write pulse low
                st          <= WR0b;
            end
            WR0b: begin
                sram_we_n <= 1'b1;    // end write pulse (>=8 ns satisfied)
                st        <= WR1;
            end
            WR1: begin
                sram_addr <= wr_addr_lat + 19'd1;  // FIX: use latched base, not
                                                     // live/recomputed wr_byte0
                dq_out    <= {4'b0, wpix[11:8]};
                dq_oe     <= 1'b1;
                sram_ce_n <= 1'b0;
                sram_oe_n <= 1'b1;
                sram_we_n <= 1'b0;
                st        <= WR1b;
            end
            WR1b: begin
                sram_we_n <= 1'b1;
                dq_oe     <= 1'b0;
                if (wr_idx < (H_ACT*V_ACT-1)) wr_idx <= wr_idx + 1'b1;
                st        <= ARB;     // revisit ARB after each pixel
            end

            // ---------------- DISPLAY FILL READ ----------------
            RD0: begin
                sram_addr <= rd_byte0;
                dq_oe     <= 1'b0;
                sram_ce_n <= 1'b0;
                sram_oe_n <= 1'b0;    // SRAM drives bus
                sram_we_n <= 1'b1;
                st        <= RD0b;
            end
            RD0b: begin
                rd_low <= dq_in;      // sample low byte
                st     <= RD1;
            end
            RD1: begin
                sram_addr <= rd_byte0 + 19'd1;
                sram_ce_n <= 1'b0;
                sram_oe_n <= 1'b0;
                sram_we_n <= 1'b1;
                st        <= RD1b;
            end
            RD1b: begin
                lb_we    <= 1'b1;
                lb_waddr <= {f_half, f_col};
                lb_wdata <= {dq_in[3:0], rd_low};  // reassemble pixel
                if (f_col == (H_ACT-1)) begin
                    fill_done   <= 1'b1;
                    fill_active <= 1'b0;
                    st          <= ARB;
                end else begin
                    f_col <= f_col + 1'b1;
                    st    <= ARB;
                end
            end

            // ---------------- LBPH READ (one pixel) ----------------
            LRD0: begin
                sram_addr <= lbph_byte0;
                dq_oe     <= 1'b0;
                sram_ce_n <= 1'b0;
                sram_oe_n <= 1'b0;
                sram_we_n <= 1'b1;
                st        <= LRD0b;
            end
            LRD0b: begin
                lbph_low <= dq_in;    // low byte
                st       <= LRD1;
            end
            LRD1: begin
                sram_addr <= lbph_byte0 + 19'd1;
                sram_ce_n <= 1'b0;
                sram_oe_n <= 1'b0;
                sram_we_n <= 1'b1;
                st        <= LRD1b;
            end
            LRD1b: begin
                lbph_pixel <= {dq_in[3:0], lbph_low};  // reassemble RGB444
                lbph_valid <= 1'b1;                    // strobe result
                st         <= ARB;
            end

            default: st <= ARB;
            endcase
        end
    end

endmodule