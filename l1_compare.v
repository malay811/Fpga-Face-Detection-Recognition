//============================================================================
// l1_compare.v  (two-identity version)
//----------------------------------------------------------------------------
// Streams the live 3776-bin LBPH histogram once, computes L1 distance to
// BOTH enrolled identities (Malay, Nilesh) in parallel using two independent
// accumulators fed by the same live_data stream -- no need to scan twice.
//
//   d_malay  = sum |live[i] - malay_hist[i]|
//   d_nilesh = sum |live[i] - nilesh_hist[i]|
//   best_id  = (d_malay <= d_nilesh) ? MALAY(0) : NILESH(1)
//   best_dist= min(d_malay, d_nilesh)
//   match    = (best_dist < THRESHOLD)     // else: neither -> no name shown
//
// THRESHOLD = 3793 (from LBPH_TwoIdentity_Threshold.py sweep: 93.7% known
// accept, 90.2% stranger reject -- chosen to favour not flickering "unknown"
// on Malay/Nilesh's own faces over a slightly higher stranger false-accept).
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module l1_compare #(
    parameter integer NBINS      = 3776,
    parameter integer THRESHOLD  = 3793,
    parameter         MALAY_FILE  = "lbph_hist_Malay.mem",
    parameter         NILESH_FILE = "lbph_hist_Nilesh.mem"
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    output reg  [11:0] live_addr,
    input  wire [7:0]  live_data,
    output reg  [15:0] dist,       // best_dist (min of the two)
    output reg         match,      // 1 = recognised as SOMEONE (best_dist < THRESHOLD)
    output reg         match_id,   // 0 = MALAY, 1 = NILESH (valid only if match=1)
    output reg         done
);

    // two enrolled ROMs, same address space, read in parallel
    (* rom_style = "block" *) reg [7:0] enr_malay  [0:NBINS-1];
    (* rom_style = "block" *) reg [7:0] enr_nilesh [0:NBINS-1];
    initial begin
        $readmemh(MALAY_FILE,  enr_malay);
        $readmemh(NILESH_FILE, enr_nilesh);
    end

    // FSM
    localparam [1:0] S_IDLE=0, S_RUN=1, S_FIN=2;
    reg [1:0] st;

    reg [12:0] drive_i;    // 0..3776 (drives live_addr and enr addr)
    reg [12:0] samp_i;     // trails drive_i by 1
    reg [7:0]  enr_m_r, enr_n_r;   // registered enr_malay[i-1], enr_nilesh[i-1]
    reg [15:0] acc_m, acc_n;

    wire [7:0] adiff_m = (live_data > enr_m_r) ? (live_data - enr_m_r)
                                               : (enr_m_r - live_data);
    wire [7:0] adiff_n = (live_data > enr_n_r) ? (live_data - enr_n_r)
                                               : (enr_n_r - live_data);

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; done <= 0; match <= 0; match_id <= 0; dist <= 0;
            drive_i <= 0; samp_i <= 0; acc_m <= 0; acc_n <= 0;
            enr_m_r <= 0; enr_n_r <= 0; live_addr <= 0;
        end else begin
            done <= 1'b0;

            case (st)
            S_IDLE: begin
                if (start) begin
                    drive_i <= 0; samp_i <= 0; acc_m <= 0; acc_n <= 0;
                    live_addr <= 0;
                    st <= S_RUN;
                end
            end

            // Drive addr i, register both enr[i]. Next cycle live_data and
            // enr_m_r/enr_n_r both valid for bin i; accumulate both sums.
            S_RUN: begin
                if (drive_i < NBINS) begin
                    live_addr <= drive_i[11:0];
                    enr_m_r   <= enr_malay [drive_i[11:0]];
                    enr_n_r   <= enr_nilesh[drive_i[11:0]];
                    drive_i   <= drive_i + 1'b1;
                end

                // Accumulate previously-sampled bin, both identities
                if (samp_i > 0 && samp_i <= NBINS) begin
                    acc_m <= acc_m + {8'd0, adiff_m};
                    acc_n <= acc_n + {8'd0, adiff_n};
                end

                samp_i <= samp_i + 1'b1;

                if (samp_i == NBINS)
                    st <= S_FIN;
            end

            S_FIN: begin
                if (acc_m <= acc_n) begin
                    dist     <= acc_m;
                    match_id <= 1'b0;              // MALAY
                    match    <= (acc_m < THRESHOLD[15:0]);
                end else begin
                    dist     <= acc_n;
                    match_id <= 1'b1;              // NILESH
                    match    <= (acc_n < THRESHOLD[15:0]);
                end
                done <= 1'b1;
                st   <= S_IDLE;
            end
            endcase
        end
    end

endmodule