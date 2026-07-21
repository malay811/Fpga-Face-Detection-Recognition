//============================================================================
// line_buffer_3x3.v
//----------------------------------------------------------------------------
// Builds a sliding 3x3 pixel window from a raster-scan grayscale stream, for
// neighbourhood operations (HOG gradient, LBP). A KxK window on width-W images
// needs K-1 line buffers of depth W; here K=3 => 2 line buffers of depth W.
//
// Data movement each valid pixel:
//   new pixel -> row0 (current). Reading the same column from the two line
//   buffers gives the pixel one row up (row1) and two rows up (row2). A 3-deep
//   shift register on each of the three rows forms the 3 columns:
//
//        p22 p21 p20   <- row two-up  (oldest row)
//        p12 p11 p10   <- row one-up
//        p02 p01 p00   <- current row (newest)
//
//   p11 is the window centre. p*0 = newest column (current pixel col).
//
// window_valid asserts only once >=2 full rows have been written AND >=3 pixels
// of the current row are in the column shift regs, i.e. a full 3x3 exists.
// Border pixels (first/last col, first/last row) are excluded by valid, matching
// the Python reference which skips the 1-pixel border.
//============================================================================
`timescale 1ns / 1ps

module line_buffer_3x3 #(
    parameter integer WIDTH  = 320,
    parameter integer HEIGHT = 240
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] pix_in,
    input  wire       pix_valid,
    // 3x3 window outputs (row-major: p[row][col], row0=current)
    output reg  [7:0] p00, output reg [7:0] p01, output reg [7:0] p02,
    output reg  [7:0] p10, output reg [7:0] p11, output reg [7:0] p12,
    output reg  [7:0] p20, output reg [7:0] p21, output reg [7:0] p22,
    output reg        window_valid,
    output reg [9:0]  win_col,   // column of the window CENTRE (0..WIDTH-1)
    output reg [9:0]  win_row    // row    of the window CENTRE (0..HEIGHT-1)
);

    // ---- two line buffers (inferred BRAM) ----
    (* ram_style = "block" *) reg [7:0] lb0 [0:WIDTH-1];  // one row up
    (* ram_style = "block" *) reg [7:0] lb1 [0:WIDTH-1];  // two rows up

    reg [9:0] col;    // current write column (0..WIDTH-1)
    reg [9:0] row;    // current write row
    reg [8:0] rows_done; // how many full rows written (saturates)

    // read-back values from the line buffers at the current column
    reg [7:0] lb0_q, lb1_q;

    // column shift registers for the three rows (newest in [0])
    // current row
    reg [7:0] c0_0, c0_1, c0_2;
    // one row up
    reg [7:0] r1_0, r1_1, r1_2;
    // two rows up
    reg [7:0] r2_0, r2_1, r2_2;

    always @(posedge clk) begin
        if (rst) begin
            col          <= 10'd0;
            row          <= 10'd0;
            rows_done    <= 9'd0;
            window_valid <= 1'b0;
            {c0_0,c0_1,c0_2} <= 24'd0;
            {r1_0,r1_1,r1_2} <= 24'd0;
            {r2_0,r2_1,r2_2} <= 24'd0;
        end else begin
            window_valid <= 1'b0;

            if (pix_valid) begin
                // Read the two upper rows at this column BEFORE overwriting lb0.
                lb0_q = lb0[col];   // pixel one row up, same column
                lb1_q = lb1[col];   // pixel two rows up, same column

                // Shift the two upper rows down the memory chain:
                //   lb1 gets what lb0 held (row one-up becomes two-up next frame
                //   line), lb0 gets the incoming pixel.
                lb1[col] <= lb0_q;
                lb0[col] <= pix_in;

                // Advance the 3 column shift registers (newest at *_0).
                c0_2 <= c0_1; c0_1 <= c0_0; c0_0 <= pix_in;
                r1_2 <= r1_1; r1_1 <= r1_0; r1_0 <= lb0_q;
                r2_2 <= r2_1; r2_1 <= r2_0; r2_0 <= lb1_q;

                // Drive window outputs (centre = column one-back, row one-up).
                // Newest column is *_0; centre column is *_1; oldest is *_2.
                p00 <= pix_in; p01 <= c0_0; p02 <= c0_1;
                p10 <= lb0_q;  p11 <= r1_0; p12 <= r1_1;
                p20 <= lb1_q;  p21 <= r2_0; p22 <= r2_1;

                // A full 3x3 exists once >=2 rows already written and we are at
                // column >=2 (three columns loaded). Centre is at (col-1,row-1).
                if ((rows_done >= 9'd2) && (col >= 10'd2)) begin
                    window_valid <= 1'b1;
                    win_col      <= col - 10'd1;
                    win_row      <= row - 10'd1;
                end

                // advance column / row counters
                if (col == WIDTH-1) begin
                    col <= 10'd0;
                    if (row != HEIGHT-1) row <= row + 1'b1;
                    if (rows_done < 9'd300) rows_done <= rows_done + 1'b1;
                end else begin
                    col <= col + 1'b1;
                end
            end
        end
    end

endmodule