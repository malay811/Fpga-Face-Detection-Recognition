//============================================================================
// linebuffer.v
//----------------------------------------------------------------------------
// Dual-clock, dual-port line buffer between the SRAM controller (50 MHz write)
// and the VGA display (25.2 MHz read). Holds a ping-pong pair of rows so the
// display can 2x-upscale 320x240 -> 640x480: while VGA reads one half, the
// SRAM controller fills the other.
//
// Storage: 1024 x 12-bit = two 512-entry halves.
//   waddr / raddr = {half_select, column}.
//   Write side (clk_w = 50 MHz) driven by sram_ctrl.
//   Read side  (clk_r = 25.2 MHz) driven by display_reader / VGA.
//
// This is the standard Xilinx-recommended CDC-through-memory structure:
// a true dual-port BRAM with independent clocks. Vivado infers a BRAM tile
// from this pattern via the ram_style="block" attribute. The 1-cycle
// registered read output matches BRAM read latency; downstream logic
// (display enable) is delayed 1 cycle to align.
//============================================================================
`timescale 1ns / 1ps

module linebuffer (
    // ---- Write port: 50 MHz, from sram_ctrl ----
    input  wire        clk_w,
    input  wire        we,          // write enable (lb_we)
    input  wire [9:0]  waddr,       // {half, col[8:0]}
    input  wire [11:0] wdata,       // RGB444 pixel from SRAM read

    // ---- Read port: 25.2 MHz, to display ----
    input  wire        clk_r,
    input  wire [9:0]  raddr,       // {disp_half, col}
    output reg  [11:0] rdata        // registered pixel out (1-cycle latency)
);

    // 1024 x 12-bit dual-port memory. ram_style="block" tells Vivado to map
    // this to a Block RAM primitive rather than distributed LUTRAM.
    (* ram_style = "block" *) reg [11:0] mem [0:1023];

    // Write port (clk_w domain): synchronous write when enabled.
    always @(posedge clk_w) begin
        if (we) mem[waddr] <= wdata;
    end

    // Read port (clk_r domain): unconditional registered read every cycle.
    // Display needs a continuous pixel stream, so no read-enable gating.
    // Address at cycle N -> data at cycle N+1 (BRAM read latency).
    always @(posedge clk_r) begin
        rdata <= mem[raddr];
    end

endmodule