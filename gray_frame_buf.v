//============================================================================
// gray_frame_buf.v
//----------------------------------------------------------------------------
// On-chip grayscale frame store for detection + recognition. The completed
// SRAM frame is grayscaled once (frame_loader, inside scan_controller) and
// written here; the window scanner and the LBPH crop then read from here at
// one pixel/cycle with zero SRAM contention (SRAM stays busy with camera +
// display).
//
// 320x240 = 76,800 bytes -> inferred BRAM (~17 BRAM36). Simple dual-port:
//   write port  (port A): loader only, during the load phase
//   read  port  (port B): scanner OR lbph crop (different phases), 1-cycle
//                         registered read (BRAM). Consumers account for the
//                         1-cycle latency.
//============================================================================
`timescale 1ns / 1ps

module gray_frame_buf #(
    parameter integer NPIX = 76800,   // 320*240
    parameter integer AW   = 17       // ceil(log2(76800)) = 17
)(
    input  wire            clk,
    // write port (frame_loader)
    input  wire            we,
    input  wire [AW-1:0]   waddr,
    input  wire [7:0]      wdata,
    // read port (scanner / lbph crop)
    input  wire [AW-1:0]   raddr,
    output reg  [7:0]      rdata      // registered (1-cycle latency)
);

    (* ram_style = "block" *) reg [7:0] mem [0:NPIX-1];

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end

endmodule