//============================================================================
// cdc_coords.v  (two-identity version)
//----------------------------------------------------------------------------
// Carries the detection result bundle {box_x, box_y, box_valid, match,
// match_id} from the 50 MHz detection domain to the 25.2 MHz pixel domain
// for osd_overlay. match_id added: 0=MALAY, 1=NILESH (valid only if match=1).
//
// Same closed-loop req/ack MCP handshake as before; bus width grew by 1 bit
// (19 -> 20) to carry match_id -- no change to the handshake logic itself.
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module cdc_coords (
    // ---- detection domain (50 MHz) ----
    input  wire        src_clk,
    input  wire        src_rst,
    input  wire        load,        // pulse: new result bundle to send
    input  wire [8:0]  box_x_i,
    input  wire [7:0]  box_y_i,
    input  wire        valid_i,
    input  wire        match_i,
    input  wire        match_id_i,  // 0=MALAY, 1=NILESH
    output wire        src_ready,   // high when a new load can be accepted
    // ---- pixel domain (25.2 MHz) ----
    input  wire        dst_clk,
    input  wire        dst_rst,
    output reg  [8:0]  box_x_o,
    output reg  [7:0]  box_y_o,
    output reg         valid_o,
    output reg         match_o,
    output reg         match_id_o,
    output reg         dst_update   // pulse: bundle refreshed
);

    localparam integer W = 20;   // 9 + 8 + 1 + 1 + 1

    reg ack;

    // ---- source domain ----
    reg [W-1:0] src_data;
    reg         req;
    (* ASYNC_REG = "TRUE" *) reg ack_s1;
    (* ASYNC_REG = "TRUE" *) reg ack_s2;

    assign src_ready = (req == ack_s2);

    always @(posedge src_clk) begin
        if (src_rst) begin
            src_data <= {W{1'b0}}; req <= 1'b0;
            ack_s1 <= 1'b0; ack_s2 <= 1'b0;
        end else begin
            ack_s1 <= ack;
            ack_s2 <= ack_s1;
            if (load && src_ready) begin
                src_data <= {box_x_i, box_y_i, valid_i, match_i, match_id_i};
                req      <= ~req;
            end
        end
    end

    // ---- destination domain ----
    (* ASYNC_REG = "TRUE" *) reg req_s1;
    (* ASYNC_REG = "TRUE" *) reg req_s2;
    reg req_s3;

    always @(posedge dst_clk) begin
        if (dst_rst) begin
            req_s1 <= 1'b0; req_s2 <= 1'b0; req_s3 <= 1'b0; ack <= 1'b0;
            box_x_o <= 9'd0; box_y_o <= 8'd0; valid_o <= 1'b0;
            match_o <= 1'b0; match_id_o <= 1'b0;
            dst_update <= 1'b0;
        end else begin
            req_s1 <= req;
            req_s2 <= req_s1;
            req_s3 <= req_s2;
            dst_update <= 1'b0;
            if (req_s2 != req_s3) begin
                {box_x_o, box_y_o, valid_o, match_o, match_id_o} <= src_data;
                ack        <= ~ack;
                dst_update <= 1'b1;
            end
        end
    end

endmodule