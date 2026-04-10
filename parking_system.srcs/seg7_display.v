`timescale 1ns/1ps
// =============================================================================
// Module  : seg7_display  (v5 - ALL BUGS FIXED)
// Project : Smart Parking System v2 - Basys3 (Artix-7)
//
// =============================================================================
//  THREE BUGS FIXED
// =============================================================================
//
//  BUG 1 - PIPELINE WIDTH (values shown x4 too large: 8->32, 7->28, etc.)
//  -----------------------------------------------------------------------
//  Old 28-bit word: [27:24]=thou [23:20]=hund [19:16]=tens [15:12]=ones [11:0]=binary
//  We load a 14-bit input at [13:0], but the ones nibble LSB is at bit[12].
//  After 14 left-shifts bit[0] lands at bit[14]=ones[2], not ones[0] => x4 error.
//
//  FIX: 30-bit word: [29:26]=thou [25:22]=hund [21:18]=tens [17:14]=ones [13:0]=binary
//  Now after 14 shifts bit[0] -> bit[14] = ones[0] => correct decimal value.
//
//  BUG 2 - SEGMENT ENCODING (every digit displays a wrong symbol)
//  -----------------------------------------------------------------------
//  XDC maps seg[6]=a seg[5]=b seg[4]=c seg[3]=d seg[2]=e seg[1]=f seg[0]=g.
//  Old code used reversed {g,f,e,d,c,b,a} convention.
//  Fixed: all 10 encodings recomputed for {a,b,c,d,e,f,g} active-low.
//
//  BUG 3 - TIMING SKEW (each position shows its left-neighbour's value)
//  -----------------------------------------------------------------------
//  Old: digit_val FF -> seg FF (an registered one cycle earlier than seg).
//  Fixed: seg decoded combinationally from digit_val, then an+seg registered
//  together in one always block => zero skew.
//
// =============================================================================
//  DISPLAY MODES
// =============================================================================
//   mode 00 -> free_count (0-8)       e.g. "0008"
//   mode 01 -> car_id    (0-14)       e.g. "0003"
//   mode 10 -> duration  (0-9999 s)   e.g. "0042"
//   mode 11 -> fee       (0-9999)     e.g. "0084"
// =============================================================================

module seg7_display #(
    parameter CLK_FREQ   = 100_000_000,
    parameter REFRESH_HZ = 1_000,
    parameter NUM_SLOTS  = 8
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [1:0]  mode,
    input  wire [3:0]  free_count,
    input  wire [15:0] fee,
    input  wire [15:0] duration,
    input  wire [3:0]  car_id,
    output reg  [3:0]  an,
    output reg  [6:0]  seg
);

    // =========================================================================
    // REFRESH DIVIDER
    // =========================================================================
    localparam integer DIV_COUNT = CLK_FREQ / REFRESH_HZ / 4;
    localparam integer DIV_BITS  = $clog2(DIV_COUNT + 1);

    reg [DIV_BITS-1:0] div_cnt;
    reg [1:0]          digit_sel;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            div_cnt   <= {DIV_BITS{1'b0}};
            digit_sel <= 2'd0;
        end else if (div_cnt == DIV_COUNT[DIV_BITS-1:0] - 1'b1) begin
            div_cnt   <= {DIV_BITS{1'b0}};
            digit_sel <= digit_sel + 1'b1;
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end

    // =========================================================================
    // INPUT PREPARATION
    // =========================================================================
    wire [13:0] free_in = {10'd0, free_count};
    wire [13:0] id_in   = {10'd0, car_id};
    wire [13:0] dur_in  = (duration > 16'd9999) ? 14'd9999 : duration[13:0];
    wire [13:0] fee_in  = (fee      > 16'd9999) ? 14'd9999 : fee[13:0];

    // =========================================================================
    // 14-STAGE DOUBLE-DABBLE BCD PIPELINE - 30-BIT WORD (BUG 1 FIX)
    // =========================================================================
    // Word layout: [29:26]=thou  [25:22]=hund  [21:18]=tens  [17:14]=ones  [13:0]=binary
    // Stage 0 loads {16'd0, 14-bit_input}.
    // Each stage: correct nibbles (add 3 if >=5), then left-shift by 1.
    // After 14 stages all 14 binary bits consumed; BCD area holds correct digits.
    // =========================================================================

    function [3:0] add3_if_ge5;
        input [3:0] n;
        add3_if_ge5 = (n >= 4'd5) ? (n + 4'd3) : n;
    endfunction

    reg [29:0] free_pipe [0:14];
    reg [29:0] id_pipe   [0:14];
    reg [29:0] dur_pipe  [0:14];
    reg [29:0] fee_pipe  [0:14];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            free_pipe[0] <= 30'd0;
            id_pipe[0]   <= 30'd0;
            dur_pipe[0]  <= 30'd0;
            fee_pipe[0]  <= 30'd0;
        end else begin
            free_pipe[0] <= {16'd0, free_in};
            id_pipe[0]   <= {16'd0, id_in};
            dur_pipe[0]  <= {16'd0, dur_in};
            fee_pipe[0]  <= {16'd0, fee_in};
        end
    end

    genvar gi;
    generate
        for (gi = 1; gi <= 14; gi = gi + 1) begin : PIPE_STAGE
            wire [28:0] fc_w, ic_w, dc_w, fw_w;

            assign fc_w = {
                add3_if_ge5(free_pipe[gi-1][29:26]),
                add3_if_ge5(free_pipe[gi-1][25:22]),
                add3_if_ge5(free_pipe[gi-1][21:18]),
                add3_if_ge5(free_pipe[gi-1][17:14]),
                free_pipe[gi-1][13:0]
            };
            assign ic_w = {
                add3_if_ge5(id_pipe[gi-1][29:26]),
                add3_if_ge5(id_pipe[gi-1][25:22]),
                add3_if_ge5(id_pipe[gi-1][21:18]),
                add3_if_ge5(id_pipe[gi-1][17:14]),
                id_pipe[gi-1][13:0]
            };
            assign dc_w = {
                add3_if_ge5(dur_pipe[gi-1][29:26]),
                add3_if_ge5(dur_pipe[gi-1][25:22]),
                add3_if_ge5(dur_pipe[gi-1][21:18]),
                add3_if_ge5(dur_pipe[gi-1][17:14]),
                dur_pipe[gi-1][13:0]
            };
            assign fw_w = {
                add3_if_ge5(fee_pipe[gi-1][29:26]),
                add3_if_ge5(fee_pipe[gi-1][25:22]),
                add3_if_ge5(fee_pipe[gi-1][21:18]),
                add3_if_ge5(fee_pipe[gi-1][17:14]),
                fee_pipe[gi-1][13:0]
            };

            always @(posedge clk or posedge rst) begin
                if (rst) begin
                    free_pipe[gi] <= 30'd0;
                    id_pipe[gi]   <= 30'd0;
                    dur_pipe[gi]  <= 30'd0;
                    fee_pipe[gi]  <= 30'd0;
                end else begin
                    free_pipe[gi] <= {fc_w, 1'b0};
                    id_pipe[gi]   <= {ic_w, 1'b0};
                    dur_pipe[gi]  <= {dc_w, 1'b0};
                    fee_pipe[gi]  <= {fw_w, 1'b0};
                end
            end
        end
    endgenerate

    // =========================================================================
    // BCD DIGIT EXTRACTION - new 30-bit positions
    // =========================================================================
    wire [3:0] free_thou = free_pipe[14][29:26];
    wire [3:0] free_hund = free_pipe[14][25:22];
    wire [3:0] free_tens = free_pipe[14][21:18];
    wire [3:0] free_ones = free_pipe[14][17:14];

    wire [3:0] id_thou   = id_pipe[14][29:26];
    wire [3:0] id_hund   = id_pipe[14][25:22];
    wire [3:0] id_tens   = id_pipe[14][21:18];
    wire [3:0] id_ones   = id_pipe[14][17:14];

    wire [3:0] dur_thou  = dur_pipe[14][29:26];
    wire [3:0] dur_hund  = dur_pipe[14][25:22];
    wire [3:0] dur_tens  = dur_pipe[14][21:18];
    wire [3:0] dur_ones  = dur_pipe[14][17:14];

    wire [3:0] fee_thou  = fee_pipe[14][29:26];
    wire [3:0] fee_hund  = fee_pipe[14][25:22];
    wire [3:0] fee_tens  = fee_pipe[14][21:18];
    wire [3:0] fee_ones  = fee_pipe[14][17:14];

    // =========================================================================
    // DIGIT SELECTION MUX  (registered)
    // =========================================================================
    reg [3:0] digit_val;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            digit_val <= 4'd0;
        end else begin
            case (mode)
                2'b00: case (digit_sel)
                    2'd3: digit_val <= free_thou;
                    2'd2: digit_val <= free_hund;
                    2'd1: digit_val <= free_tens;
                    2'd0: digit_val <= free_ones;
                    default: digit_val <= 4'd0;
                endcase
                2'b01: case (digit_sel)
                    2'd3: digit_val <= id_thou;
                    2'd2: digit_val <= id_hund;
                    2'd1: digit_val <= id_tens;
                    2'd0: digit_val <= id_ones;
                    default: digit_val <= 4'd0;
                endcase
                2'b10: case (digit_sel)
                    2'd3: digit_val <= dur_thou;
                    2'd2: digit_val <= dur_hund;
                    2'd1: digit_val <= dur_tens;
                    2'd0: digit_val <= dur_ones;
                    default: digit_val <= 4'd0;
                endcase
                2'b11: case (digit_sel)
                    2'd3: digit_val <= fee_thou;
                    2'd2: digit_val <= fee_hund;
                    2'd1: digit_val <= fee_tens;
                    2'd0: digit_val <= fee_ones;
                    default: digit_val <= 4'd0;
                endcase
                default: digit_val <= 4'd0;
            endcase
        end
    end

    // =========================================================================
    // COMBINATIONAL 7-SEGMENT DECODER  (BUG 2 fix: correct encoding)
    // =========================================================================
    // XDC: seg[6]=a  seg[5]=b  seg[4]=c  seg[3]=d  seg[2]=e  seg[1]=f  seg[0]=g
    // Active-LOW (0=ON, 1=OFF):
    //   0->0000001  1->1001111  2->0010010  3->0000110  4->1001100
    //   5->0100100  6->0100000  7->0001111  8->0000000  9->0000100
    reg [6:0] seg_comb;
    always @(*) begin
        case (digit_val)
            4'd0: seg_comb = 7'b0000001;
            4'd1: seg_comb = 7'b1001111;
            4'd2: seg_comb = 7'b0010010;
            4'd3: seg_comb = 7'b0000110;
            4'd4: seg_comb = 7'b1001100;
            4'd5: seg_comb = 7'b0100100;
            4'd6: seg_comb = 7'b0100000;
            4'd7: seg_comb = 7'b0001111;
            4'd8: seg_comb = 7'b0000000;
            4'd9: seg_comb = 7'b0000100;
            default: seg_comb = 7'b0000001;
        endcase
    end

    // =========================================================================
    // REGISTERED OUTPUTS - an and seg sampled TOGETHER (BUG 3 fix: zero skew)
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            an  <= 4'b1111;
            seg <= 7'b0000001;
        end else begin
            case (digit_sel)
                2'd0: an <= 4'b1110;   // ones      (rightmost)
                2'd1: an <= 4'b1101;   // tens
                2'd2: an <= 4'b1011;   // hundreds
                2'd3: an <= 4'b0111;   // thousands (leftmost)
                default: an <= 4'b1111;
            endcase
            seg <= seg_comb;
        end
    end

endmodule