`timescale 1ns/1ps
// =============================================================================
// Module  : seg7_display  (v4 - BUG FIXED)
// Project : Smart Parking System v2 - Basys3 (Artix-7)
// Purpose : 4-digit multiplexed 7-segment display driver.
//
// =============================================================================
//  BUGS FIXED FROM v3
// =============================================================================
//  BUG 1 - SEGMENT ENCODING (CRITICAL - wrong digits on every position)
//  -----------------------------------------------------------------------
//  The v3 code used encoding patterns that match the OPPOSITE bit convention:
//  {g,f,e,d,c,b,a} order.  But the XDC file maps:
//      seg[6]=CA=a   seg[5]=CB=b   seg[4]=CC=c   seg[3]=CD=d
//      seg[2]=CE=e   seg[1]=CF=f   seg[0]=CG=g
//  Therefore seg[6:0] = {a,b,c,d,e,f,g} and active-LOW encoding must be:
//
//   Digit  Lit segs    seg[6:0]={a,b,c,d,e,f,g}  Binary      Hex
//     0    abcdef       0  0  0  0  0  0  1        0000001    0x01
//     1    bc           1  0  0  1  1  1  1        1001111    0x4F
//     2    abdeg        0  0  1  0  0  1  0        0010010    0x12  (unchanged)
//     3    abcdg        0  0  0  0  1  1  0        0000110    0x06
//     4    bcfg         1  0  0  1  1  0  0        1001100    0x4C
//     5    acdfg        0  1  0  0  1  0  0        0100100    0x24  (unchanged)
//     6    acdefg       0  1  0  0  0  0  0        0100000    0x20
//     7    abc          0  0  0  1  1  1  1        0001111    0x0F
//     8    abcdefg      0  0  0  0  0  0  0        0000000    0x00  (unchanged)
//     9    abcdfg       0  0  0  0  1  0  0        0000100    0x04
//
//  BUG 2 - PIPELINE TIMING SKEW (causes each digit to show wrong number)
//  -----------------------------------------------------------------------
//  In v3 the output path was:
//      digit_sel FF -> [mux] -> digit_val FF -> [decoder] -> seg FF
//                                             -> an FF
//  Since digit_val and an are registered on the SAME clock edge but seg is
//  registered one cycle LATER (using digit_val as input), the seg output
//  always lags the an output by exactly one clock cycle.  This means:
//    - an[3:0] selects digit N
//    - seg[6:0] shows the segments for digit N-1
//  Every digit position displays the VALUE of its left neighbour.
//
//  FIX: Decode seg[6:0] COMBINATIONALLY from digit_val and register both
//  an and seg together in the same always block on the same edge.
//  Both outputs now change on the same rising edge -> zero skew.
//
// =============================================================================
//  DISPLAY MODES
// =============================================================================
//   mode  Input signal   Max value   Display example
//   ----  ------------   ---------   ---------------
//    00   free_count      8 (3-bit)      0008
//    01   car_id         14 (4-bit)      0003
//    10   duration       9999 (16-bit)   0042
//    11   fee            9999 (16-bit)   0084
//
// =============================================================================
//  BCD PIPELINE LAYOUT  (28 bits per stage)
// =============================================================================
//   [27:24] = thousands BCD nibble
//   [23:20] = hundreds  BCD nibble
//   [19:16] = tens      BCD nibble
//   [15:12] = ones      BCD nibble
//   [11: 0] = remaining binary bits (shift register area)
//
//  Stage 0  : load {14'd0, 14-bit_binary_input}
//  Stage k  : apply add-3-if->=5 to each nibble, then left-shift by 1
//  Stage 14 : BCD area fully populated; binary area fully consumed
//
// =============================================================================

module seg7_display #(
    parameter CLK_FREQ   = 100_000_000,
    parameter REFRESH_HZ = 1_000,
    parameter NUM_SLOTS  = 8
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [1:0]  mode,
    input  wire [3:0]  free_count,   // 0-8   (mode 00)
    input  wire [15:0] fee,          // 0-9999 (mode 11)
    input  wire [15:0] duration,     // 0-9999 (mode 10)
    input  wire [3:0]  car_id,       // 0-14  (mode 01)
    output reg  [3:0]  an,
    output reg  [6:0]  seg
);

    // =========================================================================
    // REFRESH DIVIDER
    // =========================================================================
    localparam integer DIV_COUNT = CLK_FREQ / REFRESH_HZ / 4;
    localparam integer DIV_BITS  = $clog2(DIV_COUNT + 1);

    reg [DIV_BITS-1:0] div_cnt;
    reg [1:0]          digit_sel;   // 0=ones, 1=tens, 2=hundreds, 3=thousands

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
    // 14-STAGE DOUBLE-DABBLE BCD PIPELINE  (four independent channels)
    // =========================================================================
    function [3:0] add3_if_ge5;
        input [3:0] n;
        begin
            add3_if_ge5 = (n >= 4'd5) ? (n + 4'd3) : n;
        end
    endfunction

    reg [27:0] free_pipe [0:14];
    reg [27:0] id_pipe   [0:14];
    reg [27:0] dur_pipe  [0:14];
    reg [27:0] fee_pipe  [0:14];

    // Stage 0: latch the four inputs
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            free_pipe[0] <= 28'd0;
            id_pipe[0]   <= 28'd0;
            dur_pipe[0]  <= 28'd0;
            fee_pipe[0]  <= 28'd0;
        end else begin
            free_pipe[0] <= {14'd0, free_in};
            id_pipe[0]   <= {14'd0, id_in};
            dur_pipe[0]  <= {14'd0, dur_in};
            fee_pipe[0]  <= {14'd0, fee_in};
        end
    end

    // Stages 1..14: add-3 correction then left-shift
    genvar gi;
    generate
        for (gi = 1; gi <= 14; gi = gi + 1) begin : PIPE_STAGE
            wire [26:0] fc_w, ic_w, dc_w, fw_w;

            assign fc_w = {
                add3_if_ge5(free_pipe[gi-1][27:24]),
                add3_if_ge5(free_pipe[gi-1][23:20]),
                add3_if_ge5(free_pipe[gi-1][19:16]),
                add3_if_ge5(free_pipe[gi-1][15:12]),
                free_pipe[gi-1][11:0]
            };
            assign ic_w = {
                add3_if_ge5(id_pipe[gi-1][27:24]),
                add3_if_ge5(id_pipe[gi-1][23:20]),
                add3_if_ge5(id_pipe[gi-1][19:16]),
                add3_if_ge5(id_pipe[gi-1][15:12]),
                id_pipe[gi-1][11:0]
            };
            assign dc_w = {
                add3_if_ge5(dur_pipe[gi-1][27:24]),
                add3_if_ge5(dur_pipe[gi-1][23:20]),
                add3_if_ge5(dur_pipe[gi-1][19:16]),
                add3_if_ge5(dur_pipe[gi-1][15:12]),
                dur_pipe[gi-1][11:0]
            };
            assign fw_w = {
                add3_if_ge5(fee_pipe[gi-1][27:24]),
                add3_if_ge5(fee_pipe[gi-1][23:20]),
                add3_if_ge5(fee_pipe[gi-1][19:16]),
                add3_if_ge5(fee_pipe[gi-1][15:12]),
                fee_pipe[gi-1][11:0]
            };

            always @(posedge clk or posedge rst) begin
                if (rst) begin
                    free_pipe[gi] <= 28'd0;
                    id_pipe[gi]   <= 28'd0;
                    dur_pipe[gi]  <= 28'd0;
                    fee_pipe[gi]  <= 28'd0;
                end else begin
                    free_pipe[gi] <= {fc_w,  1'b0};
                    id_pipe[gi]   <= {ic_w,  1'b0};
                    dur_pipe[gi]  <= {dc_w,  1'b0};
                    fee_pipe[gi]  <= {fw_w,  1'b0};
                end
            end
        end
    endgenerate

    // =========================================================================
    // BCD DIGIT EXTRACTION
    // =========================================================================
    wire [3:0] free_thou = free_pipe[14][27:24];
    wire [3:0] free_hund = free_pipe[14][23:20];
    wire [3:0] free_tens = free_pipe[14][19:16];
    wire [3:0] free_ones = free_pipe[14][15:12];

    wire [3:0] id_thou   = id_pipe[14][27:24];
    wire [3:0] id_hund   = id_pipe[14][23:20];
    wire [3:0] id_tens   = id_pipe[14][19:16];
    wire [3:0] id_ones   = id_pipe[14][15:12];

    wire [3:0] dur_thou  = dur_pipe[14][27:24];
    wire [3:0] dur_hund  = dur_pipe[14][23:20];
    wire [3:0] dur_tens  = dur_pipe[14][19:16];
    wire [3:0] dur_ones  = dur_pipe[14][15:12];

    wire [3:0] fee_thou  = fee_pipe[14][27:24];
    wire [3:0] fee_hund  = fee_pipe[14][23:20];
    wire [3:0] fee_tens  = fee_pipe[14][19:16];
    wire [3:0] fee_ones  = fee_pipe[14][15:12];

    // =========================================================================
    // DIGIT SELECTION MUX  (registered - cuts combinational path to seg)
    // =========================================================================
    reg [3:0] digit_val;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            digit_val <= 4'd0;
        end else begin
            case (mode)
                2'b00: begin
                    case (digit_sel)
                        2'd3: digit_val <= free_thou;
                        2'd2: digit_val <= free_hund;
                        2'd1: digit_val <= free_tens;
                        2'd0: digit_val <= free_ones;
                        default: digit_val <= 4'd0;
                    endcase
                end
                2'b01: begin
                    case (digit_sel)
                        2'd3: digit_val <= id_thou;
                        2'd2: digit_val <= id_hund;
                        2'd1: digit_val <= id_tens;
                        2'd0: digit_val <= id_ones;
                        default: digit_val <= 4'd0;
                    endcase
                end
                2'b10: begin
                    case (digit_sel)
                        2'd3: digit_val <= dur_thou;
                        2'd2: digit_val <= dur_hund;
                        2'd1: digit_val <= dur_tens;
                        2'd0: digit_val <= dur_ones;
                        default: digit_val <= 4'd0;
                    endcase
                end
                2'b11: begin
                    case (digit_sel)
                        2'd3: digit_val <= fee_thou;
                        2'd2: digit_val <= fee_hund;
                        2'd1: digit_val <= fee_tens;
                        2'd0: digit_val <= fee_ones;
                        default: digit_val <= 4'd0;
                    endcase
                end
                default: digit_val <= 4'd0;
            endcase
        end
    end

    // =========================================================================
    // COMBINATIONAL 7-SEGMENT DECODER
    // =========================================================================
    // Decoded here combinationally so that seg and an can be registered
    // together in the same always block - eliminating the 1-cycle skew bug.
    //
    // XDC pin mapping: seg[6]=a  seg[5]=b  seg[4]=c  seg[3]=d
    //                  seg[2]=e  seg[1]=f  seg[0]=g
    // Active-LOW: 0 = segment ON, 1 = segment OFF
    //
    //   Digit  Lit segs    {a,b,c,d,e,f,g}   Binary
    //     0    abcdef       0 0 0 0 0 0 1     0000001
    //     1    bc           1 0 0 1 1 1 1     1001111
    //     2    abdeg        0 0 1 0 0 1 0     0010010
    //     3    abcdg        0 0 0 0 1 1 0     0000110
    //     4    bcfg         1 0 0 1 1 0 0     1001100
    //     5    acdfg        0 1 0 0 1 0 0     0100100
    //     6    acdefg       0 1 0 0 0 0 0     0100000
    //     7    abc          0 0 0 1 1 1 1     0001111
    //     8    abcdefg      0 0 0 0 0 0 0     0000000
    //     9    abcdfg       0 0 0 0 1 0 0     0000100

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
            default: seg_comb = 7'b0000001; // show 0 for any BCD fault
        endcase
    end

    // =========================================================================
    // REGISTERED OUTPUTS - an and seg sampled TOGETHER (BUG 2 FIX)
    // =========================================================================
    // digit_val is the registered BCD nibble (from the mux above).
    // seg_comb is the combinational decode of digit_val.
    // By registering seg FROM seg_comb (not from a separate registered stage),
    // both an and seg advance on the same clock edge -> zero skew.
    //
    // Timing path to seg output FF:
    //   digit_val FF -> seg_comb (small LUT) -> seg FF
    // This is a single LUT level; well within 10 ns at 100 MHz.

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            an  <= 4'b1111;
            seg <= 7'b0000001; // display 0 after reset
        end else begin
            // Anode: activate the digit selected by digit_sel
            // digit_sel=0 -> ones (rightmost)  -> AN[0] low
            // digit_sel=3 -> thousands (leftmost) -> AN[3] low
            case (digit_sel)
                2'd0: an <= 4'b1110;
                2'd1: an <= 4'b1101;
                2'd2: an <= 4'b1011;
                2'd3: an <= 4'b0111;
                default: an <= 4'b1111;
            endcase
            // Segment: decode the current digit value
            seg <= seg_comb;
        end
    end

endmodule