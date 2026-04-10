`timescale 1ns/1ps
// =============================================================================
// Module  : debounce
// Project : Smart Parking System v2 - Basys3 (Artix-7)
// Purpose : Button debouncer + synchroniser + single-cycle rising-edge pulse.
//
// Design notes:
//   ? Two flip-flop synchroniser prevents metastability on async button input.
//   ? 20-bit saturating counter provides ~10 ms debounce window at 100 MHz.
//     Counter resets whenever the synced input level matches the stable output,
//     and the stable output flips only when the counter saturates.
//   ? btn_pulse is a guaranteed single-cycle HIGH on each stable rising edge.
//
// Parameters:
//   DEBOUNCE_BITS - width of the debounce counter (20 ? ~10 ms @ 100 MHz)
// =============================================================================

module debounce #(
    parameter DEBOUNCE_BITS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire btn_in,       // Raw button (active-high)
    output reg  btn_stable,   // Debounced level
    output wire btn_pulse     // Single-cycle rising-edge pulse
);

    // ?? Two-stage synchroniser ???????????????????????????????????????????
    reg sync0, sync1;
    always @(posedge clk or posedge rst) begin
        if (rst) { sync1, sync0 } <= 2'b00;
        else     { sync1, sync0 } <= { sync0, btn_in };
    end

    // ?? Debounce counter ?????????????????????????????????????????????????
    reg [DEBOUNCE_BITS-1:0] cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt        <= {DEBOUNCE_BITS{1'b0}};
            btn_stable <= 1'b0;
        end else if (sync1 == btn_stable) begin
            // Input matches output - keep counter at zero
            cnt <= {DEBOUNCE_BITS{1'b0}};
        end else begin
            cnt <= cnt + 1'b1;
            if (cnt == {DEBOUNCE_BITS{1'b1}}) begin
                // Counter saturated - accept the new level
                btn_stable <= sync1;
                cnt        <= {DEBOUNCE_BITS{1'b0}};
            end
        end
    end

    // ?? Rising-edge detector - one clock wide ????????????????????????????
    reg btn_prev;
    always @(posedge clk or posedge rst) begin
        if (rst) btn_prev <= 1'b0;
        else     btn_prev <= btn_stable;
    end

    assign btn_pulse = btn_stable & ~btn_prev;

endmodule