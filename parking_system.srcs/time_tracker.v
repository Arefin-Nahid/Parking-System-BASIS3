`timescale 1ns/1ps
// =============================================================================
// Module  : time_tracker
// Project : Smart Parking System v2 - Basys3 (Artix-7)
// Purpose : Global wall-clock counter + per-car entry-time memory.
//
// Key features:
//   ? Free-running 32-bit tick counter, divided from CLK_FREQ ? 1 Hz ticks.
//   ? Separate 32-bit entry_mem[ID] per car - all cars tracked in parallel
//     automatically because duration = current_time - entry_time[ID].
//   ? record_en / record_id : latch current_time into entry_mem[record_id].
//   ? query_id              : read-port - entry_time_out = entry_mem[query_id].
//     Separate query and record IDs allow the FSM to record one car's entry
//     while simultaneously asking the ALU to calculate another car's duration.
//   ? overflow_flag         : one-cycle pulse when current_time wraps (32-bit
//     limit = ~136 years at 1 Hz, but we flag it for safety).
//
// Parameters:
//   CLK_FREQ  - system oscillator frequency   (Hz)
//   TICK_DIV  - clock cycles per time unit    (set to CLK_FREQ for 1 Hz)
//   NUM_CARS  - number of car IDs supported
//   ID_BITS   - log2(NUM_CARS)
// =============================================================================

module time_tracker #(
    parameter CLK_FREQ = 100_000_000,
    parameter TICK_DIV = 100_000_000,   // 1 second per time unit
    parameter NUM_CARS = 16,
    parameter ID_BITS  = 4
)(
    input  wire                clk,
    input  wire                rst,
    // Write port - store entry time for a car
    input  wire                record_en,    // Pulse: latch current_time ? entry_mem[record_id]
    input  wire [ID_BITS-1:0]  record_id,    // Car whose entry time is being recorded
    // Read port - retrieve entry time for duration calculation
    input  wire [ID_BITS-1:0]  query_id,     // Car whose entry time we want
    // Outputs
    output reg  [31:0]         current_time, // Global seconds counter
    output wire [31:0]         entry_time_out,// entry_mem[query_id]  (combinational)
    output reg                 overflow_flag  // One-cycle pulse on 32-bit wrap
);

    // ?? Clock divider - 1 Hz tick ????????????????????????????????????????
    // Use exactly TICK_DIV counts.  $clog2 gives minimum bit width.
    localparam DIV_BITS = (TICK_DIV > 1) ? $clog2(TICK_DIV) : 1;
    reg [DIV_BITS-1:0] div_cnt;
    reg                tick;   // one-cycle HIGH every TICK_DIV clocks

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            div_cnt <= {DIV_BITS{1'b0}};
            tick    <= 1'b0;
        end else begin
            if (div_cnt == (TICK_DIV[DIV_BITS-1:0] - 1'b1)) begin
                div_cnt <= {DIV_BITS{1'b0}};
                tick    <= 1'b1;
            end else begin
                div_cnt <= div_cnt + 1'b1;
                tick    <= 1'b0;
            end
        end
    end

    // ?? 32-bit global second counter ?????????????????????????????????????
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            current_time  <= 32'd0;
            overflow_flag <= 1'b0;
        end else if (tick) begin
            if (&current_time) begin          // all-ones = 0xFFFF_FFFF
                current_time  <= 32'd0;
                overflow_flag <= 1'b1;
            end else begin
                current_time  <= current_time + 32'd1;
                overflow_flag <= 1'b0;
            end
        end else begin
            overflow_flag <= 1'b0;
        end
    end

    // ?? Per-car entry-time memory ?????????????????????????????????????????
    // All cars tracked in parallel: duration for any car is always
    //   current_time - entry_mem[car_id]
    // No special hardware needed for parallel tracking.
    reg [31:0] entry_mem [0:NUM_CARS-1];

    integer idx;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (idx = 0; idx < NUM_CARS; idx = idx + 1)
                entry_mem[idx] <= 32'd0;
        end else if (record_en) begin
            entry_mem[record_id] <= current_time;
        end
    end

    // Combinational read - available same cycle query_id changes
    assign entry_time_out = entry_mem[query_id];

endmodule