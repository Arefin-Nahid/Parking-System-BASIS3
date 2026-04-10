`timescale 1ns/1ps
// =============================================================================
// Module  : control_unit
// Project : Smart Parking System v2 - Basys3 (Artix-7)
// Purpose : Central FSM coordinating all parking system transactions.
//
// ?????????????????????????????????????????????????????????????????????????????
//  ARCHITECTURE OVERVIEW
// ?????????????????????????????????????????????????????????????????????????????
//  The FSM is TRANSACTION-BASED: it returns to IDLE after each entry or exit.
//  This is the correct design because:
//    ? Multiple cars can be parked simultaneously (parallel tracking via memory)
//    ? The parking_memory module is the authoritative source of truth
//    ? The FSM only needs to be active during the brief moments a car
//      enters or exits - it is otherwise free to handle the next transaction
//
// ?????????????????????????????????????????????????????????????????????????????
//  STATE DIAGRAM
// ?????????????????????????????????????????????????????????????????????????????
//
//                          ????????????????
//                          ?     IDLE     ?????????????????????????
//                          ????????????????                       ?
//                    btn_entry ?      ? btn_exit                  ?
//                 ????????????          ????????????????          ?
//                 ?  READ_ID ?          ? EXIT_REQUEST ?          ?
//                 ????????????          ????????????????          ?
//              [bad ID]?                       ?[bad ID]           ?
//                   ERROR                   ERROR                  ?
//                      ?                       ?[!inside?ERROR]    ?
//               ??????????????         ?????????????????         ?
//               ?CHECK_ENTRY ?         ?  CHECK_EXIT   ?         ?
//               ??????????????         ?????????????????         ?
//           [inside?ERROR]                     ?                   ?
//                 ????????????         ????????????????          ?
//                 ?CHECK_FULL?         ?CALC_DURATION ?          ?
//                 ????????????         ????????????????          ?
//           [full?ERROR]                        ?(1-cyc wait)      ?
//               ?????????????         ????????????????           ?
//               ? FIND_SLOT ?         ?   CALC_FEE   ?           ?
//               ?????????????         ????????????????           ?
//                    ?(1-cyc wait)              ?(1-cyc wait)      ?
//             ???????????????         ????????????????           ?
//             ? ASSIGN_SLOT ?         ? WAIT_PAYMENT ?           ?
//             ???????????????         ????????????????           ?
//                    ?                     [btn_pay]               ?
//             ???????????????         ????????????????           ?
//             ? START_TIMER ?         ?  FREE_SLOT   ?           ?
//             ???????????????         ????????????????           ?
//                    ?                           ?                 ?
//                    ???????????????????????????????????????????????
//
// ?????????????????????????????????????????????????????????????????????????????
//  DISPLAY MODE SCHEDULE
// ?????????????????????????????????????????????????????????????????????????????
//   IDLE / ERROR          : mode=00 (Free Slots)
//   READ_ID?START_TIMER   : mode=01 (Car ID being processed)
//   CALC_DURATION+        : mode=10 (Duration)
//   CALC_FEE+WAIT_PAYMENT : mode=11 (Fee)
//   FREE_SLOT             : mode=00 (Free Slots, updated)
//
// ?????????????????????????????????????????????????????????????????????????????
//  CORNER CASES
// ?????????????????????????????????????????????????????????????????????????????
//   1. Duplicate car entry    ? CHECK_ENTRY:  mem_car_inside ? ERROR(DUP_ENTRY)
//   2. Parking full           ? CHECK_FULL:   ALU result ? ERROR(FULL)
//   3. Exit without entry     ? CHECK_EXIT:  !mem_car_inside ? ERROR(NO_ENTRY)
//   4. Exit without payment   ? WAIT_PAYMENT: locked until btn_pay
//   5. Slot overwrite         ? parking_memory write guard (hardware)
//   6. Button bounce          ? debounce module (upstream)
//   7. Simultaneous entry+exit? IDLE: entry checked first (priority)
//   8. Invalid ID (0xF)       ? READ_ID / EXIT_REQUEST ? ERROR(INVALID_ID)
//   9. Timer overflow         ? START_TIMER: overflow_flag ? ERROR(OVERFLOW)
//  10. Reset at any time      ? async rst: all state cleared, back to IDLE
// =============================================================================

module control_unit #(
    parameter NUM_SLOTS = 8,
    parameter NUM_CARS  = 16,
    parameter ID_BITS   = 4,
    parameter SLOT_BITS = 3,
    parameter [ID_BITS-1:0] EMPTY_ID = {ID_BITS{1'b1}}  // 4'hF = reserved
)(
    input  wire                  clk,
    input  wire                  rst,

    // ?? Button inputs (debounced single-cycle pulses) ???????????????????
    input  wire                  btn_entry,   // Entry request
    input  wire                  btn_exit,    // Exit request
    input  wire                  btn_pay,     // Payment confirmation

    // ?? Switch input ????????????????????????????????????????????????????
    input  wire [ID_BITS-1:0]    sw_car_id,   // Car ID from switches

    // ?? Time tracker interface ???????????????????????????????????????????
    output reg                   timer_record_en,  // Pulse: record entry time
    output reg  [ID_BITS-1:0]    timer_record_id,  // Car ID to record
    output reg  [ID_BITS-1:0]    timer_query_id,   // Car ID to query (for ALU)
    // current_time is routed directly to the ALU at top-level; not needed here
    input  wire [31:0]           entry_time_in,    // entry_mem[timer_query_id]
    input  wire                  overflow_flag,

    // ?? ALU interface ????????????????????????????????????????????????????
    output reg  [2:0]            alu_opcode,
    input  wire [15:0]           alu_duration,
    input  wire [15:0]           alu_fee,
    input  wire                  alu_result_flag,
    input  wire [SLOT_BITS-1:0]  alu_free_slot,
    input  wire                  alu_free_valid,

    // ?? Memory interface ?????????????????????????????????????????????????
    output reg                   mem_write_en,
    output reg                   mem_op_assign,     // 1=ASSIGN, 0=FREE
    output reg  [ID_BITS-1:0]    mem_car_id,        // Address for reads AND writes
    output reg  [SLOT_BITS-1:0]  mem_slot_idx,      // Slot for ASSIGN
    input  wire                  mem_car_inside,    // car_status[mem_car_id]
    input  wire [NUM_SLOTS-1:0]  slot_occupied,     // full bitmask

    // ?? Display outputs ??????????????????????????????????????????????????
    output reg  [1:0]            disp_mode,
    output reg  [15:0]           disp_fee,
    output reg  [15:0]           disp_duration,
    output reg  [3:0]            disp_car_id,

    // ?? LED output ???????????????????????????????????????????????????????
    output wire [7:0]            led_slots,   // Slot occupancy ? LEDs

    // ?? Free count output (registered, for display) ??????????????????????
    output reg  [3:0]            free_count_out,  // Registered free slot count

    // ?? Status / debug outputs ???????????????????????????????????????????
    output reg  [3:0]            state_out,
    output reg                   error_out,   // Sticky until next transaction
    output reg  [3:0]            error_code
);

    // ?? State encoding ????????????????????????????????????????????????????
    localparam [3:0]
        S_IDLE          = 4'd0,
        S_READ_ID       = 4'd1,
        S_CHECK_ENTRY   = 4'd2,
        S_CHECK_FULL    = 4'd3,
        S_FIND_SLOT     = 4'd4,
        S_ASSIGN_SLOT   = 4'd5,
        S_START_TIMER   = 4'd6,
        S_EXIT_REQUEST  = 4'd8,   // Note: 4'd7 unused (no S_WAIT - see arch note)
        S_CHECK_EXIT    = 4'd9,
        S_CALC_DURATION = 4'd10,
        S_CALC_FEE      = 4'd11,
        S_WAIT_PAYMENT  = 4'd12,
        S_FREE_SLOT     = 4'd13,
        S_ERROR         = 4'd14;

    // ?? Error codes ???????????????????????????????????????????????????????
    localparam [3:0]
        ERR_NONE       = 4'd0,
        ERR_DUP_ENTRY  = 4'd1,   // Car already inside
        ERR_FULL       = 4'd2,   // All slots occupied
        ERR_NO_ENTRY   = 4'd3,   // Exit attempted without entry
        ERR_INVALID_ID = 4'd4,   // Car ID = 0xF (reserved)
        ERR_OVERFLOW   = 4'd5;   // Timer overflow detected at entry

    // ?? ALU opcode aliases ????????????????????????????????????????????????
    localparam [2:0]
        OP_NOP         = 3'b000,
        OP_CALC_DUR    = 3'b001,
        OP_CALC_FEE    = 3'b010,
        OP_CHECK_FULL  = 3'b011,
        OP_FIND_FREE   = 3'b101;

    // ?? Internal registers ????????????????????????????????????????????????
    reg [3:0]           state;
    reg [ID_BITS-1:0]   active_car;    // Car ID for the current transaction
    reg [SLOT_BITS-1:0] assigned_slot; // Slot found by FIND_SLOT
    reg [3:0]           stored_error;  // Latched error code for ERROR state
    reg                 alu_ready;     // Pipeline handshake: 0=wait, 1=sample
    reg                 error_sticky;  // Holds error_out between states
    reg [1:0] fee_wait;

    // ?? LED is always live slot occupancy ?????????????????????????????????
    assign led_slots = slot_occupied[7:0];

    // ?? Free slot count - combinational sum, then registered ????????????????
    // Combinational sum (wire-level) stays inside the always @(*)
    // The registered version cuts: sum ? disp_input ? BCD_pipe path
    integer s;
    reg [3:0] free_count_comb;  // combinational intermediate
    always @(*) begin
        free_count_comb = 4'd0;
        for (s = 0; s < NUM_SLOTS; s = s + 1)
            free_count_comb = free_count_comb + {{3{1'b0}}, ~slot_occupied[s]};
    end

    // Register free_count_comb to cut combinational path to display
    always @(posedge clk or posedge rst) begin
        if (rst)
            free_count_out <= 4'd0;
        else
            free_count_out <= free_count_comb;
    end

    // ??????????????????????????????????????????????????????????????????????
    //  MAIN FSM  - single synchronous always block
    // ??????????????????????????????????????????????????????????????????????
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state            <= S_IDLE;
            active_car       <= {ID_BITS{1'b0}};
            assigned_slot    <= {SLOT_BITS{1'b0}};
            stored_error     <= ERR_NONE;
            alu_ready        <= 1'b0;
            error_sticky     <= 1'b0;
            fee_wait         <= 2'd0;
            //free_count_out   <= 4'd0;
            // ?? output regs ??
            timer_record_en  <= 1'b0;
            timer_record_id  <= {ID_BITS{1'b0}};
            timer_query_id   <= {ID_BITS{1'b0}};
            alu_opcode       <= OP_NOP;
            mem_write_en     <= 1'b0;
            mem_op_assign    <= 1'b0;
            mem_car_id       <= {ID_BITS{1'b0}};
            mem_slot_idx     <= {SLOT_BITS{1'b0}};
            disp_mode        <= 2'b00;
            disp_fee         <= 16'd0;
            disp_duration    <= 16'd0;
            disp_car_id      <= 4'd0;
            error_out        <= 1'b0;
            error_code       <= ERR_NONE;
            state_out        <= S_IDLE;
        end else begin
            // ?? Default: de-assert all single-cycle strobes ???????????????
            timer_record_en <= 1'b0;
            mem_write_en    <= 1'b0;
            alu_opcode      <= OP_NOP;
            // Maintain sticky error until cleared in IDLE
            error_out       <= error_sticky;
            state_out       <= state;

            case (state)

                // ============================================================
                // IDLE - wait for a button press
                // Entry has priority over exit (corner case #7)
                // ============================================================
                S_IDLE: begin
                    alu_ready    <= 1'b0;
                    disp_mode    <= 2'b00;    // show free slots
                    // Clear error sticky when user starts a new transaction
                    error_sticky <= 1'b0;
                    error_out    <= 1'b0;

                    if (btn_entry) begin
                        active_car   <= sw_car_id;
                        mem_car_id   <= sw_car_id;  // pre-assert for async read
                        disp_car_id  <= sw_car_id;
                        state        <= S_READ_ID;
                    end else if (btn_exit) begin
                        active_car   <= sw_car_id;
                        mem_car_id   <= sw_car_id;
                        disp_car_id  <= sw_car_id;
                        state        <= S_EXIT_REQUEST;
                    end
                end

                // ============================================================
                // READ_ID - validate car ID (reject reserved 0xF)
                // ============================================================
                S_READ_ID: begin
                    disp_mode  <= 2'b01;    // show "CA X"
                    if (active_car == EMPTY_ID) begin
                        stored_error <= ERR_INVALID_ID;
                        state        <= S_ERROR;
                    end else begin
                        mem_car_id <= active_car;   // hold for async read
                        state      <= S_CHECK_ENTRY;
                    end
                end

                // ============================================================
                // CHECK_ENTRY - is this car already inside? (dup-entry guard)
                // mem_car_inside is combinational and valid this cycle
                // because mem_car_id was set in READ_ID
                // ============================================================
                S_CHECK_ENTRY: begin
                    mem_car_id <= active_car;
                    if (mem_car_inside) begin
                        stored_error <= ERR_DUP_ENTRY;
                        state        <= S_ERROR;
                    end else begin
                        // Issue CHECK_FULL to ALU; sample result next cycle
                        alu_opcode <= OP_CHECK_FULL;
                        alu_ready  <= 1'b0;
                        state      <= S_CHECK_FULL;
                    end
                end

                // ============================================================
                // CHECK_FULL - wait for ALU result (1-cycle pipeline)
                // alu_ready=0 ? first cycle in state, opcode was just issued
                // alu_ready=1 ? result in alu_result_flag is now valid
                // ============================================================
                S_CHECK_FULL: begin
                    if (!alu_ready) begin
                        alu_ready <= 1'b1;   // next cycle result is valid
                    end else begin
                        alu_ready <= 1'b0;
                        if (alu_result_flag) begin
                            stored_error <= ERR_FULL;
                            state        <= S_ERROR;
                        end else begin
                            alu_opcode <= OP_FIND_FREE;
                            state      <= S_FIND_SLOT;
                        end
                    end
                end

                // ============================================================
                // FIND_SLOT - priority encode lowest free slot (1-cycle ALU)
                // ============================================================
                S_FIND_SLOT: begin
                    if (!alu_ready) begin
                        alu_ready <= 1'b1;
                    end else begin
                        alu_ready <= 1'b0;
                        if (alu_free_valid) begin
                            assigned_slot <= alu_free_slot;
                            state         <= S_ASSIGN_SLOT;
                        end else begin
                            stored_error <= ERR_FULL;
                            state        <= S_ERROR;
                        end
                    end
                end

                // ============================================================
                // ASSIGN_SLOT - write to memory: mark car INSIDE, store slot
                // ============================================================
                S_ASSIGN_SLOT: begin
                    mem_write_en  <= 1'b1;
                    mem_op_assign <= 1'b1;
                    mem_car_id    <= active_car;
                    mem_slot_idx  <= assigned_slot;
                    state         <= S_START_TIMER;
                end

                // ============================================================
                // START_TIMER - record entry timestamp for this car
                // ============================================================
                S_START_TIMER: begin
                    timer_record_en <= 1'b1;
                    timer_record_id <= active_car;
                    if (overflow_flag) begin
                        // Unsafe to park (timer about to wrap) - corner case #9
                        stored_error <= ERR_OVERFLOW;
                        state        <= S_ERROR;
                    end else begin
                        state <= S_IDLE;   // Entry complete - ready for next car
                    end
                end

                // ============================================================
                // EXIT_REQUEST - validate ID for exit
                // ============================================================
                S_EXIT_REQUEST: begin
                    disp_mode <= 2'b01;    // show "CA X" while processing
                    if (active_car == EMPTY_ID) begin
                        stored_error <= ERR_INVALID_ID;
                        state        <= S_ERROR;
                    end else begin
                        mem_car_id   <= active_car;
                        state        <= S_CHECK_EXIT;
                    end
                end

                // ============================================================
                // CHECK_EXIT - is the car actually inside? (no ghost exits)
                // ============================================================
                S_CHECK_EXIT: begin
                    mem_car_id     <= active_car;
                    timer_query_id <= active_car;  // route entry_time to ALU
                    if (!mem_car_inside) begin
                        stored_error <= ERR_NO_ENTRY;
                        state        <= S_ERROR;
                    end else begin
                        alu_opcode <= OP_CALC_DUR;
                        alu_ready  <= 1'b0;
                        state      <= S_CALC_DURATION;
                    end
                end

                // ============================================================
                // CALC_DURATION - compute elapsed time (1-cycle ALU).
                // Display stays on car ID (mode 01) while calculating.
                // Duration is stored internally but NOT shown on display -
                // the user sees the fee directly instead (more realistic UX).
                // ============================================================
                S_CALC_DURATION: begin
                    disp_mode <= 2'b01;   // keep showing car ID while computing
                    if (!alu_ready) begin
                        alu_ready <= 1'b1;
                    end else begin
                        alu_ready     <= 1'b0;
                        disp_duration <= alu_duration;  // saved for internal use
                        // Immediately request fee calculation
                        alu_opcode <= OP_CALC_FEE;
                        state      <= S_CALC_FEE;
                    end
                end

                // ============================================================
                // CALC_FEE - multiply duration x rate (pipelined ALU).
                // fee_wait gives the multiplier pipeline time to settle.
                // Switches display directly to fee (mode 11) - no duration step.
                // ============================================================
                S_CALC_FEE: begin
                    if (fee_wait == 2'b10) begin
                        fee_wait  <= 2'b00;
                        disp_fee  <= alu_fee;
                        disp_mode <= 2'b11;   // show fee - first thing user sees
                        state     <= S_WAIT_PAYMENT;
                    end else begin
                        fee_wait <= fee_wait + 2'b01;
                    end
                end

                // ============================================================
                // WAIT_PAYMENT - FSM locked until BTNR (pay) is pressed.
                // Display shows fee the whole time.
                // Entry/exit buttons are ignored here (corner case #4).
                // ============================================================
                S_WAIT_PAYMENT: begin
                    disp_mode <= 2'b11;   // keep showing fee until paid
                    if (btn_pay) state <= S_FREE_SLOT;
                end

                // ============================================================
                // FREE_SLOT - release parking slot, mark car OUTSIDE
                // ============================================================
                S_FREE_SLOT: begin
                    mem_write_en  <= 1'b1;
                    mem_op_assign <= 1'b0;   // FREE operation
                    mem_car_id    <= active_car;
                    disp_mode     <= 2'b00;  // back to free slots display
                    state         <= S_IDLE;
                end

                // ============================================================
                // ERROR - display error for one cycle, then return to IDLE
                // error_sticky keeps error_out HIGH until next transaction
                // ============================================================
                S_ERROR: begin
                    error_out    <= 1'b1;
                    error_sticky <= 1'b1;
                    error_code   <= stored_error;
                    disp_mode    <= 2'b00;
                    state        <= S_IDLE;
                end

                default: state <= S_IDLE;

            endcase
        end
    end

endmodule