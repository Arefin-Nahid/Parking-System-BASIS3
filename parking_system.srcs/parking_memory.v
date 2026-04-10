`timescale 1ns/1ps
// =============================================================================
// Module  : parking_memory
// Project : Smart Parking System v2 - Basys3 (Artix-7)
// Purpose : Central state memory for all cars and all slots.
//
// ?????????????????????????????????????????????????????????????????????????????
//  TIMING FIX - v2.2
// ?????????????????????????????????????????????????????????????????????????????
//  car_inside and slot_occupied were purely combinational (assign statements).
//  Vivado placed them on the critical path:
//    mem_car_id FF ? array mux ? car_inside ? FSM branch ? state FF
//  Fix: both outputs are now REGISTERED (sampled at posedge clk).
//
//  FSM compatibility:
//    The FSM sets mem_car_id one state BEFORE reading mem_car_inside:
//      IDLE       ? sets mem_car_id = sw_car_id
//      READ_ID    ? holds mem_car_id, advances state
//      CHECK_ENTRY? reads mem_car_inside  ? registered value from READ_ID cycle
//    This is exactly 1-cycle read latency, which the FSM state pipeline
//    already provides. Identical analysis holds for CHECK_EXIT.
//
//  slot_occupied latency: 1 cycle. The ALU already uses an alu_ready
//  handshake that adds a wait cycle for every opcode - this absorbs the
//  registered slot_occupied with zero FSM changes required.
// =============================================================================

module parking_memory #(
    parameter NUM_SLOTS = 8,
    parameter NUM_CARS  = 16,
    parameter ID_BITS   = 4,
    parameter SLOT_BITS = 3
)(
    input  wire                  clk,
    input  wire                  rst,
    // Write port
    input  wire                  write_en,
    input  wire                  op_assign,
    input  wire [ID_BITS-1:0]    car_id,
    input  wire [SLOT_BITS-1:0]  slot_idx,
    // Read port - REGISTERED (1-cycle latency, breaks combinational critical paths)
    output reg                   car_inside,        // car_status[car_id], registered
    output wire [SLOT_BITS-1:0]  car_slot_out,      // car_slot[car_id], combinational (unused in timing path)
    output wire [ID_BITS-1:0]    slot_car_out,      // slot_data[slot_idx], combinational
    output reg  [NUM_SLOTS-1:0]  slot_occupied,     // registered occupancy bitmask
    output wire                  slot_taken         // slot_idx occupied? combinational
);

    localparam [ID_BITS-1:0] EMPTY_ID = {ID_BITS{1'b1}};

    // ?? Memory arrays ?????????????????????????????????????????????????????
    reg                   car_status [0:NUM_CARS-1];
    reg [ID_BITS-1:0]     slot_data  [0:NUM_SLOTS-1];
    reg [SLOT_BITS-1:0]   car_slot   [0:NUM_CARS-1];

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < NUM_CARS; i = i + 1) begin
                car_status[i] <= 1'b0;
                car_slot[i]   <= {SLOT_BITS{1'b0}};
            end
            for (i = 0; i < NUM_SLOTS; i = i + 1)
                slot_data[i] <= EMPTY_ID;

            // ?? Reset registered outputs ??????????????????????????????????
            car_inside    <= 1'b0;
            slot_occupied <= {NUM_SLOTS{1'b0}};

        end else begin
            // ?? Write logic (unchanged) ???????????????????????????????????
            if (write_en) begin
                if (op_assign) begin
                    if (!car_status[car_id] && (slot_data[slot_idx] == EMPTY_ID)) begin
                        car_status[car_id]  <= 1'b1;
                        slot_data[slot_idx] <= car_id;
                        car_slot[car_id]    <= slot_idx;
                    end
                end else begin
                    if (car_status[car_id]) begin
                        slot_data[car_slot[car_id]] <= EMPTY_ID;
                        car_status[car_id]          <= 1'b0;
                        car_slot[car_id]            <= {SLOT_BITS{1'b0}};
                    end
                end
            end

            // ?? Registered read outputs - sampled every clock ?????????????
            // car_inside: registered lookup of car_status[car_id].
            // FSM sets mem_car_id one state before reading mem_car_inside,
            // so this 1-cycle latency is already absorbed by the state pipeline.
            car_inside <= car_status[car_id];

            // slot_occupied: registered bitmask of all slot states.
            // Cuts the slot_data[] ? occ_count ? result_flag ? state path.
            begin : REG_OCC
                integer g;
                for (g = 0; g < NUM_SLOTS; g = g + 1)
                    slot_occupied[g] <= (slot_data[g] != EMPTY_ID);
            end
        end
    end

    // ?? Combinational reads (not on timing-critical paths) ????????????????
    assign car_slot_out = car_slot[car_id];
    assign slot_car_out = slot_data[slot_idx];
    assign slot_taken   = (slot_data[slot_idx] != EMPTY_ID);

endmodule