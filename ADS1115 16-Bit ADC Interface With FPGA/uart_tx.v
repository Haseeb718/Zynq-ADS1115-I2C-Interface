// ============================================================================
// UART Transmitter
// ============================================================================
//
// Ported from iCE40/iCEBreaker to Zynq-7010 (50 MHz clock)
//
// CHANGES FROM ORIGINAL:
//   1. Updated default CLOCKS_PER_BIT: 50MHz / 115200 = 434
//   2. Widened baud_counter from [7:0] to [9:0] (needs to count to 434)
//
// Original source: https://github.com/holla2040/Agentic_Verilog_iCE40_iCEBreaker
//
// ============================================================================

module uart_tx #(
    parameter CLOCKS_PER_BIT = 434   // 50 MHz / 115200 baud = 434  (was 104 for 12MHz)
) (
    input  wire       clk,           // System clock (50 MHz)
    input  wire [7:0] data_i,        // Byte to transmit
    input  wire       start_i,       // Start transmission (single clock pulse)

    output reg        busy_o,        // High during transmission
    output reg        tx_o           // UART transmit line
);

    // ========================================================================
    // STATE MACHINE DEFINITIONS
    // ========================================================================

    localparam STATE_IDLE  = 2'd0;
    localparam STATE_START = 2'd1;
    localparam STATE_DATA  = 2'd2;
    localparam STATE_STOP  = 2'd3;

    reg [1:0] state = STATE_IDLE;

    // ========================================================================
    // REGISTERS
    // ========================================================================

    reg [7:0] tx_byte      = 8'd0;
    reg [9:0] baud_counter = 10'd0;  // Widened from [7:0] to [9:0] (needs to count to 434)
    reg [2:0] bit_index    = 3'd0;

    // ========================================================================
    // UART TRANSMITTER STATE MACHINE
    // 8N1 format: 1 start bit, 8 data bits (LSB first), 1 stop bit
    // ========================================================================

    always @(posedge clk) begin
        case (state)

            // ----------------------------------------------------------------
            // IDLE: Line held HIGH, wait for start pulse
            // ----------------------------------------------------------------
            STATE_IDLE: begin
                tx_o         <= 1'b1;
                baud_counter <= 10'd0;
                bit_index    <= 3'd0;
                busy_o       <= 1'b0;

                if (start_i) begin
                    tx_byte <= data_i;
                    busy_o  <= 1'b1;
                    state   <= STATE_START;
                end
            end

            // ----------------------------------------------------------------
            // START BIT: Drive line LOW for one baud period
            // ----------------------------------------------------------------
            STATE_START: begin
                tx_o <= 1'b0;

                if (baud_counter == CLOCKS_PER_BIT - 1) begin
                    baud_counter <= 10'd0;
                    state        <= STATE_DATA;
                end else begin
                    baud_counter <= baud_counter + 1'b1;
                end
            end

            // ----------------------------------------------------------------
            // DATA BITS: Send 8 bits LSB first
            // ----------------------------------------------------------------
            STATE_DATA: begin
                tx_o <= tx_byte[0];

                if (baud_counter == CLOCKS_PER_BIT - 1) begin
                    baud_counter <= 10'd0;

                    if (bit_index == 3'd7) begin
                        bit_index <= 3'd0;
                        state     <= STATE_STOP;
                    end else begin
                        bit_index <= bit_index + 1'b1;
                        tx_byte   <= tx_byte >> 1;
                    end
                end else begin
                    baud_counter <= baud_counter + 1'b1;
                end
            end

            // ----------------------------------------------------------------
            // STOP BIT: Drive line HIGH for one baud period
            // ----------------------------------------------------------------
            STATE_STOP: begin
                tx_o <= 1'b1;

                if (baud_counter == CLOCKS_PER_BIT - 1) begin
                    baud_counter <= 10'd0;
                    busy_o       <= 1'b0;
                    state        <= STATE_IDLE;
                end else begin
                    baud_counter <= baud_counter + 1'b1;
                end
            end

            default: begin
                state <= STATE_IDLE;
                tx_o  <= 1'b1;
            end
        endcase
    end

endmodule
