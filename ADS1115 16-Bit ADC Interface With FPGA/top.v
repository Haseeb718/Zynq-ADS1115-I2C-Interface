// ============================================================================
// ADS1115 I2C ADC Reader - Top Level Module
// ============================================================================
//
// Ported from iCE40/iCEBreaker to Zynq-7010 (50 MHz clock)
//
// CHANGES FROM ORIGINAL:
//   1. CLOCKS_PER_INTERVAL updated: 200ms at 50MHz = 10,000,000 clocks
//   2. interval_counter widened from [23:0] to [24:0]
//   3. Removed btn_addr / ads_addr (iCEBreaker-specific)
//   4. uart_tx instantiated with CLOCKS_PER_BIT=434 (50 MHz)
//   5. Added 4 debug output ports for ILA (scl_in, sda_in, scl_oe, sda_oe)
//   6. Added state output port for ILA
//   7. Added dbg_adc_value[15:0] output port for ILA
//
// NOTE: UART is optional. To remove it, delete the uart_tx instantiation
//       and all uart_* signals. The ILA probe and PS-side access continue
//       to work unchanged without UART.
//
// Original source: https://github.com/holla2040/Agentic_Verilog_iCE40_iCEBreaker
//
// ============================================================================

module top (
    input  wire       clk,           // 50 MHz system clock

    // I2C interface
    inout  wire       scl,           // I2C clock
    inout  wire       sda,           // I2C data

    // UART output (optional - see note above)
    output wire       uart_tx,       // UART transmit pin

    // Debug outputs for ILA
    output wire       dbg_scl_in,    // Actual SCL line state
    output wire       dbg_sda_in,    // Actual SDA line state
    output wire       dbg_scl_oe,    // 1 = master driving SCL low
    output wire       dbg_sda_oe,    // 1 = master driving SDA low
    output wire [4:0] dbg_state,     // Current state machine state
    output wire       i2c_ack,       // ACK/NACK from I2C master
    output wire [15:0] dbg_adc_value // ADC value for ILA
);

    // ========================================================================
    // TIMING PARAMETERS
    // ========================================================================

    localparam CLOCKS_PER_INTERVAL = 25'd10000000;  // 200ms at 50MHz

    localparam STARTUP_MSG_LEN = 10;
    localparam ADC_MSG_LEN     = 8;

    // ========================================================================
    // ADS1115 I2C PARAMETERS
    // ========================================================================

    localparam ADS1115_ADDR_W = 8'h90;  // 0x48 << 1 | 0 (ADDR tied to GND)
    localparam ADS1115_ADDR_R = 8'h91;  // 0x48 << 1 | 1

    localparam REG_CONVERSION = 8'h00;
    localparam REG_CONFIG     = 8'h01;

    // Config 0xC2C3:
    //   MUX=100 (AIN0 single-ended), PGA=001 (+-4.096V), MODE=0 (continuous)
    //   DR=110 (250 SPS)
    localparam CONFIG_MSB = 8'hC2;
    localparam CONFIG_LSB = 8'hC3;

    // ========================================================================
    // I2C COMMAND DEFINITIONS
    // ========================================================================

    localparam CMD_NONE  = 3'd0;
    localparam CMD_START = 3'd1;
    localparam CMD_STOP  = 3'd2;
    localparam CMD_WRITE = 3'd3;
    localparam CMD_READ  = 3'd4;

    // ========================================================================
    // INTERNAL SIGNALS
    // ========================================================================

    reg  [7:0] uart_data  = 8'd0;
    reg        uart_start = 1'b0;
    wire       uart_busy;

    reg  [2:0] i2c_cmd      = CMD_NONE;
    reg  [7:0] i2c_data_out = 8'd0;
    reg        i2c_ack_send = 1'b1;
    reg        i2c_start    = 1'b0;
    wire [7:0] i2c_data_in;
    wire       i2c_busy;

    reg [7:0] startup_msg [0:9];
    reg [7:0] adc_msg     [0:7];

    // ========================================================================
    // STATE DEFINITIONS
    // ========================================================================

    localparam ST_STARTUP   = 5'd0;
    localparam ST_CFG_START = 5'd1;
    localparam ST_CFG_ADDR  = 5'd2;
    localparam ST_CFG_REG   = 5'd3;
    localparam ST_CFG_MSB   = 5'd4;
    localparam ST_CFG_LSB   = 5'd5;
    localparam ST_CFG_STOP  = 5'd6;
    localparam ST_PTR_START = 5'd7;
    localparam ST_PTR_ADDR  = 5'd8;
    localparam ST_PTR_REG   = 5'd9;
    localparam ST_PTR_STOP  = 5'd10;
    localparam ST_IDLE      = 5'd11;
    localparam ST_RD_START  = 5'd12;
    localparam ST_RD_ADDR   = 5'd13;
    localparam ST_RD_MSB    = 5'd14;
    localparam ST_RD_LSB    = 5'd15;
    localparam ST_RD_STOP   = 5'd16;
    localparam ST_SEND_ADC  = 5'd17;
    localparam ST_ERROR     = 5'd18;

    reg [4:0]  state            = ST_STARTUP;
    reg [3:0]  msg_index        = 4'd0;
    reg        seen_busy        = 1'b0;
    reg        error_flag       = 1'b0;
    reg [15:0] adc_value        = 16'd0;
    reg [24:0] interval_counter = 25'd0;
    reg        interval_tick    = 1'b0;

    // Debug assignments
    assign dbg_state     = state;
    assign dbg_adc_value = adc_value;

    // ========================================================================
    // HEX TO ASCII
    // ========================================================================

    function [7:0] hex_to_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 10)
                hex_to_ascii = 8'h30 + nibble;
            else
                hex_to_ascii = 8'h41 + (nibble - 10);
        end
    endfunction

    // ========================================================================
    // INITIALIZATION
    // ========================================================================

    initial begin
        startup_msg[0] = 8'h0D;  // \r
        startup_msg[1] = 8'h0A;  // \n
        startup_msg[2] = "a";
        startup_msg[3] = "d";
        startup_msg[4] = "s";
        startup_msg[5] = "1";
        startup_msg[6] = "1";
        startup_msg[7] = "1";
        startup_msg[8] = "5";
        startup_msg[9] = 8'h0A;  // \n
    end

    // ========================================================================
    // MODULE INSTANTIATIONS
    // ========================================================================

    uart_tx #(
        .CLOCKS_PER_BIT(434)     // 50 MHz / 115200 baud
    ) u_uart (
        .clk(clk),
        .data_i(uart_data),
        .start_i(uart_start),
        .busy_o(uart_busy),
        .tx_o(uart_tx)
    );

    i2c_master u_i2c (
        .clk(clk),
        .scl(scl),
        .sda(sda),
        .cmd_i(i2c_cmd),
        .data_i(i2c_data_out),
        .ack_i(i2c_ack_send),
        .start_i(i2c_start),
        .data_o(i2c_data_in),
        .ack_o(i2c_ack),
        .busy_o(i2c_busy),
        .dbg_scl_in(dbg_scl_in),
        .dbg_sda_in(dbg_sda_in),
        .dbg_scl_oe(dbg_scl_oe),
        .dbg_sda_oe(dbg_sda_oe)
    );

    // ========================================================================
    // INTERVAL TIMER (200ms)
    // ========================================================================

    always @(posedge clk) begin
        interval_tick <= 1'b0;

        if (interval_counter == CLOCKS_PER_INTERVAL - 1) begin
            interval_counter <= 25'd0;
            interval_tick    <= 1'b1;
        end else begin
            interval_counter <= interval_counter + 1'b1;
        end
    end

    // ========================================================================
    // MAIN STATE MACHINE
    // ========================================================================
    //
    // seen_busy pattern:
    //   - Fire i2c_start in state A, transition to state B
    //   - In state B: clear seen_busy, wait for (!i2c_busy && seen_busy)
    //   This ensures the I2C master has time to assert busy_o before
    //   seen_busy is cleared.
    //
    // ========================================================================

    always @(posedge clk) begin
        uart_start <= 1'b0;
        i2c_start  <= 1'b0;

        if (i2c_busy) begin
            seen_busy <= 1'b1;
        end

        case (state)

            // ================================================================
            // STARTUP: Send "\r\nads1115\n" via UART
            // ================================================================
            ST_STARTUP: begin
                if (!uart_busy && !uart_start) begin
                    if (msg_index < STARTUP_MSG_LEN) begin
                        uart_data  <= startup_msg[msg_index];
                        uart_start <= 1'b1;
                        msg_index  <= msg_index + 1'b1;
                    end else begin
                        msg_index <= 4'd0;
                        state     <= ST_CFG_START;
                    end
                end
            end

            // ================================================================
            // CONFIGURE ADS1115: Write config register 0x01 = 0xC2C3
            // ================================================================
            ST_CFG_START: begin
                i2c_cmd    <= CMD_START;
                i2c_start  <= 1'b1;
                seen_busy  <= 1'b0;
                error_flag <= 1'b0;
                state      <= ST_CFG_ADDR;
            end

            ST_CFG_ADDR: begin
                if (!i2c_busy && seen_busy) begin
                    i2c_cmd      <= CMD_WRITE;
                    i2c_data_out <= ADS1115_ADDR_W;
                    i2c_start    <= 1'b1;
                    seen_busy    <= 1'b0;
                    state        <= ST_CFG_REG;
                end
            end

            ST_CFG_REG: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        seen_busy  <= 1'b0;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd      <= CMD_WRITE;
                        i2c_data_out <= REG_CONFIG;
                        i2c_start    <= 1'b1;
                        seen_busy    <= 1'b0;
                        state        <= ST_CFG_MSB;
                    end
                end
            end

            ST_CFG_MSB: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        seen_busy  <= 1'b0;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd      <= CMD_WRITE;
                        i2c_data_out <= CONFIG_MSB;
                        i2c_start    <= 1'b1;
                        seen_busy    <= 1'b0;
                        state        <= ST_CFG_LSB;
                    end
                end
            end

            ST_CFG_LSB: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        seen_busy  <= 1'b0;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd      <= CMD_WRITE;
                        i2c_data_out <= CONFIG_LSB;
                        i2c_start    <= 1'b1;
                        seen_busy    <= 1'b0;
                        state        <= ST_CFG_STOP;
                    end
                end
            end

            ST_CFG_STOP: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        seen_busy  <= 1'b0;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd   <= CMD_STOP;
                        i2c_start <= 1'b1;
                        seen_busy <= 1'b0;
                        state     <= ST_PTR_START;
                    end
                end
            end

            // ================================================================
            // SET POINTER to conversion register 0x00
            // ================================================================
            ST_PTR_START: begin
                if (!i2c_busy && seen_busy) begin
                    i2c_cmd   <= CMD_START;
                    i2c_start <= 1'b1;
                    seen_busy <= 1'b0;
                    state     <= ST_PTR_ADDR;
                end
            end

            ST_PTR_ADDR: begin
                if (!i2c_busy && seen_busy) begin
                    i2c_cmd      <= CMD_WRITE;
                    i2c_data_out <= ADS1115_ADDR_W;
                    i2c_start    <= 1'b1;
                    seen_busy    <= 1'b0;
                    state        <= ST_PTR_REG;
                end
            end

            ST_PTR_REG: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        seen_busy  <= 1'b0;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd      <= CMD_WRITE;
                        i2c_data_out <= REG_CONVERSION;
                        i2c_start    <= 1'b1;
                        seen_busy    <= 1'b0;
                        state        <= ST_PTR_STOP;
                    end
                end
            end

            ST_PTR_STOP: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        seen_busy  <= 1'b0;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd   <= CMD_STOP;
                        i2c_start <= 1'b1;
                        seen_busy <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end
            end

            // ================================================================
            // IDLE: Wait for 200ms interval tick
            // ================================================================
            ST_IDLE: begin
                if (!i2c_busy && seen_busy) begin
                    seen_busy <= 1'b0;
                end

                if (interval_tick && !seen_busy) begin
                    i2c_cmd    <= CMD_START;
                    i2c_start  <= 1'b1;
                    seen_busy  <= 1'b0;
                    error_flag <= 1'b0;
                    state      <= ST_RD_START;
                end
            end

            // ================================================================
            // READ ADC: START -> 0x91 -> MSB(ACK) -> LSB(NACK) -> STOP
            // ================================================================
            ST_RD_START: begin
                if (!i2c_busy && seen_busy) begin
                    i2c_cmd      <= CMD_WRITE;
                    i2c_data_out <= ADS1115_ADDR_R;
                    i2c_start    <= 1'b1;
                    seen_busy    <= 1'b0;
                    state        <= ST_RD_ADDR;
                end
            end

            ST_RD_ADDR: begin
                if (!i2c_busy && seen_busy) begin
                    if (i2c_ack) begin
                        error_flag <= 1'b1;
                        i2c_cmd    <= CMD_STOP;
                        i2c_start  <= 1'b1;
                        seen_busy  <= 1'b0;
                        state      <= ST_ERROR;
                    end else begin
                        i2c_cmd      <= CMD_READ;
                        i2c_ack_send <= 1'b0;   // ACK after MSB (more to read)
                        i2c_start    <= 1'b1;
                        seen_busy    <= 1'b0;
                        state        <= ST_RD_MSB;
                    end
                end
            end

            ST_RD_MSB: begin
                if (!i2c_busy && seen_busy) begin
                    adc_value[15:8] <= i2c_data_in;
                    i2c_cmd         <= CMD_READ;
                    i2c_ack_send    <= 1'b1;    // NACK after LSB (last byte)
                    i2c_start       <= 1'b1;
                    seen_busy       <= 1'b0;
                    state           <= ST_RD_LSB;
                end
            end

            ST_RD_LSB: begin
                if (!i2c_busy && seen_busy) begin
                    adc_value[7:0] <= i2c_data_in;
                    i2c_cmd        <= CMD_STOP;
                    i2c_start      <= 1'b1;
                    seen_busy      <= 1'b0;
                    state          <= ST_RD_STOP;
                end
            end

            ST_RD_STOP: begin
                if (!i2c_busy && seen_busy) begin
                    // Format "0xNNNN\r\n"
                    adc_msg[0] <= "0";
                    adc_msg[1] <= "x";
                    adc_msg[2] <= hex_to_ascii(adc_value[15:12]);
                    adc_msg[3] <= hex_to_ascii(adc_value[11:8]);
                    adc_msg[4] <= hex_to_ascii(adc_value[7:4]);
                    adc_msg[5] <= hex_to_ascii(adc_value[3:0]);
                    adc_msg[6] <= 8'h0D;  // \r
                    adc_msg[7] <= 8'h0A;  // \n

                    msg_index <= 4'd0;
                    seen_busy <= 1'b0;
                    state     <= ST_SEND_ADC;
                end
            end

            // ================================================================
            // SEND ADC VALUE via UART: "0xNNNN\r\n"
            // ================================================================
            ST_SEND_ADC: begin
                if (!uart_busy && !uart_start) begin
                    if (msg_index < ADC_MSG_LEN) begin
                        uart_data  <= adc_msg[msg_index];
                        uart_start <= 1'b1;
                        msg_index  <= msg_index + 1'b1;
                    end else begin
                        msg_index <= 4'd0;
                        state     <= ST_IDLE;
                    end
                end
            end

            // ================================================================
            // ERROR: Send 'E' via UART and return to idle
            // ================================================================
            ST_ERROR: begin
                if (!i2c_busy && seen_busy) begin
                    seen_busy <= 1'b0;
                end

                if (!uart_busy && !uart_start && !seen_busy) begin
                    uart_data  <= "E";
                    uart_start <= 1'b1;
                    state      <= ST_IDLE;
                end
            end

            default: begin
                state <= ST_STARTUP;
            end
        endcase
    end

endmodule
