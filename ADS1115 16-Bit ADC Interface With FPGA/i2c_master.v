// ============================================================================
// I2C Master Controller
// ============================================================================
//
// Ported from iCE40/iCEBreaker to Zynq-7010 (50 MHz clock)
//
// CHANGES FROM ORIGINAL:
//   1. Removed SB_IO primitives (iCE40-specific)
//      Replaced with Xilinx IOBUF primitives for proper tristate on Zynq
//   2. Updated HALF_PERIOD: 50MHz / 100kHz / 2 = 250 clocks
//   3. Widened timer register from [6:0] to [8:0] to hold value up to 250
//   4. Added debug output ports for ILA connection in block design
//
// OPEN-DRAIN IMPLEMENTATION (Xilinx IOBUF):
//   IOBUF .I=0 always (we only ever drive LOW)
//   IOBUF .T=~oe (T=1 means Hi-Z, T=0 means drive)
//   So when oe=0 -> T=1 -> Hi-Z (pulled high by external 4.7k resistor)
//      when oe=1 -> T=0 -> drive LOW
//
// Pull-ups: Add 4.7k resistors from SCL and SDA to 3.3V on the PCB,
//           OR enable in XDC: set_property PULLUP true [get_ports scl]
//
// Original source: https://github.com/holla2040/Agentic_Verilog_iCE40_iCEBreaker
//
// ============================================================================

module i2c_master (
    input  wire       clk,           // System clock (50 MHz)

    // I2C bus (directly to package pins)
    inout  wire       scl,           // I2C clock line
    inout  wire       sda,           // I2C data line

    // Command interface
    input  wire [2:0] cmd_i,         // Command to execute
    input  wire [7:0] data_i,        // Data byte to write
    input  wire       ack_i,         // ACK to send on read (0=ACK, 1=NACK)
    input  wire       start_i,       // Start command execution (single pulse)

    output reg  [7:0] data_o,        // Data byte read
    output reg        ack_o,         // ACK received (0=ACK, 1=NACK)
    output reg        busy_o,        // High while executing command

    // Debug ports for ILA
    output wire       dbg_scl_in,    // Actual SCL line state
    output wire       dbg_sda_in,    // Actual SDA line state
    output wire       dbg_scl_oe,    // 1 = master driving SCL low
    output wire       dbg_sda_oe     // 1 = master driving SDA low
);

    // ========================================================================
    // COMMAND DEFINITIONS
    // ========================================================================

    localparam CMD_NONE  = 3'd0;
    localparam CMD_START = 3'd1;
    localparam CMD_STOP  = 3'd2;
    localparam CMD_WRITE = 3'd3;
    localparam CMD_READ  = 3'd4;

    // ========================================================================
    // TIMING PARAMETERS
    // ========================================================================
    //
    // I2C standard mode: 100 kHz
    // At 50 MHz system clock:
    //   Full bit period = 50,000,000 / 100,000 = 500 clocks
    //   Half period     = 250 clocks

    localparam HALF_PERIOD = 250;

    // ========================================================================
    // IOBUF PRIMITIVES FOR OPEN-DRAIN I/O
    // ========================================================================
    //
    // IOBUF ports:
    //   IO = connects to the actual package pin (inout)
    //   O  = output from pin into logic (what is currently on the line)
    //   I  = input to pin from logic (always 0 for open-drain)
    //   T  = tristate control (1=Hi-Z/release, 0=drive)
    //        NOTE: T is active HIGH for Hi-Z (opposite of oe signal)

    reg  scl_oe = 1'b0;
    reg  sda_oe = 1'b0;
    wire scl_in;
    wire sda_in;

    IOBUF scl_iobuf (
        .IO(scl),
        .O(scl_in),
        .I(1'b0),
        .T(~scl_oe)
    );

    IOBUF sda_iobuf (
        .IO(sda),
        .O(sda_in),
        .I(1'b0),
        .T(~sda_oe)
    );

    // Debug port assignments
    assign dbg_scl_in = scl_in;
    assign dbg_sda_in = sda_in;
    assign dbg_scl_oe = scl_oe;
    assign dbg_sda_oe = sda_oe;

    // ========================================================================
    // STATE MACHINE
    // ========================================================================

    localparam S_IDLE        = 4'd0;
    localparam S_START_1     = 4'd1;   // SDA high, SCL high, wait
    localparam S_START_2     = 4'd2;   // SDA falls (START condition)
    localparam S_STOP_1      = 4'd3;   // SCL rises
    localparam S_STOP_2      = 4'd4;   // SDA rises (STOP condition)
    localparam S_STOP_3      = 4'd5;   // Hold
    localparam S_WRITE_BIT   = 4'd6;   // Set SDA, pull SCL low
    localparam S_WRITE_SCL_H = 4'd7;   // Release SCL high, clock in
    localparam S_WRITE_ACK   = 4'd8;   // Release SDA, pull SCL low
    localparam S_WRITE_ACK_H = 4'd9;   // Release SCL, sample ACK
    localparam S_READ_BIT    = 4'd10;  // Release SDA, pull SCL low
    localparam S_READ_SCL_H  = 4'd11;  // Release SCL, sample bit
    localparam S_READ_ACK    = 4'd12;  // Drive ACK/NACK, pull SCL low
    localparam S_READ_ACK_H  = 4'd13;  // Release SCL high

    reg [3:0] state     = S_IDLE;
    reg [8:0] timer     = 9'd0;        // Widened from [6:0] to hold 250
    reg [7:0] shift_reg = 8'd0;
    reg [2:0] bit_count = 3'd0;
    reg       send_ack  = 1'b0;

    // ========================================================================
    // MAIN STATE MACHINE
    // ========================================================================

    always @(posedge clk) begin
        case (state)

            S_IDLE: begin
                busy_o <= 1'b0;
                timer  <= 9'd0;
                scl_oe <= 1'b0;
                sda_oe <= 1'b0;

                if (start_i) begin
                    busy_o <= 1'b1;

                    case (cmd_i)
                        CMD_START: begin
                            sda_oe <= 1'b0;
                            scl_oe <= 1'b0;
                            state  <= S_START_1;
                        end

                        CMD_STOP: begin
                            sda_oe <= 1'b1;
                            scl_oe <= 1'b1;
                            state  <= S_STOP_1;
                        end

                        CMD_WRITE: begin
                            shift_reg <= data_i;
                            bit_count <= 3'd0;
                            state     <= S_WRITE_BIT;
                        end

                        CMD_READ: begin
                            send_ack  <= ack_i;
                            shift_reg <= 8'd0;
                            bit_count <= 3'd0;
                            sda_oe    <= 1'b0;
                            state     <= S_READ_BIT;
                        end

                        default: begin
                            busy_o <= 1'b0;
                        end
                    endcase
                end
            end

            // ----------------------------------------------------------------
            // START CONDITION: SDA falls while SCL HIGH
            // ----------------------------------------------------------------

            S_START_1: begin
                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    sda_oe <= 1'b1;   // Pull SDA low (START)
                    state  <= S_START_2;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            S_START_2: begin
                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    scl_oe <= 1'b1;   // Pull SCL low (ready for data)
                    state  <= S_IDLE;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            // ----------------------------------------------------------------
            // STOP CONDITION: SDA rises while SCL HIGH
            // ----------------------------------------------------------------

            S_STOP_1: begin
                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    scl_oe <= 1'b0;   // Release SCL high
                    state  <= S_STOP_2;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            S_STOP_2: begin
                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    sda_oe <= 1'b0;   // Release SDA high (STOP)
                    state  <= S_STOP_3;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            S_STOP_3: begin
                if (timer == HALF_PERIOD - 1) begin
                    timer <= 9'd0;
                    state <= S_IDLE;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            // ----------------------------------------------------------------
            // WRITE BYTE (MSB first)
            // ----------------------------------------------------------------

            S_WRITE_BIT: begin
                scl_oe <= 1'b1;              // SCL low
                sda_oe <= ~shift_reg[7];      // Drive data bit

                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    scl_oe <= 1'b0;           // Release SCL high
                    state  <= S_WRITE_SCL_H;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            S_WRITE_SCL_H: begin
                if (timer == HALF_PERIOD - 1) begin
                    timer     <= 9'd0;
                    scl_oe    <= 1'b1;        // Pull SCL low
                    shift_reg <= {shift_reg[6:0], 1'b0};

                    if (bit_count == 3'd7) begin
                        state <= S_WRITE_ACK;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                        state     <= S_WRITE_BIT;
                    end
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            S_WRITE_ACK: begin
                sda_oe <= 1'b0;   // Release SDA for slave ACK
                scl_oe <= 1'b1;   // SCL low

                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    scl_oe <= 1'b0;   // Release SCL high
                    state  <= S_WRITE_ACK_H;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            S_WRITE_ACK_H: begin
                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    ack_o  <= sda_in;  // Sample ACK (0=ACK, 1=NACK)
                    scl_oe <= 1'b1;    // Pull SCL low
                    state  <= S_IDLE;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            // ----------------------------------------------------------------
            // READ BYTE (MSB first)
            // ----------------------------------------------------------------

            S_READ_BIT: begin
                scl_oe <= 1'b1;   // SCL low
                sda_oe <= 1'b0;   // Release SDA for slave to drive

                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    scl_oe <= 1'b0;   // Release SCL high
                    state  <= S_READ_SCL_H;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            S_READ_SCL_H: begin
                if (timer == HALF_PERIOD - 1) begin
                    timer     <= 9'd0;
                    shift_reg <= {shift_reg[6:0], sda_in};  // Sample bit
                    scl_oe    <= 1'b1;  // Pull SCL low

                    if (bit_count == 3'd7) begin
                        data_o <= {shift_reg[6:0], sda_in};
                        state  <= S_READ_ACK;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                        state     <= S_READ_BIT;
                    end
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            S_READ_ACK: begin
                scl_oe <= 1'b1;
                sda_oe <= ~send_ack;  // 0=drive ACK low, 1=release for NACK

                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    scl_oe <= 1'b0;   // Release SCL high
                    state  <= S_READ_ACK_H;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            S_READ_ACK_H: begin
                if (timer == HALF_PERIOD - 1) begin
                    timer  <= 9'd0;
                    scl_oe <= 1'b1;
                    sda_oe <= 1'b0;
                    state  <= S_IDLE;
                end else begin
                    timer <= timer + 1'b1;
                end
            end

            default: begin
                state <= S_IDLE;
            end
        endcase
    end

endmodule
