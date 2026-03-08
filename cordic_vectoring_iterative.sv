`timescale 1ns / 1ps
/*
    cordic_vectoring_iterative.sv
    SV module which implements synthesisable iterative CORDIC in vectoring mode
    
    Circular vectoring mode:
        Given inputs X_IN and Y_IN, computes:
            ANGLE_OUT = atan2(Y_IN, X_IN)
            MAG_OUT   = (1/K) * sqrt(X_IN^2 + Y_IN^2)
        where K = product(cos(atan(2^-i))) ~ 0.6073 for N iterations

    The gain 1/K is NOT compensated; the magnitude output includes
    the amplification factor. The caller must multiply by K if an exact
    magnitude is required. This keeps the core minimal.
    
    Process:
        At each step, rotate the vector (x, y) towards the x-axis
        (drive y towards zero), accumulating the rotation angle:
    
        For i = 0 to NUM_ITERATIONS-1:
            if y_i < 0:
                x_{i+1} = x_i - (y_i >> i)
                y_{i+1} = y_i + (x_i >> i)
                z_{i+1} = z_i - atan(2^-i)
            else:
                x_{i+1} = x_i + (y_i >> i)
                y_{i+1} = y_i - (x_i >> i)
                z_{i+1} = z_i + atan(2^-i)
    
    Brendan Lynskey 2025
*/

module cordic_vectoring_iterative
# (
    parameter DATA_WIDTH      = 16,
    parameter NUM_ITERATIONS  = 14
)
(
    input           SRST,
    input           CLK,
    input           CE,

    input  logic signed [DATA_WIDTH-1:0]  X_IN,
    input  logic signed [DATA_WIDTH-1:0]  Y_IN,
    output logic signed [DATA_WIDTH-1:0]  ANGLE_OUT,      // atan2(Y_IN, X_IN)
    output logic signed [DATA_WIDTH-1:0]  MAG_OUT,        // (1/K) * magnitude

    input  logic    start,
    output logic    done
);

// ============================================================
// Atan lookup table (same encoding as rotation module)
// ============================================================
logic signed [DATA_WIDTH-1:0] ATAN_LUT [0:NUM_ITERATIONS-1];

initial begin
    ATAN_LUT[ 0] = 16'sd8192;   // atan(2^0)  = 45.000 deg
    ATAN_LUT[ 1] = 16'sd4836;   // atan(2^-1) = 26.565 deg
    ATAN_LUT[ 2] = 16'sd2555;   // atan(2^-2) = 14.036 deg
    ATAN_LUT[ 3] = 16'sd1297;   // atan(2^-3) =  7.125 deg
    ATAN_LUT[ 4] = 16'sd651;    // atan(2^-4) =  3.576 deg
    ATAN_LUT[ 5] = 16'sd326;    // atan(2^-5) =  1.790 deg
    ATAN_LUT[ 6] = 16'sd163;    // atan(2^-6) =  0.895 deg
    ATAN_LUT[ 7] = 16'sd81;     // atan(2^-7) =  0.448 deg
    ATAN_LUT[ 8] = 16'sd41;     // atan(2^-8) =  0.224 deg
    ATAN_LUT[ 9] = 16'sd20;     // atan(2^-9) =  0.112 deg
    ATAN_LUT[10] = 16'sd10;     // atan(2^-10)=  0.056 deg
    ATAN_LUT[11] = 16'sd5;      // atan(2^-11)=  0.028 deg
    ATAN_LUT[12] = 16'sd3;      // atan(2^-12)=  0.014 deg
    ATAN_LUT[13] = 16'sd1;      // atan(2^-13)=  0.007 deg
end

// ============================================================
// Internal registers
// ============================================================
logic signed [DATA_WIDTH+1:0]  x, y, z;    // Extended for overflow margin
logic [$clog2(NUM_ITERATIONS)-1:0] iter;

// Pre-rotation quadrant tracking
logic x_was_negative;
logic y_was_negative;

// FSM
enum {S_IDLE, S_PREROTATE, S_ITERATE, S_OUTPUT} state;

always_ff @(posedge CLK) begin
    if (SRST) begin
        state     <= S_IDLE;
        ANGLE_OUT <= '0;
        MAG_OUT   <= '0;
        done      <= 1'b0;

    end else begin

        if (CE) begin

            unique case(state)

            // On start:
            // Sample inputs and prepare for iteration
            // CORDIC vectoring converges when the initial vector
            // is in Q1 or Q4 (x >= 0).  If x < 0, reflect into
            // Q1/Q4 and adjust the angle after iteration.
            S_IDLE: begin
                done <= 1'b0;

                if (start) begin
                    x_was_negative <= X_IN[DATA_WIDTH-1];
                    y_was_negative <= Y_IN[DATA_WIDTH-1];

                    if (X_IN[DATA_WIDTH-1]) begin
                        // x negative: reflect through origin (negate both)
                        x <= -{{2{X_IN[DATA_WIDTH-1]}}, X_IN};
                        y <= -{{2{Y_IN[DATA_WIDTH-1]}}, Y_IN};
                    end else begin
                        x <= {{2{X_IN[DATA_WIDTH-1]}}, X_IN};
                        y <= {{2{Y_IN[DATA_WIDTH-1]}}, Y_IN};
                    end

                    z    <= '0;
                    iter <= '0;

                    state <= S_ITERATE;
                end
            end

            // Main CORDIC iteration loop
            // Drive y towards zero; accumulate rotation in z
            S_ITERATE: begin
                if (y[DATA_WIDTH+1]) begin
                    // y < 0: rotate counter-clockwise
                    x <= x - (y >>> iter);
                    y <= y + (x >>> iter);
                    z <= z - {{2{ATAN_LUT[iter][DATA_WIDTH-1]}}, ATAN_LUT[iter]};
                end else begin
                    // y >= 0: rotate clockwise
                    x <= x + (y >>> iter);
                    y <= y - (x >>> iter);
                    z <= z + {{2{ATAN_LUT[iter][DATA_WIDTH-1]}}, ATAN_LUT[iter]};
                end

                if (iter == NUM_ITERATIONS - 1)
                    state <= S_OUTPUT;
                else
                    iter <= iter + 1;
            end

            // Post-correction and output
            // If the original x was negative, the angle must be adjusted
            // by +/- pi depending on the sign of the original y
            S_OUTPUT: begin
                state   <= S_IDLE;
                MAG_OUT <= x[DATA_WIDTH-1:0];

                if (x_was_negative) begin
                    // Adjust angle: if original y >= 0, angle = z + pi; else angle = z - pi
                    if (y_was_negative)
                        ANGLE_OUT <= z[DATA_WIDTH-1:0] - (1 <<< (DATA_WIDTH-1));
                    else
                        ANGLE_OUT <= z[DATA_WIDTH-1:0] + (1 <<< (DATA_WIDTH-1));
                end else begin
                    ANGLE_OUT <= z[DATA_WIDTH-1:0];
                end

                done <= 1'b1;
            end

            endcase

        end // CE
    end // not SRST
end // always_ff

endmodule
