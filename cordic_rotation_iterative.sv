`timescale 1ns / 1ps
/*
    cordic_rotation_iterative.sv
    SV module which implements synthesisable iterative CORDIC in rotation mode
    
    Circular rotation mode:
        Given an input angle THETA, computes cos(THETA) and sin(THETA).

    The CORDIC iterations amplify by 1/K where K = product(cos(atan(2^-i)))
    ~ 0.6073. Pre-compensation initialises X to K in fixed-point so that
    outputs directly represent cos and sin in Q1.(DATA_WIDTH-1) format.
    
    Angle representation:
        Signed fixed-point, where the full positive range [0, 2^(DATA_WIDTH-1))
        maps to [0, pi). Negative values map to (-pi, 0).
        The atan LUT stores atan(2^-i) in this format.
    
    Process:
        For i = 0 to NUM_ITERATIONS-1:
            if z_i >= 0:
                x_{i+1} = x_i - (y_i >> i)
                y_{i+1} = y_i + (x_i >> i)
                z_{i+1} = z_i - atan(2^-i)
            else:
                x_{i+1} = x_i + (y_i >> i)
                y_{i+1} = y_i - (x_i >> i)
                z_{i+1} = z_i + atan(2^-i)
    
    Brendan Lynskey 2025
*/

module cordic_rotation_iterative
# (
    parameter DATA_WIDTH      = 16,
    parameter NUM_ITERATIONS  = 14
)
(
    input           SRST,
    input           CLK,
    input           CE,

    input  logic signed [DATA_WIDTH-1:0]  THETA_IN,       // Input angle
    output logic signed [DATA_WIDTH-1:0]  COS_OUT,        // cos(THETA)
    output logic signed [DATA_WIDTH-1:0]  SIN_OUT,        // sin(THETA)

    input  logic    start,
    output logic    done
);

// ============================================================
// Atan lookup table
// ============================================================
// atan(2^-i) in fixed-point angle representation
// Full-scale = pi, so entry = round(atan(2^-i) / pi * 2^(DATA_WIDTH-1))
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
// CORDIC gain pre-compensation
// ============================================================
// K = product(cos(atan(2^-i))) for i=0..NUM_ITERATIONS-1
// For 14 iterations: K ~ 0.60725
// In Q1.(DATA_WIDTH-2) fixed-point: round(0.60725 * 2^(DATA_WIDTH-2))
localparam signed [DATA_WIDTH-1:0] INIT_X = 16'sd9949;   // K in Q1.14

// ============================================================
// Internal registers
// ============================================================
logic signed [DATA_WIDTH+1:0]  x, y, z;    // Extended to prevent overflow
logic [$clog2(NUM_ITERATIONS)-1:0] iter;

// Quadrant pre-rotation flags
logic [1:0] quadrant;

// FSM
enum {S_IDLE, S_PREROTATE, S_ITERATE, S_OUTPUT} state;

always_ff @(posedge CLK) begin
    if (SRST) begin
        state    <= S_IDLE;
        COS_OUT  <= '0;
        SIN_OUT  <= '0;
        done     <= 1'b0;

    end else begin

        if (CE) begin

            unique case(state)

            // On start:
            // Sample angle input, determine quadrant for pre-rotation
            // CORDIC converges over (-pi/2, pi/2); angles outside
            // this range require quadrant mapping
            S_IDLE: begin
                done <= 1'b0;

                if (start) begin
                    // Store original quadrant for post-correction
                    quadrant <= THETA_IN[DATA_WIDTH-1:DATA_WIDTH-2];
                    
                    // Initialise x = K, y = 0, z = input angle
                    x    <= {{2{INIT_X[DATA_WIDTH-1]}}, INIT_X};
                    y    <= '0;
                    z    <= {{2{THETA_IN[DATA_WIDTH-1]}}, THETA_IN};
                    iter <= '0;

                    state <= S_PREROTATE;
                end
            end

            // Pre-rotate into the range (-pi/2, pi/2)
            // If angle is in Q2 or Q3, rotate by +/- pi
            S_PREROTATE: begin
                case (quadrant)
                    2'b01: begin
                        // Q2: angle in (pi/2, pi) — rotate by -pi, negate x and y
                        x <= -x;
                        y <= -y;
                        z <= z - (1 <<< (DATA_WIDTH-1)); // subtract pi
                    end
                    2'b10: begin
                        // Q3: angle in (-pi, -pi/2) — rotate by +pi, negate x and y
                        x <= -x;
                        y <= -y;
                        z <= z + (1 <<< (DATA_WIDTH-1)); // add pi
                    end
                    default: begin
                        // Q1 or Q4: already in range, no adjustment
                    end
                endcase
                state <= S_ITERATE;
            end

            // Main CORDIC iteration loop
            // Direction of rotation determined by sign of residual angle z
            S_ITERATE: begin
                if (z >= 0) begin
                    x <= x - (y >>> iter);
                    y <= y + (x >>> iter);
                    z <= z - {{2{ATAN_LUT[iter][DATA_WIDTH-1]}}, ATAN_LUT[iter]};
                end else begin
                    x <= x + (y >>> iter);
                    y <= y - (x >>> iter);
                    z <= z + {{2{ATAN_LUT[iter][DATA_WIDTH-1]}}, ATAN_LUT[iter]};
                end

                if (iter == NUM_ITERATIONS - 1)
                    state <= S_OUTPUT;
                else
                    iter <= iter + 1;
            end

            // Register outputs
            S_OUTPUT: begin
                state   <= S_IDLE;
                COS_OUT <= x[DATA_WIDTH-1:0];
                SIN_OUT <= y[DATA_WIDTH-1:0];
                done    <= 1'b1;
            end

            endcase

        end // CE
    end // not SRST
end // always_ff

endmodule
