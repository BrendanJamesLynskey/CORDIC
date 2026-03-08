`timescale 1ns / 1ps
/*
    cordic_hyperbolic_iterative.sv
    SV module which implements synthesisable iterative CORDIC in hyperbolic mode
    
    Hyperbolic rotation mode:
        Given an input angle THETA (hyperbolic argument), computes:
            X_out = K_h * cosh(THETA)
            Y_out = K_h * sinh(THETA)
        where K_h is the hyperbolic CORDIC gain.
    
    The hyperbolic CORDIC uses the iteration:
        if z_i >= 0:
            x_{i+1} = x_i + (y_i >> i)     (note: + not - for hyperbolic)
            y_{i+1} = y_i + (x_i >> i)
            z_{i+1} = z_i - atanh(2^-i)
        else:
            x_{i+1} = x_i - (y_i >> i)
            y_{i+1} = y_i - (x_i >> i)
            z_{i+1} = z_i + atanh(2^-i)
    
    IMPORTANT: Hyperbolic CORDIC requires iteration index to start at 1
    (not 0), and iterations at indices 4, 13, 40, ... (3k+1) must be
    repeated to guarantee convergence.  This module implements the repeat
    schedule for indices up to the configured iteration count.
    
    Convergence range: |THETA| < atanh(1) ~ 1.1182 (in the natural domain)
    
    Applications:
        - sinh/cosh computation
        - Exponential: exp(x) = cosh(x) + sinh(x)
        - Square root: sqrt(a) via hyperbolic vectoring mode with
          appropriate initialisation (not implemented here; see report)
        - ln(x) via hyperbolic vectoring mode
    
    Brendan Lynskey 2025
*/

module cordic_hyperbolic_iterative
# (
    parameter DATA_WIDTH      = 16,
    parameter NUM_ITERATIONS  = 14      // Iterations before repeat expansion
)
(
    input           SRST,
    input           CLK,
    input           CE,

    input  logic signed [DATA_WIDTH-1:0]  THETA_IN,       // Hyperbolic angle
    output logic signed [DATA_WIDTH-1:0]  COSH_OUT,       // K_h * cosh(THETA)
    output logic signed [DATA_WIDTH-1:0]  SINH_OUT,       // K_h * sinh(THETA)

    input  logic    start,
    output logic    done
);

// ============================================================
// Atanh lookup table
// ============================================================
// atanh(2^-i) in fixed-point, starting from i=1
// Full-scale representation: value * 2^(DATA_WIDTH-2) for Q2.(DATA_WIDTH-2)
// atanh(2^-i) = 0.5 * ln((1+2^-i)/(1-2^-i))
logic signed [DATA_WIDTH-1:0] ATANH_LUT [0:NUM_ITERATIONS-1];

initial begin
    ATANH_LUT[ 0] = 16'sd9000;   // atanh(2^-1) = 0.5493
    ATANH_LUT[ 1] = 16'sd4185;   // atanh(2^-2) = 0.2554
    ATANH_LUT[ 2] = 16'sd2059;   // atanh(2^-3) = 0.1257
    ATANH_LUT[ 3] = 16'sd1025;   // atanh(2^-4) = 0.0626
    ATANH_LUT[ 4] = 16'sd512;    // atanh(2^-5) = 0.0313
    ATANH_LUT[ 5] = 16'sd256;    // atanh(2^-6) = 0.0156
    ATANH_LUT[ 6] = 16'sd128;    // atanh(2^-7) = 0.0078
    ATANH_LUT[ 7] = 16'sd64;     // atanh(2^-8) = 0.0039
    ATANH_LUT[ 8] = 16'sd32;     // atanh(2^-9) = 0.0020
    ATANH_LUT[ 9] = 16'sd16;     // atanh(2^-10)= 0.0010
    ATANH_LUT[10] = 16'sd8;      // atanh(2^-11)= 0.0005
    ATANH_LUT[11] = 16'sd4;      // atanh(2^-12)= 0.0002
    ATANH_LUT[12] = 16'sd2;      // atanh(2^-13)= 0.0001
    ATANH_LUT[13] = 16'sd1;      // atanh(2^-14)= 0.00006
end

// ============================================================
// Iteration schedule
// ============================================================
// Hyperbolic CORDIC: shift index = i+1 (starting from 1)
// Repeat iterations at indices 4, 13, 40, ... (3k+1)
// Total schedule length = NUM_ITERATIONS + number of repeats
localparam MAX_SCHEDULE = NUM_ITERATIONS + 3;   // Up to 3 repeats for 14 iterations

// Shift amounts for each scheduled step (pre-computed at elaboration)
// Index into ATANH_LUT and shift amount for each step
logic [$clog2(NUM_ITERATIONS):0] schedule_shift [0:MAX_SCHEDULE-1];
logic [$clog2(NUM_ITERATIONS):0] schedule_lut   [0:MAX_SCHEDULE-1];
logic [$clog2(MAX_SCHEDULE):0]   total_steps;

// Build schedule at elaboration time
// Note: initial blocks for array init are supported by Vivado/Quartus for synthesis
initial begin
    int step, lut_idx, shift_val;
    int repeat_check;
    
    step = 0;
    lut_idx = 0;
    
    for (int i = 1; i <= NUM_ITERATIONS && step < MAX_SCHEDULE; i++) begin
        schedule_shift[step] = i;
        schedule_lut[step]   = lut_idx;
        step++;
        lut_idx++;
        
        // Check if this index requires a repeat (indices 4, 13, 40, ...)
        repeat_check = i;
        if (repeat_check == 4 || repeat_check == 13) begin
            if (step < MAX_SCHEDULE) begin
                schedule_shift[step] = i;
                schedule_lut[step]   = lut_idx - 1;  // Same LUT entry
                step++;
            end
        end
    end
    
    total_steps = step;
    
    // Fill unused schedule entries with zero
    for (int j = step; j < MAX_SCHEDULE; j++) begin
        schedule_shift[j] = 0;
        schedule_lut[j]   = 0;
    end
end

// ============================================================
// Internal registers
// ============================================================
logic signed [DATA_WIDTH+1:0]  x, y, z;
logic [$clog2(MAX_SCHEDULE):0] step_cnt;

// FSM
enum {S_IDLE, S_ITERATE, S_OUTPUT} state;

always_ff @(posedge CLK) begin
    if (SRST) begin
        state    <= S_IDLE;
        COSH_OUT <= '0;
        SINH_OUT <= '0;
        done     <= 1'b0;

    end else begin

        if (CE) begin

            unique case(state)

            S_IDLE: begin
                done <= 1'b0;

                if (start) begin
                    // Initialise: x = 1.0 (in fixed-point), y = 0, z = input
                    // No gain pre-compensation — caller applies K_h correction
                    x <= (1 <<< (DATA_WIDTH - 2));
                    y <= '0;
                    z <= {{2{THETA_IN[DATA_WIDTH-1]}}, THETA_IN};
                    step_cnt <= '0;

                    state <= S_ITERATE;
                end
            end

            // Hyperbolic micro-rotation
            // Note the sign difference from circular CORDIC:
            //   Circular:    x_{i+1} = x_i -/+ (y_i >> i)
            //   Hyperbolic:  x_{i+1} = x_i +/- (y_i >> i)   (same sign as y update)
            S_ITERATE: begin
                if (z >= 0) begin
                    x <= x + (y >>> schedule_shift[step_cnt]);
                    y <= y + (x >>> schedule_shift[step_cnt]);
                    z <= z - {{2{ATANH_LUT[schedule_lut[step_cnt]][DATA_WIDTH-1]}},
                              ATANH_LUT[schedule_lut[step_cnt]]};
                end else begin
                    x <= x - (y >>> schedule_shift[step_cnt]);
                    y <= y - (x >>> schedule_shift[step_cnt]);
                    z <= z + {{2{ATANH_LUT[schedule_lut[step_cnt]][DATA_WIDTH-1]}},
                              ATANH_LUT[schedule_lut[step_cnt]]};
                end

                if (step_cnt == total_steps - 1)
                    state <= S_OUTPUT;
                else
                    step_cnt <= step_cnt + 1;
            end

            S_OUTPUT: begin
                state    <= S_IDLE;
                COSH_OUT <= x[DATA_WIDTH-1:0];
                SINH_OUT <= y[DATA_WIDTH-1:0];
                done     <= 1'b1;
            end

            endcase

        end // CE
    end // not SRST
end // always_ff

endmodule
