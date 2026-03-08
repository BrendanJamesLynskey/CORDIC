`timescale 1ns / 1ps
/*
    cordic_rotation_pipelined.sv
    SV module which implements synthesisable fully-pipelined CORDIC in rotation mode
    
    Each CORDIC iteration is mapped to a dedicated pipeline stage.
    After an initial fill latency of NUM_ITERATIONS + 2 cycles,
    the module produces one sin/cos result per clock cycle.
    
    Architecture:
        Stage 0:          Pre-rotation (quadrant mapping)
        Stages 1..N:      CORDIC micro-rotation (one per iteration)
        Stage N+1:        Output registration
    
    This trades area (N copies of the adder/shifter datapath) for
    throughput (one result per clock).  Appropriate for DSP datapaths
    requiring sustained trigonometric computation, such as NCOs,
    DDS, and digital downconverters.
    
    Interface:
        DATA_VALID_IN strobes once per input sample; DATA_VALID_OUT
        emerges NUM_ITERATIONS + 2 cycles later, aligned with the
        corresponding COS_OUT/SIN_OUT.
    
    Brendan Lynskey 2025
*/

module cordic_rotation_pipelined
# (
    parameter DATA_WIDTH      = 16,
    parameter NUM_ITERATIONS  = 14
)
(
    input           SRST,
    input           CLK,
    input           CE,

    input  logic signed [DATA_WIDTH-1:0]  THETA_IN,
    input  logic                          DATA_VALID_IN,
    output logic signed [DATA_WIDTH-1:0]  COS_OUT,
    output logic signed [DATA_WIDTH-1:0]  SIN_OUT,
    output logic                          DATA_VALID_OUT
);

// ============================================================
// Atan lookup table
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

// CORDIC gain pre-compensation: K in Q1.(DATA_WIDTH-2)
localparam signed [DATA_WIDTH-1:0] INIT_X = 16'sd9949;   // K in Q1.14

// ============================================================
// Pipeline registers
// ============================================================
// Stage 0 is pre-rotation; stages 1..NUM_ITERATIONS are micro-rotations
localparam PIPE_DEPTH = NUM_ITERATIONS + 1;

logic signed [DATA_WIDTH+1:0]  x_pipe [0:PIPE_DEPTH];
logic signed [DATA_WIDTH+1:0]  y_pipe [0:PIPE_DEPTH];
logic signed [DATA_WIDTH+1:0]  z_pipe [0:PIPE_DEPTH];
logic                          valid_pipe [0:PIPE_DEPTH];

// ============================================================
// Stage 0: Input registration and quadrant pre-rotation
// ============================================================
always_ff @(posedge CLK) begin
    if (SRST) begin
        x_pipe[0]     <= '0;
        y_pipe[0]     <= '0;
        z_pipe[0]     <= '0;
        valid_pipe[0] <= 1'b0;
    end else if (CE) begin
        valid_pipe[0] <= DATA_VALID_IN;

        if (DATA_VALID_IN) begin
            // Quadrant pre-rotation: map angle into (-pi/2, pi/2)
            case (THETA_IN[DATA_WIDTH-1:DATA_WIDTH-2])
                2'b01: begin
                    // Q2: negate x,y initial and subtract pi from z
                    x_pipe[0] <= -{{2{INIT_X[DATA_WIDTH-1]}}, INIT_X};
                    y_pipe[0] <= '0;
                    z_pipe[0] <= {{2{THETA_IN[DATA_WIDTH-1]}}, THETA_IN}
                                 - (1 <<< (DATA_WIDTH-1));
                end
                2'b10: begin
                    // Q3: negate x,y initial and add pi to z
                    x_pipe[0] <= -{{2{INIT_X[DATA_WIDTH-1]}}, INIT_X};
                    y_pipe[0] <= '0;
                    z_pipe[0] <= {{2{THETA_IN[DATA_WIDTH-1]}}, THETA_IN}
                                 + (1 <<< (DATA_WIDTH-1));
                end
                default: begin
                    // Q1/Q4: no adjustment
                    x_pipe[0] <= {{2{INIT_X[DATA_WIDTH-1]}}, INIT_X};
                    y_pipe[0] <= '0;
                    z_pipe[0] <= {{2{THETA_IN[DATA_WIDTH-1]}}, THETA_IN};
                end
            endcase
        end else begin
            x_pipe[0] <= '0;
            y_pipe[0] <= '0;
            z_pipe[0] <= '0;
        end
    end
end

// ============================================================
// Stages 1..NUM_ITERATIONS: CORDIC micro-rotations
// ============================================================
genvar i;
generate
    for (i = 0; i < NUM_ITERATIONS; i++) begin : gen_cordic_stage

        always_ff @(posedge CLK) begin
            if (SRST) begin
                x_pipe[i+1]     <= '0;
                y_pipe[i+1]     <= '0;
                z_pipe[i+1]     <= '0;
                valid_pipe[i+1] <= 1'b0;
            end else if (CE) begin
                valid_pipe[i+1] <= valid_pipe[i];

                if (z_pipe[i] >= 0) begin
                    x_pipe[i+1] <= x_pipe[i] - (y_pipe[i] >>> i);
                    y_pipe[i+1] <= y_pipe[i] + (x_pipe[i] >>> i);
                    z_pipe[i+1] <= z_pipe[i] - {{2{ATAN_LUT[i][DATA_WIDTH-1]}}, ATAN_LUT[i]};
                end else begin
                    x_pipe[i+1] <= x_pipe[i] + (y_pipe[i] >>> i);
                    y_pipe[i+1] <= y_pipe[i] - (x_pipe[i] >>> i);
                    z_pipe[i+1] <= z_pipe[i] + {{2{ATAN_LUT[i][DATA_WIDTH-1]}}, ATAN_LUT[i]};
                end
            end
        end

    end
endgenerate

// ============================================================
// Output registration
// ============================================================
always_ff @(posedge CLK) begin
    if (SRST) begin
        COS_OUT        <= '0;
        SIN_OUT        <= '0;
        DATA_VALID_OUT <= 1'b0;
    end else if (CE) begin
        COS_OUT        <= x_pipe[NUM_ITERATIONS][DATA_WIDTH-1:0];
        SIN_OUT        <= y_pipe[NUM_ITERATIONS][DATA_WIDTH-1:0];
        DATA_VALID_OUT <= valid_pipe[NUM_ITERATIONS];
    end
end

endmodule
