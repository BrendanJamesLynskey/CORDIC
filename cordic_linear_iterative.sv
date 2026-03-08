`timescale 1ns / 1ps
/*
    cordic_linear_iterative.sv
    SV module which implements synthesisable iterative CORDIC in linear mode
    
    Linear CORDIC modes:
    
    Rotation mode (MODE_SELECT = 0):
        Computes Y_OUT = Y_IN + X_IN * THETA_IN (fixed-point multiply)
        using only shift-and-add operations — no multiplier required.
        X_OUT passes through unchanged.
    
    Vectoring mode (MODE_SELECT = 1):
        Computes THETA_OUT = THETA_IN + Y_IN / X_IN (fixed-point divide)
        using only shift-and-subtract operations.
        Drives Y towards zero, accumulating the quotient in Z.
    
    The linear CORDIC iteration is:
        Rotation:
            x_{i+1} = x_i                     (x unchanged)
            y_{i+1} = y_i + sigma_i * (x_i >> i)
            z_{i+1} = z_i - sigma_i * 2^-i
            sigma_i = sign(z_i)
        
        Vectoring:
            x_{i+1} = x_i                     (x unchanged)
            y_{i+1} = y_i - sigma_i * (x_i >> i)
            z_{i+1} = z_i + sigma_i * 2^-i
            sigma_i = sign(y_i)  (drives y to zero)
    
    There is no CORDIC gain in linear mode (K_linear = 1).
    No lookup table is required — the angle increments are simply 2^-i.
    
    This is the simplest CORDIC variant.  Its primary application is
    multiplier-free multiply/divide in area-constrained designs.
    
    Brendan Lynskey 2025
*/

module cordic_linear_iterative
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
    input  logic signed [DATA_WIDTH-1:0]  Z_IN,           // Angle/accumulator input
    
    output logic signed [DATA_WIDTH-1:0]  X_OUT,
    output logic signed [DATA_WIDTH-1:0]  Y_OUT,
    output logic signed [DATA_WIDTH-1:0]  Z_OUT,

    input  logic    mode_select,                           // 0 = rotation, 1 = vectoring
    input  logic    start,
    output logic    done
);

// ============================================================
// Internal registers
// ============================================================
logic signed [DATA_WIDTH+1:0]  x, y, z;
logic [$clog2(NUM_ITERATIONS)-1:0] iter;
logic mode_reg;

// FSM
enum {S_IDLE, S_ITERATE, S_OUTPUT} state;

always_ff @(posedge CLK) begin
    if (SRST) begin
        state  <= S_IDLE;
        X_OUT  <= '0;
        Y_OUT  <= '0;
        Z_OUT  <= '0;
        done   <= 1'b0;

    end else begin

        if (CE) begin

            unique case(state)

            S_IDLE: begin
                done <= 1'b0;

                if (start) begin
                    x        <= {{2{X_IN[DATA_WIDTH-1]}}, X_IN};
                    y        <= {{2{Y_IN[DATA_WIDTH-1]}}, Y_IN};
                    z        <= {{2{Z_IN[DATA_WIDTH-1]}}, Z_IN};
                    iter     <= '0;
                    mode_reg <= mode_select;

                    state <= S_ITERATE;
                end
            end

            S_ITERATE: begin
                if (mode_reg == 1'b0) begin
                    // Rotation mode: drive z towards zero
                    if (z >= 0) begin
                        // x unchanged
                        y <= y + (x >>> iter);
                        z <= z - (1 <<< (DATA_WIDTH - 2 - iter));
                    end else begin
                        y <= y - (x >>> iter);
                        z <= z + (1 <<< (DATA_WIDTH - 2 - iter));
                    end
                end else begin
                    // Vectoring mode: drive y towards zero
                    if (y >= 0) begin
                        // x unchanged
                        y <= y - (x >>> iter);
                        z <= z + (1 <<< (DATA_WIDTH - 2 - iter));
                    end else begin
                        y <= y + (x >>> iter);
                        z <= z - (1 <<< (DATA_WIDTH - 2 - iter));
                    end
                end

                if (iter == NUM_ITERATIONS - 1)
                    state <= S_OUTPUT;
                else
                    iter <= iter + 1;
            end

            S_OUTPUT: begin
                state <= S_IDLE;
                X_OUT <= x[DATA_WIDTH-1:0];
                Y_OUT <= y[DATA_WIDTH-1:0];
                Z_OUT <= z[DATA_WIDTH-1:0];
                done  <= 1'b1;
            end

            endcase

        end // CE
    end // not SRST
end // always_ff

endmodule
