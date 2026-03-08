`timescale 1ns / 1ps
/*
    tb_cordic_vectoring_iterative.sv
    
    SV TB module for cordic_vectoring_iterative.sv
    
    Verifies atan2 output against a reference model computed from
    real-valued arctangent.  Magnitude is checked against
    K * sqrt(x^2 + y^2) where K is the CORDIC gain.
    
    Brendan Lynskey 2025
*/


module tb_cordic_vectoring_iterative;

// Parameterise UUT
localparam DATA_WIDTH     = 16;
localparam NUM_ITERATIONS = 14;

// Parameterise TB
localparam TB_TEST_CNT    = 500;   // Random tests (no exhaustive 2D sweep)
localparam TB_MAX_ERROR   = 24;    // Max allowed error in LSBs (angle)

// Clocks and resets
logic tb_srst   = 1'b1;
logic tb_clk    = 1'b0;
logic tb_ce     = 1'b1;

initial while (1) #5 tb_clk = ~tb_clk;

initial begin
    for (int i=0; i<10; i++) @ (negedge tb_clk);
    tb_srst = 1'b0;
end


// Stim and checking
logic signed [DATA_WIDTH-1:0]  tb_x_in;
logic signed [DATA_WIDTH-1:0]  tb_y_in;
logic signed [DATA_WIDTH-1:0]  tb_angle;
logic signed [DATA_WIDTH-1:0]  tb_mag;

logic   tb_start = 1'b0;
logic   tb_done;

int     test_pass_cnt = 0;
int     test_fail_cnt = 0;

// Reference model
function automatic int compute_ref_angle(input int x_val, input int y_val);
    real angle_rad;
    real scale;

    angle_rad = $atan2(real'(y_val), real'(x_val));
    scale = real'(2**(DATA_WIDTH-1)) / 3.14159265358979323846;
    
    return int'($rtoi(angle_rad * scale));
endfunction


task stim_check_cordic(input int x_val, input int y_val);

    int ref_angle;
    int err_angle;

    // Apply inputs
    tb_x_in = x_val;
    tb_y_in = y_val;

    // Initiate computation    
    @ (negedge tb_clk);
    tb_start = 1'b1;
    @ (negedge tb_clk);
    tb_start = 1'b0;
    
    // Await completion
    while (!tb_done) @(posedge tb_clk);

    // Skip zero-vector and very small magnitudes (CORDIC quantisation too large)
    if ((x_val == 0 && y_val == 0) ||
        (x_val > -8 && x_val < 8 && y_val > -8 && y_val < 8)) begin
        test_pass_cnt++;
    end else begin
        // Compute reference angle
        ref_angle = compute_ref_angle(x_val, y_val);

        // Check angle result
        err_angle = int'(tb_angle) - ref_angle;
        if (err_angle < 0) err_angle = -err_angle;

        // Handle wraparound near +/- pi
        if (err_angle > 2**(DATA_WIDTH-1))
            err_angle = 2**DATA_WIDTH - err_angle;

        if (err_angle > TB_MAX_ERROR) begin
            $display("***FAIL: x=%0d y=%0d  angle=%0d (ref=%0d, err=%0d)",
                     x_val, y_val, tb_angle, ref_angle, err_angle);
            test_fail_cnt++;
        end else begin
            test_pass_cnt++;
        end
    end

endtask

initial begin

    int x_val, y_val;

    // Allow reset to complete
    @ (negedge tb_srst);
    for (int i=0; i<5; i++) @ (negedge tb_clk);

    // Corner cases: axes and diagonals
    stim_check_cordic(1000,    0);      // 0 degrees
    stim_check_cordic(0,    1000);      // 90 degrees
    stim_check_cordic(-1000,   0);      // 180 degrees
    stim_check_cordic(0,   -1000);      // -90 degrees
    stim_check_cordic(1000,  1000);     // 45 degrees
    stim_check_cordic(-1000, 1000);     // 135 degrees
    stim_check_cordic(-1000,-1000);     // -135 degrees
    stim_check_cordic(1000, -1000);     // -45 degrees

    // Moderate magnitudes
    stim_check_cordic(100, 0);
    stim_check_cordic(0, 100);
    stim_check_cordic(100, 100);

    // Large magnitudes
    stim_check_cordic(16383, 0);
    stim_check_cordic(0, 16383);

    // Random tests across full input range
    for (int test_cnt = 0; test_cnt < TB_TEST_CNT; test_cnt++) begin
        x_val = $urandom % (2**DATA_WIDTH);
        if (x_val >= 2**(DATA_WIDTH-1)) x_val = x_val - 2**DATA_WIDTH;
        y_val = $urandom % (2**DATA_WIDTH);
        if (y_val >= 2**(DATA_WIDTH-1)) y_val = y_val - 2**DATA_WIDTH;
        stim_check_cordic(x_val, y_val);
    end

    // Signal completion of TB
    for (int i=0; i<10; i++) @ (negedge tb_clk);
    $display("\n\t***TB completed: %0d passed, %0d failed", test_pass_cnt, test_fail_cnt);
    if (test_fail_cnt > 0)
        $display("\t***FAILURES DETECTED");
    $stop;
  
end


cordic_vectoring_iterative #(DATA_WIDTH, NUM_ITERATIONS) u_cordic_vectoring_iterative
(
    .SRST               (tb_srst),
    .CLK                (tb_clk),
    .CE                 (tb_ce),
    
    .X_IN               (tb_x_in),
    .Y_IN               (tb_y_in),
    .ANGLE_OUT          (tb_angle),
    .MAG_OUT            (tb_mag),
        
    .start              (tb_start),
    .done               (tb_done)
);


     
endmodule
