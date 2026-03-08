`timescale 1ns / 1ps
/*
    tb_cordic_rotation_iterative.sv
    
    SV TB module for cordic_rotation_iterative.sv
    
    Verifies sin/cos output against a reference model computed from
    real-valued trigonometric functions.  Error tolerance accounts for
    fixed-point quantisation and CORDIC approximation.
    
    Brendan Lynskey 2025
*/


module tb_cordic_rotation_iterative;

// Parameterise UUT
localparam DATA_WIDTH     = 16;
localparam NUM_ITERATIONS = 14;

// Parameterise TB
localparam TB_TEST_CNT    = 0;     // Zero for exhaustive angle sweep
localparam TB_ANGLE_STEP  = 16;    // Step size for exhaustive sweep (smaller = more tests)
localparam TB_MAX_ERROR   = 10;    // Max allowed error in LSBs

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
logic signed [DATA_WIDTH-1:0]  tb_theta;
logic signed [DATA_WIDTH-1:0]  tb_cos;
logic signed [DATA_WIDTH-1:0]  tb_sin;

logic   tb_start = 1'b0;
logic   tb_done;

int     test_pass_cnt = 0;
int     test_fail_cnt = 0;

task stim_check_cordic(input int theta_val);

    real theta_rad;
    real scale;
    int ref_cos, ref_sin;
    int err_cos, err_sin;

    // Apply angle
    tb_theta = theta_val;

    // Initiate computation
    @ (negedge tb_clk);
    tb_start = 1'b1;
    @ (negedge tb_clk);
    tb_start = 1'b0;

    // Await completion
    while (!tb_done) @(posedge tb_clk);

    // Compute reference
    theta_rad = real'(theta_val) * 3.14159265358979323846 / real'(2**(DATA_WIDTH-1));
    scale = real'(2**(DATA_WIDTH-2));
    ref_cos = int'($rtoi($cos(theta_rad) * scale));
    ref_sin = int'($rtoi($sin(theta_rad) * scale));

    // Check results
    err_cos = int'(tb_cos) - ref_cos;
    err_sin = int'(tb_sin) - ref_sin;

    if (err_cos < 0) err_cos = -err_cos;
    if (err_sin < 0) err_sin = -err_sin;

    if (err_cos > TB_MAX_ERROR || err_sin > TB_MAX_ERROR) begin
        $display("***FAIL: theta=%0d  cos=%0d (ref=%0d, err=%0d)  sin=%0d (ref=%0d, err=%0d)",
                 theta_val, tb_cos, ref_cos, err_cos, tb_sin, ref_sin, err_sin);
        test_fail_cnt++;
    end else begin
        test_pass_cnt++;
    end

endtask

initial begin

    int theta_val;

    // Allow reset to complete
    @ (negedge tb_srst);
    for (int i=0; i<5; i++) @ (negedge tb_clk);

    if (TB_TEST_CNT == 0) begin
        // Exhaustive sweep across full angle range
        for (theta_val = -(2**(DATA_WIDTH-1)); theta_val < (2**(DATA_WIDTH-1)); theta_val += TB_ANGLE_STEP)
            stim_check_cordic(theta_val);
            
    end else begin
        // Corner cases
        stim_check_cordic(0);                           // 0 degrees
        stim_check_cordic(2**(DATA_WIDTH-2));           // 45 degrees (pi/4)
        stim_check_cordic(2**(DATA_WIDTH-1) - 1);      // ~180 degrees
        stim_check_cordic(-(2**(DATA_WIDTH-1)));        // -180 degrees
        stim_check_cordic(-(2**(DATA_WIDTH-2)));        // -45 degrees
        // Quadrant boundary straddling (Issue 3: ±90° transitions)
        stim_check_cordic(2**(DATA_WIDTH-2) + 1);      // just above 45 deg
        stim_check_cordic(2**(DATA_WIDTH-2) - 1);      // just below 45 deg
        stim_check_cordic(2**(DATA_WIDTH-1) / 2);      // 90 deg (Q1/Q2 boundary)
        stim_check_cordic(2**(DATA_WIDTH-1) / 2 + 1);  // just above 90 deg
        stim_check_cordic(2**(DATA_WIDTH-1) / 2 - 1);  // just below 90 deg
        stim_check_cordic(-(2**(DATA_WIDTH-1) / 2));    // -90 deg
        stim_check_cordic(-(2**(DATA_WIDTH-1) / 2) + 1);// just above -90 deg
        stim_check_cordic(-(2**(DATA_WIDTH-1) / 2) - 1);// just below -90 deg

        // Random tests
        for (int test_cnt = 0; test_cnt < TB_TEST_CNT; test_cnt++) begin
            theta_val = $signed($urandom % (2**DATA_WIDTH));
            stim_check_cordic(theta_val);
        end        
    end

    // Signal completion of TB
    for (int i=0; i<10; i++) @ (negedge tb_clk);
    $display("\n\t***TB completed: %0d passed, %0d failed", test_pass_cnt, test_fail_cnt);
    if (test_fail_cnt > 0)
        $display("\t***FAILURES DETECTED");
    $stop;
  
end


cordic_rotation_iterative #(DATA_WIDTH, NUM_ITERATIONS) u_cordic_rotation_iterative
(
    .SRST               (tb_srst),
    .CLK                (tb_clk),
    .CE                 (tb_ce),
    
    .THETA_IN           (tb_theta),
    .COS_OUT            (tb_cos),
    .SIN_OUT            (tb_sin),
        
    .start              (tb_start),
    .done               (tb_done)
);


     
endmodule
