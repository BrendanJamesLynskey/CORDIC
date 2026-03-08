`timescale 1ns / 1ps
/*
    tb_cordic_hyperbolic_iterative.sv
    
    SV TB module for cordic_hyperbolic_iterative.sv
    
    Verifies sinh/cosh output against reference values computed from
    real-valued hyperbolic functions.  Tests are restricted to the
    convergence range |theta| < atanh(1) ~ 1.1182.
    
    Brendan Lynskey 2025
*/


module tb_cordic_hyperbolic_iterative;

// Parameterise UUT
localparam DATA_WIDTH     = 16;
localparam NUM_ITERATIONS = 14;

// Parameterise TB
localparam TB_TEST_CNT    = 500;
localparam TB_MAX_ERROR   = 8;     // Wider tolerance for hyperbolic

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
logic signed [DATA_WIDTH-1:0]  tb_cosh;
logic signed [DATA_WIDTH-1:0]  tb_sinh;

logic   tb_start = 1'b0;
logic   tb_done;

int     test_pass_cnt = 0;
int     test_fail_cnt = 0;

// Convergence range in fixed-point
// atanh(1) ~ 1.1182, in Q2.(DATA_WIDTH-2) format:
// max_theta = round(1.1182 * 2^(DATA_WIDTH-2)) = round(1.1182 * 16384) ~ 18320
localparam int MAX_THETA_FP = 18320;


task stim_check_cordic(input int theta_val);

    real theta_real;
    real scale;
    real kh;
    int  ref_cosh_kh, ref_sinh_kh;
    int  err_cosh, err_sinh;

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
    // Angle encoding: theta_real = theta_val / 2^(DATA_WIDTH-2)
    theta_real = real'(theta_val) / real'(2**(DATA_WIDTH-2));
    scale = real'(2**(DATA_WIDTH-2));

    // The output includes K_h factor, so apply K_h to reference
    // K_h for 14 iterations (with repeats at 4,13) ~ 0.82816
    kh = 0.82816;
    ref_cosh_kh = int'($rtoi($cosh(theta_real) * scale * kh));
    ref_sinh_kh = int'($rtoi($sinh(theta_real) * scale * kh));

    err_cosh = int'(tb_cosh) - ref_cosh_kh;
    err_sinh = int'(tb_sinh) - ref_sinh_kh;
    if (err_cosh < 0) err_cosh = -err_cosh;
    if (err_sinh < 0) err_sinh = -err_sinh;

    if (err_cosh > TB_MAX_ERROR || err_sinh > TB_MAX_ERROR) begin
        $display("***FAIL: theta=%0d  cosh=%0d (ref_kh=%0d, err=%0d)  sinh=%0d (ref_kh=%0d, err=%0d)",
                 theta_val, tb_cosh, ref_cosh_kh, err_cosh, tb_sinh, ref_sinh_kh, err_sinh);
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

    // Corner cases
    stim_check_cordic(0);                       // theta = 0
    stim_check_cordic(MAX_THETA_FP / 2);        // Mid-range positive
    stim_check_cordic(-MAX_THETA_FP / 2);       // Mid-range negative
    stim_check_cordic(MAX_THETA_FP - 100);      // Near convergence limit
    stim_check_cordic(-(MAX_THETA_FP - 100));   // Near negative limit

    // Random tests within convergence range
    for (int test_cnt = 0; test_cnt < TB_TEST_CNT; test_cnt++) begin
        theta_val = ($urandom % (2 * MAX_THETA_FP)) - MAX_THETA_FP;
        stim_check_cordic(theta_val);
    end

    // Signal completion of TB
    for (int i=0; i<10; i++) @ (negedge tb_clk);
    $display("\n\t***TB completed: %0d passed, %0d failed", test_pass_cnt, test_fail_cnt);
    if (test_fail_cnt > 0)
        $display("\t***FAILURES DETECTED");
    $stop;
  
end


cordic_hyperbolic_iterative #(DATA_WIDTH, NUM_ITERATIONS) u_cordic_hyperbolic_iterative
(
    .SRST               (tb_srst),
    .CLK                (tb_clk),
    .CE                 (tb_ce),
    
    .THETA_IN           (tb_theta),
    .COSH_OUT           (tb_cosh),
    .SINH_OUT           (tb_sinh),
        
    .start              (tb_start),
    .done               (tb_done)
);


     
endmodule
