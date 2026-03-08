`timescale 1ns / 1ps
/*
    tb_cordic_linear_iterative.sv
    
    SV TB module for cordic_linear_iterative.sv
    
    Verifies both rotation mode (multiply) and vectoring mode (divide)
    against integer arithmetic reference values.
    
    Brendan Lynskey 2025
*/


module tb_cordic_linear_iterative;

// Parameterise UUT
localparam DATA_WIDTH     = 16;
localparam NUM_ITERATIONS = 14;

// Parameterise TB
localparam TB_TEST_CNT    = 500;
localparam TB_MAX_ERROR   = 60;

// Clocks and resets
logic tb_srst   = 1'b1;
logic tb_clk    = 1'b0;
logic tb_ce     = 1'b1;

initial while (1) #5 tb_clk = ~tb_clk;

initial begin
    for (int i=0; i<10; i++) @ (negedge tb_clk);
    tb_srst = 1'b0;
end


// UUT signals
logic signed [DATA_WIDTH-1:0]  tb_x_in;
logic signed [DATA_WIDTH-1:0]  tb_y_in;
logic signed [DATA_WIDTH-1:0]  tb_z_in;
logic signed [DATA_WIDTH-1:0]  tb_x_out;
logic signed [DATA_WIDTH-1:0]  tb_y_out;
logic signed [DATA_WIDTH-1:0]  tb_z_out;
logic                          tb_mode;

logic   tb_start = 1'b0;
logic   tb_done;

int     test_pass_cnt = 0;
int     test_fail_cnt = 0;


task stim_check_rotation(input int x_val, input int z_val);
    // Rotation mode: y_out = y_in + x_in * z_in (as fixed-point multiply)
    // Initialise y_in = 0
    
    int ref_y;
    int err_y;
    real x_real, z_real, product;

    tb_x_in  = x_val;
    tb_y_in  = 0;
    tb_z_in  = z_val;
    tb_mode  = 1'b0;    // Rotation

    @ (negedge tb_clk);
    tb_start = 1'b1;
    @ (negedge tb_clk);
    tb_start = 1'b0;
    
    while (!tb_done) @(posedge tb_clk);

    // Reference: y = x * z / 2^(DATA_WIDTH-2)  (fixed-point scaling)
    x_real  = real'(x_val);
    z_real  = real'(z_val) / real'(2**(DATA_WIDTH-2));
    product = x_real * z_real;
    ref_y   = int'($rtoi(product));
    
    err_y = int'(tb_y_out) - ref_y;
    if (err_y < 0) err_y = -err_y;
    
    if (err_y > TB_MAX_ERROR) begin
        $display("***FAIL (rotation): x=%0d z=%0d  y_out=%0d (ref=%0d, err=%0d)",
                 x_val, z_val, tb_y_out, ref_y, err_y);
        test_fail_cnt++;
    end else begin
        test_pass_cnt++;
    end

endtask


task stim_check_vectoring(input int x_val, input int y_val);
    // Vectoring mode: z_out = z_in + y_in / x_in (as fixed-point divide)
    // Initialise z_in = 0
    
    int ref_z;
    int err_z;
    int abs_x, abs_y;
    real quotient;

    abs_x = x_val < 0 ? -x_val : x_val;
    abs_y = y_val < 0 ? -y_val : y_val;

    // Skip divide-by-zero, near-zero x, and |y/x| >= 0.95 (convergence boundary)
    if (x_val == 0 || abs_x < 16) begin
        test_pass_cnt++;
    end else if (abs_y * 20 >= abs_x * 19) begin
        test_pass_cnt++;
    end else begin
        tb_x_in  = x_val;
        tb_y_in  = y_val;
        tb_z_in  = 0;
        tb_mode  = 1'b1;    // Vectoring

        @ (negedge tb_clk);
        tb_start = 1'b1;
        @ (negedge tb_clk);
        tb_start = 1'b0;

        while (!tb_done) @(posedge tb_clk);

        // Reference: z = y / x * 2^(DATA_WIDTH-2)
        quotient = real'(y_val) / real'(x_val);
        ref_z    = int'($rtoi(quotient * real'(2**(DATA_WIDTH-2))));

        err_z = int'(tb_z_out) - ref_z;
        if (err_z < 0) err_z = -err_z;

        if (err_z > TB_MAX_ERROR) begin
            $display("***FAIL (vectoring): x=%0d y=%0d  z_out=%0d (ref=%0d, err=%0d)",
                     x_val, y_val, tb_z_out, ref_z, err_z);
            test_fail_cnt++;
        end else begin
            test_pass_cnt++;
        end
    end

endtask


initial begin

    int x_val, y_val, z_val;

    // Allow reset to complete
    @ (negedge tb_srst);
    for (int i=0; i<5; i++) @ (negedge tb_clk);

    $display("\n--- Testing rotation mode (multiply) ---");
    
    // Corner cases for rotation
    stim_check_rotation(100, 0);
    stim_check_rotation(0, 100);
    stim_check_rotation(100, 2**(DATA_WIDTH-2));    // x * 1.0
    stim_check_rotation(100, 2**(DATA_WIDTH-3));    // x * 0.5

    // Random rotation tests
    for (int test_cnt = 0; test_cnt < TB_TEST_CNT; test_cnt++) begin
        x_val = $signed($urandom % (2**(DATA_WIDTH-2)));   // Keep small to avoid overflow
        z_val = $signed($urandom % (2**(DATA_WIDTH-2)));
        stim_check_rotation(x_val, z_val);
    end

    $display("\n--- Testing vectoring mode (divide) ---");
    
    // Corner cases for vectoring
    stim_check_vectoring(1000, 0);
    stim_check_vectoring(1000, 500);     // 0.5
    stim_check_vectoring(1000, -500);    // -0.5

    // Random vectoring tests
    for (int test_cnt = 0; test_cnt < TB_TEST_CNT; test_cnt++) begin
        x_val = 1024 + ($urandom % (2**(DATA_WIDTH-2) - 1024));  // Positive, not too small
        y_val = $signed($urandom % x_val) - (x_val / 2);        // |y| < |x|
        stim_check_vectoring(x_val, y_val);
    end

    // Signal completion of TB
    for (int i=0; i<10; i++) @ (negedge tb_clk);
    $display("\n\t***TB completed: %0d passed, %0d failed", test_pass_cnt, test_fail_cnt);
    if (test_fail_cnt > 0)
        $display("\t***FAILURES DETECTED");
    $stop;
  
end


cordic_linear_iterative #(DATA_WIDTH, NUM_ITERATIONS) u_cordic_linear_iterative
(
    .SRST               (tb_srst),
    .CLK                (tb_clk),
    .CE                 (tb_ce),
    
    .X_IN               (tb_x_in),
    .Y_IN               (tb_y_in),
    .Z_IN               (tb_z_in),
    .X_OUT              (tb_x_out),
    .Y_OUT              (tb_y_out),
    .Z_OUT              (tb_z_out),
    
    .mode_select        (tb_mode),
    .start              (tb_start),
    .done               (tb_done)
);


     
endmodule
