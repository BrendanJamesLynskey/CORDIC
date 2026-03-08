`timescale 1ns / 1ps
/*
    tb_cordic_rotation_pipelined.sv

    SV TB module for cordic_rotation_pipelined.sv

    Verifies pipelined sin/cos output.  Inputs are streamed continuously
    and results collected after the pipeline fill latency.

    Brendan Lynskey 2025
*/


module tb_cordic_rotation_pipelined;

// Parameterise UUT
localparam DATA_WIDTH     = 16;
localparam NUM_ITERATIONS = 14;

// Parameterise TB
localparam TB_ANGLE_STEP  = 64;    // Step size for angle sweep
localparam TB_MAX_ERROR   = 10;    // Max allowed error in LSBs

// Circular buffer depth: enough for pipeline latency + margin
localparam FIFO_DEPTH     = 64;

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
logic signed [DATA_WIDTH-1:0]  tb_theta;
logic                          tb_valid_in;
logic signed [DATA_WIDTH-1:0]  tb_cos;
logic signed [DATA_WIDTH-1:0]  tb_sin;
logic                          tb_valid_out;

int     test_pass_cnt = 0;
int     test_fail_cnt = 0;

// Reference circular buffer to match pipeline latency
int ref_cos_buf [0:FIFO_DEPTH-1];
int ref_sin_buf [0:FIFO_DEPTH-1];
int fifo_wr_ptr = 0;
int fifo_rd_ptr = 0;
int fifo_count  = 0;

// Output checker — runs continuously
int chk_exp_cos, chk_exp_sin;
int chk_err_cos, chk_err_sin;

always @(posedge tb_clk) begin
    if (tb_valid_out && !tb_srst) begin
        if (fifo_count > 0) begin
            chk_exp_cos = ref_cos_buf[fifo_rd_ptr];
            chk_exp_sin = ref_sin_buf[fifo_rd_ptr];
            fifo_rd_ptr = (fifo_rd_ptr + 1) % FIFO_DEPTH;
            fifo_count  = fifo_count - 1;

            chk_err_cos = int'(tb_cos) - chk_exp_cos;
            chk_err_sin = int'(tb_sin) - chk_exp_sin;
            if (chk_err_cos < 0) chk_err_cos = -chk_err_cos;
            if (chk_err_sin < 0) chk_err_sin = -chk_err_sin;

            if (chk_err_cos > TB_MAX_ERROR || chk_err_sin > TB_MAX_ERROR) begin
                $display("***FAIL: cos=%0d (ref=%0d, err=%0d)  sin=%0d (ref=%0d, err=%0d)",
                         tb_cos, chk_exp_cos, chk_err_cos, tb_sin, chk_exp_sin, chk_err_sin);
                test_fail_cnt++;
            end else begin
                test_pass_cnt++;
            end
        end
    end
end

initial begin

    int theta_val;
    real theta_rad, scale;
    int ref_c, ref_s;

    tb_valid_in = 1'b0;
    tb_theta    = '0;

    // Allow reset to complete
    @ (negedge tb_srst);
    for (int i=0; i<5; i++) @ (negedge tb_clk);

    // Stream angle values continuously
    for (theta_val = -(2**(DATA_WIDTH-1)); theta_val < (2**(DATA_WIDTH-1)); theta_val += TB_ANGLE_STEP) begin
        @ (negedge tb_clk);
        tb_theta    = theta_val;
        tb_valid_in = 1'b1;

        // Compute and push expected result into circular buffer
        theta_rad = real'(theta_val) * 3.14159265358979323846 / real'(2**(DATA_WIDTH-1));
        scale = real'(2**(DATA_WIDTH-2));
        ref_c = int'($rtoi($cos(theta_rad) * scale));
        ref_s = int'($rtoi($sin(theta_rad) * scale));

        ref_cos_buf[fifo_wr_ptr] = ref_c;
        ref_sin_buf[fifo_wr_ptr] = ref_s;
        fifo_wr_ptr = (fifo_wr_ptr + 1) % FIFO_DEPTH;
        fifo_count  = fifo_count + 1;
    end

    // Deassert valid and wait for pipeline to drain
    @ (negedge tb_clk);
    tb_valid_in = 1'b0;

    for (int i=0; i < NUM_ITERATIONS + 20; i++) @ (negedge tb_clk);

    // Signal completion
    $display("\n\t***TB completed: %0d passed, %0d failed", test_pass_cnt, test_fail_cnt);
    if (test_fail_cnt > 0)
        $display("\t***FAILURES DETECTED");
    if (fifo_count > 0)
        $display("\t***WARNING: %0d results never emerged from pipeline", fifo_count);
    $stop;

end


cordic_rotation_pipelined #(DATA_WIDTH, NUM_ITERATIONS) u_cordic_rotation_pipelined
(
    .SRST               (tb_srst),
    .CLK                (tb_clk),
    .CE                 (tb_ce),

    .THETA_IN           (tb_theta),
    .DATA_VALID_IN      (tb_valid_in),
    .COS_OUT            (tb_cos),
    .SIN_OUT            (tb_sin),
    .DATA_VALID_OUT     (tb_valid_out)
);



endmodule
