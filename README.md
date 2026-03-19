# CORDIC

Synthesisable SystemVerilog implementations of the CORDIC (COordinate Rotation DIgital Computer) algorithm, with self-checking testbenches.

Five modules are provided, covering the three CORDIC coordinate systems (circular, hyperbolic, linear) and the two operating modes (rotation and vectoring), plus a fully-pipelined variant for high-throughput applications.

---

## Implementations

| Module | Coordinate System | Mode | Function | Latency |
|---|---|---|---|---|
| `cordic_rotation_iterative` | Circular | Rotation | sin, cos | N + 3 cycles |
| `cordic_vectoring_iterative` | Circular | Vectoring | atan2, magnitude | N + 2 cycles |
| `cordic_rotation_pipelined` | Circular | Rotation | sin, cos | N + 2 cycles (pipelined) |
| `cordic_hyperbolic_iterative` | Hyperbolic | Rotation | sinh, cosh | N + repeats + 2 cycles |
| `cordic_linear_iterative` | Linear | Both | multiply, divide | N + 2 cycles |

All iterative modules share a common interface pattern (CLK, SRST, CE, start/done handshake) and are parameterised by data width `DATA_WIDTH` and iteration count `NUM_ITERATIONS`.  The pipelined module replaces the start/done handshake with DATA_VALID_IN/DATA_VALID_OUT flow control and produces one result per clock after the pipeline fills.

---

## File Structure

```
CORDIC/
├── cordic_rotation_iterative.sv            # Circular rotation (sin/cos), iterative
├── cordic_vectoring_iterative.sv           # Circular vectoring (atan2/mag), iterative
├── cordic_rotation_pipelined.sv            # Circular rotation (sin/cos), pipelined
├── cordic_hyperbolic_iterative.sv          # Hyperbolic rotation (sinh/cosh), iterative
├── cordic_linear_iterative.sv              # Linear mode (multiply/divide), iterative
├── tb_cordic_rotation_iterative.sv         # Testbench — circular rotation
├── tb_cordic_vectoring_iterative.sv        # Testbench — circular vectoring
├── tb_cordic_rotation_pipelined.sv         # Testbench — pipelined rotation
├── tb_cordic_hyperbolic_iterative.sv       # Testbench — hyperbolic
├── tb_cordic_linear_iterative.sv           # Testbench — linear
├── cordic_report.md                        # Technical report
└── README.md
```

---

## Simulation

Testbenches are compatible with **Icarus Verilog** (`iverilog -g2012`).  All TBs use `$stop` (enter `finish` at the interactive prompt, or pipe `printf 'finish\n' | vvp ...`).

```bash
# Circular rotation (iterative)
iverilog -g2012 -o sim_rot_iter tb_cordic_rotation_iterative.sv cordic_rotation_iterative.sv
vvp sim_rot_iter

# Circular vectoring (iterative)
iverilog -g2012 -o sim_vec_iter tb_cordic_vectoring_iterative.sv cordic_vectoring_iterative.sv
vvp sim_vec_iter

# Circular rotation (pipelined)
iverilog -g2012 -o sim_rot_pipe tb_cordic_rotation_pipelined.sv cordic_rotation_pipelined.sv
vvp sim_rot_pipe

# Hyperbolic (iterative)
iverilog -g2012 -o sim_hyp tb_cordic_hyperbolic_iterative.sv cordic_hyperbolic_iterative.sv
vvp sim_hyp

# Linear (iterative)
iverilog -g2012 -o sim_linear tb_cordic_linear_iterative.sv cordic_linear_iterative.sv
vvp sim_linear
```

The circular rotation testbench defaults to an exhaustive sweep across the full angle range at 16-bit width (`TB_TEST_CNT = 0`).  The vectoring, hyperbolic, and linear testbenches use corner cases plus random trials (`TB_TEST_CNT = 500`), since exhaustive 2D input sweeps are impractical at 16-bit width.

---

## Algorithm Summary

### Circular Rotation Mode (sin/cos)

Given an input angle θ, iteratively rotate a unit vector by successively smaller atan(2⁻ⁱ) micro-rotations until the residual angle is driven to zero.  The x and y components of the final vector are cos(θ) and sin(θ), scaled by the CORDIC gain K ≈ 0.6073 (pre-compensated in the initial x value).  Quadrant pre-rotation maps arbitrary angles into the convergence range (−π/2, π/2).

### Circular Vectoring Mode (atan2/magnitude)

Given an input vector (x, y), iteratively rotate it towards the positive x-axis by driving y to zero.  The accumulated rotation angle is atan2(y, x).  The final x value is (1/K) × √(x² + y²).  The initial vector is reflected into Q1/Q4 if x is negative, with a post-correction of ±π on the output angle.

### Pipelined Rotation Mode

Identical algorithm to the iterative rotation module, but each micro-rotation is mapped to a dedicated pipeline stage.  After an initial fill latency of NUM_ITERATIONS + 2 cycles, the module sustains one sin/cos result per clock cycle.  Trades NUM_ITERATIONS copies of the adder/shifter datapath for throughput — appropriate for NCOs, DDS, and digital downconverters where sustained trigonometric throughput is required.

### Hyperbolic Rotation Mode (sinh/cosh)

The hyperbolic variant replaces the circular micro-rotation with the identity x' = x ± y·2⁻ⁱ, y' = y ± x·2⁻ⁱ (same sign on both updates, unlike circular).  The lookup table stores atanh(2⁻ⁱ) instead of atan(2⁻ⁱ), and iteration indices start at 1 (not 0).  Convergence requires repeating certain iterations (at indices 4, 13, 40, ...) — this schedule is hard-coded in the module.  The convergence range is |θ| < atanh(1) ≈ 1.1182.  Applications include sinh/cosh, exp(x) = cosh(x) + sinh(x), and (via vectoring mode) ln(x) and √x.

### Linear Mode (multiply/divide)

The simplest CORDIC variant — no lookup table, no gain compensation, no trigonometric functions.  In rotation mode, computes y_out = y_in + x_in × z_in using only shifts and adds (multiplier-free multiply).  In vectoring mode, computes z_out = z_in + y_in / x_in using only shifts and subtracts (divider-free divide).  The angle increments are simply 2⁻ⁱ.  Primary application is area-constrained designs where neither a hardware multiplier nor a divider is available.

---

## CORDIC Gain

The circular CORDIC gain K = ∏ cos(atan(2⁻ⁱ)) ≈ 0.60725 (for ≥14 iterations) is a constant that scales the output magnitude.  In rotation mode, the initial x register is set to K so that the CORDIC gain (1/K) cancels, producing outputs directly in the desired fixed-point format.  In vectoring mode, the magnitude output is scaled by 1/K ≈ 1.6468; the caller may multiply by K if an exact magnitude is required, or absorb the gain into the system budget.

The hyperbolic gain K_h = ∏ √(1 − 2⁻²ⁱ) ≈ 0.828 is similarly constant and can be pre-compensated or absorbed.

The linear mode has no gain (K_linear = 1).

---

## Fixed-Point Format

All modules use signed fixed-point arithmetic.  The circular modules use an angle encoding where the full positive range [0, 2^(DATA_WIDTH−1)) maps to [0, π), with negative values mapping to (−π, 0).  Data outputs are in Q1.(DATA_WIDTH−2) format.  The linear module uses Q2.(DATA_WIDTH−2) for the z (angle/accumulator) path.

The atan and atanh lookup tables are hard-coded arrays initialised in `initial` blocks for 16-bit data width.  For other widths, the LUT values should be regenerated — a Python script or spreadsheet formula suffices: `round(atan(2^-i) / π × 2^(DATA_WIDTH-1))`.

---

## Synthesis Results

Target: Xilinx Artix-7 (xc7a35tcpg236-1) | Tool: Vivado 2025.2

| Module | LUTs | FFs | BRAM | DSP | Fmax (MHz) |
|--------|------|-----|------|-----|------------|
| cordic_rotation_iterative | 170 | 95 | 0 | 0 | 198.3 |
| cordic_vectoring_iterative | 180 | 92 | 0 | 0 | 170.7 |
| cordic_linear_iterative | 136 | 109 | 0 | 0 | 194.4 |
| cordic_hyperbolic_iterative | 131 | 95 | 0 | 0 | 176.7 |
| cordic_rotation_pipelined | 745 | 784 | 0 | 0 | 272.9 |

*Auto-generated by Vivado batch synthesis. Clock target: 100 MHz. Default parameterisation (16-bit data width).*
