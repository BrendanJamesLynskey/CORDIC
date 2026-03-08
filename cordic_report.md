# CORDIC in Digital Hardware: History, Theory, and Implementation Tradeoffs

---

## 1. Introduction

The CORDIC algorithm (COordinate Rotation DIgital Computer) is a class of shift-and-add methods for computing trigonometric, hyperbolic, logarithmic, and linear functions using only additions, subtractions, bit-shifts, and a small lookup table.  No multiplier is required.  This makes CORDIC uniquely suited to hardware implementations where area, power, or silicon resources are constrained — precisely the contexts where a dedicated multiplier would be too expensive.

CORDIC is not one algorithm but a family: by varying the coordinate system (circular, hyperbolic, linear) and the operating mode (rotation, vectoring), the same iterative structure computes sin, cos, atan2, sinh, cosh, exp, ln, sqrt, multiply, and divide.  This generality, combined with the minimal hardware footprint, has made CORDIC a foundational building block in digital signal processing, communications, navigation, and scientific computing for over six decades.

This report surveys the history, mathematical foundations, convergence properties, and hardware implementation tradeoffs of the CORDIC algorithm family, and documents the design choices in the five SystemVerilog modules provided in this repository.

---

## 2. Historical Context

### 2.1 Volder's Original CORDIC (1959)

Jack E. Volder developed the CORDIC algorithm in 1956–1959 while working at Convair (later General Dynamics) on the navigation computer for the B-58 Hustler bomber.  The problem was to compute real-time trigonometric functions for coordinate conversion in an airborne navigation system, using the binary arithmetic hardware available in late-1950s avionics — essentially adders, shifters, and registers, but no multiplier.

Volder's insight was that an arbitrary rotation of a 2D vector can be decomposed into a sequence of elementary rotations by angles atan(2⁻ⁱ), each of which requires only a shift and an add/subtract.  The direction of each micro-rotation is chosen to drive a residual angle towards zero (rotation mode) or a residual y-component towards zero (vectoring mode).  The algorithm was published in the September 1959 IRE Transactions on Electronic Computers under the title "The CORDIC Trigonometric Computing Technique."

The original hardware implementation was the Athena navigation computer, which used CORDIC for all coordinate transformations.  It operated at a clock rate of approximately 70 kHz and completed a full sin/cos computation in roughly 20 iterations — a fraction of a millisecond, which was adequate for real-time navigation.

### 2.2 Walther's Unified CORDIC (1971)

Volder's original work covered circular rotations (sin, cos, atan).  In 1971, John S. Walther at Hewlett-Packard generalised the algorithm to a unified framework encompassing three coordinate systems:

- **Circular** (m = 1): sin, cos, atan, magnitude, phase
- **Hyperbolic** (m = −1): sinh, cosh, atanh, exp, ln, sqrt
- **Linear** (m = 0): multiply, divide

Walther showed that all three systems share the same iterative structure, differing only in the sign of the cross-term in the x-update and the contents of the lookup table.  This unified view was published in the 1971 Spring Joint Computer Conference proceedings as "A Unified Algorithm for Elementary Functions."

The unified CORDIC became the computational engine of the HP-35 scientific calculator (1972) — the first handheld calculator capable of transcendental functions — and established CORDIC as the standard approach for function evaluation in resource-constrained processors.

### 2.3 CORDIC in Signal Processing and Communications

Through the 1980s and 1990s, CORDIC found extensive use in digital signal processing:

- **Direct Digital Synthesis (DDS):** CORDIC replaced ROM-based sine lookup tables in numerically-controlled oscillators (NCOs) for communications transmitters and receivers, offering continuous phase resolution without the memory cost of a large ROM.
- **FFT Butterfly Twiddle Factors:** CORDIC was used to generate the complex twiddle factors in FFT processors, avoiding the storage of a full sine/cosine table.
- **Radar and Sonar:** Coordinate conversion between Cartesian and polar forms (atan2 and magnitude) for target detection and tracking.
- **FPGA DSP:** As FPGAs became capable of implementing complex signal processing chains, CORDIC became the standard IP block for trigonometric computation in the absence of floating-point units.

Xilinx introduced the CORDIC IP core in its LogiCORE library in the early 2000s, providing a configurable, parameterised CORDIC generator for Virtex and Spartan families.  Altera (now Intel) offered a similar megafunction.  These IP cores remain widely used.

### 2.4 Decline in General-Purpose Processors

In general-purpose processors, CORDIC was largely displaced by polynomial approximation and table-lookup methods for transcendental function evaluation by the mid-1990s.  Once hardware multipliers became cheap (and later, fused multiply-add units became standard), the shift-and-add approach lost its advantage in throughput.  Intel's x87 FPU used CORDIC internally through the Pentium era, but modern x86 implementations use range reduction plus minimax polynomial evaluation.

CORDIC remains dominant, however, in FPGA, ASIC, and embedded DSP contexts where multiplier resources are limited or absent.

---

## 3. Mathematical Foundation

### 3.1 The Generalised CORDIC Iteration

The unified CORDIC iteration operates on a three-component state vector (x, y, z):

```
x_{i+1} = x_i − m · σ_i · y_i · 2^{−s_i}
y_{i+1} = y_i + σ_i · x_i · 2^{−s_i}
z_{i+1} = z_i − σ_i · e_i
```

where:
- **m** selects the coordinate system: m = 1 (circular), m = −1 (hyperbolic), m = 0 (linear)
- **σ_i ∈ {−1, +1}** is the rotation direction, chosen based on the sign of z (rotation mode) or y (vectoring mode)
- **s_i** is the shift amount at iteration i
- **e_i** is the elementary angle from the lookup table

For each coordinate system:

| System | m | s_i | e_i | LUT |
|---|---|---|---|---|
| Circular | 1 | i | atan(2⁻ⁱ) | atan table |
| Hyperbolic | −1 | i+1 | atanh(2⁻⁽ⁱ⁺¹⁾) | atanh table |
| Linear | 0 | i | 2⁻ⁱ | None (power of 2) |

### 3.2 Rotation Mode vs Vectoring Mode

**Rotation mode** drives the residual angle z towards zero:
- σ_i = sign(z_i)
- Application: given an angle, compute sin/cos/sinh/cosh, or perform a multiply

**Vectoring mode** drives the y-component towards zero:
- σ_i = −sign(y_i)
- Application: given a vector, compute its angle and magnitude, or perform a divide

### 3.3 The CORDIC Gain

Each micro-rotation scales the vector by a factor:

```
Circular:    K_i = √(1 + 2^{−2i})
Hyperbolic:  K_i = √(1 − 2^{−2(i+1)})
Linear:      K_i = 1
```

The cumulative gain is the product over all iterations:

```
K_circular   = ∏ cos(atan(2^{−i}))  ≈ 0.6073  (reciprocal 1/K ≈ 1.6468)
K_hyperbolic = ∏ √(1 − 2^{−2(i+1)}) ≈ 0.8282  (reciprocal ≈ 1.2075)
K_linear     = 1
```

The circular gain converges rapidly — after 14 iterations it is stable to the precision of a 16-bit datapath.  The CORDIC iterations amplify the vector by 1/K ≈ 1.6468, so pre-compensation (initialising x to K) cancels this gain.  In vectoring mode the 1/K factor is either corrected post-hoc or absorbed into the system gain budget.

### 3.4 Convergence Range

The total rotation range of the CORDIC algorithm is the sum of all elementary angles:

```
Circular:    |θ_max| = Σ atan(2^{-i}) ≈ 1.7433 rad ≈ 99.88°
Hyperbolic:  |θ_max| = Σ atanh(2^{-(i+1)}) ≈ 1.1182
Linear:      |z_max| = Σ 2^{-i} ≈ 2.0 (approaches 2)
```

For the circular system, the convergence range exceeds π/2 = 1.5708 but does not reach π.  Full-circle coverage requires a quadrant pre-rotation: if the input angle lies outside (−π/2, π/2), reflect the initial vector by 180° and adjust the angle accordingly.

For the hyperbolic system, the convergence range is limited to |θ| < atanh(1) ≈ 1.1182.  Extending beyond this range requires double-rotation or argument reduction techniques.

### 3.5 Convergence Guarantee for Hyperbolic CORDIC

The hyperbolic CORDIC has a well-known convergence defect: unlike the circular case, the sum of elementary angles does not monotonically cover the convergence range unless certain iterations are repeated.  Specifically, at shift index k, if atanh(2⁻ᵏ) > Σ_{j>k} atanh(2⁻ʲ), a gap exists and convergence fails for certain inputs.

Walther showed that repeating iterations at indices 4, 13, 40, 121, ... (the sequence 3k+1 where each term is three times the previous plus one) eliminates all gaps.  This adds a small number of extra iterations (at most 3 for a 16-bit datapath) but guarantees convergence over the full range.

### 3.6 Precision and Error Analysis

CORDIC accumulates two sources of error:

1. **Truncation error** from the arithmetic right shifts (the discarded LSBs at each iteration).
2. **Approximation error** from the finite number of iterations (the residual angle z_N ≠ 0).

For an N-bit datapath with N iterations, both errors are bounded to approximately 1 LSB, giving a total worst-case error of roughly 2–3 LSBs.  This is adequate for most DSP applications.  Higher precision requires either more iterations (with correspondingly wider internal registers) or post-correction techniques such as a final linear interpolation step.

Guard bits (extending the internal registers by 2–4 bits beyond the output width) reduce truncation error and are standard practice.  All modules in this repository use 2 guard bits.

---

## 4. The CORDIC Function Repertoire

The following table summarises the functions computable by the six CORDIC configurations:

| System | Mode | Initialisation | Output |
|---|---|---|---|
| Circular, Rotation | x₀ = K, y₀ = 0, z₀ = θ | x_N = cos(θ), y_N = sin(θ) |
| Circular, Vectoring | x₀ = x, y₀ = y, z₀ = 0 | z_N = atan2(y,x), x_N = (1/K)·√(x²+y²) |
| Hyperbolic, Rotation | x₀ = 1.0, y₀ = 0, z₀ = θ | x_N = K_h·cosh(θ), y_N = K_h·sinh(θ) |
| Hyperbolic, Vectoring | x₀ = x, y₀ = y, z₀ = 0 | z_N = atanh(y/x), x_N = (1/K_h)·√(x²−y²) |
| Linear, Rotation | x₀ = x, y₀ = 0, z₀ = z | y_N = x·z (multiply) |
| Linear, Vectoring | x₀ = x, y₀ = y, z₀ = 0 | z_N = y/x (divide) |

Derived functions:

- **exp(θ)** = cosh(θ) + sinh(θ): sum the x and y outputs of hyperbolic rotation
- **ln(x)**: hyperbolic vectoring with x₀ = x+1, y₀ = x−1 yields z = atanh((x−1)/(x+1)) = ½ ln(x)
- **√x**: hyperbolic vectoring with x₀ = x+¼, y₀ = x−¼ yields x_N = K_h · √(x)

---

## 5. Hardware Architecture Options

### 5.1 Iterative (Bit-Serial)

The simplest architecture reuses a single set of adders and shifters across all iterations.  The state vector (x, y, z) is stored in registers, and an FSM sequences through N iterations plus any pre/post-processing steps.

**Latency:** N + overhead cycles (typically 2–3 for pre-rotation and output registration).
**Area:** Minimal — one adder/subtractor per component (x, y, z), a barrel shifter or MUX-based shifter, and the LUT (typically a small ROM or localparam array).
**Throughput:** One result per N cycles.

This is the architecture used by all iterative modules in this repository.

### 5.2 Pipelined (Unrolled)

Each iteration is instantiated as a separate pipeline stage.  After the pipeline fills, one result emerges per clock cycle.

**Latency:** N + overhead cycles (same as iterative for the first result).
**Area:** N × (area of one iteration) — approximately N times the iterative area.
**Throughput:** One result per clock cycle after fill.

The area cost is justified in applications requiring sustained high throughput: DDS/NCO, FFT twiddle-factor generation, and real-time phased-array beamforming.  The pipelined rotation module in this repository demonstrates this architecture.

### 5.3 Semi-Pipelined (Folded)

A compromise: instantiate K pipeline stages (where K divides N) and iterate N/K times through the folded pipeline.  This gives K× the throughput of the iterative architecture at K× the area.  Not implemented here but straightforward to derive from the pipelined module by adding a recirculation path.

### 5.4 Redundant Arithmetic

The critical path of each CORDIC iteration is the carry-propagate addition.  For high-clock-frequency targets, the adders can be replaced with carry-save or signed-digit arithmetic, eliminating the carry chain.  The carry-save residual is converted to binary only at the final output stage.  This is the same technique used in high-radix SRT dividers and multiplier arrays.

Redundant arithmetic CORDIC can achieve clock rates 2–3× higher than conventional CORDIC at modest area overhead (the carry-save registers are wider but the adders are simpler).  This is primarily relevant to ASIC implementations at advanced process nodes.

### 5.5 Angle Recoding

Analogous to Booth encoding in multiplication, angle recoding techniques reduce the number of non-zero rotation decisions, skipping iterations where σ_i = 0 is beneficial.  The most common approach is the "double rotation" method, which uses micro-rotations of ±2·atan(2⁻ⁱ) to halve the iteration count.  This doubles the hardware per stage but halves the latency.

### 5.6 Word-Serial (Bit-at-a-Time)

For extremely area-constrained designs (e.g., sensor nodes), the additions can be performed serially, one bit per clock cycle.  The total latency becomes N × DATA_WIDTH clocks, which is very high but the area is minimal — a single full adder plus shift registers.  This is rarely used in practice but appears in academic literature as a lower bound on CORDIC area.

---

## 6. PPA Tradeoffs

### 6.1 Latency

| Architecture | Latency | Throughput |
|---|---|---|
| Iterative | N + 2–3 cycles | 1 result / (N+3) cycles |
| Pipelined | N + 2 cycles (first result) | 1 result / cycle |
| Semi-pipelined (K stages) | N/K iterations × K stages | 1 result / (N/K) cycles |
| Redundant-arithmetic iterative | N + 2–3 cycles (higher fmax) | 1 result / (N+3) cycles |
| Word-serial | N × W cycles | 1 result / (N×W) cycles |

### 6.2 Area

The dominant area components of a CORDIC iteration are:

- Two or three adder/subtractors (for x, y, and optionally z updates)
- A barrel shifter or iteration-dependent MUX for the 2⁻ⁱ shift
- The atan/atanh lookup table (typically 14–20 entries of DATA_WIDTH bits)
- Registers for x, y, z (plus guard bits)

For the iterative architecture, the total area is roughly 3×(DATA_WIDTH-bit adder) + LUT + registers.  For a 16-bit datapath this is approximately 500–800 LUTs on a modern FPGA (Xilinx 7-series or Intel Cyclone V), which is smaller than a single DSP48 multiplier block.

The pipelined architecture multiplies this by N (the iteration count).  For 14 iterations at 16 bits, this is roughly 7,000–11,000 LUTs — still modest by modern FPGA standards, but a significant fraction of a small device.

### 6.3 Power

CORDIC's power consumption is dominated by the toggle rate of the adder inputs.  The iterative architecture toggles the same adders N times per computation.  The pipelined architecture toggles N sets of adders once per computation, with the same total switching energy but spread across more logic.  In practice, the pipelined architecture consumes slightly more static power (more transistors) but similar dynamic power per result.

Compared to a multiplier-based trigonometric implementation (polynomial evaluation using DSP blocks), CORDIC typically consumes less power per computation because the shift-and-add operations are simpler than full multiply-accumulate operations.

### 6.4 Timing (Critical Path)

The critical path of one CORDIC iteration is: barrel shift → adder → register.  For a 16-bit datapath, this is typically 3–5 ns on a modern FPGA, supporting clock rates of 200–300 MHz.  The barrel shifter can be replaced with a fixed shift at each pipeline stage (in the pipelined architecture) or a MUX tree (in the iterative architecture), which is faster but less flexible.

### 6.5 Summary Table

| Property | Iterative | Pipelined | Comparison to MUL-based |
|---|---|---|---|
| Latency (16-bit) | ~17 cycles | ~17 cycles (first), 1/cycle after | MUL-based: 4–8 cycles |
| Area (16-bit FPGA) | ~600 LUTs | ~8,000 LUTs | MUL-based: 1–2 DSP48 + ~400 LUTs |
| Fmax | 200–300 MHz | 200–300 MHz | DSP-limited: ~400 MHz |
| Power/result | Low | Low | Medium (DSP switching) |
| Multiplier required | No | No | Yes |
| Function generality | sin/cos/atan2/sinh/cosh/... | sin/cos (one function per instance) | Per-function polynomial |

---

## 7. CORDIC vs Alternatives

### 7.1 ROM Lookup Table

A ROM storing pre-computed sin/cos values indexed by angle is the simplest approach.  For 10-bit angle resolution and 16-bit output, this requires a 1024×16 ROM (2 KB per function).  Latency is 1 clock cycle.

**Advantage:** Minimal latency; trivial implementation.
**Disadvantage:** Memory grows exponentially with angle resolution.  At 16-bit angle resolution, the ROM would be 64K × 16 bits (128 KB) — impractical.  Interpolation (ROM + linear correction) reduces the table size but adds a multiplier.

CORDIC provides arbitrary angle resolution with no ROM growth, making it preferable above ~10–12 bits of angle precision.

### 7.2 Polynomial Approximation

Chebyshev or minimax polynomial evaluation of sin/cos using a hardware multiplier.  A degree-4 polynomial provides ~16-bit accuracy over a reduced range (after argument reduction).

**Advantage:** Low latency (4–6 multiply-accumulate cycles); high throughput if pipelined.
**Disadvantage:** Requires a hardware multiplier (DSP block); separate implementations needed for each function; polynomial coefficient storage.

Polynomial methods dominate when a multiplier is already available and only sin/cos are needed.  CORDIC wins when multiplier resources are scarce, when multiple functions are needed from the same hardware, or when atan2/vectoring is required (polynomial atan2 is awkward).

### 7.3 Hybrid Approaches

Many practical designs combine CORDIC with other methods:
- **Coarse ROM + fine CORDIC:** A small ROM provides a coarse angle lookup; CORDIC refines the residual.  This reduces the CORDIC iteration count (and latency) while keeping the ROM small.
- **CORDIC for angle computation, polynomial for sin/cos:** Use CORDIC vectoring for atan2 and magnitude, but polynomial evaluation for sin/cos where a multiplier is available.

---

## 8. Implementation Notes for This Repository

Five modules are provided, covering the principal CORDIC configurations:

- `cordic_rotation_iterative.sv` — circular rotation, iterative; computes sin/cos with gain pre-compensation.
- `cordic_vectoring_iterative.sv` — circular vectoring, iterative; computes atan2 and magnitude.
- `cordic_rotation_pipelined.sv` — circular rotation, fully pipelined; one sin/cos result per clock.
- `cordic_hyperbolic_iterative.sv` — hyperbolic rotation, iterative; computes sinh/cosh with repeat schedule.
- `cordic_linear_iterative.sv` — linear mode, iterative; computes multiply (rotation) or divide (vectoring).

### 8.1 Angle Encoding

The circular modules encode angles such that the full positive range [0, 2^(DATA_WIDTH−1)) maps to [0, π), and the full negative range maps to (−π, 0].  This is a natural signed fixed-point representation where 1 LSB ≈ π / 2^(DATA_WIDTH−1) radians.

The atan lookup table entries are pre-computed as:
```
ATAN_LUT[i] = round(atan(2^{-i}) / π × 2^{DATA_WIDTH-1})
```

For 16-bit data width:
```
ATAN_LUT[0] = 8192   (atan(1)      = π/4 = 45°)
ATAN_LUT[1] = 4836   (atan(0.5)    ≈ 26.57°)
ATAN_LUT[2] = 2555   (atan(0.25)   ≈ 14.04°)
...
ATAN_LUT[13] = 1     (atan(2^{-13}) ≈ 0.007°)
```

### 8.2 Gain Pre-Compensation

In the rotation modules, the initial x register is set to K in the Q1.(DATA_WIDTH−2) fixed-point format.  The CORDIC iterations amplify by 1/K, so starting with K produces unity-scaled outputs.  For 14 iterations:
```
K ≈ 0.60725
INIT_X = round(K × 2^{DATA_WIDTH-2}) = 9949 (for DATA_WIDTH=16)
```

This ensures that cos(θ) and sin(θ) appear directly at the output without requiring a post-multiply.

### 8.3 Quadrant Pre-Rotation

The convergence range of circular CORDIC is (−π/2, π/2), which covers only half the circle.  For angles outside this range, the module applies a pre-rotation of ±π before entering the CORDIC loop:

- Quadrant 2 (angle ∈ (π/2, π)): negate x₀ and y₀, subtract π from z₀
- Quadrant 3 (angle ∈ (−π, −π/2)): negate x₀ and y₀, add π to z₀

This is equivalent to rotating by 180° and then running CORDIC on the residual.  The pre-rotation costs one pipeline stage (pipelined) or one FSM state (iterative).

### 8.4 Vectoring Mode: Full-Circle atan2

The vectoring module handles all four quadrants by reflecting negative-x inputs through the origin before iteration (making x positive), then adjusting the output angle by ±π based on the original signs of x and y.  This produces the standard atan2(y, x) result over the full range (−π, π].

### 8.5 Hyperbolic Iteration Schedule

The hyperbolic module implements the Walther repeat schedule: iterations at shift indices 4 and 13 are performed twice.  The schedule is pre-computed at elaboration time and stored in `schedule_shift[]` and `schedule_lut[]` arrays, which the FSM indexes sequentially.  For 14 base iterations, the total step count is 16 (14 base + 2 repeats).

### 8.6 Pipelined Architecture Detail

The pipelined module instantiates NUM_ITERATIONS micro-rotation stages using a `generate` loop.  Each stage is a simple registered adder/subtractor pair.  The shift amount is a compile-time constant at each stage (since i is known), so the barrel shifter degenerates to a wiring-only operation — no logic is consumed for the shift in the pipelined case.  This is a significant area advantage over the iterative architecture, where a variable-shift barrel shifter or MUX is required.

A `valid_pipe[]` shift register tracks data validity through the pipeline, and the output is registered one additional cycle for clean timing.  The total pipeline depth is NUM_ITERATIONS + 2 (pre-rotation + N stages + output register).

---

## 9. References

- Volder, J.E. (1959). "The CORDIC Trigonometric Computing Technique." *IRE Transactions on Electronic Computers*, EC-8(3), pp. 330–334.
- Walther, J.S. (1971). "A Unified Algorithm for Elementary Functions." *Proceedings of the Spring Joint Computer Conference*, AFIPS, pp. 379–385.
- Meher, P.K., Valls, J., Juang, T.-B., Sridharan, K., and Maharatna, K. (2009). "50 Years of CORDIC: Algorithms, Architectures, and Applications." *IEEE Transactions on Circuits and Systems I*, 56(9), pp. 1893–1907.
- Andraka, R. (1998). "A Survey of CORDIC Algorithms for FPGA Based Computers." *Proceedings of the ACM/SIGDA International Symposium on FPGAs*, pp. 191–200.
- Hu, Y.H. (1992). "CORDIC-Based VLSI Architectures for Digital Signal Processing." *IEEE Signal Processing Magazine*, 9(3), pp. 16–35.
- Xilinx (2017). *CORDIC v6.0 LogiCORE IP Product Guide* (PG105).
- Ercegovac, M.D. and Lang, T. (2004). *Digital Arithmetic*. Morgan Kaufmann. Chapter 11: CORDIC Algorithm and Implementations.
- Muller, J.-M. (2006). *Elementary Functions: Algorithms and Implementation*. 2nd ed. Birkhäuser. Chapter 6: Shift-and-Add Algorithms.
