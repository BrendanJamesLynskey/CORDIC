#!/usr/bin/env python3
"""Verify CORDIC LUT values, INIT_X gain constant, and K_h."""
import math

DATA_WIDTH = 16
NUM_ITERATIONS = 14

print("=== ATAN_LUT verification ===")
print("Encoding: round(atan(2^-i) / pi * 2^(DATA_WIDTH-1))")
print()

expected_atan = []
for i in range(NUM_ITERATIONS):
    val = round(math.atan(2**-i) / math.pi * 2**(DATA_WIDTH-1))
    expected_atan.append(val)
    print(f"  ATAN_LUT[{i:2d}] = {val}")

# Hard-coded values from RTL
rtl_atan = [8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 3, 1]

print()
print("Comparison with RTL values:")
for i in range(NUM_ITERATIONS):
    match = "OK" if expected_atan[i] == rtl_atan[i] else f"MISMATCH (RTL={rtl_atan[i]})"
    print(f"  [{i:2d}] expected={expected_atan[i]:6d}  RTL={rtl_atan[i]:6d}  {match}")

print()
print("=== ATANH_LUT verification ===")
print("Encoding: round(atanh(2^-(i+1)) * 2^(DATA_WIDTH-2))")
print()

expected_atanh = []
for i in range(NUM_ITERATIONS):
    val = round(math.atanh(2**(-(i+1))) * 2**(DATA_WIDTH-2))
    expected_atanh.append(val)
    print(f"  ATANH_LUT[{i:2d}] = {val}")

# Hard-coded values from RTL (cordic_hyperbolic_iterative.sv)
rtl_atanh = [9000, 4185, 2059, 1025, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1]

print()
print("Comparison with RTL values:")
for i in range(NUM_ITERATIONS):
    match = "OK" if expected_atanh[i] == rtl_atanh[i] else f"MISMATCH (RTL={rtl_atanh[i]})"
    print(f"  [{i:2d}] expected={expected_atanh[i]:6d}  RTL={rtl_atanh[i]:6d}  {match}")

print()
print("=== INIT_X verification ===")
print("INIT_X = round(K * 2^(DATA_WIDTH-2))")
print("K = product(cos(atan(2^-i))) for i=0..13")
print("CORDIC iterations amplify by 1/K, so initialising with K cancels the gain.")
print()

K = 1.0
for i in range(NUM_ITERATIONS):
    K *= math.cos(math.atan(2**-i))

init_x = round(K * 2**(DATA_WIDTH-2))
print(f"  K      = {K:.10f}")
print(f"  1/K    = {1/K:.10f}")
print(f"  INIT_X = {init_x}")
print(f"  RTL    = 9949")
print(f"  {'OK' if init_x == 9949 else 'MISMATCH'}")

print()
print("=== K_h (hyperbolic gain) verification ===")
# K_h = product of sqrt(1 - 2^(-2i)) for i=1..N, with repeats at 4,13
# For the iteration schedule: i=1..14, with repeats at 4 and 13
K_h = 1.0
for i in range(1, NUM_ITERATIONS + 1):
    K_h *= math.sqrt(1 - 2**(-2*i))
    if i == 4 or i == 13:
        K_h *= math.sqrt(1 - 2**(-2*i))  # repeat

print(f"  K_h (with repeats at 4,13) = {K_h:.10f}")
print(f"  RTL TB uses kh = 0.82816")
print(f"  Difference: {abs(K_h - 0.82816):.6f}")
