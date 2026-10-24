"""
NumPy reference (golden) SVD used to validate the FPGA Jacobi SVD accelerator.

Run:
    python numpy_reference.py

Compares against the same 3x3 test matrix used in the FPGA testbench
(tb/tb_jacobi_svd_3x3.sv) and reports reconstruction error.
"""

import numpy as np

A = np.array([
    [3, 1, 2],
    [2, 6, 4],
    [1, 5, 9]
], dtype=float)

U, S, Vt = np.linalg.svd(A)

print("U =")
print(U)

print("\nSigma =")
print(S)

print("\nVt =")
print(Vt)

A_rec = U @ np.diag(S) @ Vt
print("\nReconstructed Matrix")
print(A_rec)

print("\nMaximum Error")
print(np.max(np.abs(A - A_rec)))

print("\nRMSE")
print(np.sqrt(np.mean((A - A_rec) ** 2)))
