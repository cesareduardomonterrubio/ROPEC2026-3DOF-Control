"""
LMI Stability Verification for Takagi-Sugeno PDC (ROPEC 2026)
Verifies global asymptotic stability including cross-terms (i <= j)
for both continuous-time and discrete-time (Ts = 5 ms) implementations.
"""

import cvxpy as cp
import numpy as np
from scipy import signal

def comprobar_estabilidad_LMI_PDC_continuo(TensorA, TensorB, TensorK):
    """
    Solves continuous LMI with cross-terms:
    G_ij^T * P + P * G_ij < 0  for all i <= j
    """
    num_reglas = len(TensorA)
    Cq = np.array([[1, 0, 0, 0, 0, 0],
                   [0, 1, 0, 0, 0, 0],
                   [0, 0, 1, 0, 0, 0]])

    A_aug_list, B_aug_list, K_list = [], [], []

    print("\n[LMI CONTINUOUS] Reconstructing augmented subsystems (9 states)...")
    for i in range(num_reglas):
        A_aug = np.block([
            [TensorA[i], np.zeros((6, 3))],
            [-Cq, np.zeros((3, 3))]
        ])
        B_aug = np.block([
            [TensorB[i]],
            [np.zeros((3, 3))]
        ])
        A_aug_list.append(A_aug)
        B_aug_list.append(B_aug)
        K_list.append(TensorK[i])

    n_estados = A_aug_list[0].shape[0]
    P = cp.Variable((n_estados, n_estados), symmetric=True)
    eps = 1e-5
    restricciones = [P >> np.eye(n_estados) * eps]

    for i in range(num_reglas):
        for j in range(i, num_reglas):
            if i == j:
                A_cl = A_aug_list[i] - B_aug_list[i] @ K_list[i]
                restricciones.append(A_cl.T @ P + P @ A_cl << -np.eye(n_estados) * eps)
            else:
                G_ij = 0.5 * ((A_aug_list[i] - B_aug_list[i] @ K_list[j]) +
                              (A_aug_list[j] - B_aug_list[j] @ K_list[i]))
                restricciones.append(G_ij.T @ P + P @ G_ij << -np.eye(n_estados) * eps)

    problema = cp.Problem(cp.Minimize(0), restricciones)
    problema.solve(solver=cp.SCS)

    if problema.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        eigenvalues = np.linalg.eigvals(P.value)
        print("SUCCESS! Continuous PDC system is globally asymptotically stable.")
        print(f"Min Eigenvalue of P_cont: {np.min(eigenvalues):.4e}")
        return P.value
    else:
        print("FAILED: No common positive definite matrix found. Status:", problema.status)
        return None

def comprobar_estabilidad_LMI_PDC_discreto(TensorA, TensorB, TensorK_d, dt=5e-3):
    """
    Solves discrete LMI with cross-terms:
    G_d,ij^T * P * G_d,ij - P < 0  for all i <= j
    """
    num_reglas = len(TensorA)
    Cq = np.array([[1, 0, 0, 0, 0, 0],
                   [0, 1, 0, 0, 0, 0],
                   [0, 0, 1, 0, 0, 0]])

    Ad_aug_list, Bd_aug_list, Kd_list = [], [], []

    print("\n[LMI DISCRETE] Discretizing and augmenting subsystems (Ts = 5 ms)...")
    for i in range(num_reglas):
        sys_d = signal.cont2discrete((TensorA[i], TensorB[i], np.zeros((1, 6)), np.zeros((1, 3))), dt, method='zoh')
        Ad_aug = np.block([
            [sys_d[0], np.zeros((6, 3))],
            [-dt * Cq, np.eye(3)]
        ])
        Bd_aug = np.block([
            [sys_d[1]],
            [np.zeros((3, 3))]
        ])
        Ad_aug_list.append(Ad_aug)
        Bd_aug_list.append(Bd_aug)
        Kd_list.append(TensorK_d[i])

    n_estados = Ad_aug_list[0].shape[0]
    P = cp.Variable((n_estados, n_estados), symmetric=True)
    eps = 1e-5
    restricciones = [P >> np.eye(n_estados) * eps]

    for i in range(num_reglas):
        for j in range(i, num_reglas):
            if i == j:
                A_cl = Ad_aug_list[i] - Bd_aug_list[i] @ Kd_list[i]
                restricciones.append(A_cl.T @ P @ A_cl - P << -np.eye(n_estados) * eps)
            else:
                G_ij = 0.5 * ((Ad_aug_list[i] - Bd_aug_list[i] @ Kd_list[j]) +
                              (Ad_aug_list[j] - Bd_aug_list[j] @ Kd_list[i]))
                restricciones.append(G_ij.T @ P @ G_ij - P << -np.eye(n_estados) * eps)

    problema = cp.Problem(cp.Minimize(0), restricciones)
    problema.solve(solver=cp.SCS)

    if problema.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        eigenvalues = np.linalg.eigvals(P.value)
        print("SUCCESS! Discrete PDC system is globally asymptotically stable.")
        print(f"Min Eigenvalue of P_disc: {np.min(eigenvalues):.4e}")
        return P.value
    else:
        print("FAILED: No common discrete P matrix found. Status:", problema.status)
        return None

if __name__ == "__main__":
    print("Loading tensors from .npz files...")
    datos_modelo = np.load("Modelo_difuso_3R25reglas.npz")
    datos_ganancias = np.load("Control_and_Observer_Gains.npz")

    TensorA = datos_modelo['A']
    TensorB = datos_modelo['B']
    TensorK = datos_ganancias['TensorK_continuo']
    TensorK_d = datos_ganancias['TensorK_discreto']

    P_cont = comprobar_estabilidad_LMI_PDC_continuo(TensorA, TensorB, TensorK)
    P_disc = comprobar_estabilidad_LMI_PDC_discreto(TensorA, TensorB, TensorK_d, dt=5e-3)
