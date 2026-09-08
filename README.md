# Performance Comparison of Takagi-Sugeno PDC and Super-Twisting Algorithms for a 3-DOF Anthropomorphic Manipulator

[![Conference](https://img.shields.io/badge/IEEE-ROPEC_2026-blue.svg)](https://ropec.org/)
[![Python](https://img.shields.io/badge/Python-3.9%2B-brightgreen.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Supplementary repository containing mathematical models, dynamic parameters, and LMI stability verification scripts for the paper:

> **Cesar Eduardo Monterrubio Morales and Juan Anzurez Marin**, *"Performance Comparison of Takagi-Sugeno PDC and Super-Twisting Algorithms for a 3-DOF Anthropomorphic Manipulator"*, in *Proceedings of the 2026 IEEE International Autumn Meeting on Power, Electronics and Computing (ROPEC 2026)*, Morelia, Mexico.

---

## Overview

Due to strict page constraints in the IEEE conference format, explicit representations of the $9 \times 9$ Lyapunov matrices ($P$) and full parametric formulations could not be printed directly in the manuscript. 

This repository ensures the scientific transparency and reproducibility of the comparative study by providing:
1. The optimization scripts solving the Linear Matrix Inequalities (LMIs) for both continuous and discrete PDC implementations.
2. The dynamic model formulations ($M(q)$, $C(q,\dot{q})$, $G(q)$) and physical actuator parameters.
3. The exact tuning gains, observer configurations, and simulation conditions.

---

## Repository Structure

```text
├── README.md                          # Repository overview and documentation
├── lmi_stability_check.py             # Convex optimization script (CVXPY) for PDC stability
├── System_Parameters_and_Matrices.md   # Dynamic matrices, actuator specs, and controller gains
└── requirements.txt                   # Required Python libraries
