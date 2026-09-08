# Manipulator Dynamics and Physical Parameters

## 1. Identified Physical and Actuator Parameters

### Manipulator Geometrical Parameters
* **Link Lengths:** $l_1 = 0.176\text{ m}$, $l_2 = 0.150\text{ m}$, $l_3 = 0.176\text{ m}$

### Actuator Electromechanical Parameters (OLS Identification)
* **Armature Resistances:**
  * Joint 1: $R_2 = 17.0392\ \Omega$
  * Joint 2: $R_1 = 17.7635\ \Omega$
  * Joint 3: $R_2 = 17.0392\ \Omega$
* **Torque and Back-EMF Constants:**
  * Joint 1: $K_2 = 0.7034\text{ V}\cdot\text{s/rad}$
  * Joint 2: $K_1 = 1.7357\text{ V}\cdot\text{s/rad}$
  * Joint 3: $K_2 = 0.7034\text{ V}\cdot\text{s/rad}$
* **Rotor Inertias:**
  * Joint 1: $J_2 = 0.875 \times 10^{-3}\text{ kg}\cdot\text{m}^2$
  * Joint 2: $J_1 = 4.362 \times 10^{-3}\text{ kg}\cdot\text{m}^2$
  * Joint 3: $J_2 = 0.875 \times 10^{-3}\text{ kg}\cdot\text{m}^2$
* **Viscous Friction Coefficients:**
  * Joint 1: $b_2 = 16.805 \times 10^{-3}\text{ N}\cdot\text{m}\cdot\text{s}$
  * Joint 2: $b_1 = 37.366 \times 10^{-3}\text{ N}\cdot\text{m}\cdot\text{s}$
  * Joint 3: $b_2 = 16.805 \times 10^{-3}\text{ N}\cdot\text{m}\cdot\text{s}$
* **Coulomb Friction Magnitudes:**
  * Joint 1: $T_{c2} = 30.058 \times 10^{-3}\text{ N}\cdot\text{m}$
  * Joint 2: $T_{c1} = 55.317 \times 10^{-3}\text{ N}\cdot\text{m}$
  * Joint 3: $T_{c2} = 30.058 \times 10^{-3}\text{ N}\cdot\text{m}$
* **Motor Armature Inductances (Full Electromechanical Model):**
  * Joint 1: $L_2 = 132.9145 \times 10^{-3}\text{ H}$
  * Joint 2: $L_1 = 168.544 \times 10^{-3}\text{ H}$
  * Joint 3: $L_2 = 132.9145 \times 10^{-3}\text{ H}$

---

## 2. Dynamic Model Formulations

The non-reduced electromechanical equations governing the system are defined by:
$$M_t(q)\ddot{q} + C(q,\dot{q})\dot{q} + B_{eff}\dot{q} + G(q) = \tau$$
where $M_t(q) = M(q) + J_m$.

### A. Total Inertia Matrix $M_t(q) \in \mathbb{R}^{3 \times 3}$
$$M_t(q) = \begin{bmatrix} M_{00} + J_2 & M_{01} & M_{02} \\ M_{01} & M_{11} + J_1 & M_{12} \\ M_{02} & M_{12} & M_{22} + J_2 \end{bmatrix}$$

* $M_{00}(q_2, q_3) = 4.66 \times 10^{-7}\sin(2q_2) + 2.08 \times 10^{-5}[\sin(q_3) - \sin(2q_2 + q_3)] + 1.224 \times 10^{-5}\sin(2q_2 + 2q_3) - 5.964 \times 10^{-3}\cos(2q_2) + 7.486 \times 10^{-3}[\cos(2q_2 + q_3) - \cos(q_3)] - 4.184 \times 10^{-3}\cos(2q_2 + 2q_3) + 11.29 \times 10^{-3}$
* $M_{01}(q_2, q_3) = 5.844 \times 10^{-8}\sin(q_2) - 8.381 \times 10^{-7}\sin(q_2 + q_3) + 5.439 \times 10^{-6}\cos(q_2) + 2.409 \times 10^{-5}\cos(q_2 + q_3)$
* $M_{02}(q_2, q_3) = -8.381 \times 10^{-7}\sin(q_2 + q_3) + 2.409 \times 10^{-5}\cos(q_2 + q_3)$
* $M_{11}(q_3) = 4.159 \times 10^{-5}\sin(q_3) - 14.97 \times 10^{-3}\cos(q_3) + 20.46 \times 10^{-3}$
* $M_{12}(q_3) = 2.08 \times 10^{-5}\sin(q_3) - 7.486 \times 10^{-3}\cos(q_3) + 8.492 \times 10^{-3}$
* $M_{22} = 8.492 \times 10^{-3}$

---

### B. Centrifugal and Coriolis Matrix $C(q,\dot{q}) \in \mathbb{R}^{3 \times 3}$
$$C(q,\dot{q}) = \begin{bmatrix} C_{00} & C_{01} & C_{02} \\ C_{10} & C_{11} & C_{12} \\ C_{20} & C_{21} & 0 \end{bmatrix}$$

* $C_{00} = \dot{q}_2 \left(5.964 \times 10^{-3}\sin(2q_2) - 7.486 \times 10^{-3}\sin(2q_2 + q_3) + 4.184 \times 10^{-3}\sin(2q_2 + 2q_3) + 4.66 \times 10^{-7}\cos(2q_2) - 2.08 \times 10^{-5}\cos(2q_2 + q_3) + 1.224 \times 10^{-5}\cos(2q_2 + 2q_3)\right) + \dot{q}_3 \left(3.743 \times 10^{-3}(\sin(q_3) - \sin(2q_2 + q_3)) + 4.184 \times 10^{-3}\sin(2q_2 + 2q_3) + 1.04 \times 10^{-5}(\cos(q_3) - \cos(2q_2 + q_3)) + 1.224 \times 10^{-5}\cos(2q_2 + 2q_3)\right)$
* $C_{01} = \dot{q}_1 \left(5.964 \times 10^{-3}\sin(2q_2) - 7.486 \times 10^{-3}\sin(2q_2 + q_3) + 4.184 \times 10^{-3}\sin(2q_2 + 2q_3) + 4.66 \times 10^{-7}\cos(2q_2) - 2.08 \times 10^{-5}\cos(2q_2 + q_3) + 1.224 \times 10^{-5}\cos(2q_2 + 2q_3)\right) + \dot{q}_2 \left(-5.439 \times 10^{-6}\sin(q_2) - 2.409 \times 10^{-5}\sin(q_2 + q_3) + 5.844 \times 10^{-8}\cos(q_2) - 8.381 \times 10^{-7}\cos(q_2 + q_3)\right) + \dot{q}_3 \left(-2.409 \times 10^{-5}\sin(q_2 + q_3) - 8.381 \times 10^{-7}\cos(q_2 + q_3)\right)$
* $C_{02} = \dot{q}_1 \left(3.743 \times 10^{-3}(\sin(q_3) - \sin(2q_2 + q_3)) + 4.184 \times 10^{-3}\sin(2q_2 + 2q_3) + 1.04 \times 10^{-5}(\cos(q_3) - \cos(2q_2 + q_3)) + 1.224 \times 10^{-5}\cos(2q_2 + 2q_3)\right) + (\dot{q}_2 + \dot{q}_3)\left(-2.409 \times 10^{-5}\sin(q_2 + q_3) - 8.381 \times 10^{-7}\cos(q_2 + q_3)\right)$
* $C_{10} = \dot{q}_1 \left(-5.964 \times 10^{-3}\sin(2q_2) + 7.486 \times 10^{-3}\sin(2q_2 + q_3) - 4.184 \times 10^{-3}\sin(2q_2 + 2q_3) - 4.66 \times 10^{-7}\cos(2q_2) + 2.08 \times 10^{-5}\cos(2q_2 + q_3) - 1.224 \times 10^{-5}\cos(2q_2 + 2q_3)\right)$
* $C_{11} = \dot{q}_3 \left(7.486 \times 10^{-3}\sin(q_3) + 2.08 \times 10^{-5}\cos(q_3)\right)$
* $C_{12} = (\dot{q}_2 + \dot{q}_3)\left(7.486 \times 10^{-3}\sin(q_3) + 2.08 \times 10^{-5}\cos(q_3)\right)$
* $C_{20} = \dot{q}_1 \left(-3.743 \times 10^{-3}(\sin(q_3) - \sin(2q_2 + q_3)) - 4.184 \times 10^{-3}\sin(2q_2 + 2q_3) - 1.04 \times 10^{-5}(\cos(q_3) - \cos(2q_2 + q_3)) - 1.224 \times 10^{-5}\cos(2q_2 + 2q_3)\right)$
* $C_{21} = -\dot{q}_2 \left(7.486 \times 10^{-3}\sin(q_3) + 2.08 \times 10^{-5}\cos(q_3)\right)$

---

### C. Gravity Vector $G(q) \in \mathbb{R}^3$
$$G(q) = \begin{bmatrix} 0 \\ 0.7979\sin(q_2) - 0.4896\sin(q_2 + q_3) + 2.433 \times 10^{-5}\cos(q_2) - 1.36 \times 10^{-3}\cos(q_2 + q_3) \\ -0.4896\sin(q_2 + q_3) - 1.36 \times 10^{-3}\cos(q_2 + q_3) \end{bmatrix}$$

---

### D. Electromechanical Coupling Matrices
* **Effective Viscous Damping Matrix ($B_{eff}$):**
  $$B_{eff} = \text{diag}\left(b_2 + \frac{K_2^2}{R_2},\ b_1 + \frac{K_1^2}{R_1},\ b_2 + \frac{K_2^2}{R_2}\right) \approx \text{diag}(0.04588,\ 0.20700,\ 0.04588)\text{ N}\cdot\text{m}\cdot\text{s}$$
* **Voltage-to-Torque Scaling Matrix ($\Lambda^{-1}$):**
  $$\Lambda^{-1} = \text{diag}\left(\frac{K_2}{R_2},\ \frac{K_1}{R_1},\ \frac{K_2}{R_2}\right) \approx \text{diag}(0.04128,\ 0.09771,\ 0.04128)\text{ N}\cdot\text{m/V}$$
* **Torque-to-Voltage Scaling Matrix ($\Lambda$):**
  $$\Lambda = \text{diag}\left(\frac{R_2}{K_2},\ \frac{R_1}{K_1},\ \frac{R_2}{K_2}\right) \approx \text{diag}(24.2240,\ 10.2342,\ 24.2240)\text{ V/(N}\cdot\text{m)}$$
