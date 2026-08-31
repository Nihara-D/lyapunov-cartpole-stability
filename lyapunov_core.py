"""
The Bowl and the Ball — Lyapunov stability for a cart-pole balancing robot.

Core physics, LQR controller design, and Lyapunov verification.
No hand-solved trajectory needed: we build V(x) = x^T P x from the LQR
Riccati solution, then verify numerically that V is always shrinking
whenever the system is off-balance.
"""
import numpy as np
from scipy.linalg import solve_continuous_are

# ---------------------------------------------------------------------------
# 1. System parameters (cart-pole / inverted pendulum on a cart)
# ---------------------------------------------------------------------------
M = 1.0     # cart mass (kg)
m = 0.2     # pole mass (kg)
l = 0.5     # distance from pivot to pole center of mass (m)
g = 9.81    # gravity (m/s^2)

# State vector: z = [x, x_dot, theta, theta_dot]
#   x      = cart position
#   theta  = pole angle from upright (0 = balanced)


def nonlinear_dynamics(z, u):
    """True nonlinear cart-pole dynamics, upright equilibrium at theta=0."""
    x, x_dot, theta, theta_dot = z
    s, c = np.sin(theta), np.cos(theta)

    denom = M + m * s**2
    x_ddot = (u + m * s * (l * theta_dot**2 - g * c)) / denom
    theta_ddot = (
        -u * c - m * l * theta_dot**2 * s * c + (M + m) * g * s
    ) / (l * denom)

    return np.array([x_dot, x_ddot, theta_dot, theta_ddot])


# ---------------------------------------------------------------------------
# 2. Linearize about the upright equilibrium (theta=0, all rates=0, u=0)
# ---------------------------------------------------------------------------
A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m * g / M, 0],
    [0, 0, 0, 1],
    [0, 0, (M + m) * g / (M * l), 0],
])
B = np.array([[0], [1 / M], [0], [-1 / (M * l)]])

# ---------------------------------------------------------------------------
# 3. LQR design -> also hands us the Lyapunov function for free
# ---------------------------------------------------------------------------
Q = np.diag([1.0, 1.0, 10.0, 1.0])   # penalize angle error most
R = np.array([[0.5]])

P = solve_continuous_are(A, B, Q, R)   # P is our Lyapunov matrix
K = np.linalg.solve(R, B.T @ P)        # LQR gain, u = -K @ z


def control(z):
    return float((-K @ z).item())


def V(z):
    """Lyapunov candidate: V(x) = x^T P x  (our 'bowl')."""
    z = np.asarray(z)
    return float(z @ P @ z)


def V_dot_analytic_linear(z):
    """V_dot predicted by the *linear* closed-loop model: 2 x^T P (A-BK) x."""
    Acl = A - B @ K
    return float(2 * z @ P @ (Acl @ z))


def V_dot_true(z, u=None):
    """V_dot computed along the *true nonlinear* dynamics with the LQR law."""
    if u is None:
        u = control(z)
    zdot = nonlinear_dynamics(z, u)
    return float(2 * z @ P @ zdot)


if __name__ == "__main__":
    print("Linearized A:\n", A)
    print("Linearized B:\n", B.ravel())
    print("\nLyapunov matrix P (from LQR Riccati solution):\n", np.round(P, 3))
    print("\nLQR gain K:", np.round(K, 3))
    print("\nEigenvalues of closed-loop A-BK:", np.linalg.eigvals(A - B @ K))
