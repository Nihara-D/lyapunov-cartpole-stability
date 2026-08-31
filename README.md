# The Bowl and the Ball

### Proving a cart-pole robot never loses its balance - without solving its equations of motion

This is a complete, self-contained project with working code: a self-balancing robot, a Lyapunov "energy bowl," and numerical verification that the bowl only ever guides the robot downhill.

## Visual Proof & Outputs

### 1. Core LQR & Matrices Setup

Matrix definitions (`A`, `B`, `K`, `P`) and closed-loop eigenvalue verification confirming local stability.

![Core Setup](1.png)


---

### 2. State Sampling & Region of Attraction

20,000 random state evaluations checking `V̇ < 0`, calculating the certified region of attraction (`V ≤ 21.4923`), and verifying monotonic decay during push recovery.

![Verification Log](2.png)

---

### 3. Stability Verification Plots

Static visualization showing non-positive `V̇` across sampled states and monotonic decay of `V(t)` over time.

![Proof Summary Plot](3.png)
![Proof Summary Plot](proof_summary.png)

---

### 4. Cart-Pole & Lyapunov Bowl Animation

Live trajectory rendering of the cart-pole stabilizing side-by-side with the state trajectory rolling down the 3D Lyapunov energy bowl surface.

![Lyapunov Bowl Animation](lyapunov_animation.gif)

---

## Files

| File                     | What it does                                                                                       |
| ------------------------ | -------------------------------------------------------------------------------------------------- |
| `lyapunov_core.py`       | Cart-pole physics, LQR controller, and Lyapunov function `V(x) = xᵀPx`                             |
| `lyapunov_verify.py`     | Samples 20,000 states, checks `V̇ < 0`, estimates the certified region, and outputs `sim_data.npz` |
| `lyapunov_animate.py`    | Renders `lyapunov_animation.gif`                                                                   |
| `lyapunov_proof_plot.py` | Renders the stability proof plots                                                                  |
| `sim_data.npz`           | Cached simulation dataset                                                                          |
| `1.png`                  | Core LQR / matrix output                                                                           |
| `1.svg`                  | Vector version of the core setup                                                                   |
| `2.png`                  | Verification output                                                                                |
| `3.png`                  | Proof summary                                                                                      |
| `lyapunov_animation.gif` | Animated cart-pole + Lyapunov bowl                                                                 |

## The idea, in code form

1. **Don't solve the dynamics.** The cart-pole's true equations of motion are nonlinear and messy. We never solve them in closed form.
2. **Build the bowl.** Linearize near the balanced position, solve the LQR Riccati equation for `P`, and use `V(x) = xᵀPx` as the Lyapunov function.
3. **Prove it only rolls downhill.** Sample thousands of random off-balance states and check that `V̇(x) < 0` using the true nonlinear dynamics with the LQR controller.
4. **Find where the bowl ends.** States near the largest tested tilt angle (`≈34°`) begin violating `V̇ < 0`. The largest tested sublevel set without violations gives the certified region of attraction.
5. **Watch it happen.** The animation shows the pole recovering from a `20°` push while the red ball simultaneously rolls toward the bottom of the Lyapunov bowl.

## Results from this run

* **20,000 sampled states**
* **19 violations** (`0.095%`), all near `|θ| ≈ 34°`
* **Certified region:** `V(x) ≤ 21.4923`
* **Push recovery:** `V` drops from `8.19` to `0.00026` in 6 seconds
* Final angle: approximately **0.13°** from vertical

## Execution

Install dependencies:

```cmd
pip install scipy matplotlib pillow numpy
```

Run **one command at a time** in Windows Command Prompt:

```cmd
python lyapunov_core.py
```

After it finishes:

```cmd
python lyapunov_verify.py
```

After it finishes:

```cmd
python lyapunov_proof_plot.py
```

Finally:

```cmd
python lyapunov_animate.py
```

The final command generates:

```text
lyapunov_animation.gif
```

