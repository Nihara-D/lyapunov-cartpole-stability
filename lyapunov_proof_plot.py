import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

data = np.load("sim_data.npz")
t, V_traj = data["t"], data["V_traj"]
Vs, Vdots = data["Vs"], data["Vdots"]

fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

# Panel 1: V(t) monotonically decreasing during push-recovery
axes[0].plot(t, V_traj, color="#c53030", lw=2)
axes[0].set_title("V(x(t)) during push-recovery\n(the ball's height over time)")
axes[0].set_xlabel("time (s)")
axes[0].set_ylabel("V(x) = xᵀPx")
axes[0].grid(alpha=0.3)

# Panel 2: scatter of V vs V_dot across 20,000 random sampled states
sc = axes[1].scatter(Vs, Vdots, s=3, alpha=0.25, c=(Vdots < 0),
                      cmap="RdYlGn", vmin=0, vmax=1)
axes[1].axhline(0, color="black", lw=1)
axes[1].set_title("V̇ vs V across 20,000 sampled states\n(proof: no equations of motion solved)")
axes[1].set_xlabel("V(x)")
axes[1].set_ylabel("V̇(x)  (should stay ≤ 0)")
axes[1].grid(alpha=0.3)

plt.tight_layout()
plt.savefig("proof_summary.png", dpi=150)
print("Saved proof_summary.png")
