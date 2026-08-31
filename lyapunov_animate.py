import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from lyapunov_core import P, l as pole_len

data = np.load("sim_data.npz")
t = data["t"]
z = data["z"]          # shape (4, T): x, x_dot, theta, theta_dot
V_traj = data["V_traj"]

x_cart = z[0]
theta = z[2]
theta_dot = z[3]

# ---------------------------------------------------------------------------
# Precompute the Lyapunov "bowl" surface as a function of (theta, theta_dot),
# holding x=0, x_dot=0 -- a 2D slice through the true 4D bowl, since a 4D
# bowl can't be drawn. This slice is what the trajectory actually rides on
# for most of the recovery, since |x|, |x_dot| stay small.
# ---------------------------------------------------------------------------
th_range = np.linspace(-0.7, 0.7, 60)
thd_range = np.linspace(-3.0, 3.0, 60)
TH, THD = np.meshgrid(th_range, thd_range)


def V_slice(th, thd):
    zz = np.array([0.0, 0.0, th, thd])
    return zz @ P @ zz


V_surf = np.vectorize(V_slice)(TH, THD)
V_on_traj = np.array([V_slice(th, thd) for th, thd in zip(theta, theta_dot)])

# ---------------------------------------------------------------------------
# Figure layout
# ---------------------------------------------------------------------------
fig = plt.figure(figsize=(12, 5.5))
fig.suptitle("The Bowl and the Ball — Lyapunov Stability of a Cart-Pole Robot",
             fontsize=13, fontweight="bold")

ax_pend = fig.add_subplot(1, 2, 1)
ax_bowl = fig.add_subplot(1, 2, 2, projection="3d")

# --- Pendulum panel setup ---
ax_pend.set_xlim(-2.2, 2.2)
ax_pend.set_ylim(-0.3, 1.3)
ax_pend.set_aspect("equal")
ax_pend.set_title("Cart-pole recovering from a push")
ax_pend.axhline(0, color="#888", lw=1)

cart_w, cart_h = 0.35, 0.2
cart_patch = plt.Rectangle((-cart_w/2, -cart_h/2), cart_w, cart_h,
                            fc="#2b6cb0", ec="black", zorder=5)
ax_pend.add_patch(cart_patch)
pole_line, = ax_pend.plot([], [], lw=4, color="#c53030", zorder=4)
bob_dot, = ax_pend.plot([], [], "o", color="#c53030", markersize=10, zorder=6)
time_text = ax_pend.text(0.02, 0.95, "", transform=ax_pend.transAxes, fontsize=10)

# --- Bowl panel setup ---
ax_bowl.plot_surface(TH, THD, V_surf, cmap="viridis", alpha=0.55,
                      linewidth=0, antialiased=True)
ax_bowl.set_xlabel("theta (rad)")
ax_bowl.set_ylabel("theta_dot (rad/s)")
ax_bowl.set_zlabel("V(x) = xᵀPx")
ax_bowl.set_title("Lyapunov function: always rolling downhill")
ball_dot, = ax_bowl.plot([], [], [], "o", color="red", markersize=8, zorder=10)
trail_line, = ax_bowl.plot([], [], [], color="red", lw=2, alpha=0.8)

n_frames = len(t)


def init():
    pole_line.set_data([], [])
    bob_dot.set_data([], [])
    ball_dot.set_data([], [])
    ball_dot.set_3d_properties([])
    trail_line.set_data([], [])
    trail_line.set_3d_properties([])
    return pole_line, bob_dot, ball_dot, trail_line, cart_patch


def update(i):
    xc = x_cart[i]
    th = theta[i]

    cart_patch.set_xy((xc - cart_w/2, -cart_h/2))

    bob_x = xc + 2 * pole_len * np.sin(th)
    bob_y = 2 * pole_len * np.cos(th)
    pole_line.set_data([xc, bob_x], [0, bob_y])
    bob_dot.set_data([bob_x], [bob_y])
    time_text.set_text(f"t = {t[i]:.2f}s   theta = {np.degrees(th):5.1f} deg")

    ball_dot.set_data([theta[i]], [theta_dot[i]])
    ball_dot.set_3d_properties([V_on_traj[i]])

    trail_line.set_data(theta[:i+1], theta_dot[:i+1])
    trail_line.set_3d_properties(V_on_traj[:i+1])

    return pole_line, bob_dot, ball_dot, trail_line, cart_patch


ax_bowl.view_init(elev=28, azim=-60)

anim = FuncAnimation(fig, update, frames=n_frames, init_func=init,
                      interval=25, blit=False)

plt.tight_layout(rect=[0, 0, 1, 0.94])

out_path = "lyapunov_animation.gif"
anim.save(out_path, writer=PillowWriter(fps=25))
print("Saved animation to", out_path)
