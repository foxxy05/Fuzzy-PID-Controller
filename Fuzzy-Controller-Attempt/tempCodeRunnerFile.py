import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

# -------------------------------
# 1. Fuzzy system definitions
# -------------------------------

NUM_SETS = 7
labels = ['NB', 'NM', 'NS', 'ZO', 'PS', 'PM', 'PB']

# Error membership functions [a, b, c]
error_mf_points = np.array([
    [-180.0, -180.0,  -90.0],   # NB
    [-180.0,  -90.0,    0.0],   # NM
    [ -90.0,    0.0,   90.0],   # NS
    [ -20.0,    0.0,   20.0],   # ZO
    [   0.0,   20.0,   40.0],   # PS
    [  20.0,   70.0,  140.0],   # PM
    [  90.0,  180.0,  180.0]    # PB
])

# Delta-error membership functions [a, b, c]
delta_error_mf_points = np.array([
    [-1200.0, -1200.0,  -680.0],  # NB
    [ -950.0,  -400.0,     0.0],  # NM
    [ -240.0,  -100.0,     0.0],  # NS
    [  -80.0,     0.0,    80.0],  # ZO
    [    0.0,   100.0,   240.0],  # PS
    [    0.0,   400.0,   950.0],  # PM
    [  680.0,  1200.0,  1200.0]   # PB
])

# Output centroids (for both Kp and Kd)
OUTPUT_CENTROIDS = np.array([-.20, -0.15, -0.075, 0.0, 0.075, 0.15, 0.20])

# Rule bases (indices 0..6 → NB..PB)
KP_Rule_Base = np.array([
    [3, 4, 5, 6, 6, 6, 5],
    [2, 3, 4, 6, 5, 5, 4],
    [1, 2, 3, 5, 4, 4, 1],
    [2, 1, 1, 2, 2, 1, 2],
    [5, 4, 3, 1, 3, 2, 1],
    [6, 5, 4, 2, 3, 2, 1],
    [5, 6, 5, 2, 2, 1, 3]
])

KD_Rule_Base = np.array([
    [2, 1, 0, 0, 0, 1, 2],
    [3, 2, 1, 0, 1, 2, 3],
    [4, 3, 2, 1, 2, 3, 4],
    [5, 4, 3, 3, 3, 4, 5],
    [6, 5, 4, 1, 2, 3, 4],
    [6, 6, 5, 4, 4, 3, 1],
    [6, 6, 6, 5, 4, 4, 3]
])


def tri_mf(x, a, b, c):
    """Triangular membership function μ(x; a,b,c)."""
    if x < a or x > c:
        return 0.0
    if a == b and x <= b:   # left shoulder
        return 1.0
    if b == c and x >= b:   # right shoulder
        return 1.0
    if a <= x <= b:
        return (x - a) / (b - a) if b != a else 1.0
    if b < x <= c:
        return (c - x) / (c - b) if c != b else 1.0
    return 0.0


def fuzzify_error(e):
    return np.array([tri_mf(e, *error_mf_points[i]) for i in range(NUM_SETS)])


def fuzzify_delta_error(de):
    return np.array([tri_mf(de, *delta_error_mf_points[i]) for i in range(NUM_SETS)])


def infer_output(mu_e, mu_de, rule_base):
    """Mamdani min-max inference; returns aggregated μ_out[0..6]."""
    aggregated = np.zeros(NUM_SETS)
    for i in range(NUM_SETS):      # over Δe sets
        for j in range(NUM_SETS):  # over e sets
            alpha = min(mu_de[i], mu_e[j])
            if alpha <= 0:
                continue
            out_idx = rule_base[i, j]
            aggregated[out_idx] = max(aggregated[out_idx], alpha)
    return aggregated


def defuzzify(aggregated):
    num = np.sum(aggregated * OUTPUT_CENTROIDS)
    den = np.sum(aggregated)
    return num / den if den != 0 else 0.0


def compute_dKp_dKd(e, de):
    """Single-point evaluation: (error, delta_error) -> (delta_Kp, delta_Kd)."""
    mu_e = fuzzify_error(e)
    mu_de = fuzzify_delta_error(de)

    agg_kp = infer_output(mu_e, mu_de, KP_Rule_Base)
    agg_kd = infer_output(mu_e, mu_de, KD_Rule_Base)

    dKp = defuzzify(agg_kp)
    dKd = defuzzify(agg_kd)
    return dKp, dKd


# -------------------------------
# 2. Sweep full input space
# -------------------------------

# Resolution: adjust step sizes to trade off density vs runtime
ERROR_MIN, ERROR_MAX, ERROR_STEP = -180.0, 180.0, 5.0
DE_MIN, DE_MAX, DE_STEP = -1200.0, 1200.0, 50.0

errors = np.arange(ERROR_MIN, ERROR_MAX + 1e-9, ERROR_STEP)
derrors = np.arange(DE_MIN, DE_MAX + 1e-9, DE_STEP)

records = []  # list of dicts: for CSV + analysis

for e in errors:
    for de in derrors:
        dKp, dKd = compute_dKp_dKd(e, de)
        records.append({
            "error": e,
            "delta_error": de,
            "delta_Kp": dKp,
            "delta_Kd": dKd
        })

print(f"Generated {len(records)} grid points.")


# -------------------------------
# 3. Save results to CSV
# -------------------------------

csv_filename = "fuzzy_surface_all_cases.csv"
with open(csv_filename, mode="w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["error", "delta_error", "delta_Kp", "delta_Kd"])
    writer.writeheader()
    for row in records:
        writer.writerow(row)

print(f"Saved grid results to {csv_filename}")


# -------------------------------
# 4. Convert to arrays for plotting
# -------------------------------

E_vals = np.array([r["error"] for r in records])
DE_vals = np.array([r["delta_error"] for r in records])
dKp_vals = np.array([r["delta_Kp"] for r in records])
dKd_vals = np.array([r["delta_Kd"] for r in records])

# reshape into 2D grids for surface/heatmap
E_grid = errors
DE_grid = derrors
E_mesh, DE_mesh = np.meshgrid(E_grid, DE_grid, indexing="ij")  # [N_e, N_de][web:20][web:29]

dKp_grid = dKp_vals.reshape(len(E_grid), len(DE_grid))
dKd_grid = dKd_vals.reshape(len(E_grid), len(DE_grid))


# -------------------------------
# 5. Plot 3D surface + heatmap
# -------------------------------

def make_surface_and_heatmap(Z, zlabel, fname_prefix):
    # 3D surface
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')  # 3D projection[web:14][web:16]
    surf = ax.plot_surface(E_mesh, DE_mesh, Z,
                           cmap="coolwarm", linewidth=0, antialiased=True)
    ax.set_xlabel("Error (deg)")
    ax.set_ylabel("Delta Error (deg/s)")
    ax.set_zlabel(zlabel)
    ax.set_title(f"{zlabel} vs Error & Delta Error")
    fig.colorbar(surf, shrink=0.5, aspect=10, label=zlabel)
    plt.tight_layout()
    plt.savefig(f"{fname_prefix}_surface.png", dpi=300)
    plt.close(fig)

    # 2D heatmap
    fig2, ax2 = plt.subplots(figsize=(8, 6))
    im = ax2.imshow(Z.T, origin="lower",
                    extent=[ERROR_MIN, ERROR_MAX, DE_MIN, DE_MAX],
                    aspect="auto", cmap="coolwarm")
    ax2.set_xlabel("Error (deg)")
    ax2.set_ylabel("Delta Error (deg/s)")
    ax2.set_title(f"{zlabel} Heatmap")
    cbar = fig2.colorbar(im, ax=ax2)
    cbar.set_label(zlabel)
    plt.tight_layout()
    plt.savefig(f"{fname_prefix}_heatmap.png", dpi=300)
    plt.close(fig2)


make_surface_and_heatmap(dKp_grid, "Delta Kp", "delta_kp")
make_surface_and_heatmap(dKd_grid, "Delta Kd", "delta_kd")

print("Generated plots: delta_kp_surface.png, delta_kp_heatmap.png, "
      "delta_kd_surface.png, delta_kd_heatmap.png")
