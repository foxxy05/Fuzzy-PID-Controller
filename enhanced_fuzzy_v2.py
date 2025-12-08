import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# ============================================================================
# AUTOMATIC CSV PATH RESOLUTION - FINDS CSV AUTOMATICALLY
# ============================================================================

csv_path = None
search_paths = [
    "fuzzy_surface_all_cases.csv",              # Same folder
    "../fuzzy_surface_all_cases.csv",           # Parent folder
    "../../fuzzy_surface_all_cases.csv",        # Grandparent folder
    "./data/fuzzy_surface_all_cases.csv",       # data subfolder
    "./output/fuzzy_surface_all_cases.csv",     # output subfolder
    "../data/fuzzy_surface_all_cases.csv",      # ../data
    "../output/fuzzy_surface_all_cases.csv",    # ../output
]

print("="*100)
print("SEARCHING FOR CSV FILE...")
print("="*100)

for path in search_paths:
    if os.path.exists(path):
        csv_path = path
        print(f"✓ Found CSV at: {os.path.abspath(path)}")
        break

if csv_path is None:
    print("\n✗ ERROR: Could not find fuzzy_surface_all_cases.csv")
    print("\nSearched in these locations:")
    for path in search_paths:
        abs_path = os.path.abspath(path)
        print(f"  - {abs_path}")
    print("\n⚠️  SOLUTIONS:")
    print("  1. Copy fuzzy_surface_all_cases.csv to the same folder as this script")
    print("  2. OR edit this script and set csv_path manually:")
    print("     csv_path = r'D:/path/to/your/fuzzy_surface_all_cases.csv'")
    exit()

# ============================================================================
# CONFIGURATION - SET YOUR VALUES HERE!
# ============================================================================

Kp_base = 1.0   # CHANGE THIS TO YOUR VALUE!
Kd_base = 0.5   # CHANGE THIS TO YOUR VALUE!
DESIRED_DELTA_RATIO = 0.10

# ============================================================================
# LOAD AND CALCULATE
# ============================================================================

df = pd.read_csv(csv_path)
df['Kp'] = Kp_base * (1.0 + df['delta_Kp'])
df['Kd'] = Kd_base * (1.0 + df['delta_Kd'])

# ============================================================================
# ANALYSIS
# ============================================================================

print("\n" + "="*100)
print("ENHANCED FUZZY TUNING ANALYSIS")
print("="*100)

print("\n[1] CONFIGURATION:")
print(f"    Kp_base = {Kp_base}")
print(f"    Kd_base = {Kd_base}")
print(f"    Desired delta/base ratio = {DESIRED_DELTA_RATIO:.2%}")

print("\n[2] DELTA RANGES (From Fuzzy System):")
print(f"    delta_Kp: [{df['delta_Kp'].min():.4f}, {df['delta_Kp'].max():.4f}]")
print(f"    delta_Kd: [{df['delta_Kd'].min():.4f}, {df['delta_Kd'].max():.4f}]")

print("\n[3] ACTUAL KP AND KD RANGES:")
print(f"    Kp: [{df['Kp'].min():.4f}, {df['Kp'].max():.4f}]")
print(f"    Kd: [{df['Kd'].min():.4f}, {df['Kd'].max():.4f}]")

print("\n[4] PERCENTAGE CHANGES FROM BASE:")
Kp_change_max = ((df['Kp'].max() - Kp_base) / Kp_base) * 100
Kp_change_min = ((df['Kp'].min() - Kp_base) / Kp_base) * 100
Kd_change_max = ((df['Kd'].max() - Kd_base) / Kd_base) * 100
Kd_change_min = ((df['Kd'].min() - Kd_base) / Kd_base) * 100
print(f"    Kp changes: {Kp_change_min:+.1f}% to {Kp_change_max:+.1f}%")
print(f"    Kd changes: {Kd_change_min:+.1f}% to {Kd_change_max:+.1f}%")

print("\n[5] DELTA/BASE RATIOS:")
ratio_Kp = df['delta_Kp'].max() / Kp_base
ratio_Kd = df['delta_Kd'].max() / Kd_base
print(f"    Max delta_Kp / Kp_base = {ratio_Kp:.4f}  (desired: {DESIRED_DELTA_RATIO:.4f})")
print(f"    Max delta_Kd / Kd_base = {ratio_Kd:.4f}  (desired: {DESIRED_DELTA_RATIO:.4f})")

print("\n[6] SCALING RECOMMENDATIONS:")
if ratio_Kp > DESIRED_DELTA_RATIO:
    scaling_down = DESIRED_DELTA_RATIO / ratio_Kp
    scaling_up = ratio_Kp / DESIRED_DELTA_RATIO
    print(f"    ⚠️  Kp ratio EXCEEDS desired!")
    print(f"    Scale centroids DOWN by: {scaling_down:.3f}×")
    print(f"    OR Scale Kp_base UP by: {scaling_up:.3f}×")
else:
    print(f"    ✓ Kp ratio OK")

if ratio_Kd > DESIRED_DELTA_RATIO:
    scaling_down = DESIRED_DELTA_RATIO / ratio_Kd
    scaling_up = ratio_Kd / DESIRED_DELTA_RATIO
    print(f"    ⚠️  Kd ratio EXCEEDS desired!")
    print(f"    Scale centroids DOWN by: {scaling_down:.3f}×")
    print(f"    OR Scale Kd_base UP by: {scaling_up:.3f}×")
else:
    print(f"    ✓ Kd ratio OK")

print("\n[7] SAMPLE DATA:")
sample_df = df.iloc[[0, len(df)//2, -1]][['error', 'delta_error', 'delta_Kp', 'delta_Kd', 'Kp', 'Kd']]
print(sample_df.to_string())

# ============================================================================
# VISUALIZATIONS
# ============================================================================

print("\n[8] GENERATING VISUALIZATIONS...")

errors = sorted(df['error'].unique())
delta_errors = sorted(df['delta_error'].unique())

delta_Kp_grid = df.pivot_table(values='delta_Kp', index='delta_error', columns='error').values
delta_Kd_grid = df.pivot_table(values='delta_Kd', index='delta_error', columns='error').values
Kp_grid = df.pivot_table(values='Kp', index='delta_error', columns='error').values
Kd_grid = df.pivot_table(values='Kd', index='delta_error', columns='error').values

# 2D Heatmaps
fig, axes = plt.subplots(2, 2, figsize=(16, 12))
fig.suptitle(f'Fuzzy Analysis (Kp_base={Kp_base}, Kd_base={Kd_base})', fontsize=16, fontweight='bold')

axes[0, 0].imshow(delta_Kp_grid, aspect='auto', origin='lower', cmap='RdBu_r')
axes[0, 0].set_title('Delta Kp (Fuzzy Output)', fontweight='bold')
axes[0, 1].imshow(delta_Kd_grid, aspect='auto', origin='lower', cmap='RdBu_r')
axes[0, 1].set_title('Delta Kd (Fuzzy Output)', fontweight='bold')
axes[1, 0].imshow(Kp_grid, aspect='auto', origin='lower', cmap='viridis')
axes[1, 0].set_title(f'Actual Kp (Base={Kp_base})', fontweight='bold')
axes[1, 1].imshow(Kd_grid, aspect='auto', origin='lower', cmap='viridis')
axes[1, 1].set_title(f'Actual Kd (Base={Kd_base})', fontweight='bold')

plt.tight_layout()
plt.savefig('fuzzy_analysis_heatmaps.png', dpi=150)
print("  ✓ Saved: fuzzy_analysis_heatmaps.png")
plt.close()

# 3D Surface Plots
X, Y = np.meshgrid(errors, delta_errors)
fig2 = plt.figure(figsize=(16, 12))

ax1 = fig2.add_subplot(2, 2, 1, projection='3d')
ax1.plot_surface(X, Y, delta_Kp_grid, cmap='RdBu_r', alpha=0.8)
ax1.set_title('Delta Kp 3D', fontweight='bold')

ax2 = fig2.add_subplot(2, 2, 2, projection='3d')
ax2.plot_surface(X, Y, delta_Kd_grid, cmap='RdBu_r', alpha=0.8)
ax2.set_title('Delta Kd 3D', fontweight='bold')

ax3 = fig2.add_subplot(2, 2, 3, projection='3d')
ax3.plot_surface(X, Y, Kp_grid, cmap='viridis', alpha=0.8)
ax3.set_title(f'Actual Kp 3D', fontweight='bold')

ax4 = fig2.add_subplot(2, 2, 4, projection='3d')
ax4.plot_surface(X, Y, Kd_grid, cmap='viridis', alpha=0.8)
ax4.set_title(f'Actual Kd 3D', fontweight='bold')

plt.savefig('fuzzy_analysis_3d_surfaces.png', dpi=150)
print("  ✓ Saved: fuzzy_analysis_3d_surfaces.png")

# Save data
output_df = df[['error', 'delta_error', 'delta_Kp', 'delta_Kd', 'Kp', 'Kd']]
output_df.to_csv("fuzzy_surface_with_gains.csv", index=False)
print("  ✓ Saved: fuzzy_surface_with_gains.csv")

print("\n" + "="*100)
print("COMPLETE!")
print("="*100)
