import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ============================================================================
# CONFIGURATION - THESE ARE THE VALUES YOU NEED TO PROVIDE!
# ============================================================================

# YOUR BASE GAINS (from your C code - find these values!)
Kp_base = 10.0   # What is Kp_base in your C code? CHANGE THIS!
Kd_base = 5.0    # What is Kd_base in your C code? CHANGE THIS!

# DESIRED DELTA/BASE RATIO (the "filtering factor")
# This controls how much the fuzzy system adjusts your gains
# Typical values: 0.05 (5%), 0.10 (10%), 0.15 (15%)
DESIRED_DELTA_RATIO = 0.10  # Adjust this based on your requirements

# ============================================================================
# LOAD AND CALCULATE
# ============================================================================

df = pd.read_csv("fuzzy_surface_all_cases.csv")

# Calculate actual Kp and Kd using multiplicative scaling
df['Kp'] = Kp_base * (1.0 + df['delta_Kp'])
df['Kd'] = Kd_base * (1.0 + df['delta_Kd'])

# ============================================================================
# ANALYSIS AND REPORTING
# ============================================================================

print("="*100)
print("ENHANCED FUZZY TUNING ANALYSIS")
print("="*100)

print("\n[1] CONFIGURATION:")
print(f"    Kp_base = {Kp_base}")
print(f"    Kd_base = {Kd_base}")
print(f"    Desired delta/base ratio = {DESIRED_DELTA_RATIO:.2%}")

print("\n[2] DELTA RANGES (From Fuzzy System):")
print(f"    delta_Kp: [{df['delta_Kp'].min():.4f}, {df['delta_Kp'].max():.4f}]  range={df['delta_Kp'].max()-df['delta_Kp'].min():.4f}")
print(f"    delta_Kd: [{df['delta_Kd'].min():.4f}, {df['delta_Kd'].max():.4f}]  range={df['delta_Kd'].max()-df['delta_Kd'].min():.4f}")

print("\n[3] ACTUAL KP AND KD RANGES (With Multiplicative Scaling):")
print(f"    Kp: [{df['Kp'].min():.4f}, {df['Kp'].max():.4f}]  range={df['Kp'].max()-df['Kp'].min():.4f}")
print(f"    Kd: [{df['Kd'].min():.4f}, {df['Kd'].max():.4f}]  range={df['Kd'].max()-df['Kd'].min():.4f}")

print("\n[4] PERCENTAGE CHANGES FROM BASE:")
Kp_change_max = ((df['Kp'].max() - Kp_base) / Kp_base) * 100
Kp_change_min = ((df['Kp'].min() - Kp_base) / Kp_base) * 100
Kd_change_max = ((df['Kd'].max() - Kd_base) / Kd_base) * 100
Kd_change_min = ((df['Kd'].min() - Kd_base) / Kd_base) * 100

print(f"    Kp changes: {Kp_change_min:+.1f}% to {Kp_change_max:+.1f}%")
print(f"    Kd changes: {Kd_change_min:+.1f}% to {Kd_change_max:+.1f}%")

print("\n[5] DELTA/BASE RATIOS (Critical for Multiplicative Scaling):")
ratio_Kp = df['delta_Kp'].max() / Kp_base
ratio_Kd = df['delta_Kd'].max() / Kd_base
print(f"    Max delta_Kp / Kp_base = {ratio_Kp:.4f}  (desired: {DESIRED_DELTA_RATIO:.4f})")
print(f"    Max delta_Kd / Kd_base = {ratio_Kd:.4f}  (desired: {DESIRED_DELTA_RATIO:.4f})")

print("\n[6] SCALING RECOMMENDATIONS:")
if ratio_Kp > DESIRED_DELTA_RATIO:
    scaling_down = DESIRED_DELTA_RATIO / ratio_Kp
    scaling_up = ratio_Kp / DESIRED_DELTA_RATIO
    print(f"    ⚠️  Kp ratio EXCEEDS desired!")
    print(f"    To achieve {DESIRED_DELTA_RATIO:.2%} ratio:")
    print(f"      • Scale centroids DOWN by: {scaling_down:.3f}×")
    print(f"      • Scale Kp_base UP by:  {scaling_up:.3f}×")
elif ratio_Kp < DESIRED_DELTA_RATIO:
    print(f"    ✓ Kp ratio is BELOW desired")
else:
    print(f"    ✓ Kp ratio MATCHES desired!")

if ratio_Kd > DESIRED_DELTA_RATIO:
    scaling_down = DESIRED_DELTA_RATIO / ratio_Kd
    scaling_up = ratio_Kd / DESIRED_DELTA_RATIO
    print(f"    ⚠️  Kd ratio EXCEEDS desired!")
    print(f"    To achieve {DESIRED_DELTA_RATIO:.2%} ratio:")
    print(f"      • Scale centroids DOWN by: {scaling_down:.3f}×")
    print(f"      • Scale Kd_base UP by:  {scaling_up:.3f}×")
elif ratio_Kd < DESIRED_DELTA_RATIO:
    print(f"    ✓ Kd ratio is BELOW desired")
else:
    print(f"    ✓ Kd ratio MATCHES desired!")

print("\n[7] SAMPLE DATA POINTS:")
sample_indices = [0, len(df)//4, len(df)//2, 3*len(df)//4, -1]
sample_df = df.iloc[sample_indices][['error', 'delta_error', 'delta_Kp', 'delta_Kd', 'Kp', 'Kd']]
print(sample_df.to_string())

# ============================================================================
# CREATE HEATMAPS FOR DELTAS AND ACTUAL GAINS
# ============================================================================

print("\n[8] GENERATING VISUALIZATIONS...")

# Get unique error and delta_error values for reshaping
errors = sorted(df['error'].unique())
delta_errors = sorted(df['delta_error'].unique())

# Create 2D grids for heatmaps
delta_Kp_grid = df.pivot_table(values='delta_Kp', index='delta_error', columns='error').values
delta_Kd_grid = df.pivot_table(values='delta_Kd', index='delta_error', columns='error').values
Kp_grid = df.pivot_table(values='Kp', index='delta_error', columns='error').values
Kd_grid = df.pivot_table(values='Kd', index='delta_error', columns='error').values

# Create figure with 4 heatmaps (2x2)
fig, axes = plt.subplots(2, 2, figsize=(16, 12))
fig.suptitle(f'Fuzzy Analysis: Deltas vs Actual Gains (Kp_base={Kp_base}, Kd_base={Kd_base})', fontsize=16, fontweight='bold')

# Plot 1: Delta Kp (fuzzy output)
im1 = axes[0, 0].imshow(delta_Kp_grid, aspect='auto', origin='lower', cmap='RdBu_r')
axes[0, 0].set_title(f'Delta Kp (Fuzzy Output)', fontsize=13, fontweight='bold')
axes[0, 0].set_xlabel('Error (deg)', fontsize=11)
axes[0, 0].set_ylabel('Delta Error (deg/s)', fontsize=11)
cbar1 = plt.colorbar(im1, ax=axes[0, 0])
cbar1.set_label('Δ Kp', rotation=270, labelpad=20)

# Plot 2: Delta Kd (fuzzy output)
im2 = axes[0, 1].imshow(delta_Kd_grid, aspect='auto', origin='lower', cmap='RdBu_r')
axes[0, 1].set_title(f'Delta Kd (Fuzzy Output)', fontsize=13, fontweight='bold')
axes[0, 1].set_xlabel('Error (deg)', fontsize=11)
axes[0, 1].set_ylabel('Delta Error (deg/s)', fontsize=11)
cbar2 = plt.colorbar(im2, ax=axes[0, 1])
cbar2.set_label('Δ Kd', rotation=270, labelpad=20)

# Plot 3: Actual Kp (after applying base gain)
im3 = axes[1, 0].imshow(Kp_grid, aspect='auto', origin='lower', cmap='viridis')
axes[1, 0].set_title(f'Actual Kp (Base={Kp_base})', fontsize=13, fontweight='bold')
axes[1, 0].set_xlabel('Error (deg)', fontsize=11)
axes[1, 0].set_ylabel('Delta Error (deg/s)', fontsize=11)
cbar3 = plt.colorbar(im3, ax=axes[1, 0])
cbar3.set_label('Kp', rotation=270, labelpad=20)

# Plot 4: Actual Kd (after applying base gain)
im4 = axes[1, 1].imshow(Kd_grid, aspect='auto', origin='lower', cmap='viridis')
axes[1, 1].set_title(f'Actual Kd (Base={Kd_base})', fontsize=13, fontweight='bold')
axes[1, 1].set_xlabel('Error (deg)', fontsize=11)
axes[1, 1].set_ylabel('Delta Error (deg/s)', fontsize=11)
cbar4 = plt.colorbar(im4, ax=axes[1, 1])
cbar4.set_label('Kd', rotation=270, labelpad=20)

plt.tight_layout()
plt.savefig('fuzzy_analysis_heatmaps.png', dpi=150, bbox_inches='tight')
print(f"  ✓ Saved: fuzzy_analysis_heatmaps.png")
plt.close()

# ============================================================================
# CREATE 3D SURFACE PLOTS
# ============================================================================

X, Y = np.meshgrid(errors, delta_errors)

fig2 = plt.figure(figsize=(16, 12))
fig2.suptitle(f'3D Surfaces: Deltas vs Actual Gains (Kp_base={Kp_base}, Kd_base={Kd_base})', fontsize=16, fontweight='bold')

# 3D plot 1: Delta Kp surface
ax1 = fig2.add_subplot(2, 2, 1, projection='3d')
surf1 = ax1.plot_surface(X, Y, delta_Kp_grid, cmap='RdBu_r', alpha=0.8, edgecolor='none')
ax1.set_title(f'Delta Kp (Fuzzy Output)', fontsize=12, fontweight='bold')
ax1.set_xlabel('Error (deg)', fontsize=10)
ax1.set_ylabel('Delta Error (deg/s)', fontsize=10)
ax1.set_zlabel('Δ Kp', fontsize=10)

# 3D plot 2: Delta Kd surface
ax2 = fig2.add_subplot(2, 2, 2, projection='3d')
surf2 = ax2.plot_surface(X, Y, delta_Kd_grid, cmap='RdBu_r', alpha=0.8, edgecolor='none')
ax2.set_title(f'Delta Kd (Fuzzy Output)', fontsize=12, fontweight='bold')
ax2.set_xlabel('Error (deg)', fontsize=10)
ax2.set_ylabel('Delta Error (deg/s)', fontsize=10)
ax2.set_zlabel('Δ Kd', fontsize=10)

# 3D plot 3: Actual Kp surface
ax3 = fig2.add_subplot(2, 2, 3, projection='3d')
surf3 = ax3.plot_surface(X, Y, Kp_grid, cmap='viridis', alpha=0.8, edgecolor='none')
ax3.set_title(f'Actual Kp (Base={Kp_base})', fontsize=12, fontweight='bold')
ax3.set_xlabel('Error (deg)', fontsize=10)
ax3.set_ylabel('Delta Error (deg/s)', fontsize=10)
ax3.set_zlabel('Kp', fontsize=10)

# 3D plot 4: Actual Kd surface
ax4 = fig2.add_subplot(2, 2, 4, projection='3d')
surf4 = ax4.plot_surface(X, Y, Kd_grid, cmap='viridis', alpha=0.8, edgecolor='none')
ax4.set_title(f'Actual Kd (Base={Kd_base})', fontsize=12, fontweight='bold')
ax4.set_xlabel('Error (deg)', fontsize=10)
ax4.set_ylabel('Delta Error (deg/s)', fontsize=10)
ax4.set_zlabel('Kd', fontsize=10)

plt.tight_layout()
plt.savefig('fuzzy_analysis_3d_surfaces.png', dpi=150, bbox_inches='tight')
print(f"  ✓ Saved: fuzzy_analysis_3d_surfaces.png")
plt.close()

# ============================================================================
# SAVE COMPLETE DATA WITH KP AND KD
# ============================================================================

output_cols = ['error', 'delta_error', 'delta_Kp', 'delta_Kd', 'Kp', 'Kd']
output_df = df[output_cols].copy()
output_df.to_csv("fuzzy_surface_with_gains.csv", index=False)
print(f"  ✓ Saved: fuzzy_surface_with_gains.csv")

# ============================================================================
# FINAL SUMMARY
# ============================================================================

print("\n" + "="*100)
print("ANALYSIS COMPLETE")
print("="*100)
print("""
FILES GENERATED:
  1. fuzzy_analysis_heatmaps.png     - 2D heatmaps (deltas vs actual gains)
  2. fuzzy_analysis_3d_surfaces.png  - 3D surface plots (all 4 values)
  3. fuzzy_surface_with_gains.csv    - Data export with Kp and Kd

NEXT STEPS:
  1. Review section [6] SCALING RECOMMENDATIONS
  2. Review the generated images to visualize your fuzzy system
  3. If ratio > desired: scale down centroids or up base gains
  4. Update Python centroids and C code values
  5. Re-run this script to verify improvements
""")
