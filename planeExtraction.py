import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def setup_plot(ax, title):
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=12, pad=10, fontweight='bold')
    ax.set_xlim(-2, 8)
    ax.set_ylim(-2, 8)
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")

def draw_robot(ax, x, y, angle_deg, color='black'):
    """Draws a simple robot marker."""
    circle = patches.Circle((x, y), 0.3, facecolor='white', edgecolor=color, zorder=5)
    ax.add_patch(circle)
    # Direction indicator
    rad = np.radians(angle_deg)
    ax.arrow(x, y, 0.4*np.cos(rad), 0.4*np.sin(rad), 
             head_width=0.2, head_length=0.2, fc=color, ec=color, zorder=6)

def generate_nosp_figure():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    
    # --- GEOMETRY DEFINITIONS ---
    # Wall 1: Horizontal from (0,0) to (4,0)
    # Wall 2: Angled 135 degrees relative to Wall 1. 
    # If Wall 1 is 0 deg, Wall 2 is at 45 deg relative to x-axis (since 180-135 = 45 turn).
    
    # Ground Truth Coordinates
    origin = np.array([0, 0])
    corner = np.array([4, 0])
    
    # Wall 2 direction (45 degrees relative to X, making the internal angle 135)
    w2_len = 4
    angle_rad = np.radians(45) 
    end_point = corner + np.array([w2_len * np.cos(angle_rad), w2_len * np.sin(angle_rad)])
    
    gt_x = [origin[0], corner[0], end_point[0]]
    gt_y = [origin[1], corner[1], end_point[1]]

    # --- PLOT 1: MANHATTAN WORLD (FAILURE CASE) ---
    setup_plot(ax1, "Standard Manhattan World (MW)\n(Assumption: Orthogonal 90°)")
    
    # 1. Ground Truth Walls (faint grey)
    ax1.plot(gt_x, gt_y, color='gray', linewidth=8, alpha=0.3, label='Ground Truth')
    
    # 2. MW Estimated Walls (Red Dashed - Snapping to 90)
    # It tracks the first wall well, but snaps the second wall to 90 degrees (vertical)
    mw_end_point = corner + np.array([0, w2_len]) # Vertical
    ax1.plot([origin[0], corner[0], mw_end_point[0]], 
             [origin[1], corner[1], mw_end_point[1]], 
             color='#d62728', linestyle='--', linewidth=3, label='MW Estimation')
    
    # 3. Trajectory Drift (Curving to try and match)
    traj_x = np.linspace(0, 4, 10)
    traj_y = np.zeros(10)
    # Curve part
    theta = np.linspace(0, np.pi/2, 10)
    curve_x = 4 + 0.5 * np.cos(theta - np.pi/2) # Small offset
    curve_y = 0 + 0.5 * np.sin(theta - np.pi/2) # Small offset essentially turning 90
    
    ax1.text(4.2, 2.0, "Severe Map Distortion\n(Snapping 135° $\u2192$ 90°)", 
             color='#d62728', fontsize=10, ha='left')

    # Annotate Angle
    arc = patches.Arc((4, 0), 1.5, 1.5, angle=0, theta1=0, theta2=90, color='#d62728', linewidth=2)
    ax1.add_patch(arc)
    ax1.text(4.3, 0.5, r"$90^\circ$", color='#d62728', fontsize=11, fontweight='bold')

    # Legend
    ax1.legend(loc='upper left')

    # --- PLOT 2: NOSP (SUCCESS CASE) ---
    setup_plot(ax2, "Proposed NOSP Approach\n(Constraint: Structural $\\alpha \\approx 135^\\circ$)")
    
    # 1. Ground Truth Walls (faint grey)
    ax2.plot(gt_x, gt_y, color='gray', linewidth=8, alpha=0.3, label='Ground Truth')
    
    # 2. NOSP Estimated Walls (Blue Dashed - Matches GT)
    ax2.plot(gt_x, gt_y, color='#1f77b4', linestyle='--', linewidth=3, label='NOSP Estimation')
    
    # 3. Dominant Vectors (The Graph Nodes)
    # Vector d1 (Wall 1 normal - pointing South for visualization sake, or along wall)
    # Usually normals are used, but for map visualization, showing the wall direction is clearer.
    # Let's show direction vectors d_a and d_b
    
    # d_a along X axis
    ax2.arrow(2, -0.5, 1, 0, head_width=0.3, head_length=0.3, fc='black', ec='black', width=0.05)
    ax2.text(2.5, -1.0, r"$\mathbf{d}_a$", fontsize=12, ha='center')
    
    # d_b along 45 deg
    arrow_start = corner + np.array([1, -0.5]) # offset slightly
    ax2.arrow(5.5, 1.5, 0.707, 0.707, head_width=0.3, head_length=0.3, fc='black', ec='black', width=0.05)
    ax2.text(6.5, 2.0, r"$\mathbf{d}_b$", fontsize=12, ha='center')

    # Annotate Angle
    arc2 = patches.Arc((4, 0), 2.0, 2.0, angle=0, theta1=0, theta2=45, color='#1f77b4', linewidth=2)
    ax2.add_patch(arc2)
    ax2.text(5.2, 0.3, r"$\alpha_{ab} = 135^\circ$", color='#1f77b4', fontsize=11, fontweight='bold')

    # Robot at corner
    draw_robot(ax2, 4, 0, 22.5, color='black')
    
    # Residual Text
    ax2.text(0.2, 6.5, r"$\mathbf{r}_{ang} = \mathbf{d}_a^T \mathbf{d}_b - \cos(135^\circ) \approx 0$", 
             fontsize=10, bbox=dict(facecolor='white', alpha=0.8, edgecolor='#1f77b4'))
    
    ax2.legend(loc='upper left')

    plt.tight_layout()
    plt.savefig('nosp_concept_diagram.png', dpi=300)
    print("Figure saved as 'nosp_concept_diagram.png'")

if __name__ == "__main__":
    generate_nosp_figure()