import matplotlib.pyplot as plt
import numpy as np
import cv2

# Set up formatting for a scientific-style figure
plt.rcParams.update({
    'font.family': 'serif',
    'axes.labelsize': 10,
    'axes.titlesize': 12,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'figure.titlesize': 14,
    'text.usetex': False # Set to True if you have LaTeX installed for nicer fonts
})

# Helper function to create a synthetic IR image with nests and stains
def generate_synthetic_ir(height=300, width=400, scenario="fp_removal"):
    img = np.full((height, width), 180, dtype=np.uint8) # Grey wall
    # Add subtle wall texture/noise
    noise = np.random.randint(-10, 10, (height, width)).astype(np.uint8)
    img = cv2.add(img, noise)
    
    nests = []
    stains = []

    if scenario == "fp_removal":
        # True Nests (dark, textured)
        nests.append(cv2.ellipse(np.zeros_like(img), (100, 80), (40, 25), 0, 0, 360, 255, -1))
        nests.append(cv2.ellipse(np.zeros_like(img), (300, 120), (35, 22), 0, 0, 360, 255, -1))
        # False Positive Stains (dark, less textured, irregular)
        stains.append(cv2.ellipse(np.zeros_like(img), (80, 200), (30, 15), 20, 0, 360, 255, -1))
        stains.append(cv2.ellipse(np.zeros_like(img), (250, 220), (25, 12), -10, 0, 360, 255, -1))
        stains.append(cv2.ellipse(np.zeros_like(img), (350, 60), (20, 10), 5, 0, 360, 255, -1))
    
    elif scenario == "tp_preservation":
        # Nests at different distances (scales)
        # Foreground (large)
        nests.append(cv2.ellipse(np.zeros_like(img), (100, 220), (60, 35), 0, 0, 360, 255, -1))
        nests.append(cv2.ellipse(np.zeros_like(img), (300, 230), (55, 32), 0, 0, 360, 255, -1))
        # Mid-ground (medium)
        nests.append(cv2.ellipse(np.zeros_like(img), (200, 130), (35, 20), 0, 0, 360, 255, -1))
        # Background (small)
        nests.append(cv2.ellipse(np.zeros_like(img), (80, 60), (20, 12), 0, 0, 360, 255, -1))
        nests.append(cv2.ellipse(np.zeros_like(img), (320, 70), (18, 10), 0, 0, 360, 255, -1))

    # Draw objects onto the IR image
    for mask in nests:
        texture = np.random.randint(40, 80, (height, width)).astype(np.uint8)
        img = np.where(mask > 0, texture, img)
    for mask in stains:
        texture = np.random.randint(60, 100, (height, width)).astype(np.uint8)
        img = np.where(mask > 0, texture, img)

    return img, nests, stains

# Helper function to create color masks for segmentation
def create_segmentation_overlay(img, nests, stains, nest_color=(0, 0, 255), stain_color=(255, 165, 0)):
    overlay = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    alpha = 0.6 # Transparency
    
    mask_layer = np.zeros_like(overlay)
    for mask in nests:
        mask_layer[mask > 0] = nest_color
    for mask in stains:
        mask_layer[mask > 0] = stain_color
        
    cv2.addWeighted(mask_layer, alpha, overlay, 1 - alpha, 0, overlay)
    return overlay

# Helper function to generate a synthetic depth map
def generate_depth_map(height, width, nests, stains, scenario):
    # Background gradient (warm colors = far)
    depth = np.tile(np.linspace(150, 220, height, dtype=np.uint8).reshape(-1, 1), (1, width))
    
    # Nests are concave -> closer at center (cool colors)
    for mask in nests:
        # Find centroid
        M = cv2.moments(mask)
        cX, cY = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        # Create a radial gradient for concavity
        Y, X = np.ogrid[:height, :width]
        dist_from_center = np.sqrt((X - cX)**2 + (Y - cY)**2)
        # Normalize and invert distance for depth (closer = lower value)
        nest_depth_profile = 20 + (dist_from_center / np.sqrt(40**2 + 25**2)) * 100 
        depth = np.where(mask > 0, np.clip(nest_depth_profile, 0, 255).astype(np.uint8), depth)

    # Stains are flat -> same depth as the wall (no change to background depth)
    # (depth remains the background value where stains are)

    # Apply a colormap (e.g., PLASMA or JET for blue=near, red/yellow=far)
    depth_color = cv2.applyColorMap(depth, cv2.COLORMAP_PLASMA)
    return depth_color, nests, stains

# Function to draw depth cues (arrows, circles, X's)
def draw_depth_overlays(ax, nests, stains, scenario):
    if scenario == "fp_removal":
        # Arrows for TPs (Nests)
        for mask in nests:
            M = cv2.moments(mask)
            cX, cY = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            ax.arrow(cX-20, cY-30, 15, 25, color='blue', head_width=8, linewidth=2)
        # 'X' for FPs (Stains)
        for mask in stains:
            M = cv2.moments(mask)
            cX, cY = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            ax.text(cX, cY, 'X', color='red', fontsize=20, ha='center', va='center', fontweight='bold')
            
    elif scenario == "tp_preservation":
        # Center-surround circles and Delta-d arrows for a few nests
        examples_indices = [0, 2, 4] # A large, medium, and small nest
        for i in examples_indices:
            mask = nests[i]
            M = cv2.moments(mask)
            cX, cY = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            # Get rough radius
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            _, radius = cv2.minEnclosingCircle(contours[0])
            
            # Center and Periphery circles
            center_circle = plt.Circle((cX, cY), radius*0.4, color='blue', alpha=0.7, linewidth=2, fill=True)
            periphery_circle = plt.Circle((cX, cY), radius*0.8, color='green', alpha=0.5, linewidth=2, fill=False)
            ax.add_patch(periphery_circle)
            ax.add_patch(center_circle)

            # Delta-d Arrow and Label (only for one clear example)
            if i == 0:
                ax.annotate('', xy=(cX+radius, cY), xytext=(cX+radius, cY-radius*0.6),
                            arrowprops=dict(arrowstyle='<->', color='blue', lw=2))
                ax.text(cX+radius+5, cY-radius*0.3, r'$\Delta d$', color='blue', fontsize=14)


# --- Main Plotting Logic ---
fig, axes = plt.subplots(2, 4, figsize=(16, 7))

# --- Row 1: FP Removal ---
scenario1 = "fp_removal"
ir_img1, nests1, stains1 = generate_synthetic_ir(scenario=scenario1)
seg_mask1 = create_segmentation_overlay(ir_img1, nests1, stains1)
depth_map1, _, _ = generate_depth_map(300, 400, nests1, stains1, scenario1)
final_det1 = create_segmentation_overlay(ir_img1, nests1, []) # Only nests remain

axes[0, 0].imshow(ir_img1, cmap='gray')
axes[0, 1].imshow(seg_mask1)
axes[0, 2].imshow(depth_map1)
draw_depth_overlays(axes[0, 2], nests1, stains1, scenario1)
axes[0, 3].imshow(final_det1)

# --- Row 2: TP Preservation ---
scenario2 = "tp_preservation"
ir_img2, nests2, stains2 = generate_synthetic_ir(scenario=scenario2) # stains2 is empty
seg_mask2 = create_segmentation_overlay(ir_img2, nests2, stains2)
depth_map2, _, _ = generate_depth_map(300, 400, nests2, stains2, scenario2)
final_det2 = create_segmentation_overlay(ir_img2, nests2, stains2) # All nests remain

axes[1, 0].imshow(ir_img2, cmap='gray')
axes[1, 1].imshow(seg_mask2)
axes[1, 2].imshow(depth_map2)
draw_depth_overlays(axes[1, 2], nests2, stains2, scenario2)
axes[1, 3].imshow(final_det2)

# --- Formatting ---
cols = ['IR image', 'IR-SegNet masks\n(appearance only)', 'Depth map with overlay', 'Final detections after\ndepth validation']
for ax, col_name in zip(axes[0], cols):
    ax.set_title(col_name, fontweight='bold')

for ax in axes.flatten():
    ax.set_xticks([])
    ax.set_yticks([])

# Figure Caption
fig.text(0.5, 0.01, 'Figure 14 – Qualitative examples of depth-based refinement. Row 1: Removal of wall stains (orange masks) that lack the concave geometry of true nests. Row 2: Preservation of multiple true nests (blue masks) at different distances, which all satisfy the geometric validation criteria.', 
         ha='center', fontsize=12)

plt.tight_layout()
plt.subplots_adjust(bottom=0.1) # Make room for the caption
plt.show()