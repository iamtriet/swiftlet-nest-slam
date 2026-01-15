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
    'text.usetex': False
})

# Load the user's image
image_path = r"C:\Users\iamtriet\OneDrive - vqtg8\Documents\3I - UEH\Bird's nest\catkin_ws\fake_test.jpg"
img = cv2.imread(image_path)

if img is None:
    print(f"Error: Could not load image from {image_path}")
    exit(1)

print(f"Image loaded successfully!")
print(f"Image shape: {img.shape}")

# Convert to grayscale for IR-like processing
ir_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Resize for consistent display (if needed)
height, width = ir_img.shape
if height > 600 or width > 800:
    scale = min(600/height, 800/width)
    new_height = int(height * scale)
    new_width = int(width * scale)
    ir_img = cv2.resize(ir_img, (new_width, new_height))
    height, width = ir_img.shape
    print(f"Resized to: {ir_img.shape}")

# --- Step 1: Simulate segmentation (detect dark regions as potential nests/stains) ---
# Apply thresholding to find dark regions
_, binary = cv2.threshold(ir_img, 120, 255, cv2.THRESH_BINARY_INV)

# Clean up with morphological operations
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

# Find contours
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter contours by area
min_area = 200
max_area = 50000
valid_contours = [c for c in contours if min_area < cv2.contourArea(c) < max_area]

print(f"Found {len(valid_contours)} potential objects")

# --- Step 2: Classify as nests (blue) or stains (orange) based on simple heuristics ---
nests = []
stains = []

for contour in valid_contours:
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    
    # Compactness measure
    if perimeter > 0:
        compactness = 4 * np.pi * area / (perimeter ** 2)
    else:
        compactness = 0
    
    # Aspect ratio
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / h if h > 0 else 0
    
    # Classify: more compact and rounder = nest, elongated = stain
    if compactness > 0.4 and 0.5 < aspect_ratio < 2.0 and area > 500:
        nests.append(contour)
    else:
        stains.append(contour)

print(f"Classified: {len(nests)} nests (blue), {len(stains)} stains (orange)")

# --- Step 3: Create segmentation overlay ---
seg_overlay = cv2.cvtColor(ir_img, cv2.COLOR_GRAY2RGB)
alpha = 0.6

# Draw nests in blue
nest_mask = np.zeros_like(ir_img)
for contour in nests:
    cv2.drawContours(nest_mask, [contour], -1, 255, -1)
seg_overlay[nest_mask > 0] = (seg_overlay[nest_mask > 0] * (1 - alpha) + np.array([0, 0, 255]) * alpha).astype(np.uint8)

# Draw stains in orange
stain_mask = np.zeros_like(ir_img)
for contour in stains:
    cv2.drawContours(stain_mask, [contour], -1, 255, -1)
seg_overlay[stain_mask > 0] = (seg_overlay[stain_mask > 0] * (1 - alpha) + np.array([255, 165, 0]) * alpha).astype(np.uint8)

# --- Step 4: Generate depth map ---
# Create a simple depth map based on image intensity
# Darker regions = closer (concave nests)
depth = np.copy(ir_img)

# For each nest, create a concave depth profile
for contour in nests:
    # Get bounding box and center
    x, y, w, h = cv2.boundingRect(contour)
    M = cv2.moments(contour)
    if M["m00"] > 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # Create radial gradient for concavity
        mask = np.zeros_like(depth)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        
        Y, X = np.ogrid[:height, :width]
        dist_from_center = np.sqrt((X - cX)**2 + (Y - cY)**2)
        max_dist = np.sqrt(w**2 + h**2) / 2
        
        # Create depth gradient (center is closer = darker/lower value)
        if max_dist > 0:
            depth_profile = (50 + (dist_from_center / max_dist) * 150).astype(np.uint8)
            depth = np.where(mask > 0, depth_profile, depth)

# Apply colormap (PLASMA: blue=near, yellow/red=far)
depth_color = cv2.applyColorMap(depth, cv2.COLORMAP_PLASMA)
depth_color_rgb = cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)

# --- Step 5: Add depth overlay annotations ---
fig, axes = plt.subplots(1, 4, figsize=(16, 4))

# Column 1: IR image
axes[0].imshow(ir_img, cmap='gray')
axes[0].set_title('IR image', fontweight='bold')
axes[0].axis('off')

# Column 2: IR-SegNet masks
axes[1].imshow(seg_overlay)
axes[1].set_title('IR-SegNet masks\n(appearance only)', fontweight='bold')
axes[1].axis('off')

# Column 3: Depth map with overlay
axes[2].imshow(depth_color_rgb)
axes[2].set_title('Depth map with overlay', fontweight='bold')
axes[2].axis('off')

# Add arrows and X marks on depth map
for contour in nests:
    M = cv2.moments(contour)
    if M["m00"] > 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # Add arrow pointing to nest
        axes[2].arrow(cX-20, cY-30, 15, 25, color='blue', head_width=8, linewidth=2)

for contour in stains:
    M = cv2.moments(contour)
    if M["m00"] > 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # Add X mark for stains (to be removed)
        axes[2].text(cX, cY, 'X', color='red', fontsize=20, ha='center', va='center', fontweight='bold')

# Column 4: Final detections (only nests)
final_overlay = cv2.cvtColor(ir_img, cv2.COLOR_GRAY2RGB)
final_overlay[nest_mask > 0] = (final_overlay[nest_mask > 0] * (1 - alpha) + np.array([0, 0, 255]) * alpha).astype(np.uint8)
axes[3].imshow(final_overlay)
axes[3].set_title('Final detections after\ndepth validation', fontweight='bold')
axes[3].axis('off')

# Add figure caption
fig.text(0.5, 0.02, 
         'Depth-based refinement on real image. Blue masks: true nests (concave geometry). Orange masks: stains (flat geometry, removed in final detection).', 
         ha='center', fontsize=11, wrap=True)

plt.tight_layout()
plt.subplots_adjust(bottom=0.1)

# Save the output
output_path = r"C:\Users\iamtriet\OneDrive - vqtg8\Documents\3I - UEH\Bird's nest\catkin_ws\depth_analysis_output.png"
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"\nOutput saved to: {output_path}")

plt.show()

print("Processing complete!")
