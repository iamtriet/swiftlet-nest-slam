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
print(f"Image dtype: {img.dtype}")

# Convert to grayscale for IR-like processing
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Create figure with multiple views
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

# Original image (BGR to RGB for display)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
axes[0, 0].imshow(img_rgb)
axes[0, 0].set_title('Original Image (RGB)', fontweight='bold')
axes[0, 0].axis('off')

# Grayscale version
axes[0, 1].imshow(img_gray, cmap='gray')
axes[0, 1].set_title('Grayscale (IR-like)', fontweight='bold')
axes[0, 1].axis('off')

# Apply edge detection (Canny)
edges = cv2.Canny(img_gray, 50, 150)
axes[1, 0].imshow(edges, cmap='gray')
axes[1, 0].set_title('Edge Detection (Canny)', fontweight='bold')
axes[1, 0].axis('off')

# Create a pseudo-depth map using intensity gradient
# Darker areas are closer (lower depth values)
depth_pseudo = 255 - img_gray  # Invert so dark = near
depth_color = cv2.applyColorMap(depth_pseudo, cv2.COLORMAP_PLASMA)
depth_color_rgb = cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)
axes[1, 1].imshow(depth_color_rgb)
axes[1, 1].set_title('Pseudo-Depth Map\n(Blue=Near, Yellow=Far)', fontweight='bold')
axes[1, 1].axis('off')

# Add figure title
fig.suptitle(f'Analysis of: {image_path.split("/")[-1]}', fontsize=14, fontweight='bold')

plt.tight_layout()
plt.show()

print("\nProcessing complete!")
