#!/usr/bin/env python3
"""
IR Preprocessing Pipeline Visualizer
Shows each step of the enhancement process with detailed metrics

Usage:
    python visualize_preprocessing_pipeline.py <image_path>

Example:
    python visualize_preprocessing_pipeline.py test_ir.jpg
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import sys
import os

def calculate_metrics(image):
    """Calculate image quality metrics"""
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    metrics = {
        'mean': np.mean(gray),
        'std': np.std(gray),
        'min': np.min(gray),
        'max': np.max(gray),
        'contrast': np.max(gray) - np.min(gray)
    }
    return metrics

def enhance_infrared_image_stepwise(ir_image):
    """
    Enhanced IR image preprocessing with step-by-step outputs
    Returns a dictionary with each processing step
    """
    steps = {}
    
    # Handle both grayscale and color input
    if len(ir_image.shape) == 3:
        ir_image = cv2.cvtColor(ir_image, cv2.COLOR_BGR2GRAY)
    
    steps['0_original'] = {
        'image': ir_image.copy(),
        'title': 'Original IR Image',
        'description': 'Raw infrared camera output\nTypically low contrast, darker',
        'metrics': calculate_metrics(ir_image)
    }
    
    # Step 1: Normalize to full 0-255 range
    ir_norm = cv2.normalize(ir_image, None, 0, 255, cv2.NORM_MINMAX)
    steps['1_normalized'] = {
        'image': ir_norm.copy(),
        'title': 'Step 1: Normalization',
        'description': 'Stretch to full 0-255 range\nMaximizes dynamic range',
        'metrics': calculate_metrics(ir_norm)
    }
    
    # Step 2: CLAHE (Contrast Limited Adaptive Histogram Equalization)
    clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(ir_norm.astype(np.uint8))
    steps['2_clahe'] = {
        'image': enhanced.copy(),
        'title': 'Step 2: CLAHE',
        'description': 'Adaptive contrast enhancement\nclipLimit=4.0, tiles=8×8\nBoosts local contrast',
        'metrics': calculate_metrics(enhanced)
    }
    
    # Step 3: Sharpen edges
    kernel = np.array([[-1, -1, -1],
                       [-1,  9, -1],
                       [-1, -1, -1]])
    sharpened = cv2.filter2D(enhanced, -1, kernel)
    sharpened = np.clip(sharpened, 0, 255).astype(np.uint8)
    steps['3_sharpened'] = {
        'image': sharpened.copy(),
        'title': 'Step 3: Edge Sharpening',
        'description': 'Laplacian kernel sharpening\nEmphasizes edges and texture\nEnhances nest boundaries',
        'metrics': calculate_metrics(sharpened)
    }
    
    # Step 4: Bilateral filter
    bilateral = cv2.bilateralFilter(sharpened, 5, 50, 50)
    steps['4_bilateral'] = {
        'image': bilateral.copy(),
        'title': 'Step 4: Bilateral Filtering',
        'description': 'Edge-preserving denoising\nd=5, σ_color=50, σ_space=50\nReduces noise, keeps edges',
        'metrics': calculate_metrics(bilateral)
    }
    
    # Step 5: Convert to BGR
    enhanced_bgr = cv2.cvtColor(bilateral, cv2.COLOR_GRAY2BGR)
    steps['5_bgr'] = {
        'image': enhanced_bgr.copy(),
        'title': 'Step 5: Grayscale → BGR',
        'description': 'Convert to 3-channel\nRequired for input\nR=G=B (grayscale preserved)',
        'metrics': calculate_metrics(enhanced_bgr)
    }
    
    # Step 6: Gamma correction
    gamma = 1.3
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 
                      for i in range(256)]).astype("uint8")
    gamma_corrected = cv2.LUT(enhanced_bgr, table)
    steps['6_gamma'] = {
        'image': gamma_corrected.copy(),
        'title': 'Step 6: Gamma Correction',
        'description': f'Gamma = {gamma}\nBrightens mid-tones\nFinal output',
        'metrics': calculate_metrics(gamma_corrected)
    }
    
    return steps

def create_histogram(image, color='gray'):
    """Create histogram for an image"""
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
    return hist.flatten()

def visualize_pipeline(image_path, save_output=True):
    """
    Create comprehensive visualization of the preprocessing pipeline
    """
    # Read image
    ir_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if ir_image is None:
        print(f"❌ Failed to read image: {image_path}")
        return
    
    print("="*60)
    print("📷 IR Preprocessing Pipeline Visualizer")
    print("="*60)
    print(f"   Input: {image_path}")
    print(f"   Size: {ir_image.shape[1]}×{ir_image.shape[0]} pixels")
    print("="*60)
    
    # Process image step-by-step
    steps = enhance_infrared_image_stepwise(ir_image)
    
    # Create comprehensive figure
    fig = plt.figure(figsize=(20, 12))
    gs = GridSpec(3, 7, figure=fig, hspace=0.4, wspace=0.3)
    
    # Plot each step
    step_keys = ['0_original', '1_normalized', '2_clahe', '3_sharpened', 
                 '4_bilateral', '5_bgr', '6_gamma']
    
    for idx, key in enumerate(step_keys):
        step_data = steps[key]
        
        # Main image (row 0-1)
        ax_img = fig.add_subplot(gs[0:2, idx])
        
        if len(step_data['image'].shape) == 3:
            # Convert BGR to RGB for display
            display_img = cv2.cvtColor(step_data['image'], cv2.COLOR_BGR2RGB)
            ax_img.imshow(display_img)
        else:
            ax_img.imshow(step_data['image'], cmap='gray', vmin=0, vmax=255)
        
        ax_img.set_title(step_data['title'], fontweight='bold', fontsize=10)
        ax_img.axis('off')
        
        # Add description text
        ax_img.text(0.5, -0.05, step_data['description'], 
                   transform=ax_img.transAxes,
                   ha='center', va='top', fontsize=8,
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
        
        # Histogram (row 2)
        ax_hist = fig.add_subplot(gs[2, idx])
        hist = create_histogram(step_data['image'])
        ax_hist.fill_between(range(256), hist, alpha=0.7, color='steelblue')
        ax_hist.set_xlim([0, 255])
        ax_hist.set_ylim([0, max(hist) * 1.1])
        ax_hist.set_xlabel('Intensity', fontsize=8)
        ax_hist.set_ylabel('Frequency', fontsize=8)
        ax_hist.tick_params(labelsize=7)
        ax_hist.grid(True, alpha=0.3)
        
        # Add metrics text
        metrics = step_data['metrics']
        metrics_text = f"μ={metrics['mean']:.1f}\nσ={metrics['std']:.1f}\nRange=[{metrics['min']},{metrics['max']}]"
        ax_hist.text(0.98, 0.98, metrics_text,
                    transform=ax_hist.transAxes,
                    ha='right', va='top', fontsize=7,
                    bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.3))
        
        print(f"✅ Step {idx}: {step_data['title']}")
        print(f"   Mean: {metrics['mean']:.2f}, Std: {metrics['std']:.2f}, "
              f"Range: [{metrics['min']}, {metrics['max']}]")
    
    # Overall title
    fig.suptitle('IR Image Preprocessing Pipeline - Step-by-Step Visualization',
                fontsize=16, fontweight='bold', y=0.98)
    
    # Add pipeline flow diagram at bottom
    fig.text(0.5, 0.02, 
            '→ Original IR → Normalize → CLAHE → Sharpen → Bilateral Filter → BGR Convert → Gamma Correct → Final Output →',
            ha='center', fontsize=11, style='italic',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    plt.subplots_adjust(top=0.95, bottom=0.08)
    
    # Save output
    if save_output:
        output_path = os.path.splitext(image_path)[0] + '_pipeline_visualization.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print("="*60)
        print(f"💾 Visualization saved to: {output_path}")
        print("="*60)
    
    plt.show()
    
    # Create side-by-side comparison
    print("\n📊 Creating before/after comparison...")
    create_before_after_comparison(steps, image_path, save_output)

def create_before_after_comparison(steps, image_path, save_output=True):
    """
    Create a detailed before/after comparison
    """
    original = steps['0_original']['image']
    final = steps['6_gamma']['image']
    
    # Convert final to grayscale for fair comparison
    if len(final.shape) == 3:
        final_gray = cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
    else:
        final_gray = final
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    
    # Row 1: Images
    axes[0, 0].imshow(original, cmap='gray', vmin=0, vmax=255)
    axes[0, 0].set_title('Before: Original IR Image', fontweight='bold', fontsize=12)
    axes[0, 0].axis('off')
    
    axes[0, 1].imshow(final_gray, cmap='gray', vmin=0, vmax=255)
    axes[0, 1].set_title('After: Enhanced', fontweight='bold', fontsize=12)
    axes[0, 1].axis('off')
    
    # Difference map
    diff = cv2.absdiff(original, final_gray)
    axes[0, 2].imshow(diff, cmap='hot', vmin=0, vmax=255)
    axes[0, 2].set_title('Absolute Difference', fontweight='bold', fontsize=12)
    axes[0, 2].axis('off')
    
    # Row 2: Histograms
    hist_original = create_histogram(original)
    hist_final = create_histogram(final_gray)
    
    axes[1, 0].fill_between(range(256), hist_original, alpha=0.7, color='darkblue', label='Original')
    axes[1, 0].set_title('Original Histogram', fontweight='bold')
    axes[1, 0].set_xlabel('Intensity')
    axes[1, 0].set_ylabel('Frequency')
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].fill_between(range(256), hist_final, alpha=0.7, color='darkgreen', label='Enhanced')
    axes[1, 1].set_title('Enhanced Histogram', fontweight='bold')
    axes[1, 1].set_xlabel('Intensity')
    axes[1, 1].set_ylabel('Frequency')
    axes[1, 1].grid(True, alpha=0.3)
    
    # Overlaid comparison
    axes[1, 2].fill_between(range(256), hist_original, alpha=0.5, color='blue', label='Original')
    axes[1, 2].fill_between(range(256), hist_final, alpha=0.5, color='green', label='Enhanced')
    axes[1, 2].set_title('Histogram Comparison', fontweight='bold')
    axes[1, 2].set_xlabel('Intensity')
    axes[1, 2].set_ylabel('Frequency')
    axes[1, 2].legend()
    axes[1, 2].grid(True, alpha=0.3)
    
    plt.suptitle('Before vs After Comparison', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if save_output:
        output_path = os.path.splitext(image_path)[0] + '_before_after.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"💾 Comparison saved to: {output_path}")
    
    plt.show()

def create_3d_surface_plot(steps, image_path):
    """
    Create 3D surface plots showing intensity changes
    """
    from mpl_toolkits.mplot3d import Axes3D
    
    original = steps['0_original']['image']
    final = steps['6_gamma']['image']
    
    if len(final.shape) == 3:
        final = cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
    
    # Downsample for visualization
    scale = 4
    original_small = cv2.resize(original, (original.shape[1]//scale, original.shape[0]//scale))
    final_small = cv2.resize(final, (final.shape[1]//scale, final.shape[0]//scale))
    
    fig = plt.figure(figsize=(16, 6))
    
    # Create meshgrid
    y, x = np.meshgrid(range(original_small.shape[0]), range(original_small.shape[1]), indexing='ij')
    
    # Original surface
    ax1 = fig.add_subplot(131, projection='3d')
    surf1 = ax1.plot_surface(x, y, original_small, cmap='viridis', alpha=0.8)
    ax1.set_title('Original IR (3D Intensity Surface)', fontweight='bold')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Intensity')
    fig.colorbar(surf1, ax=ax1, shrink=0.5)
    
    # Enhanced surface
    ax2 = fig.add_subplot(132, projection='3d')
    surf2 = ax2.plot_surface(x, y, final_small, cmap='plasma', alpha=0.8)
    ax2.set_title('Enhanced (3D Intensity Surface)', fontweight='bold')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Intensity')
    fig.colorbar(surf2, ax=ax2, shrink=0.5)
    
    # Difference surface
    ax3 = fig.add_subplot(133, projection='3d')
    diff = final_small.astype(float) - original_small.astype(float)
    surf3 = ax3.plot_surface(x, y, diff, cmap='RdBu', alpha=0.8)
    ax3.set_title('Intensity Change (Enhanced - Original)', fontweight='bold')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Δ Intensity')
    fig.colorbar(surf3, ax=ax3, shrink=0.5)
    
    plt.suptitle('3D Intensity Surface Visualization', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    output_path = os.path.splitext(image_path)[0] + '_3d_surfaces.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"💾 3D surfaces saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    # Set matplotlib style
    plt.rcParams.update({
        'font.family': 'serif',
        'axes.labelsize': 10,
        'axes.titlesize': 12,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'figure.titlesize': 14,
    })
    
    if len(sys.argv) < 2:
        print("Usage: python visualize_preprocessing_pipeline.py <image_path>")
        print("\nExample:")
        print("  python visualize_preprocessing_pipeline.py test_ir.jpg")
        sys.exit(1)
    
    image_path = sys.argv[1]
    
    if not os.path.exists(image_path):
        print(f"❌ Error: Image not found: {image_path}")
        sys.exit(1)
    
    # Run visualization
    visualize_pipeline(image_path, save_output=True)
    
    # Ask if user wants 3D visualization
    print("\n" + "="*60)
    print("🎨 Pipeline visualization complete!")
    print("="*60)
