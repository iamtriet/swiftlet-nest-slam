#!/usr/bin/env python3
"""
IR Image Preprocessing Tool
Applies the same enhancement used in SSL_SLAM IR+YOLO system
- CLAHE (Contrast Limited Adaptive Histogram Equalization)
- Edge sharpening
- Bilateral filtering (noise reduction)
- Gamma correction

Usage:
    python3 preprocess_ir_images.py <input_folder> <output_folder>
    
Example:
    python3 preprocess_ir_images.py ir_raw/ ir_enhanced/
"""

import cv2
import numpy as np
import os
import sys
from pathlib import Path

def enhance_infrared_image(ir_image):
    """
    Enhanced IR image preprocessing - same as SSL_SLAM system
    Makes IR images more compatible with RGB-trained YOLO models
    """
    # Handle both grayscale and color input
    if len(ir_image.shape) == 3:
        ir_image = cv2.cvtColor(ir_image, cv2.COLOR_BGR2GRAY)
    
    # 1. Normalize to full 0-255 range
    ir_norm = cv2.normalize(ir_image, None, 0, 255, cv2.NORM_MINMAX)
    
    # 2. Aggressive CLAHE (Contrast Limited Adaptive Histogram Equalization)
    # clipLimit=4.0 prevents over-amplification of noise
    clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(ir_norm.astype(np.uint8))
    
    # 3. Sharpen edges to enhance features
    kernel = np.array([[-1, -1, -1],
                       [-1,  9, -1],
                       [-1, -1, -1]])
    enhanced = cv2.filter2D(enhanced, -1, kernel)
    enhanced = np.clip(enhanced, 0, 255).astype(np.uint8)
    
    # 4. Bilateral filter (reduces noise while preserving edges)
    enhanced = cv2.bilateralFilter(enhanced, 5, 50, 50)
    
    # 5. Convert grayscale to 3-channel BGR (for YOLO compatibility)
    enhanced_bgr = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)
    
    # 6. Gamma correction (brighten image, γ=1.3)
    gamma = 1.3
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 
                      for i in range(256)]).astype("uint8")
    enhanced_bgr = cv2.LUT(enhanced_bgr, table)
    
    return enhanced_bgr

def process_folder(input_folder, output_folder, show_preview=False):
    """
    Process all images in input folder and save to output folder
    """
    # Create output folder if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)
    
    # Supported image formats
    image_extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff']
    
    # Find all image files
    input_path = Path(input_folder)
    image_files = []
    for ext in image_extensions:
        image_files.extend(input_path.glob(f'*{ext}'))
        image_files.extend(input_path.glob(f'*{ext.upper()}'))
    
    if len(image_files) == 0:
        print(f"❌ No images found in {input_folder}")
        return
    
    print("=" * 60)
    print("📷 IR Image Preprocessing Tool")
    print("=" * 60)
    print(f"   Input folder: {input_folder}")
    print(f"   Output folder: {output_folder}")
    print(f"   Found {len(image_files)} images")
    print("=" * 60)
    print("   Processing steps:")
    print("   1. Normalization")
    print("   2. CLAHE (clipLimit=4.0)")
    print("   3. Edge sharpening")
    print("   4. Bilateral filtering")
    print("   5. Gamma correction (γ=1.3)")
    print("=" * 60)
    
    # Process each image
    processed_count = 0
    for image_file in sorted(image_files):
        try:
            # Read image
            ir_image = cv2.imread(str(image_file), cv2.IMREAD_GRAYSCALE)
            if ir_image is None:
                print(f"⚠️  Failed to read: {image_file.name}")
                continue
            
            # Apply enhancement
            enhanced = enhance_infrared_image(ir_image)
            
            # Save enhanced image
            output_path = os.path.join(output_folder, image_file.name)
            cv2.imwrite(output_path, enhanced)
            
            processed_count += 1
            print(f"✅ [{processed_count}/{len(image_files)}] {image_file.name}")
            
            # Optional: show preview
            if show_preview:
                # Create side-by-side comparison
                ir_display = cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)
                comparison = np.hstack([ir_display, enhanced])
                
                # Add labels
                cv2.putText(comparison, "Original", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(comparison, "Enhanced", (ir_image.shape[1] + 10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow("IR Enhancement - Press any key to continue", comparison)
                cv2.waitKey(500)  # Show for 500ms
                
        except Exception as e:
            print(f"❌ Error processing {image_file.name}: {e}")
    
    if show_preview:
        cv2.destroyAllWindows()
    
    print("=" * 60)
    print(f"✅ Completed! Processed {processed_count}/{len(image_files)} images")
    print(f"📁 Enhanced images saved to: {os.path.abspath(output_folder)}")
    print("=" * 60)

def process_single_image(input_path, output_path=None, show=True):
    """
    Process a single image and optionally display it
    """
    # Read image
    ir_image = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)
    if ir_image is None:
        print(f"❌ Failed to read image: {input_path}")
        return
    
    print(f"📷 Processing: {input_path}")
    print(f"   Original size: {ir_image.shape}")
    
    # Apply enhancement
    enhanced = enhance_infrared_image(ir_image)
    
    # Save if output path provided
    if output_path:
        cv2.imwrite(output_path, enhanced)
        print(f"✅ Saved enhanced image to: {output_path}")
    
    # Show comparison
    if show:
        ir_display = cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)
        comparison = np.hstack([ir_display, enhanced])
        
        cv2.putText(comparison, "Original IR", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(comparison, "Enhanced (SLAM Processing)", (ir_image.shape[1] + 10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        cv2.imshow("IR Enhancement Comparison - Press any key to close", comparison)
        print("👁️  Showing comparison (press any key to close)")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    process_single_image('image copy.png', output_path='enhanced_test2_ir_image.png', show=True)
