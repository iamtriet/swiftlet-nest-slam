#!/usr/bin/env python3
"""
Visualize SLAM Export Session Data
Shows 3D map and detected objects using matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
import sys
import os

def load_pcd(filename):
    """Load PCD file (ASCII format)"""
    points = []
    data_section = False
    
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('DATA'):
                data_section = True
                continue
            
            if data_section:
                parts = line.strip().split()
                if len(parts) >= 3:
                    try:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        points.append([x, y, z])
                    except:
                        continue
    
    return np.array(points)

def visualize_session(session_dir):
    """Visualize complete session data"""
    
    # Load object data
    objects_file = os.path.join(session_dir, 'detected_objects.json')
    with open(objects_file, 'r') as f:
        objects_data = json.load(f)
    
    # Load SLAM map if it exists
    map_file = os.path.join(session_dir, 'slam_map.pcd')
    map_points = None
    if os.path.exists(map_file):
        print(f"📊 Loading SLAM map from {map_file}...")
        map_points = load_pcd(map_file)
        print(f"   ✅ Loaded {len(map_points)} map points")
    else:
        print(f"ℹ️  No SLAM map found (run session longer for map to be generated)")
    
    # Load object positions
    objects_file_pcd = os.path.join(session_dir, 'object_positions.pcd')
    object_points = None
    if os.path.exists(objects_file_pcd):
        print(f"📍 Loading object positions...")
        object_points = load_pcd(objects_file_pcd)
        print(f"   ✅ Loaded {len(object_points)} objects")
    
    # Create visualization
    fig = plt.figure(figsize=(16, 12))
    
    # === PLOT 1: Enhanced 3D View with Grid ===
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    
    # Plot SLAM map if available
    if map_points is not None and len(map_points) > 0:
        # Subsample for performance (plot every Nth point)
        subsample = max(1, len(map_points) // 10000)
        map_subset = map_points[::subsample]
        ax1.scatter(map_subset[:, 0], map_subset[:, 1], map_subset[:, 2], 
                   c='lightgray', s=1, alpha=0.4, label='SLAM Map', depthshade=True)
        print(f"   ✅ Plotted {len(map_subset)} map points (subsampled from {len(map_points)})")
    
    # Plot object positions with enhanced 3D representation
    if object_points is not None and len(object_points) > 0:
        # Determine bounds for grid
        x_min, x_max = object_points[:, 0].min(), object_points[:, 0].max()
        y_min, y_max = object_points[:, 1].min(), object_points[:, 1].max()
        z_min, z_max = object_points[:, 2].min(), object_points[:, 2].max()
        
        # Add padding
        padding = 0.5
        x_min, x_max = x_min - padding, x_max + padding
        y_min, y_max = y_min - padding, y_max + padding
        
        # Create a grid floor at z_min
        if map_points is None:  # Only show grid if no SLAM map
            grid_x = np.linspace(x_min, x_max, 10)
            grid_y = np.linspace(y_min, y_max, 10)
            grid_X, grid_Y = np.meshgrid(grid_x, grid_y)
            grid_Z = np.ones_like(grid_X) * (z_min - 0.1)
            ax1.plot_surface(grid_X, grid_Y, grid_Z, alpha=0.1, color='lightblue', 
                           edgecolor='gray', linewidth=0.5)
        
        # Plot vertical lines from objects to floor
        for pos in object_points:
            ax1.plot([pos[0], pos[0]], [pos[1], pos[1]], [z_min - 0.1, pos[2]], 
                    'r--', alpha=0.3, linewidth=1)
        
        # Plot objects as large stars
        ax1.scatter(object_points[:, 0], object_points[:, 1], object_points[:, 2],
                   c='red', s=300, marker='*', edgecolors='darkred', linewidths=2,
                   label='Detected Objects', zorder=100)
        
        # Add object labels with boxes
        for i, (obj, pos) in enumerate(zip(objects_data['objects'], object_points)):
            ax1.text(pos[0], pos[1], pos[2] + 0.1, f"  G#{obj['global_id']}", 
                    fontsize=9, color='darkred', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7, edgecolor='red'))
        
        # Add distance lines between nearest objects (optional, for top 3)
        sorted_objs = sorted(zip(objects_data['objects'], object_points), 
                           key=lambda x: x[0]['detection_count'], reverse=True)
        if len(sorted_objs) >= 2:
            # Draw line between top 2 detected objects
            obj1_pos = sorted_objs[0][1]
            obj2_pos = sorted_objs[1][1]
            ax1.plot([obj1_pos[0], obj2_pos[0]], 
                    [obj1_pos[1], obj2_pos[1]], 
                    [obj1_pos[2], obj2_pos[2]], 
                    'b--', alpha=0.5, linewidth=2, label='Top objects distance')
            
            # Calculate and display distance
            dist = np.linalg.norm(obj1_pos - obj2_pos)
            mid_point = (obj1_pos + obj2_pos) / 2
            ax1.text(mid_point[0], mid_point[1], mid_point[2], 
                    f'{dist:.2f}m', fontsize=8, color='blue', 
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    ax1.set_xlabel('X (meters)', fontweight='bold')
    ax1.set_ylabel('Y (meters)', fontweight='bold')
    ax1.set_zlabel('Z (meters)', fontweight='bold')
    
    title = '3D SLAM Map with Object Positions' if map_points is not None else '3D Object Positions in Space'
    ax1.set_title(title, fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # Set viewing angle for better perspective
    ax1.view_init(elev=20, azim=45)
    
    # === PLOT 2: Enhanced Top-Down View (X-Y plane) ===
    ax2 = fig.add_subplot(2, 2, 2)
    
    if map_points is not None and len(map_points) > 0:
        subsample = max(1, len(map_points) // 5000)
        map_subset = map_points[::subsample]
        ax2.scatter(map_subset[:, 0], map_subset[:, 1], 
                   c='lightgray', s=2, alpha=0.4, label='SLAM Map')
    
    if object_points is not None and len(object_points) > 0:
        # Color code by detection count
        detection_counts = [obj['detection_count'] for obj in objects_data['objects']]
        max_count = max(detection_counts)
        
        scatter = ax2.scatter(object_points[:, 0], object_points[:, 1],
                   c=detection_counts, cmap='YlOrRd', s=300, marker='*', 
                   edgecolors='black', linewidths=2, label='Objects', zorder=10,
                   vmin=0, vmax=max_count)
        
        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax2, pad=0.02)
        cbar.set_label('Detection Count', fontweight='bold')
        
        # Add labels with arrows
        for i, (obj, pos) in enumerate(zip(objects_data['objects'], object_points)):
            ax2.annotate(f"G#{obj['global_id']}\n({obj['detection_count']})", 
                        (pos[0], pos[1]), 
                        xytext=(10, 10), textcoords='offset points',
                        fontsize=8, color='darkred', fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.4', facecolor='yellow', alpha=0.7),
                        arrowprops=dict(arrowstyle='->', color='red', lw=1.5))
        
        # Draw grid
        ax2.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
        
        # Add distance scale bar
        x_range = object_points[:, 0].max() - object_points[:, 0].min()
        scale_length = max(0.5, round(x_range * 0.2, 1))  # 20% of range
        x_pos = object_points[:, 0].min() + 0.1
        y_pos = object_points[:, 1].min() + 0.1
        ax2.plot([x_pos, x_pos + scale_length], [y_pos, y_pos], 
                'k-', linewidth=3)
        ax2.text(x_pos + scale_length/2, y_pos - 0.15, f'{scale_length}m',
                ha='center', fontsize=9, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    ax2.set_xlabel('X (meters)', fontweight='bold')
    ax2.set_ylabel('Y (meters)', fontweight='bold')
    ax2.set_title('Top-Down View (Color = Detection Count)', fontsize=14, fontweight='bold')
    ax2.axis('equal')
    
    # Add compass rose
    if object_points is not None and len(object_points) > 0:
        x_max, y_max = object_points[:, 0].max(), object_points[:, 1].max()
        compass_x, compass_y = x_max - 0.3, y_max - 0.3
        ax2.arrow(compass_x, compass_y, 0.2, 0, head_width=0.08, head_length=0.05, 
                 fc='blue', ec='blue', alpha=0.6)
        ax2.text(compass_x + 0.25, compass_y, 'X', fontsize=10, fontweight='bold', color='blue')
        ax2.arrow(compass_x, compass_y, 0, 0.2, head_width=0.08, head_length=0.05,
                 fc='green', ec='green', alpha=0.6)
        ax2.text(compass_x, compass_y + 0.25, 'Y', fontsize=10, fontweight='bold', color='green')
    
    # === PLOT 3: Side View (X-Z plane) for 3D Position Clarity ===
    ax3 = fig.add_subplot(2, 2, 3)
    
    if map_points is not None and len(map_points) > 0:
        subsample = max(1, len(map_points) // 5000)
        map_subset = map_points[::subsample]
        ax3.scatter(map_subset[:, 0], map_subset[:, 2], 
                   c='lightgray', s=2, alpha=0.4, label='SLAM Map')
    
    if object_points is not None and len(object_points) > 0:
        # Color code by detection count
        detection_counts = [obj['detection_count'] for obj in objects_data['objects']]
        max_count = max(detection_counts)
        
        scatter = ax3.scatter(object_points[:, 0], object_points[:, 2],
                   c=detection_counts, cmap='YlOrRd', s=300, marker='*', 
                   edgecolors='black', linewidths=2, label='Objects', zorder=10,
                   vmin=0, vmax=max_count)
        
        # Add labels
        for i, (obj, pos) in enumerate(zip(objects_data['objects'], object_points)):
            ax3.annotate(f"G#{obj['global_id']}", 
                        (pos[0], pos[2]), 
                        xytext=(5, 5), textcoords='offset points',
                        fontsize=8, color='darkred', fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
        
        # Draw ground line
        x_min, x_max = object_points[:, 0].min(), object_points[:, 0].max()
        z_ground = object_points[:, 2].min() - 0.2
        ax3.axhline(y=z_ground, color='brown', linestyle='--', linewidth=2, 
                   alpha=0.5, label='Ground Reference')
        
        # Draw vertical position lines
        for pos in object_points:
            ax3.plot([pos[0], pos[0]], [z_ground, pos[2]], 
                    'r--', alpha=0.3, linewidth=1)
        
        ax3.grid(True, alpha=0.3, linestyle='--')
    
    ax3.set_xlabel('X (meters)', fontweight='bold')
    ax3.set_ylabel('Z (meters) - Height', fontweight='bold')
    ax3.set_title('Side View - Object Heights', fontsize=14, fontweight='bold')
    ax3.legend(loc='best', fontsize=9)
    
    # Sort objects by detection count
    sorted_objects = sorted(objects_data['objects'], 
                          key=lambda x: x['detection_count'], reverse=True)
    
    global_ids = [f"G#{obj['global_id']}" for obj in sorted_objects]
    detection_counts = [obj['detection_count'] for obj in sorted_objects]
    
    bars = ax3.barh(global_ids, detection_counts, color='steelblue', edgecolor='black')
    ax3.set_xlabel('Detection Count')
    ax3.set_title('Objects by Detection Count', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3, axis='x')
    
    # Add value labels on bars
    for i, (bar, count) in enumerate(zip(bars, detection_counts)):
        ax3.text(count, i, f' {count}', va='center', fontsize=9, fontweight='bold')
    
    # === PLOT 4: Detection Statistics and Object Table ===
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.axis('off')
    
    # Load statistics
    stats_file = os.path.join(session_dir, 'tracking_statistics.json')
    with open(stats_file, 'r') as f:
        stats = json.load(f)
    
    # Sort objects by detection count
    sorted_objects = sorted(objects_data['objects'], 
                          key=lambda x: x['detection_count'], reverse=True)
    
    # Create detailed table
    summary_text = f"""SESSION SUMMARY
{'='*60}
Timestamp: {objects_data['session_timestamp']}
Camera: {objects_data['camera_mode'].upper()}
Tracking: {stats['tracking_mode']}
Total Frames: {stats['total_frames_processed']}
Average FPS: {stats['average_fps']:.2f}
Unique Objects: {objects_data['total_unique_objects']}

OBJECT POSITIONS (sorted by detection count)
{'='*60}
ID  | Position (X, Y, Z) meters     | Detections | Duration
{'─'*60}
"""
    
    for obj in sorted_objects:
        pos = obj['position']
        summary_text += f"G#{obj['global_id']:<2} | ({pos['x']:>5.2f}, {pos['y']:>5.2f}, {pos['z']:>5.2f}) | {obj['detection_count']:>10} | {obj['duration_seconds']:>6.1f}s\n"
    
    # Add distance matrix for top objects
    if len(sorted_objects) >= 3:
        summary_text += f"\n{'='*60}\nDISTANCES BETWEEN TOP 3 OBJECTS (meters)\n{'='*60}\n"
        top_3 = sorted_objects[:3]
        for i in range(len(top_3)):
            for j in range(i+1, len(top_3)):
                pos_i = np.array([top_3[i]['position']['x'], 
                                 top_3[i]['position']['y'],
                                 top_3[i]['position']['z']])
                pos_j = np.array([top_3[j]['position']['x'],
                                 top_3[j]['position']['y'],
                                 top_3[j]['position']['z']])
                dist = np.linalg.norm(pos_i - pos_j)
                summary_text += f"G#{top_3[i]['global_id']} ↔ G#{top_3[j]['global_id']}: {dist:.2f}m\n"
    
    ax4.text(0.05, 0.95, summary_text, transform=ax4.transAxes,
            fontsize=8, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8, edgecolor='orange'))
    
    # Main title
    fig.suptitle(f'Semantic SLAM Session Analysis - {objects_data["total_unique_objects"]} Objects Detected',
                fontsize=16, fontweight='bold', y=0.98)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Save figure
    output_file = os.path.join(session_dir, 'visualization.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✅ Visualization saved: {output_file}")
    
    # Show interactive plot
    print("\n🎨 Showing interactive plot (close window to exit)...")
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        session_dir = sys.argv[1]
    else:
        # Find most recent session
        exports_dir = os.path.expanduser('~/catkin_ws/slam_exports')
        sessions = [d for d in os.listdir(exports_dir) if d.startswith('session_')]
        if not sessions:
            print("❌ No sessions found in ~/catkin_ws/slam_exports/")
            sys.exit(1)
        
        sessions.sort(reverse=True)
        session_dir = os.path.join(exports_dir, sessions[0])
    
    print(f"📂 Visualizing session: {session_dir}\n")
    visualize_session(session_dir)
