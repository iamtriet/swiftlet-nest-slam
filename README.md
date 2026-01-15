# Swiftlet Nest SLAM System

A ROS-based Simultaneous Localization and Mapping (SLAM) system for swiftlet nest detection and mapping using semantic segmentation, object detection/segmentation, and infrared imaging with solid-state LiDAR.

![Laboratory Result](laboratory%20result.png)

## Features

- **Multi-mode SLAM Support**: Semantic SLAM, MMS-SLAM, and SSL-SLAM implementations
- **IR Image Processing**: Enhanced infrared image processing for nest detection
- **Deep Learning Integration**: Real-time object detection with tracking
- **3D Mapping**: Point cloud generation and visualization
- **Session Recording**: Automatic export of detected objects and tracking statistics
- **Performance Monitoring**: Built-in diagnostics and performance checking tools

## Prerequisites

### Hardware Requirements
- Intel RealSense Camera (D435/D455 recommended)
- Computer with GPU support (for YOLO inference)

### Software Requirements
- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- CUDA (for GPU acceleration)

### Dependencies
- OpenCV
- PyTorch
- YOLOv8 (Ultralytics)
- RealSense SDK
- PCL (Point Cloud Library)
- ROS packages: `realsense-ros`, `tf`, `cv_bridge`

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/iamtriet/swiftlet-nest-slam.git
cd swiftlet-nest-slam
```

### 2. Install ROS Dependencies
```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
sudo apt-get install ros-$ROS_DISTRO-cv-bridge
```

### 3. Install Python Dependencies
```bash
pip install -r requirements.txt
# If requirements.txt doesn't exist, install manually:
pip install torch torchvision opencv-python ultralytics pyrealsense2 numpy open3d
```

### 4. Build the Workspace
```bash
./rebuild.sh
# Or manually:
catkin_make
source devel/setup.bash
```

## Usage

### Quick Start

#### 1. Basic Semantic SLAM
```bash
./start_semantic_slam.sh
```

#### 2. Semantic SLAM with IR Enhancement
```bash
./start_semantic_slam_IR.sh
```

#### 3. Semantic SLAM with YOLO Detection
```bash
./start_semantic_slam_IR_YOLO.sh
```

#### 4. Full System with Object Tracking (Recommended)
```bash
./start_semantic_slam_IR_YOLO_tracking_optimized.sh
```

#### 5. MMS-SLAM Mode
```bash
./start_mms_slam.sh
```

### Advanced Usage

#### IR Image Preprocessing
Process infrared images for better nest detection:
```bash
python preprocess_ir_images.py
```

#### Visualize Preprocessing Pipeline
```bash
python visualize_preprocessing_pipeline.py
```

#### Model Testing
```bash
python modelTesting.py
```

#### Plane Extraction
```bash
python planeExtraction.py
```

### Monitoring and Diagnostics

#### Check System Performance
```bash
./check_performance.sh
```

#### Check MMS-SLAM Status
```bash
./check_mms_status.sh
```

#### Check TF Transforms
```bash
./check_tf_transforms.sh
```

#### Diagnose Semantic SLAM
```bash
./diagnose_semantic_slam.sh
```

## Project Structure

```
.
├── src/
│   ├── mms_slam/          # MMS-SLAM implementation
│   ├── realsense-ros/     # RealSense camera drivers
│   └── ssl_slam/          # SSL-SLAM implementation
├── slam_exports/          # Session data and visualizations
├── raw-data/              # Raw IR images
├── pre-processing/        # Preprocessed images
├── images/                # Sample images and results
├── training_plots/        # Training visualization plots
├── scripts/               # Shell scripts for launching
├── *.py                   # Python utilities
└── README.md
```

## Session Data

SLAM sessions are automatically exported to `slam_exports/session_YYYYMMDD_HHMMSS/` containing:
- `detected_objects.json` - List of detected nests/objects
- `object_positions.pcd` - 3D point cloud data
- `tracking_statistics.json` - Tracking metrics
- `visualization.png` - Session visualization
- `README.txt` - Session summary

### Visualize Session Data
```bash
cd slam_exports
python visualize_session.py
```

See [VISUALIZATION_GUIDE.md](slam_exports/VISUALIZATION_GUIDE.md) for details.

## Configuration

### Camera Settings
Reset camera if needed:
```bash
python reset_camera.py
```

### Training
Verify training setup:
```bash
./verify_training_setup.sh
```

Create training curves:
```bash
python create_training_curves.py
python create_pr_curves.py
```

### Model Comparison
Compare model performance:
```bash
python model_comparison_result.py
```

## Troubleshooting

### Camera Not Detected
1. Check USB connection
2. Run `reset_camera.py`
3. Verify RealSense SDK installation: `rs-enumerate-devices`

### Low Detection Accuracy
1. Ensure proper lighting conditions
2. Check IR preprocessing settings
3. Verify YOLO model weights are loaded (`best.pt`)

### TF Transform Errors
```bash
./check_tf_transforms.sh
```

### Performance Issues
- Enable GPU acceleration
- Reduce image resolution
- Use optimized tracking mode: `start_semantic_slam_IR_YOLO_tracking_optimized.sh`

## Development

### Build System
```bash
./rebuild.sh  # Clean build
catkin_make   # Incremental build
```

### Debug Mode
```bash
./debug_mms_slam.sh
```

## Citation

If you use this work in your research, please cite:
```bibtex
@software{swiftlet_nest_slam,
  author = {iamtriet},
  title = {Swiftlet Nest SLAM System},
  year = {2025},
  url = {https://github.com/iamtriet/swiftlet-nest-slam}
}
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS community
- Intel RealSense team
- Ultralytics YOLO team
- Open-source SLAM research community

## Contact

For questions and support, please open an issue on the GitHub repository.

---

**Note**: This is an active research project. Features and APIs may change.
