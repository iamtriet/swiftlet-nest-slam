#!/bin/bash
# Verify swiftlet training toolkit installation and dependencies

echo "🔍 Verifying Swiftlet Training Toolkit Installation"
echo "══════════════════════════════════════════════════════════"
echo

# Check toolkit files
echo "📦 Checking Toolkit Files..."
files=(
    "TRAIN_YOLO_GUIDE.md"
    "SWIFTLET_TRAINING_WORKFLOW.md"
    "SWIFTLET_TRAINING_SUMMARY.md"
    "optimize_camera_dark.py"
    "capture_training_images.py"
    "convert_labelme_to_yolo.py"
    "train_swiftlet_model.py"
    "test_swiftlet_model.py"
)

all_files_present=true
for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✅ $file"
    else
        echo "  ❌ $file (MISSING)"
        all_files_present=false
    fi
done
echo

# Check Python dependencies
echo "🐍 Checking Python Dependencies..."
deps=(
    "torch"
    "ultralytics"
    "cv2"
    "PIL"
    "numpy"
    "yaml"
    "rospy"
)

all_deps_present=true
for dep in "${deps[@]}"; do
    if python3 -c "import $dep" 2>/dev/null; then
        echo "  ✅ $dep"
    else
        echo "  ❌ $dep (NOT INSTALLED)"
        all_deps_present=false
    fi
done
echo

# Check CUDA
echo "🎮 Checking GPU/CUDA..."
if python3 -c "import torch; exit(0 if torch.cuda.is_available() else 1)" 2>/dev/null; then
    gpu_name=$(python3 -c "import torch; print(torch.cuda.get_device_name(0))" 2>/dev/null)
    cuda_version=$(python3 -c "import torch; print(torch.version.cuda)" 2>/dev/null)
    echo "  ✅ GPU: $gpu_name"
    echo "  ✅ CUDA: $cuda_version"
    gpu_ok=true
else
    echo "  ⚠️  No GPU detected (training will be slow)"
    gpu_ok=false
fi
echo

# Check disk space
echo "💾 Checking Disk Space..."
available=$(df -h ~ | awk 'NR==2 {print $4}')
echo "  Available: $available"
if [ $(df ~ | awk 'NR==2 {print $4}') -gt 5242880 ]; then
    echo "  ✅ Sufficient space (>5GB)"
    disk_ok=true
else
    echo "  ⚠️  Low disk space (<5GB)"
    disk_ok=false
fi
echo

# Summary
echo "══════════════════════════════════════════════════════════"
echo "📊 SUMMARY"
echo "══════════════════════════════════════════════════════════"

if $all_files_present && $all_deps_present; then
    echo "✅ Installation: COMPLETE"
else
    echo "❌ Installation: INCOMPLETE"
fi

if $gpu_ok; then
    echo "✅ GPU: READY"
else
    echo "⚠️  GPU: NOT AVAILABLE (will be slow)"
fi

if $disk_ok; then
    echo "✅ Disk Space: SUFFICIENT"
else
    echo "⚠️  Disk Space: LOW"
fi

echo
echo "══════════════════════════════════════════════════════════"

if $all_files_present && $all_deps_present; then
    echo "🎉 READY TO START!"
    echo
    echo "📖 Read documentation:"
    echo "   cat SWIFTLET_TRAINING_SUMMARY.md"
    echo
    echo "🚀 Begin training pipeline:"
    echo "   ./optimize_camera_dark.py"
    echo
else
    echo "⚠️  INSTALLATION INCOMPLETE"
    echo
    
    if ! $all_deps_present; then
        echo "Install missing dependencies:"
        echo "   pip install ultralytics labelme opencv-python pillow numpy pyyaml"
        echo
    fi
    
    if ! $gpu_ok; then
        echo "⚠️  GPU not detected. Training will be very slow."
        echo "   Consider using Google Colab with GPU for training."
        echo
    fi
fi

echo "══════════════════════════════════════════════════════════"
