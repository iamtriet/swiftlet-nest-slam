#!/bin/bash

echo "=========================================="
echo "🚀 Performance Test & Optimization Check"
echo "=========================================="
echo ""

echo "1️⃣  Checking GPU Status:"
echo "─────────────────────────────────────────"
python3 << EOF
import torch
print(f"CUDA Available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"CUDA Version: {torch.version.cuda}")
else:
    print("⚠️  GPU NOT AVAILABLE - Will run on CPU (VERY SLOW)")
EOF

echo ""
echo "2️⃣  Testing YOLO Performance:"
echo "─────────────────────────────────────────"
python3 << 'EOF'
import time
import numpy as np
from ultralytics import YOLO

# Load model
model = YOLO('yolov8n-seg.pt')

# Test image
img = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)

# Warm up
print("Warming up...")
_ = model(img, device='cuda:0', verbose=False)

# Test FP32
print("\nTesting FP32 (normal precision)...")
times = []
for i in range(10):
    start = time.time()
    _ = model(img, device='cuda:0', verbose=False, half=False)
    times.append(time.time() - start)
avg_time_fp32 = np.mean(times) * 1000
fps_fp32 = 1000 / avg_time_fp32
print(f"  Average: {avg_time_fp32:.1f}ms ({fps_fp32:.1f} FPS)")

# Test FP16 (half precision)
print("\nTesting FP16 (half precision - 2x faster)...")
times = []
for i in range(10):
    start = time.time()
    _ = model(img, device='cuda:0', verbose=False, half=True)
    times.append(time.time() - start)
avg_time_fp16 = np.mean(times) * 1000
fps_fp16 = 1000 / avg_time_fp16
print(f"  Average: {avg_time_fp16:.1f}ms ({fps_fp16:.1f} FPS)")
print(f"  Speedup: {avg_time_fp32/avg_time_fp16:.2f}x faster with FP16!")

print("\n✅ Expected performance in full system:")
print(f"   • With GPU (FP16): {fps_fp16*0.6:.0f}-{fps_fp16*0.8:.0f} FPS")
print(f"   • With CPU: 3-5 FPS (SLOW!)")
EOF

echo ""
echo "3️⃣  Checking System Resources:"
echo "─────────────────────────────────────────"
echo "CPU Usage:"
top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print "  " 100 - $1"% used"}'

echo ""
echo "Memory:"
free -h | grep "Mem:" | awk '{print "  "$3" / "$2" used"}'

echo ""
echo "GPU Memory (if available):"
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader 2>/dev/null | awk '{print "  "$0}' || echo "  N/A"

echo ""
echo "=========================================="
echo "💡 Optimization Tips:"
echo "=========================================="
echo ""
echo "For BEST performance:"
echo "  ✅ GPU (CUDA) should be available"
echo "  ✅ FP16 enabled (half=True) = 2x speedup"
echo "  ✅ Smaller image size (640x640)"
echo "  ✅ Fewer points per object (150 max)"
echo ""
echo "Expected FPS:"
echo "  🚀 GPU + FP16: 15-25 FPS"
echo "  🐢 GPU only:   8-15 FPS"
echo "  🐌 CPU:        3-5 FPS"
echo ""
