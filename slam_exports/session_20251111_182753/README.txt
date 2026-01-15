================================================================================
SEMANTIC SLAM SESSION EXPORT
================================================================================

Session Date: 2025-11-11 18:27:53
Camera Mode: Infrared (L515 IR)
Total Frames: 690
Average FPS: 8.10
Unique Objects Detected: 16

FILES IN THIS EXPORT:
--------------------------------------------------------------------------------
1. slam_map.pcd          - 3D SLAM map point cloud
2. detected_objects.json - All unique objects with positions
3. object_positions.pcd  - Object centroids as point cloud
4. tracking_statistics.json - Session statistics
5. README.txt            - This file

HOW TO USE:
--------------------------------------------------------------------------------
View PCD files:
  pcl_viewer slam_map.pcd
  pcl_viewer object_positions.pcd

View JSON files:
  cat detected_objects.json | jq .
  cat tracking_statistics.json | jq .

Object Summary:
--------------------------------------------------------------------------------
  G#1 [bird-nest] at (0.17, -1.61, 0.06)m
       Detected 471 times
  G#2 [bird-nest] at (0.25, -1.64, -0.33)m
       Detected 627 times
  G#3 [bird-nest] at (0.04, -1.61, 0.36)m
       Detected 91 times
  G#4 [bird-nest] at (0.48, -1.21, 0.42)m
       Detected 147 times
  G#5 [bird-nest] at (0.61, -1.00, 0.62)m
       Detected 1 times
  G#6 [bird-nest] at (-0.18, -1.43, 0.19)m
       Detected 2 times
  G#7 [bird-nest] at (0.51, -1.46, -0.18)m
       Detected 3 times
  G#8 [bird-nest] at (0.55, -1.09, 0.15)m
       Detected 1 times
  G#9 [bird-nest] at (0.42, -1.50, 0.24)m
       Detected 1 times
  G#10 [bird-nest] at (0.35, -1.48, 0.57)m
       Detected 4 times
  G#11 [bird-nest] at (0.72, -0.06, 0.27)m
       Detected 3 times
  G#12 [bird-nest] at (0.76, -0.15, 0.68)m
       Detected 10 times
  G#13 [bird-nest] at (0.62, -0.11, 1.05)m
       Detected 8 times
  G#14 [bird-nest] at (0.53, 0.11, 0.73)m
       Detected 3 times
  G#15 [bird-nest] at (0.95, -0.32, 0.50)m
       Detected 1 times
  G#16 [bird-nest] at (1.06, -0.62, 0.92)m
       Detected 3 times
