================================================================================
SEMANTIC SLAM SESSION EXPORT
================================================================================

Session Date: 2025-11-12 11:35:14
Camera Mode: Infrared (L515 IR)
Total Frames: 520
Average FPS: 11.52
Unique Objects Detected: 10

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
  G#1 [bird-nest] at (0.53, -1.44, -0.02)m
       Detected 577 times
  G#2 [bird-nest] at (-0.26, -0.82, 0.06)m
       Detected 55 times
  G#3 [bird-nest] at (0.40, -1.36, 0.79)m
       Detected 25 times
  G#4 [bird-nest] at (-0.14, -1.39, 0.08)m
       Detected 33 times
  G#5 [bird-nest] at (-0.16, -0.50, -0.07)m
       Detected 5 times
  G#6 [bird-nest] at (0.04, -1.33, 0.60)m
       Detected 15 times
  G#7 [bird-nest] at (-0.49, -1.24, 0.44)m
       Detected 5 times
  G#8 [bird-nest] at (-0.49, -1.32, 0.13)m
       Detected 2 times
  G#9 [bird-nest] at (-0.10, -0.68, 0.28)m
       Detected 1 times
  G#10 [bird-nest] at (-0.15, -0.95, 0.48)m
       Detected 1 times
