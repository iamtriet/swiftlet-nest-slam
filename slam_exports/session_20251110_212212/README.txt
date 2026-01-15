================================================================================
SEMANTIC SLAM SESSION EXPORT
================================================================================

Session Date: 2025-11-10 21:22:12
Camera Mode: Infrared (L515 IR)
Total Frames: 282
Average FPS: 11.44
Unique Objects Detected: 11

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
  G#1 [bird-nest] at (0.68, -1.95, 0.43)m
       Detected 380 times
  G#2 [bird-nest] at (0.02, -0.56, 0.07)m
       Detected 193 times
  G#3 [bird-nest] at (0.56, -1.62, -0.24)m
       Detected 132 times
  G#4 [bird-nest] at (-0.49, -1.33, -0.77)m
       Detected 128 times
  G#5 [bird-nest] at (0.87, -1.83, 0.11)m
       Detected 64 times
  G#6 [bird-nest] at (0.17, -0.58, 0.36)m
       Detected 11 times
  G#7 [bird-nest] at (-0.39, -1.01, -0.23)m
       Detected 37 times
  G#8 [bird-nest] at (-0.34, -1.66, -0.13)m
       Detected 24 times
  G#9 [bird-nest] at (0.51, -1.49, -0.56)m
       Detected 1 times
  G#10 [bird-nest] at (-0.25, -1.49, -0.38)m
       Detected 2 times
  G#11 [bird-nest] at (-0.05, -1.53, 0.08)m
       Detected 2 times
