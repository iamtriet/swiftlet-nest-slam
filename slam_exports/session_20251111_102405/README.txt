================================================================================
SEMANTIC SLAM SESSION EXPORT
================================================================================

Session Date: 2025-11-11 10:24:05
Camera Mode: Infrared (L515 IR)
Total Frames: 1451
Average FPS: 10.88
Unique Objects Detected: 5

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
  G#1 [bird-nest] at (0.79, 0.28, 0.02)m
       Detected 32 times
  G#2 [bird-nest] at (0.42, 0.37, -0.06)m
       Detected 43 times
  G#3 [bird-nest] at (0.54, 0.72, 0.13)m
       Detected 5 times
  G#4 [bird-nest] at (0.57, 0.20, 0.27)m
       Detected 2 times
  G#5 [bird-nest] at (0.87, 1.04, -0.29)m
       Detected 1 times
