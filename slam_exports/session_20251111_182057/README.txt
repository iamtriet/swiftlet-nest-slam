================================================================================
SEMANTIC SLAM SESSION EXPORT
================================================================================

Session Date: 2025-11-11 18:20:57
Camera Mode: Infrared (L515 IR)
Total Frames: 155
Average FPS: 11.57
Unique Objects Detected: 8

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
  G#1 [bird-nest] at (0.21, -0.53, 0.16)m
       Detected 174 times
  G#2 [bird-nest] at (0.52, -1.37, -0.58)m
       Detected 28 times
  G#3 [bird-nest] at (-0.11, -0.74, 0.43)m
       Detected 4 times
  G#4 [bird-nest] at (0.63, -1.85, 0.75)m
       Detected 1 times
  G#5 [bird-nest] at (0.38, -1.15, 0.48)m
       Detected 2 times
  G#6 [bird-nest] at (-0.24, -0.72, -0.15)m
       Detected 1 times
  G#7 [bird-nest] at (0.35, -0.73, -0.02)m
       Detected 2 times
  G#8 [bird-nest] at (-0.01, -1.12, 0.67)m
       Detected 9 times
