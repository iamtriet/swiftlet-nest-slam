================================================================================
SEMANTIC SLAM SESSION EXPORT
================================================================================

Session Date: 2025-11-11 18:20:12
Camera Mode: Infrared (L515 IR)
Total Frames: 675
Average FPS: 25.36
Unique Objects Detected: 3

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
  G#1 [bird-nest] at (0.04, -1.30, -0.67)m
       Detected 3 times
  G#2 [bird-nest] at (0.07, -0.53, -0.20)m
       Detected 91 times
  G#3 [bird-nest] at (0.22, -0.60, 0.07)m
       Detected 2 times
