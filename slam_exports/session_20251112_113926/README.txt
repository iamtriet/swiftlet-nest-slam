================================================================================
SEMANTIC SLAM SESSION EXPORT
================================================================================

Session Date: 2025-11-12 11:39:26
Camera Mode: Infrared (L515 IR)
Total Frames: 2837
Average FPS: 12.06
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
  G#1 [bird-nest] at (0.13, -1.52, 0.15)m
       Detected 2864 times
  G#2 [bird-nest] at (0.19, -1.37, 0.76)m
       Detected 97 times
  G#3 [bird-nest] at (0.55, -1.47, 0.90)m
       Detected 1 times
