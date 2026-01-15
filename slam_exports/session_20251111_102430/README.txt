================================================================================
SEMANTIC SLAM SESSION EXPORT
================================================================================

Session Date: 2025-11-11 10:24:30
Camera Mode: Infrared (L515 IR)
Total Frames: 125
Average FPS: 12.02
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
  G#1 [bird-nest] at (-0.06, -0.76, 0.30)m
       Detected 80 times
  G#2 [bird-nest] at (0.18, -0.58, 0.23)m
       Detected 31 times
  G#3 [bird-nest] at (-0.17, -0.63, -0.03)m
       Detected 5 times
