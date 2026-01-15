================================================================================
SEMANTIC SLAM SESSION EXPORT
================================================================================

Session Date: 2025-11-11 18:24:46
Camera Mode: Infrared (L515 IR)
Total Frames: 169
Average FPS: 16.52
Unique Objects Detected: 0

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
