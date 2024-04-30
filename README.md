# ouster-ros-extras

This repo contains samples and some utility script to be used with the ouster-ros driver

## Examples:
* [Apply PCL Filters](./scripts/ros2_pcl_filters.py): This example shows how to filter the point
  cloud to remove noise or scope the size of the point cloud using PCL library in python.
* [Range Image to Point Cloud](./scripts/ros2_range_2_points_xyz.py): This example shows how to
  consume the `/ouster/range_image` topic and produce a point cloud similar to the `/ouster/points`
  by utilizing python and ouster-sdk.
