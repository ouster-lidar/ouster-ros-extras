#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2024, Ouster, Inc.
# All rights reserved.

"""
A module that shows how to subscribe to ouster point cloud and utililze pcl filters.
Author: Ussama Naal
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
from sensor_msgs.msg import PointCloud2, PointField
import point_cloud2 as pc2
import pcl


class PointCloudSubscriber(Node):

    def __init__(self):
        super().__init__('point_cloud_subscriber')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            liveliness=LivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            depth=5
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.point_cloud_callback,
            qos_profile=qos_profile)

        self.publisher = self.create_publisher(
            PointCloud2,
            '/ouster/points_filtered',
            qos_profile=qos_profile)

    def convert_pointcloud2_to_pcl(self, msg):
        data = pc2.read_points(msg, field_names="xyz")
        points = np.array(list(data), dtype=np.float32)
        cloud = pcl.PointCloud()
        cloud.from_array(points)
        return cloud

    def convert_pcl_to_pointcloud2(self, pcl_cloud, header):
        points = pcl_cloud.to_array()
        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1)
        ]
        return pc2.create_cloud(header, fields, points)

    def remove_outliers(self, point_cloud, mean_k=10, std_dev_mul_thresh=1.0):
        fil = point_cloud.make_statistical_outlier_filter()
        fil.set_negative(True)
        fil.set_mean_k(mean_k)
        fil.set_std_dev_mul_thresh(std_dev_mul_thresh)
        return fil.filter()

    def pass_through_filter(self, point_cloud, axis="x", dist=1.0):
        fil = point_cloud.make_passthrough_filter()
        fil.set_filter_field_name(axis)
        fil.set_filter_limits(-dist, dist)
        return fil.filter()

    def point_cloud_callback(self, msg):
        points = self.convert_pointcloud2_to_pcl(msg)
        # pcl_cloud = self.pass_through_filter(points)
        pcl_cloud = self.remove_outliers(points)
        mm = self.convert_pcl_to_pointcloud2(pcl_cloud, msg.header)
        self.publisher.publish(mm)

def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
