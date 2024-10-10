#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2024, Ouster, Inc.
# All rights reserved.

"""
A module that shows how to subscribe to ouster point cloud and save as OSF file.
Author: Ussama Naal
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
from sensor_msgs.msg import PointCloud2
import point_cloud2 as pc2
import pcl
from std_msgs.msg import String
from ouster.sdk.client import SensorInfo, ChanField, FieldType, FieldClass, LidarScan, destagger

class PointCloudSubscriber(Node):

    def __init__(self, output_path):
        super().__init__('point_cloud_subscriber')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            depth=5
        )
        self.metadata_sub = self.create_subscription(
            String,
            '/ouster/metadata',
            self.metadata_callback,
            qos_profile=qos_profile)
        
        self._scan_ctr = 0
        self._output_path = output_path

    def metadata_callback(self, metadata):
        import ouster.sdk.osf as osf
        self._sensor_info = SensorInfo(s=metadata.data)
        metadata = SensorInfo(metadata.data)
        self._writer = osf.Writer(self._output_path, self._sensor_info)

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

    def point_cloud_callback(self, msg):
        if self._scan_ctr > 1740:
            return

        ts = msg.header.stamp
        scan_ts = np.uint64(ts.sec * 1e9 + ts.nanosec)
        W, H = msg.width, msg.height
        scan = LidarScan(H, W)

        data = pc2.read_points(msg, field_names=["range"])
        points = np.array(list(data), dtype=np.uint32)
        points = points.reshape((H, W))
        points = destagger(self._sensor_info, points, inverse=True)
        scan.field(ChanField.RANGE)[:] = points

        data = pc2.read_points(msg, field_names=["reflectivity"])
        points = np.array(list(data), dtype=np.uint16)
        points = points.reshape((H, W))
        points = destagger(self._sensor_info, points, inverse=True)
        scan.field(ChanField.REFLECTIVITY)[:] = points

        data = pc2.read_points(msg, field_names=["ambient"])
        points = np.array(list(data), dtype=np.uint16)
        points = points.reshape((H, W))
        points = destagger(self._sensor_info, points, inverse=True)
        scan.field(ChanField.NEAR_IR)[:] = points

        data = pc2.read_points(msg, field_names=["intensity"])
        points = np.array(list(data), dtype=np.float32)
        points = points.reshape((H, W))
        points = points.astype(np.uint32)
        points = destagger(self._sensor_info, points, inverse=True)
        scan.field(ChanField.SIGNAL)[:] = points

        for i in range(len(scan.packet_timestamp)):
            scan.packet_timestamp[i] = scan_ts + i * (1e9/100/16)

        self._writer.save(0, scan, scan_ts)

        self._scan_ctr += 1
        print("process scan", self._scan_ctr)
        if self._scan_ctr > 1740:
            self._writer.close()
            print("done")

def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber("output.osf")
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
