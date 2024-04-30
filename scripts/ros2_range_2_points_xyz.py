#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2024, Ouster, Inc.
# All rights reserved.

"""
Produce the xyz point cloud from the range_image topic.
Author: Ussama Naal
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy

from sensor_msgs.msg import PointCloud2, PointField
import point_cloud2 as pc2

from ouster import client


class ImageSubscriberNode(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        self.br = CvBridge()

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

    def metadata_callback(self, metadata):
        self.sensor_info = client.SensorInfo(s=metadata.data)
        self.xyzlut = client.XYZLut(self.sensor_info)
        self.create_image_subscriber()

    def create_image_subscriber(self):

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            liveliness=LivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            depth=5
        )

        self.range_image_sub = self.create_subscription(
            Image,
            '/ouster/range_image',
            self.image_callback,
            qos_profile=qos_profile)

        self.publisher = self.create_publisher(
            PointCloud2,
            '/ouster/points_xyz',
            qos_profile=qos_profile)

    def image_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg)
        cv_image *= 4   # this is needed because we truncate the values
        # xyzlut seems to destagger by default so we re-stagger
        cv_image2 = client.destagger(self.sensor_info, cv_image, inverse=True)
        xyz = self.xyzlut(cv_image2)

        fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1)
        ]
        header = Header(stamp=msg.header.stamp, frame_id="os_sensor")
        pc_msg = pc2.create_cloud(header, fields, points=xyz.reshape((-1, 3)))
        self.publisher.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
