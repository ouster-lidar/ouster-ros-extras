from rosbags.rosbag2 import Reader
from rosbags.rosbag1 import Writer
from rosbags.typesys import get_types_from_msg
from rosbags.typesys import Stores, get_typestore

import argparse

idl_text = """
module ouster_sensor_msgs {
  module msg {
    struct PacketMsg {
      sequence<uint8> buf;
    };
  };
};
"""

msg_text = """
uint8[] buf
"""

ros2_typestore = get_typestore(Stores.ROS2_FOXY)
ros2_typestore.register(get_types_from_msg(
    msg_text, 'ouster_sensor_msgs/msg/PacketMsg'))

ros1_typestore = get_typestore(Stores.ROS1_NOETIC)
# ROS1
ros1_typestore.register(get_types_from_msg(
    msg_text, "ouster_ros/msg/PacketMsg"))


def convert(input_file, output_file):
    with Reader(input_file) as reader:
        connections = [x for x in reader.connections if x.topic in {
            '/ouster/imu_packets', '/ouster/lidar_packets', '/ouster/metadata'}]

        with Writer(output_file) as writer:
            conn0 = writer.add_connection(
                "/ouster/metadata", "std_msgs/msg/String", typestore=ros1_typestore)
            conn1 = writer.add_connection(
                "/ouster/imu_packets", "ouster_ros/msg/PacketMsg", typestore=ros1_typestore)
            conn2 = writer.add_connection(
                "/ouster/lidar_packets", "ouster_ros/msg/PacketMsg", typestore=ros1_typestore)

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                conn = None
                if connection.topic == "/ouster/metadata":
                    msg = ros2_typestore.deserialize_cdr(
                        rawdata, connection.msgtype)
                    smsg = ros1_typestore.serialize_ros1(
                        msg, "std_msgs/msg/String")
                    writer.write(conn0, timestamp, smsg)
                elif connection.topic == "/ouster/imu_packets":
                    msg = ros2_typestore.deserialize_cdr(
                        rawdata, connection.msgtype)
                    smsg = ros1_typestore.serialize_ros1(
                        msg, "ouster_ros/msg/PacketMsg")
                    writer.write(conn1, timestamp, smsg)
                elif connection.topic == "/ouster/lidar_packets":
                    msg = ros2_typestore.deserialize_cdr(
                        rawdata, connection.msgtype)
                    smsg = ros1_typestore.serialize_ros1(
                        msg, "ouster_ros/msg/PacketMsg")
                    writer.write(conn2, timestamp, smsg)
                else:
                    print("Unknown Topic:", connection.topic, "skipping..")
                    continue


def main():
    parser = argparse.ArgumentParser(description="dumps point loud")
    parser.add_argument("input_path", help="path to scan source")
    parser.add_argument("-o", "--output", default="",
                        help="output folder name for dumped pcd")
    args = parser.parse_args()
    output = args.output if args.output else F"ros1-{args.input_path}"
    convert(args.input_path, output)


if __name__ == "__main__":
    main()
