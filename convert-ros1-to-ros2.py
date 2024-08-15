from pathlib import Path
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_ros1

from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr

from rosbags.typesys import get_types_from_idl, get_types_from_msg, register_types

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

# plain dictionary to hold message definitions
add_types = {}

# add all definitions from one idl file
add_types.update(get_types_from_idl(idl_text))

# add definition from one msg file
add_types.update(get_types_from_msg(
    msg_text, 'ouster_sensor_msgs/msg/PacketMsg'))

# ROS1
ros1_msg_text = msg_text
add_types.update(get_types_from_msg(ros1_msg_text, "ouster_ros/msg/PacketMsg"))

# make types available to rosbags serializers/deserializers
register_types(add_types)


def convert(file_in, file_out):

    in_topics = ["/os_node/imu_packets", "/os_node/lidar_packets"]
    out_topics = {"/ouster/imu_packets": "ouster_sensor_msgs/msg/PacketMsg",
                  "/ouster/lidar_packets": "ouster_sensor_msgs/msg/PacketMsg"}
    
    out_2_in_dict = {"/ouster/imu_packets": "/os_node/imu_packets",
                  "/ouster/lidar_packets": "/os_node/lidar_packets"}
    in_2_out_dict = {value: key for key, value in out_2_in_dict.items()}

    # create reader instance and open for reading
    with Reader(file_in) as reader:
        connections = [x for x in reader.connections if x.topic in in_topics]

        with Writer(file_out) as writer:

            conn_dict = {}
            
            for topic_name, topic_type in out_topics.items():
                in_topic = out_2_in_dict[topic_name]
                conn_dict[in_topic] = writer.add_connection(topic_name, topic_type)


            for connection, timestamp, rawdata in reader.messages(connections=connections):
                conn = conn_dict.get(connection.topic)
                if conn is None:
                    print(F"unidentified connection topic {connection.topic}")
                    continue
                msg = deserialize_ros1(rawdata, connection.msgtype)
                out_topic = in_2_out_dict[connection.topic]
                smsg = serialize_cdr(msg, out_topics[out_topic])
                writer.write(conn, timestamp, smsg)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Convert ros1 bag format to ros2 bag format")
    parser.add_argument("input", help="ROS1 bag file")
    parser.add_argument("output", help="ROS2 bag file output name")
    args = parser.parse_args()
    convert(file_in=args.input, file_out=args.output)
