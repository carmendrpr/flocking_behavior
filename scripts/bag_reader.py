"""
bag_reader.py
"""

from typing import Any
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


def read_rosbag(filename: str) -> dict[str, list[Any]]:
    """Read a rosbag"""
    bag_reader = SequentialReader()
    storage_options = StorageOptions(uri=filename, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="", output_serialization_format="")
    bag_reader.open(storage_options, converter_options)

    topics_dict = {}
    while bag_reader.has_next():
        topic, msg, _ = bag_reader.read_next()
        if topic not in topics_dict:
            topics_dict[topic] = []
        topics_dict[topic].append(msg)
    return topics_dict


def deserialize_msgs(msgs: list[Any], msg_type: Any) -> list[Any]:
    """Deserialize messages"""
    deserialized_msgs = []
    for msg in msgs:
        deserialized_msgs.append(deserialize_message(msg, msg_type))
    return deserialized_msgs


def deserialize_rosbag(rosbag: dict[str, list[Any]],
                       msg_types: dict[str, Any]) -> dict[str, list[Any]]:
    """Deserialize messages in rosbags"""
    deserialized_msgs = {}
    for topic, msgs in rosbag.items():
        try:
            deserialized_msgs[topic] = deserialize_msgs(msgs, msg_types[topic])
        except KeyError:
            deserialized_msgs[topic] = []
    return deserialized_msgs


if __name__ == "__main__":
    from geometry_msgs.msg import PointStamped

    info = read_rosbag("rosbags/exploration_20231129_115432")
    info = deserialize_rosbag(info, {"/drone0/pose": PointStamped,
                                     "/drone0/path_length": PointStamped})
    print(info)
