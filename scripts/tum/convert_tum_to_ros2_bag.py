import os
import cv2
import rclpy
import numpy as np
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

DATASET_DIR = "/home/minor-project/ros2_test/Datasets/TUM/rgbd_dataset_freiburg1_desk"
RGB_FILE = os.path.join(DATASET_DIR, "rgb.txt")
DEPTH_FILE = os.path.join(DATASET_DIR, "depth.txt")
POSE_FILE = os.path.join(DATASET_DIR, "groundtruth.txt")

bridge = CvBridge()

def parse_assoc(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#') or len(line.strip()) == 0:
                continue
            timestamp, filename = line.strip().split()
            data.append((float(timestamp), filename))
    return data

def parse_groundtruth(file_path):
    poses = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#') or len(line.strip()) == 0:
                continue
            vals = list(map(float, line.strip().split()))
            stamp = vals[0]
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = Time(sec=int(stamp), nanosec=int((stamp - int(stamp)) * 1e9))
            pose.pose.position.x = vals[1]
            pose.pose.position.y = vals[2]
            pose.pose.position.z = vals[3]
            pose.pose.orientation.x = vals[4]
            pose.pose.orientation.y = vals[5]
            pose.pose.orientation.z = vals[6]
            pose.pose.orientation.w = vals[7]
            poses.append((stamp, pose))
    return poses

def to_ros_time(t):
    return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

def write_rosbag(output_path):
    writer = SequentialWriter()
    storage_options = StorageOptions(uri=output_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    writer.create_topic(TopicMetadata(name="/camera/image_raw", type="sensor_msgs/msg/Image", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name="/camera/image_depth", type="sensor_msgs/msg/Image", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name="/groundtruth", type="geometry_msgs/msg/PoseStamped", serialization_format="cdr"))

    rgb_data = parse_assoc(RGB_FILE)
    depth_data = parse_assoc(DEPTH_FILE)
    poses = parse_groundtruth(POSE_FILE)

    for timestamp, fname in rgb_data:
        img_path = os.path.join(DATASET_DIR, fname)
        image = cv2.imread(img_path)
        msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header.stamp = to_ros_time(timestamp)
        msg.header.frame_id = "camera_color"
        writer.write("/camera/image_raw", serialize_message(msg), msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)

    for timestamp, fname in depth_data:
        img_path = os.path.join(DATASET_DIR, fname)
        image = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        msg.header.stamp = to_ros_time(timestamp)
        msg.header.frame_id = "camera_depth"
        writer.write("/camera/image_depth", serialize_message(msg), msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)

    for stamp, pose in poses:
        writer.write("/groundtruth", serialize_message(pose), pose.header.stamp.sec * 10**9 + pose.header.stamp.nanosec)

    print("Rosbag written to", output_path)

write_rosbag("/home/minor-project/ros2_test/Datasets/TUM/tum_ros2_bag")
