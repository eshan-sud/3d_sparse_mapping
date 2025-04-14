import os
import cv2
import rclpy
import numpy as np
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge

DATASET_DIR = "/home/minor-project/ros2_test/Datasets/EuRoC/MH01/mav0"
bridge = CvBridge()

def load_csv_data(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split(',')
            data.append(parts)
    return data

def to_ros_time(t):
    sec = int(t)
    nsec = int((t - sec) * 1e9)
    return Time(sec=sec, nanosec=nsec)

def create_image_msg(timestamp_ns, img_path):
    image = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    stamp = timestamp_ns / 1e9
    msg = bridge.cv2_to_imgmsg(image, encoding="mono8")
    msg.header.stamp = to_ros_time(stamp)
    msg.header.frame_id = "cam"
    return msg

def create_imu_msg(data):
    timestamp_ns = int(data[0])
    t = timestamp_ns / 1e9
    msg = Imu()
    msg.header.stamp = to_ros_time(t)
    msg.header.frame_id = "imu"
    msg.linear_acceleration.x = float(data[1])
    msg.linear_acceleration.y = float(data[2])
    msg.linear_acceleration.z = float(data[3])
    msg.angular_velocity.x = float(data[4])
    msg.angular_velocity.y = float(data[5])
    msg.angular_velocity.z = float(data[6])
    return msg

def create_pose_msg(data):
    timestamp_ns = int(data[0])
    t = timestamp_ns / 1e9
    msg = PoseStamped()
    msg.header.stamp = to_ros_time(t)
    msg.header.frame_id = "map"
    msg.pose.position.x = float(data[1])
    msg.pose.position.y = float(data[2])
    msg.pose.position.z = float(data[3])
    msg.pose.orientation.x = float(data[4])
    msg.pose.orientation.y = float(data[5])
    msg.pose.orientation.z = float(data[6])
    msg.pose.orientation.w = float(data[7])
    return msg

def main():
    cam0_data = load_csv_data(os.path.join(DATASET_DIR, "cam0/data.csv"))
    cam1_data = load_csv_data(os.path.join(DATASET_DIR, "cam1/data.csv"))
    imu_data = load_csv_data(os.path.join(DATASET_DIR, "imu0/data.csv"))
    gt_data = load_csv_data(os.path.join(DATASET_DIR, "state_groundtruth_estimate0/data.csv"))

    writer = SequentialWriter()
    storage_options = StorageOptions(uri=DATASET_DIR + "/euroc_ros2_bag", storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    writer.create_topic(TopicMetadata(name="/cam0/image_raw", type="sensor_msgs/msg/Image", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name="/cam1/image_raw", type="sensor_msgs/msg/Image", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name="/imu", type="sensor_msgs/msg/Imu", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name="/groundtruth", type="geometry_msgs/msg/PoseStamped", serialization_format="cdr"))

    for ts, fname in cam0_data:
        path = os.path.join(DATASET_DIR, "cam0/data", fname)
        msg = create_image_msg(int(ts), path)
        writer.write("/cam0/image_raw", serialize_message(msg), msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)

    for ts, fname in cam1_data:
        path = os.path.join(DATASET_DIR, "cam1/data", fname)
        msg = create_image_msg(int(ts), path)
        writer.write("/cam1/image_raw", serialize_message(msg), msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)

    for row in imu_data:
        msg = create_imu_msg(row)
        writer.write("/imu", serialize_message(msg), msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)

    for row in gt_data:
        msg = create_pose_msg(row)
        writer.write("/groundtruth", serialize_message(msg), msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)

    print("ROS 2 bag created at:", storage_options.uri)

if __name__ == "__main__":
    main()
