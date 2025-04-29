# Convert a TUM RGB-D dataset sequence to a synced ROS 2 bag

import os
import cv2
import rclpy
import argparse
import numpy as np
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

bridge = CvBridge()


def to_ros_time(t):
    return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

def stamp_to_ns(t):
    return t.sec * 10**9 + t.nanosec

def parse_assoc_file(file_path):
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
            pose.pose.position.x = vals[1]
            pose.pose.position.y = vals[2]
            pose.pose.position.z = vals[3]
            pose.pose.orientation.x = vals[4]
            pose.pose.orientation.y = vals[5]
            pose.pose.orientation.z = vals[6]
            pose.pose.orientation.w = vals[7]
            poses.append((stamp, pose))
    return poses

def find_nearest(data, t, max_diff=0.05):
    closest = None
    smallest = float('inf')
    for ts, val in data:
        diff = abs(t - ts)
        if diff < smallest and diff < max_diff:
            smallest = diff
            closest = (ts, val)
    return closest

def write_rosbag(dataset_dir, output_bag_path):
    rgb_data = parse_assoc_file(os.path.join(dataset_dir, "rgb.txt"))
    depth_data = parse_assoc_file(os.path.join(dataset_dir, "depth.txt"))
    poses = parse_groundtruth(os.path.join(dataset_dir, "groundtruth.txt"))

    writer = SequentialWriter()
    storage_options = StorageOptions(uri=output_bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    writer.create_topic(TopicMetadata(name="/camera/image_raw", type="sensor_msgs/msg/Image", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name="/camera/image_depth", type="sensor_msgs/msg/Image", serialization_format="cdr"))
    writer.create_topic(TopicMetadata(name="/orb_slam3/camera_pose", type="geometry_msgs/msg/PoseStamped", serialization_format="cdr"))

    total_synced = 0

    for t_rgb, rgb_file in rgb_data:
        depth_match = find_nearest(depth_data, t_rgb, max_diff=0.05)
        pose_match = find_nearest(poses, t_rgb, max_diff=0.1)

        if not depth_match or not pose_match:
            continue

        t_depth, depth_file = depth_match
        t_pose, pose_msg = pose_match

        # Load images
        rgb_path = os.path.join(dataset_dir, rgb_file)
        depth_path = os.path.join(dataset_dir, depth_file)

        rgb_img = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

        if rgb_img is None or depth_img is None:
            continue

        stamp = to_ros_time(t_rgb)

        # RGB message
        rgb_msg = bridge.cv2_to_imgmsg(rgb_img, encoding="bgr8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = "map"
        writer.write("/camera/image_raw", serialize_message(rgb_msg), stamp_to_ns(stamp))

        # Depth message
        depth_msg = bridge.cv2_to_imgmsg(depth_img, encoding="passthrough")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "map"
        writer.write("/camera/image_depth", serialize_message(depth_msg), stamp_to_ns(stamp))

        # Pose message
        pose_msg.header.stamp = stamp
        writer.write("/orb_slam3/camera_pose", serialize_message(pose_msg), stamp_to_ns(stamp))

        total_synced += 1

    print(f"Wrote {total_synced} synced RGB-D-Pose messages to {output_bag_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert TUM RGB-D dataset to ROS 2 bag with synced RGB, depth, and pose")
    parser.add_argument("dataset_dir", type=str, help="Path to TUM dataset sequence folder")
    parser.add_argument("output_bag", type=str, help="Output bag file path")
    args = parser.parse_args()

    rclpy.init()
    write_rosbag(args.dataset_dir, args.output_bag)
    rclpy.shutdown()
