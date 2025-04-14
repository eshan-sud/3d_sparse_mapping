#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
from pathlib import Path

class MiDaSNode(Node):
    def __init__(self):
        super().__init__('midas_depth_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your actual camera topic
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/camera/depth_estimated', 10)

        # Load MiDaS model
        model_path = str(Path(__file__).resolve().parent.parent / 'model' / 'dpt_large_384.pt')
        self.model_type = "DPT_Large"
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.model = torch.hub.load("intel-isl/MiDaS", self.model_type)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device).eval()

        self.transform = torch.hub.load("intel-isl/MiDaS", "transforms").dpt_transform

        self.get_logger().info("MiDaS depth node initialized")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        input_tensor = self.transform(cv_image).to(self.device)

        with torch.no_grad():
            prediction = self.model(input_tensor)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=cv_image.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth_map = prediction.cpu().numpy()
        depth_norm = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
        depth_image = depth_norm.astype(np.uint8)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="mono8")
        depth_msg.header = msg.header
        self.publisher.publish(depth_msg)
