#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from PIL import Image as PILImage
from math import isnan

from custom_msgs.msg import MarkerTag


class YOLOColorInference(Node):

    def __init__(self):
        super().__init__("inference")

        self.bridge = CvBridge()
        self.model = YOLO("/home/mrmnavjet/IRC2026/ircWS/src/yolo/yolo/cone_v1.pt")

        self.latest_image = None
        self.latest_depth = None

        self.COLOR_ID = {
            "orange": 1,
            "red": 2,
            "blue": 3,
            "green": 4,
            "yellow": 5
        }

        # ---------------- UPDATED TOPICS ---------------- #
        self.image_sub = self.create_subscription(
            Image,
            "/zed/zed_node/rgb/image_rect_color",  # <-- NEW
            self.image_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",  # <-- already correct
            self.depth_callback,
            10
        )
        # ------------------------------------------------ #

        self.marker_pub = self.create_publisher(MarkerTag, "/marker_detect", 10)
        self.timer = self.create_timer(0.01, self.process_frame)

        self.get_logger().info("YOLO + ZED depth inference node started.")

    def image_callback(self, msg):
        self.latest_image = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def detect_color(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        color_ranges = {
            "orange": [(5, 120, 120), (20, 255, 255)],
            "red": [(0, 140, 120), (10, 255, 255)],
            "blue": [(90, 120, 120), (130, 255, 255)],
            "green": [(40, 120, 120), (80, 255, 255)],
            "yellow": [(22, 120, 120), (34, 255, 255)]
        }

        best_color = None
        max_pixels = 0

        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, tuple(lower), tuple(upper))
            count = cv2.countNonZero(mask)
            if count > max_pixels:
                best_color = color
                max_pixels = count

        return best_color

    def process_frame(self):
        if self.latest_image is None or self.latest_depth is None:
            return

        frame = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        depth_map = self.bridge.imgmsg_to_cv2(self.latest_depth, "32FC1")

        results = self.model.predict(PILImage.fromarray(frame))[0]

        h, w, _ = frame.shape
        img_center = w / 2

        for det in results.boxes:
            x1, y1, x2, y2 = map(int, det.xyxy[0].cpu())

            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                continue

            color_name = self.detect_color(roi)
            if color_name is None:
                continue

            cx = int(x1 + (x2-x1)/2)
            cy = int(y1 + (y2-y1)/2)

            depth_value = float(depth_map[cy, cx])

            if isnan(depth_value) or depth_value <= 0:
                continue

            # ---------------- FIXED: depth already in meters ---------------- #
            y_offset = (cx - img_center) * depth_value * 0.001
            # ---------------------------------------------------------------- #

            msg = MarkerTag()
            msg.is_found = True
            msg.id = self.COLOR_ID[color_name]
            msg.x = depth_value          # forward distance (meters)
            msg.y = y_offset             # lateral offset

            self.marker_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down inference node.")
        super().destroy_node()


def main(args=None):
        rclpy.init(args=args)
        node = YOLOColorInference()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
