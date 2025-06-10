#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import Tuple


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.
    Return:
        steer_matrix_left:  Steering matrix for Braitenberg-like control using left lane markings.
    """
    return np.random.rand(*shape)


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.
    Return:
        steer_matrix_right:  Steering matrix for Braitenberg-like control using right lane markings.
    """
    return np.random.rand(*shape)


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space.
    Return:
        mask_left_edge:   Mask for dashed-yellow line.
        mask_right_edge:  Mask for solid-white line.
    """
    h, w, _ = image.shape
    mask_left_edge = np.random.rand(h, w)
    mask_right_edge = np.random.rand(h, w)
    return mask_left_edge, mask_right_edge


class LaneDetectorNode:
    def __init__(self):
        rospy.init_node("lane_detector", anonymous=True)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)

    def callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")
            return

        left_mask, right_mask = detect_lane_markings(cv_image)
        rospy.loginfo("Lane markings detected.")
        # Optional: Hier könntest du die Steer-Matrix berechnen:
        # steer_left = get_steer_matrix_left_lane_markings(left_mask.shape)

    def run(self):
        rospy.loginfo("LaneDetectorNode is running.")
        rospy.spin()


if __name__ == "__main__":
    node = LaneDetectorNode()
    node.run()
