#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
import yaml

from duckietown.dtros import DTROS, NodeType

class DetectLaneNode(DTROS):
    def __init__(self, node_name):
        super(DetectLaneNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size=1)
        self.pub_lane = rospy.Publisher(f"/{self._vehicle_name}/detect/lane", Float64, queue_size=1)

    def cbFindLane(self, image_msg):
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Seulement détection jaune
        mask = cv2.inRange(hsv,
            (self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l),
            (self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h)
        )

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        yellow_coords = np.column_stack(np.where(mask > 0))
        if yellow_coords.size == 0:
            rospy.logwarn("❌ Ligne jaune non détectée")
            return

        center = np.mean(yellow_coords[:, 1])  # axe x (colonne)

        rospy.loginfo(f"✅ Centre ligne jaune : {center:.2f}")

        msg = Float64()
        msg.data = center
        self.pub_lane.publish(msg)

    def load_conf(self, path):
        with open(path, 'r') as f:
            self.conf = yaml.safe_load(f)

        self.hue_yellow_l = self.conf['yellow']['hl']
        self.hue_yellow_h = self.conf['yellow']['hh']
        self.saturation_yellow_l = self.conf['yellow']['sl']
        self.saturation_yellow_h = self.conf['yellow']['sh']
        self.lightness_yellow_l = self.conf['yellow']['vl']
        self.lightness_yellow_h = self.conf['yellow']['vh']

if __name__ == '__main__':
    node = DetectLaneNode(node_name='detect_lane_node')
    rospy.spin()
