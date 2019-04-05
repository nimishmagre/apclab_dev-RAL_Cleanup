#!/usr/bin/env python
import os
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge()


class ExpCalibrator:
    """
    Loading simulation scene set-up image and overly it on top of the live
        real-world video stream to copy the simulation setup to the real-world
    """
    def __init__(self, dataset_name, sample_idx):
        real_image_topic = '/camera/image_color'
        self.dataset_path = os.path.join(
            os.path.expanduser('~/apcNet_dataset/exp_dataset'),
            dataset_name)
        self.sim_image = self.read_sim_image(sample_idx)
        self.overlay_imgmsg = None
        self.real_image_sub = rospy.Subscriber(real_image_topic, Image,
                                               self.real_image_callback)
        self.overlay_img_pub = rospy.Publisher("/camera/exp_calibration",
                                               Image, queue_size=10)

    def read_sim_image(self, sample_idx):
        file_name = str(sample_idx) + '.jpeg'
        full_path = os.path.join(self.dataset_path, file_name)
        bgr_img = cv2.imread(full_path)
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        return rgb_img

    def real_image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            self.real_image = bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError, e:
            print(e)
        if self.sim_image is not None:
            alpha = 0.5
            overlay_img = np.uint8(
                alpha * self.sim_image + (1 - alpha) * self.real_image)
            self.overlay_imgmsg = bridge.cv2_to_imgmsg(overlay_img, 'rgb8')


def main(args):
    rospy.init_node("scene_calibrator", anonymous=True)
    rospy.sleep(1)
    dataset_name, sample_idx = args[1], args[2]
    calibrator = ExpCalibrator(dataset_name, sample_idx)
    print('Publishing overlay images...press ctrl+c to shut down')
    rospy.sleep(1)
    while not rospy.is_shutdown():
        calibrator.overlay_img_pub.publish(calibrator.overlay_imgmsg)
        rospy.sleep(0.033)


if __name__ == '__main__':
    main(sys.argv)
