#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()

def image_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    noise = np.random.normal(0, 10, cv_image.shape).astype(np.uint8)
    noisy_image = cv2.add(cv_image, noise)

    noisy_msg = bridge.cv2_to_imgmsg(noisy_image, encoding="bgr8")
    noisy_msg.header = msg.header  
    img_pub.publish(noisy_msg)
    # black_image = np.zeros_like(cv_image)

    # noisy_msg = bridge.cv2_to_imgmsg(black_image, encoding="bgr8")
    # noisy_msg.header = msg.header  
    # img_pub.publish(noisy_msg)

if __name__ == '__main__':
    rospy.init_node('image_noise_adder')

    # img_sub = rospy.Subscriber('/camera/image_raw', Image, image_callback)
    # img_pub = rospy.Publisher('/camera/image_noisy', Image, queue_size=10)
    img_sub = rospy.Subscriber('/kitti/camera_gray_left/image_raw', Image, image_callback)
    img_pub = rospy.Publisher('/kitti/camera_gray_left/image_raw_noisy', Image, queue_size=10)

    rospy.spin()
