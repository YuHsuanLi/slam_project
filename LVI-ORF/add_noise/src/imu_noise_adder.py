#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import copy

def imu_callback(msg):
    noisy_msg = copy.deepcopy(msg)


    # noisy_msg.linear_acceleration.x += np.random.normal(0, 0.1)
    # noisy_msg.linear_acceleration.y += np.random.normal(0, 0.1)
    # noisy_msg.linear_acceleration.z += np.random.normal(0, 0.1)


    # noisy_msg.angular_velocity.x += np.random.normal(0, 0.01)
    # noisy_msg.angular_velocity.y += np.random.normal(0, 0.01)
    # noisy_msg.angular_velocity.z += np.random.normal(0, 0.01)

    noisy_msg.linear_acceleration.x += np.random.normal(0, 10)
    noisy_msg.linear_acceleration.y += np.random.normal(0, 10)
    noisy_msg.linear_acceleration.z += np.random.normal(0, 10)


    noisy_msg.angular_velocity.x += np.random.normal(0, 10)
    noisy_msg.angular_velocity.y += np.random.normal(0, 10)
    noisy_msg.angular_velocity.z += np.random.normal(0, 10)

    # noisy_msg.orientation.x *= 0
    # noisy_msg.orientation.y *= 0
    # noisy_msg.orientation.z *= 0
    # noisy_msg.orientation.w *= 0
    
    # noisy_msg.linear_acceleration.x *= 0
    # noisy_msg.linear_acceleration.y *= 0
    # noisy_msg.linear_acceleration.z *= 0


    # noisy_msg.angular_velocity.x *= 0
    # noisy_msg.angular_velocity.y *= 0
    # noisy_msg.angular_velocity.z *= 0

    imu_pub.publish(noisy_msg)

if __name__ == '__main__':
    rospy.init_node('imu_noise_adder')

    # imu_sub = rospy.Subscriber('/imu/data_raw', Imu, imu_callback)
    # imu_pub = rospy.Publisher('/imu/data_noisy', Imu, queue_size=10)
    imu_sub = rospy.Subscriber('/imu_raw', Imu, imu_callback)
    imu_pub = rospy.Publisher('/imu_noisy', Imu, queue_size=10)

    rospy.spin()
