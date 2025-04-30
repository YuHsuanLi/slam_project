#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np
# def lidar_callback(msg):
#     points = list(pc2.read_points(msg, skip_nans=True))
#     noisy_points = []

#     for p in points:
#         noisy_p = (
#             p[0] + np.random.normal(0, 0.05),  # x noise 0.05
#             p[1] + np.random.normal(0, 0.05),  # y noise
#             p[2] + np.random.normal(0, 0.05),  # z noise
#             p[3],  # intensity 
#             p[4],
#         )
#         # noisy_p = (
#         #     p[0]*0 ,  # x noise
#         #     p[1]*0 ,  # y noise
#         #     p[2]*0 ,  # z noise
#         #     p[3],  # intensity 
#         #     p[4],
#         # )
#         noisy_points.append(noisy_p)

#     noisy_msg = pc2.create_cloud(
#         msg.header,
#         msg.fields,
#         noisy_points
#     )
#     noisy_msg.is_dense = True
#     cloud_pub.publish(noisy_msg)

def lidar_callback(msg):
    noise_std = 0.0
    points = np.array(list(pc2.read_points(msg, skip_nans=True)), dtype=np.float32)
    points[:, :3] += np.random.normal(0, noise_std, size=(points.shape[0], 3))
    noisy_msg = pc2.create_cloud(
        msg.header,
        msg.fields,
        points
    )
    noisy_msg.is_dense = True
    cloud_pub.publish(noisy_msg)
    # noise_std = 0
    # points = np.array(list(pc2.read_points(msg, skip_nans=True)))
    # noise = np.random.normal(0, noise_std, size=(points.shape[0], 3))
    # points[:, :3] += noise
    # points = list(pc2.read_points(msg, skip_nans=True))
    # # noisy_points = []
    # points = np.array(points)
    # noisy_points = points.copy()
    # # noisy_points[:, :3] += np.random.normal(0, 0.05, size=(points.shape[0], 3))
    # noisy_points[:, :3] += np.random.normal(0, 0.00, size=(points.shape[0], 3))

    # noisy_points = noisy_points.tolist()
    # noisy_points = []
    # for p in points:
    #     noisy_p = (
    #         p[0], #+ np.random.normal(0, 0.05),  # x noise 0.05
    #         p[1], #+ np.random.normal(0, 0.05),  # y noise
    #         p[2], #+ np.random.normal(0, 0.05),  # z noise
    #         p[3],  # intensity 
    #         p[4],
    #     )
        # noisy_p = (
        #     p[0]*0 ,  # x noise
        #     p[1]*0 ,  # y noise
        #     p[2]*0 ,  # z noise
        #     p[3],  # intensity 
        #     p[4],
        # )
    # noisy_points.append(noisy_p)

    # noisy_msg = pc2.create_cloud(
    #     msg.header,
    #     msg.fields,
    #     noisy_points
    # )
    # noisy_msg.is_dense = True
    # cloud_pub.publish(noisy_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_noise_adder')

    # cloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, lidar_callback)
    # cloud_pub = rospy.Publisher('/velodyne_points_noisy', PointCloud2, queue_size=10)
    cloud_sub = rospy.Subscriber('/points_raw', PointCloud2, lidar_callback)
    cloud_pub = rospy.Publisher('/points_noisy', PointCloud2, queue_size=10)

    rospy.spin()
