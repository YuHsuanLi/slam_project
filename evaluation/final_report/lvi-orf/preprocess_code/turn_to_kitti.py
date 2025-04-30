import pandas as pd
import numpy as np

# Load CSV extracted from rostopic
df = pd.read_csv("/Users/yuhsuanli/Desktop/projects/slam_project/evaluation/kitti00_odometry_lidar_interp0000.csv")

with open("traj_kitti00_lidar_0000.txt", "w") as f:
    for _, row in df.iterrows():
        # Get translation
        tx, ty, tz = row['field.pose.pose.position.x'], row['field.pose.pose.position.y'], row['field.pose.pose.position.z']
        
        # Get orientation quaternion
        qx, qy, qz, qw = row['field.pose.pose.orientation.x'], row['field.pose.pose.orientation.y'], row['field.pose.pose.orientation.z'], row['field.pose.pose.orientation.w']
        
        # Convert to rotation matrix
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw,     1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx**2 - 2*qy**2]
        ])
        
        # Construct 3x4 transformation matrix
        T = np.hstack((R, np.array([[tx], [ty], [tz]])))
        
        # Flatten and write to file
        line = ' '.join(map(str, T.flatten()))
        f.write(line + '\n')
