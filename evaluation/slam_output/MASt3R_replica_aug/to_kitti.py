import numpy as np
from scipy.spatial.transform import Rotation as R
import glob
import os

# Load your data
# Assuming format: time x y z qx qy qz qw

target_folder = '/Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/MASt3R_replica_aug/data'
output_folder = '/Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/MASt3R_replica_aug/kitti_format'
for file in glob.glob(os.path.join(target_folder, '*.txt')):
    name = os.path.basename(file)
    data = np.loadtxt(file)
    # data = np.loadtxt('/Users/yuhsuanli/Desktop/projects/slam_project/evaluation/final_report/Replica.txt')

    # Prepare list to store poses
    poses = []

    for line in data:
        time, x, y, z, qx, qy, qz, qw = line

        # Convert quaternion to rotation matrix
        rotation = R.from_quat([qx, qy, qz, qw]).as_matrix()  # 3x3

        # Create 3x4 pose matrix
        pose = np.hstack((rotation, np.array([[x], [y], [z]])))

        poses.append(pose.flatten())  # flatten to 12 numbers

    # Save poses
    poses = np.array(poses)
    np.savetxt(os.path.join(output_folder, name), poses, fmt='%.8f')
