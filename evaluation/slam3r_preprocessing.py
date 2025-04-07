# input_file = 'evaluation/slam_output/SLAM3R/Kitti00_cam_poses.txt'
# output_file = 'evaluation/slam_output/SLAM3R/Kitti00_cam_poses_processed.txt'
input_file = 'evaluation/slam_output/SLAM3R/Replica00_cam_poses.txt'
output_file = 'evaluation/slam_output/SLAM3R/Replica00_cam_poses_processed.txt'

with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
    for line in f_in:
        values = line.strip().split(',')
        trimmed_values = values[:-4]  # remove last 4 columns
        f_out.write(' '.join(trimmed_values) + '\n')

