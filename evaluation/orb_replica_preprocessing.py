# gt_file = '/Users/yuhsuanli/Desktop/projects/16833_robot_localization_and_mapping/project/gt.txt'
# input_file = '/Users/yuhsuanli/Desktop/projects/16833_robot_localization_and_mapping/project/traj_del.txt'
# output_file = 'traj_trimmed.txt'

input_file = 'evaluation/slam_output/gt/CamTraj/Replica/traj.txt'
output_file = 'evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt'
orbslam_file = 'evaluation/slam_output/ORB-SLAM/CamTraj/Replicate/Replicate00_CameraTrajectoryEuroc.txt'
orbslam_output_file = 'evaluation/slam_output/ORB-SLAM/CamTraj/Replicate/Replicate00_CameraTrajectoryEuroc_processed.txt'

with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
    for line in f_in:
        values = line.strip().split()
        trimmed_values = values[:-4]  # remove last 4 columns
        f_out.write(' '.join(trimmed_values) + '\n')

with open(orbslam_file, 'r') as f_in, open(orbslam_output_file, 'w') as f_out, open(input_file, 'r') as f_in2:
    lines = f_in.readlines()
    lines2 = f_in2.readlines()
    diff = len(lines2) - len(lines)
    padded_lines = [lines[0]] * diff + lines
    f_out.writelines(padded_lines)