import os
import sys 
import subprocess
import glob

data_folder = '/Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/MASt3R_replica_aug/kitti_format'
gt_path = "/Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt"
result_folder = '/Users/yuhsuanli/Desktop/projects/slam_project/evaluation/results/MASt3R_replica'
for file in glob.glob(os.path.join(data_folder, '*.txt')):
    kitti_file_path = file
    plot_name = os.path.join(result_folder, os.path.basename(file).replace(".txt", ".png"))
    cmd = f"evo_rpe kitti {gt_path} {kitti_file_path} -va  --plot_mode xz --save_plot {plot_name} --align --correct_scale"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    output = result.stdout.strip()
    output_file_path = os.path.join(result_folder, os.path.basename(file))
    with open(output_file_path, "w") as file:
        file.write(output)
