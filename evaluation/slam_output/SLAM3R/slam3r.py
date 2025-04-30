import os
import sys
import subprocess

folder_path = "evaluation/slam_output/SLAM3R/Replica"
gt_path = "evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt"
save_path = "evaluation/results/SLAM3R/Replica"

file_name = "cam_poses.txt"
processed_file_name = "cam_poses_processed.txt"

# Check if it's a valid directory
if not os.path.isdir(folder_path):
    print(f"Error: '{folder_path}' is not a valid directory.")
    sys.exit(1)

# List subfolders
subfolders = [os.path.join(folder_path, name) for name in os.listdir(folder_path)
              if os.path.isdir(os.path.join(folder_path, name))]

# for subfolder in subfolders:
#     file_path = os.path.join(subfolder, file_name)
#     processed_file_path = os.path.join(subfolder, processed_file_name)
#     with open(file_path, 'r') as f_in, open(processed_file_path, 'w') as f_out:
#         for line in f_in:
#             values = line.strip().split(',')
#             trimmed_values = values[:-4]  # remove last 4 columns
#             f_out.write(' '.join(trimmed_values) + '\n')

# exit(0)


for subfolder in subfolders:
    processed_file_path = os.path.join(subfolder, processed_file_name)
    subcategory = subfolder.split("/")[-1]
    plot_name = os.path.join(save_path, subcategory, "ape.png")
    output_file_path = os.path.join(save_path, subcategory, "ape.txt")
    os.makedirs(os.path.dirname(plot_name), exist_ok=True)
    cmd = f"evo_ape kitti {gt_path} {processed_file_path} -va  --plot_mode xz --save_plot {plot_name} --align --correct_scale"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    output = result.stdout.strip()
    with open(output_file_path, "w") as file:
        file.write(output)

    # plot_name = f"evaluation/results/ORB-SLAM/kitti/00/{subcategory}/rpe.png"
    # output_file_path = f"evaluation/results/ORB-SLAM/kitti/00/{subcategory}/rpe.txt"
    plot_name = os.path.join(save_path, subcategory, "rpe.png")
    output_file_path = os.path.join(save_path, subcategory, "rpe.txt")

    os.makedirs(os.path.dirname(plot_name), exist_ok=True)
    cmd = f"evo_rpe kitti {gt_path} {processed_file_path} -va  --plot_mode xz --save_plot {plot_name} --align --correct_scale"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    output = result.stdout.strip()
    with open(output_file_path, "w") as file:
        file.write(output)
