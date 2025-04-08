import os
import sys 
import subprocess

dataset = "kitti"
if dataset == "kitti":
    folder_path = "evaluation/slam_output/ORB-SLAM/Kitti00"
    gt_path = "evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt"
    save_path = "evaluation/results/ORB-SLAM/Kitti/00"
elif dataset == "replica":
    folder_path = "evaluation/slam_output/ORB-SLAM/Replica"
    gt_path = "evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt"
    save_path = "evaluation/results/ORB-SLAM/Replica"

file_name = "CameraTrajectoryEuroc.txt"
prepend_file_name = "CameraTrajectoryEuroc_prepend.txt"
kitti_name = "CameraTrajectoryEuroc.kitti"


# Check if it's a valid directory
if not os.path.isdir(folder_path):
    print(f"Error: '{folder_path}' is not a valid directory.")
    sys.exit(1)

# List subfolders
subfolders = [os.path.join(folder_path, name) for name in os.listdir(folder_path)
              if os.path.isdir(os.path.join(folder_path, name))]

with open(gt_path, "r") as f:
    lines = f.readlines()
    target_lines = len(lines)

for subfolder in subfolders:
    file_path = os.path.join(subfolder, file_name)
    prepend_file_path = os.path.join(subfolder, prepend_file_name)
    with open(file_path, "r") as f_in, open(prepend_file_path, "w") as f_out:
        lines = f_in.readlines()
        diff = target_lines - len(lines)
        padded_lines = [lines[0]] * diff + lines
        f_out.writelines(padded_lines)
exit(0)

# Loop through subfolders
# for subfolder in subfolders:
#     tum_file_path = os.path.join(subfolder, prepend_file_name)
#     cmd = f"evo_traj tum {tum_file_path} --save_as_kitti"
#     result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
#     cmd = f"mv CameraTrajectoryEuroc_prepend.kitti CameraTrajectoryEuroc.kitti"
#     result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

#     cmd = f"mv {kitti_name} {subfolder}"
#     result = subprocess.run(cmd, shell=True, capture_output=True, text=True)


for subfolder in subfolders:
    kitti_file_path = os.path.join(subfolder, kitti_name)
    subcategory = subfolder.split("/")[-1]
    # plot_name = f"evaluation/results/ORB-SLAM/kitti/00/{subcategory}/ape.png"
    # output_file_path = f"evaluation/results/ORB-SLAM/kitti/00/{subcategory}/ape.txt"
    plot_name = os.path.join(save_path, subcategory, "ape.png")
    output_file_path = os.path.join(save_path, subcategory, "ape.txt")
    os.makedirs(os.path.dirname(plot_name), exist_ok=True)
    cmd = f"evo_ape kitti {gt_path} {kitti_file_path} -va  --plot_mode xz --save_plot {plot_name} --align --correct_scale"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    output = result.stdout.strip()
    with open(output_file_path, "w") as file:
        file.write(output)

    # plot_name = f"evaluation/results/ORB-SLAM/kitti/00/{subcategory}/rpe.png"
    # output_file_path = f"evaluation/results/ORB-SLAM/kitti/00/{subcategory}/rpe.txt"
    plot_name = os.path.join(save_path, subcategory, "rpe.png")
    output_file_path = os.path.join(save_path, subcategory, "rpe.txt")

    os.makedirs(os.path.dirname(plot_name), exist_ok=True)
    cmd = f"evo_rpe kitti {gt_path} {kitti_file_path} -va  --plot_mode xz --save_plot {plot_name} --align --correct_scale"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    output = result.stdout.strip()
    with open(output_file_path, "w") as file:
        file.write(output)
