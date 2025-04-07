# step 1
# turn to kitti format
# example 
evo_traj tum evaluation/slam_output/ORB-SLAM/CamTraj/Kitti00/Kitti00_CameraTrajectoryEuroc.txt --save_as_kitti
evo_traj tum evaluation/slam_output/ORB-SLAM/CamTraj/Replicate/Replicate00_CameraTrajectoryEuroc_processed.txt --save_as_kitti

# step2 run evo_ape
# example
evo_ape kitti  evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt evaluation/slam_output/ORB-SLAM/kitti_format/Kitti00_CameraTrajectoryEuroc.kitti -va  --plot_mode xz --save_plot orb_kitti_ape.png --align --correct_scale
#
# APE w.r.t. translation part (m)
# (with Sim(3) Umeyama alignment)

#        max      72.834106
#       mean      22.247723
#     median      21.709509
#        min      0.501314
#       rmse      24.807403
#        sse      2793948.949100
#        std      10.974793
evo_rpe kitti  evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt evaluation/slam_output/ORB-SLAM/kitti_format/Kitti00_CameraTrajectoryEuroc.kitti -va  --plot_mode xz --save_plot orb_kitti_rpe.png --align --correct_scale
# RPE w.r.t. translation part (m)
# for delta = 1 (frames) using consecutive pairs
# (with Sim(3) Umeyama alignment)

#        max      67.178885
#       mean      0.296198
#     median      0.146877
#        min      0.001980
#       rmse      1.759842
#        sse      14057.478476
#        std      1.734736

evo_ape kitti evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt evaluation/slam_output/ORB-SLAM/kitti_format/Replicate00_CameraTrajectoryEuroc_processed.kitti -va  --plot_mode xz --save_plot orb_replica_ape.png --align --correct_scale
# APE w.r.t. translation part (m)
# (with Sim(3) Umeyama alignment)

#        max      0.048863
#       mean      0.003367
#     median      0.002977
#        min      0.000224
#       rmse      0.004010
#        sse      0.032155
#        std      0.002177

evo_rpe kitti evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt evaluation/slam_output/ORB-SLAM/kitti_format/Replicate00_CameraTrajectoryEuroc_processed.kitti -va  --plot_mode xz --save_plot orb_replica_rpe.png --align --correct_scale
# RPE w.r.t. translation part (m)
# for delta = 1 (frames) using consecutive pairs
# (with Sim(3) Umeyama alignment)

#        max      0.016554
#       mean      0.002210
#     median      0.001869
#        min      0.000079
#       rmse      0.002674
#        sse      0.014291
#        std      0.001504

# slam3r
# note, remove the first line of the prediction so the length is the same
evo_ape kitti  evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt evaluation/slam_output/SLAM3R/Kitti00_cam_poses_processed.txt -va  --plot_mode xz --save_plot slam3r_kitti_ape.png --align --correct_scale
# APE w.r.t. translation part (m)
# (with Sim(3) Umeyama alignment)

#        max      470.518348
#       mean      155.774207
#     median      149.181351
#        min      3.579640
#       rmse      178.723778
#        sse      145017537.372572
#        std      87.616124
evo_rpe kitti  evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt evaluation/slam_output/SLAM3R/Kitti00_cam_poses_processed.txt -va  --plot_mode xz --save_plot slam3r_kitti_rpe.png --align --correct_scale
# RPE w.r.t. translation part (m)
# for delta = 1 (frames) using consecutive pairs
# (with Sim(3) Umeyama alignment)

#        max      223.344072
#       mean      20.658982
#     median      12.295260
#        min      0.218296
#       rmse      31.520753
#        sse      4509759.191213
#        std      23.806813

evo_ape kitti  evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt evaluation/slam_output/SLAM3R/Replica00_cam_poses_processed.txt -va  --plot_mode xz --save_plot slam3r_replica_ape.png --align --correct_scale
# APE w.r.t. translation part (m)
# (with Sim(3) Umeyama alignment)

#        max      0.172215
#       mean      0.047069
#     median      0.045055
#        min      0.004340
#       rmse      0.052504
#        sse      5.513426
#        std      0.023265

evo_rpe kitti  evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt evaluation/slam_output/SLAM3R/Replica00_cam_poses_processed.txt -va  --plot_mode xz --save_plot slam3r_replica_rpe.png --align --correct_scale
# RPE w.r.t. translation part (m)
# for delta = 1 (frames) using consecutive pairs
# (with Sim(3) Umeyama alignment)

#        max      0.101776
#       mean      0.007088
#     median      0.005962
#        min      0.000393
#       rmse      0.009252
#        sse      0.171119
#        std      0.005947