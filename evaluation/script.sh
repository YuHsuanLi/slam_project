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

evo_ape kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/05.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/traj_kitti.txt -va  --plot_mode xz --save_plot lviorf_kitti_ape.png --align --correct_scale
evo_ape kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/traj_kitti00_ori.txt -va  --plot_mode xz --save_plot lviorf_kitti00_ape_ori.png --align --correct_scale
    #    max      11.640139
    #   mean      3.106679
    # median      2.651872
    #    min      0.841386
    #   rmse      3.728412
    #    sse      63110.809309
    #    std      2.061456

evo_rpe kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/traj_kitti00_ori.txt -va  --plot_mode xz --save_plot lviorf_kitti00_rpe_ori.png --align --correct_scale


evo_ape kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/traj_kitti00_black.txt -va  --plot_mode xz --save_plot lviorf_kitti00_ape_black.png --align --correct_scale
    #    max      10.536818
    #   mean      2.934048
    # median      2.561094
    #    min      0.834192
    #   rmse      3.451168
    #    sse      54073.930973
    #    std      1.817118
evo_ape kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/final_report/kitti_mast3r.txt -va  --plot_mode xz --save_plot kitti_mast3r.png --align --correct_scale
    #    max      339.941998
    #   mean      168.803801
    # median      167.188815
    #    min      5.281538
    #   rmse      190.850247
    #    sse      165364128.581690
    #    std      89.045458
evo_rpe kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/final_report/kitti_mast3r.txt -va  --plot_mode xz --save_plot kitti_mast3r_rpe.png --align --correct_scale
    #    max      107.279719
    #   mean      0.877368
    # median      0.869680
    #    min      0.001850
    #   rmse      1.832924
    #    sse      15249.267546
    #    std      1.609297

evo_ape kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/final_report/kitti_orbslam3.txt -va  --plot_mode xz --save_plot kitti_orbslam3_ape.png --align --correct_scale
    #    max      1.963896
    #   mean      0.777931
    # median      0.761275
    #    min      0.043690
    #   rmse      0.856029
    #    sse      3326.846428
    #    std      0.357224
evo_rpe kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/final_report/kitti_orbslam3.txt -va  --plot_mode xz --save_plot kitti_orbslam3_rpe.png --align --correct_scale
    #  max      0.299969
    #   mean      0.018686
    # median      0.014566
    #    min      0.000592
    #   rmse      0.027105
    #    sse      3.334735
    #    std      0.019635
evo_ape kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/final_report/replica_mast3r.txt -va  --plot_mode xz --save_plot slam3r_replica_ape.png --align --correct_scale
    #    max      0.033678
    #   mean      0.012770
    # median      0.012079
    #    min      0.000150
    #   rmse      0.013988
    #    sse      0.391343
    #    std      0.005710

evo_rpe kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Replica/traj_processed.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/final_report/replica_mast3r.txt -va  --plot_mode xz --save_plot slam3r_replica_rpe.png --align --correct_scale
    #    max      0.029502
    #   mean      0.001919
    # median      0.001550
    #    min      0.000098
    #   rmse      0.002509
    #    sse      0.012581
    #    std      0.001615

evo_ape kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/traj_kitti00_imu.txt -va  --plot_mode xz --save_plot lviorf_kitti00_ape_imu.png --align --correct_scale

evo_ape kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/traj_kitti00_imu_10.txt -va  --plot_mode xz --save_plot lviorf_kitti00_ape_imu_10.png --align --correct_scale

evo_ape kitti  /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/slam_output/gt/CamTraj/Kitti/dataset/poses/00.txt /Users/yuhsuanli/Desktop/projects/slam_project/evaluation/traj_kitti00_lidar.txt -va  --plot_mode xz --save_plot lviorf_kitti00_ape_lidar.png --align --correct_scale
    #    max      339.744640
    #   mean      169.763138
    # median      172.139266
    #    min      15.028323
    #   rmse      189.177670
    #    sse      162478385.963584
    #    std      83.478546