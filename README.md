## LVI-ORF
### environment
docker: reference https://github.com/TixiaoShan/LVI-SAM/blob/master/docker/docker_start.md
1. docker pull tyoung96/lvi_sam:1.0
2. docker build -t lvi_sam:1.0 .
3. docker run -it \
--gpus all \
--privileged \
--net=host \
--ipc=host \
--shm-size=1gb \
--name=lviorf \
-e DISPLAY=$DISPLAY \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-e NVIDIA_VISIBLE_DEVICES=all \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/.Xauthority:/root/.Xauthority \
-v {your lviorf path}:/home/catkin_ws/src/lviorf \
-v {your add_noise path}:/home/catkin_ws/src/add_noise \
tyoung96/lvi_sam:1.0 /bin/bash

inside the docker
apt install ros-melodic-desktop-full
apt install libgeographic-dev
source /opt/ros/melodic/setup.bash
(under /home/catkin_ws)
catkin_make -j8
source devel/setup.bash

### Run (basic no further noise)
(suggest use tmux)
0: /home/catkin_ws/src# roslaunch lviorf run_kitti.launch    
1: rostopic echo -p /lviorf/mapping/odometry > result.csv
2: rosbag play kitti_2011_10_03_drive_0027_synced.bag
(kitt05 can be download by wget --no-check-certificate 'https://drive.google.com/uc?export=download&id=1uLSVayprhJcTqvY5q8lgRoWl8nNjkWAM' -O kitti_2011_09_30_drive_0018_synced.bag)
for other route, please follow https://github.com/TixiaoShan/LIO-SAM/tree/master/config/doc/kitti2bag

### Run (with noise)
(under /home/catkin_ws)
catkin_make
source devel/setup.bash
modify 
LVI-ORF/lviorf/config/pinhole/kitti_params_camera.yaml
LVI-ORF/lviorf/config/pinhole/kitti_params_lidar.yaml
according to you want to add noise to IMU/visual/Lidar, modify the topic name
3: rosrun add_noise src/lidar_noise_adder.py
(this should be run before rosbag play)