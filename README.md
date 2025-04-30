
# LVI-ORF

This project adapts and extends the LVI-ORF framework for experiments with noise injection on the KITTI dataset.

---

##  Environment Setup (via Docker)

We recommend using Docker for a consistent runtime environment. The base image is compatible with [LVI-SAM's Docker guide](https://github.com/TixiaoShan/LVI-SAM/blob/master/docker/docker_start.md).

### 1. Pull or Build Docker Image

```bash
docker pull tyoung96/lvi_sam:1.0
```

### 2. Run Docker Container

```bash
docker run -it \
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
  -v {your_lviorf_path}:/home/catkin_ws/src/lviorf \
  -v {your_add_noise_path}:/home/catkin_ws/src/add_noise \
  tyoung96/lvi_sam:1.0 /bin/bash
```

> Replace `{your_lviorf_path}` and `{your_add_noise_path}` with the absolute paths on your machine.

### 3. Inside Docker (First-Time Setup)

```bash
apt update
apt install ros-melodic-desktop-full libgeographic-dev
source /opt/ros/melodic/setup.bash
cd /home/catkin_ws
catkin_make -j8
source devel/setup.bash
```

---

## Run (Basic - Without Noise)

> It's recommended to use `tmux` to manage multiple terminals.

**Terminal 0: Launch LVI-ORF**
```bash
roslaunch lviorf run_kitti.launch
```

**Terminal 1: Record Result**
```bash
rostopic echo -p /lviorf/mapping/odometry > result.csv
```

**Terminal 2: Play ROS bag**
```bash
rosbag play kitti_2011_10_03_drive_0027_synced.bag
```

To download KITTI 05:
```bash
wget --no-check-certificate \
  'https://drive.google.com/uc?export=download&id=1uLSVayprhJcTqvY5q8lgRoWl8nNjkWAM' \
  -O kitti_2011_09_30_drive_0018_synced.bag
```

For additional routes, follow:  
https://github.com/TixiaoShan/LIO-SAM/tree/master/config/doc/kitti2bag

---

## Run (With Noise Injection)

### Step 1: Rebuild Workspace

```bash
cd /home/catkin_ws
catkin_make
source devel/setup.bash
```

### Step 2: Configure Noise Parameters

Modify the following config files as needed to enable noise for IMU, visual, or LiDAR:
- `LVI-ORF/lviorf/config/pinhole/kitti_params_camera.yaml`
- `LVI-ORF/lviorf/config/pinhole/kitti_params_lidar.yaml`

> Update the topic names to point to the noisy versions.

### Step 3: Launch the Noise Injection Node

Run **before** `rosbag play`:

```bash
rosrun add_noise src/lidar_noise_adder.py
```



