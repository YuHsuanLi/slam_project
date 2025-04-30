import argparse
import os
import cv2
import imgaug.augmenters as iaa
import shutil

# +
import argparse
import os
import cv2
import imgaug.augmenters as iaa

def apply_transform(image, transform, severity, rain_size, rain_speed):
    if transform == "fog":
        augmenter = iaa.Fog()#alpha=(0.1 * severity, 0.2 * severity))
    elif transform == "blur":
        augmenter = iaa.GaussianBlur(sigma=(severity, severity + 0.5))
    elif transform == "saturation":
        factor = max(0.1, 1.0 - 0.2 * severity)
        augmenter = iaa.MultiplySaturation((factor, factor))
    elif transform == "contrast":
        factor = max(0.1, 1.0 - 0.2 * severity)
        augmenter = iaa.LinearContrast((factor, factor))
    elif transform == "lowlight":
        factor = max(0.1, 1.0 - 0.6 * severity)
        augmenter = iaa.Multiply((factor, factor))
    elif transform == "rain":
        augmenter = iaa.Rain(drop_size=(rain_size, rain_size + 0.05),
                             speed=(rain_speed, rain_speed + 0.05))
    elif transform == "clouds":
        augmenter = iaa.Clouds()#alpha=(0.2 * severity, 0.5 * severity))
    elif transform == "snow":
        augmenter = iaa.Snowflakes(flake_size=(0.1 * severity, 0.2 * severity),
                                   speed=(0.01 * severity, 0.03 * severity))
    elif transform == "jpeg":
        augmenter = iaa.JpegCompression(compression=(70*severity, 70*severity))
    elif transform == "noise":
        augmenter = iaa.AdditivePoissonNoise(lam=(10*severity, 20*severity))
    elif transform == "grayscale":
        augmented = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return augmented
    else:
        raise ValueError(f"Unsupported transform: {transform}")
    
    return augmenter(image=image)


# -

# cap = cv2.VideoCapture(r"/home/yujing/code/SLAM/slam_project/SLAM3R/Replica_demo/room0/image_0/%06d.png")
# ret = True

img_path = r"/home/yujing/code/SLAM/slam_project/SLAM3R/data/Replica_demo/room0_{0}/{1}.png"

augmentations = [
#     'fog', 
    'blur', 
    'saturation', 
    'contrast', 
    'lowlight', 
#     'rain', 
#     'clouds', 
#     'snow', 
    'jpeg', 
    'noise', 
    "grayscale",
]
for augmentation in augmentations:
    os.makedirs(r"/home/yujing/code/SLAM/slam_project/SLAM3R/data/Replica_demo/room0_{0}".format(augmentation), exist_ok=True)
    # shutil.copy(r"/home/yujing/code/SLAM/slam_project/SLAM3R/Replica_demo\room0\times.txt", r"C:\Files\CMU\Courses\SLAM\Replica\room0_{0}\times.txt".format(augmentation))

frame_id = 0
root = r"/home/yujing/code/SLAM/slam_project/SLAM3R/data/Replica_demo/room0"
for frame_id in range(len(os.listdir(root))):
    frame = cv2.imread(r"/home/yujing/code/SLAM/slam_project/SLAM3R/data/Replica_demo/room0/frame{0}.jpg".format(str(frame_id).zfill(6)))
    for augmentation in augmentations:
        augmented = apply_transform(frame, severity=1, 
                        transform=augmentation,
                        rain_size=0.1,
                        rain_speed=0.2,
                       )
        cv2.imwrite(img_path.format(augmentation, str(frame_id).zfill(6)), augmented)
    frame_id += 1


