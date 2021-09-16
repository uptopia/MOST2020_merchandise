# MOST2020 Merchandise Stock and Reorientation

## About Submodule
First thing you need to do after clone, run
```bash
git submodule update --init --recursive 
```
Update submodule to the version tracked by super repo, run
```bash
git submodule update --recursive 
```
If you want to update the submodule to track the latest version, run
```bash
cd /path/to/submodule
git pull orign main
```

## Dependencies
* Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* Install [cv_bridge for Python3]()

## Software environment
* Ubuntu 18.04
* ROS Melodic
* Python 3.6.9
* opencv 4.5.1 cv2 (?)
* cv_bridge (python3 待測試) (?)
* Install pcl (?)
* Install cv_bridge(?)

## Function Blocks
### 1. 部件分割 affordance_ros
   + (1) `part_detection: YOLOv4, YOLOv4-tiny` 
      - popcorn_f, popcorn_b, juice
   + (2) `part_sematic_seg: DeepLab v3 Plus` 
      - plant, handle, cap, body 
  
### 2. 雙臂機器人 timda_dual_arm

### 3. 物件姿態與點雲 obj_pose_cloud

### 4. 工具選取策略 tool selection: vacuum gripper/ gripper
   + (1) which tool to use
   + (2) position, theta

### 5. 物件上架與擺正策略 object reorientation
   + (1) object cloud
   + (2) object orientatoin 
       (vector: juice[body->cap]; popcorn[plate->handle])
