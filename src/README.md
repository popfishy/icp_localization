A simple localization framework that can re-localize in built maps. 

## 1. Dependencies for localization module
```
python >= 3.5
open3d == 0.13.0
```

## 2. Build

Clone the repository and catkin_make:

```
    cd LOCALIZATION
    catkin_make
    source devel/setup.bash
```

## 3. Merge pointcloud

修改isRGBcloud值最终输出点云类型

isRGBcloud = false 生成普通PointXYZI点云结果

isRGBcloud = true 生成彩色PointXYZRGB点云结果

```
source devel/setup.bash
rosrun localization merge
```

## 4. Load your map

修改localization.launch中的map路径

```
source devel/setup.bash
roslaunch localization localization.launch
```

## 5. Publish your scan

```
source devel/setup.bash
rosrun localization publish_cur_scan
```

Please modify `/path/to/your/map.pcd` to your own map point cloud file path.

Wait for 3~5 seconds until the map cloud shows up in RVIZ;

## Based on FAST_LIO_LOCALIZATION(https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)
