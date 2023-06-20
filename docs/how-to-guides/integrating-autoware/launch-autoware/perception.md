# Perception mode

!!! warning

    Under Construction

Autowareの起動時に次のように`perception_mode`を指定することで、どのセンサ構成で認識をするかを切り替えることができます。

```bash
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=YOUR_VEHICLE sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP \
    perception_mode:=lidar
```

## LiDAR

`perception_mode:=lidar`

## Radar

`perception_mode:=radar`

## Camera LiDAR fusion

`perception_mode:=camera_lidar_fusion`

## Camera LiDAR Radar fusion 

`perception_mode:=camera_lidar_radar_fusion`

## LiDAR Radar fusion 

`perception_mode:=lidar_radar_fusion`