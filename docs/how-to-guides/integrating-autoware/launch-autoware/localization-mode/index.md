# Localization mode

!!! warning

    Under Construction

AutowareのLocalizationコンポーネントは複数の異なるセンサ構成で動く位置推定手法を提供しています。

| localization mode | method  | map type        |
|-------------------|---------|-----------------|
| LiDAR-based       | NDT     | point cloud map |
| camera-based      | YabLoc  | vector map      |
| GNSS/IMU-based    | Eagleye | -               |

## How to launch LiDAR-based localizer (default)

Autowreのデフォルトでは自己位置推定は[ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_scan_matcher)が起動します。

## How to launch camera-based localizer

カメラベースの位置推定手法としてはYabLocが利用できます。 YabLocの詳細は [YabLoc Guide](yabloc-guide.md)を見てください。

YabLocをndt_scan_matcherに代わってpose_estimatorとして利用する場合はAutowareの起動時に次のように`localization_mode:=camera`を追加してください。
デフォルトでは`localization_mode:=lidar`になっています。
コマンドライン引数を指定することにより、自動でYabLocのノードが起動し、NDTのノードは起動しないようになります。

```bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=YOUR_VEHICLE \
  sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP \
  localization_mode:=camera # Add this argument
```

## How to launch GNSS/IMU-based localizer

GNSS/IMU-based localizerとしてEagleyeが利用可能です。
Eagleyeの詳細は [Eagleye Guide](eagleye-guide.md)を見てください。

Eagleyeはtwist_estimatorとしてのオプションでもあり、pose_estimatorとtwist_estimator両方の代替オプションでもあります。
現状ではEagleyeの利用するためにはコマンドライン引数での指定に加えてlaunchファイルの修正が必要です。

### Available Options

Eagleye has a function for position estimation and twist estimation, namely `pose_estimator` and `twist_estimator`, respectively.

| localization launch                                               | twist estimator                         | pose estimator                          |
|-------------------------------------------------------------------|-----------------------------------------|-----------------------------------------|
| `tier4_localization_launch`                                       | `gyro_odometry`                         | `ndt_scan_matcher`                      |
| `map4_localization_launch/eagleye_twist_localization_launch`      | `eagleye_rt`<br>(gyro/odom/gnss fusion) | `ndt_scan_matcher`                      |
| `map4_localization_launch/eagleye_pose_twist_localization_launch` | `eagleye_rt`<br>(gyro/odom/gnss fusion) | `eagleye_rt`<br>(gyro/odom/gnss fusion) |

### 1. Modifying Autoware launch files

Please refer to `map4_localization_launch` in the `autoware.universe` package and `map4_localization_component.launch.xml` in `autoware_launch` package for information on how to modify the localization launch.


When using Eagleye, comment out `tier4_localization_component.launch.xml` and start `map4_localization_component.launch.xml` in `autoware.launch.xml`.

```xml
  <!-- Localization -->
  <group if="$(var launch_localization)">
    <!-- <include file="$(find-pkg-share autoware_launch)/launch/components/tier4_localization_component.launch.xml"/> -->
    <include file="$(find-pkg-share autoware_launch)/launch/components/map4_localization_component.launch.xml"/>
  </group>
```

### 2. Execution command

Enable Eagleye in Autoware by switching the localization module in autoware.launch.xml and the `pose_estimator_mode` parameter in `map4_localization_component.launch.xml` in `autoware.launch.xml`.

| twist_estimator_mode        | pose_estimator_mode | method                          |
|-----------------------------|---------------------|---------------------------------|
| `gyro_odom_fusion`(default) | `lidar`(default)    | NDT (default)                   |
| `gyro_odom_gnss_fusion`     | `lidar`(default)    | eagleye as twist_estiamtor      |
| `gyro_odom_fusion`(default) | `gnss`              | eagleye as pose_twist_estiamtor |
| `gyro_odom_gnss_fusion`     | `gnss`              | NOT supported                   |

In Autoware, you can set the pose estimator to GNSS by setting `pose_estimator_mode:=gnss` in `map4_localization_component.launch.xml` in `autoware_launch` package.

**Eagleyeをpose_estiamtorとして使う方法**
```bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=YOUR_VEHICLE \
  sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP \
  pose_estimator_mode:=gnss # Add this argument
```

**Eagleyeをtwist_estiamtorとして使う方法**
```bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=YOUR_VEHICLE \
  sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP \
  twist_estimator_mode:=gyro_odom_gnss_fusion # Add this argument
```