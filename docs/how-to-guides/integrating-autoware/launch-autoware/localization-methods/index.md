# Localization methods

Current localization launcher implemented by TIER IV supports multiple localization methods, both pose estimators and twist estimators.
`tier4_localization_component.launch.xml` has two arguments to select which estimators to launch:

- `pose_source` : an argument to select pose_estimator, currently supporting `ndt` (default), `yabloc`, and `eagleye`
- `twist_source` : an argument to select twist_estimator, currently supporting `gyro_odom` (default), and `eagleye`

## NDT scan matcher: a LiDAR and pointcloud map based pose estimator (default)

By default, Autoware launches [ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/ndt_scan_matcher) for pose estimator.
In order to launch this explicitly, you need to specify as follows:

```bash
ros2 launch autoware_launch autoware.launch.xml ... pose_source:=ndt ...
```

Note that currently `pose_source` is set to NDT as default, so you can skip this argument.

## Gyro Odometer: an IMU & wheel odometry based twist estimator (default)

By default, Autoware launches [gyro_odometer](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/gyro_odometer) for twist estimator.
In order to launch this explicitly, you need to specify as follows:

```bash
ros2 launch autoware_launch autoware.launch.xml ... twist_source:=gyro_odom ...
```

Note that currently `twist_source` is set to Gyro Odometer as default, so you can skip this argument.

## YabLoc: a camera and vector map based pose estimator

You can use YabLoc as a camera-based localization method.
For more details on YabLoc, please refer to the [README of YabLoc](https://github.com/autowarefoundation/autoware.universe/blob/main/localization/yabloc/README.md) in autoware.universe.

To use YabLoc as a pose_estimator, add `localization_mode:=camera` when launching Autoware.
By default, the `localization_mode` is set to `lidar`.
By specifying this command-line argument, YabLoc nodes will be automatically launched while the NDT nodes will not be started.

Here is an example of a launch command:

```bash
ros2 launch autoware_launch autoware.launch.xml ... pose_source:=yabloc ...
```

## Eagleye: a GNSS & IMU & wheel odometry based pose and twist estimator

You can use Eagleye as a GNSS & IMU & wheel odometry-based localization method. For more details on Eagleye, please refer to the [Eagleye](eagleye-guide.md).

Eagleye has a function for position estimation and twist estimation, namely `pose_estimator` and `twist_estimator`, respectively.
When running Eagleye in twist_estimator mode with other pose_estimator such as ndt_scan_matcher, Eagleye is still helpful since it can improve scan matching by providing accurate twists using GNSS doppler.

You can use Eagleye by specifying the `pose_source` and `twist_source` accordingly through command-line arguments.

**Example of using Eagleye as the pose twist estimator:**

```bash
ros2 launch autoware_launch autoware.launch.xml ... pose_source:=eagleye twist_source:=eagleye ...
```

**Example of using Eagleye as the twist estimator:**

```bash
ros2 launch autoware_launch autoware.launch.xml ... twist_source:=eagleye ...
```
