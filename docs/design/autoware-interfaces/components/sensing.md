# Sensing

![Node diagram](images/Sensing-Bus-ODD-Architecture.drawio.svg)

## Inputs

### The sensing module contains the driver for the sensor, so there is no input

## Output

### Camera image

Image data from camera. Used by the Perception.

- [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg)

### Radar tracks

Tracks from radar. Used by the Perception.

- [radar_msgs/RadarTracks](https://docs.ros.org/en/noetic/api/radar_msgs/html/msg/RadarTracks.html)

### Radar pointcloud

Pointcloud from radar. Used by the Perception.

- [sensor_msgs/PointCloud2](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)

### Lidar preprocessed pointcloud

Lidar pointcloud after preprocessing. Used by the Perception and Localization.

- [sensor_msgs/PointCloud2](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)

### Corrected IMU data

IMU data with yaw rate offset compensated. Used by the Localization.

- [sensor_msgs/Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg)

### Vehicle velocity

Velocity of the ego vehicle. Used by the Localization.

- [geometry_msgs/TwistWithCovarianceStamped](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/TwistWithCovarianceStamped.msg)

### GNSS pose

Initial pose of the ego vehicle from GNSS. Used by the Localization.

- [geometry_msgs/PoseStamped](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg)
- [geometry_msgs/PoseWithCovarianceStamped](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseWithCovarianceStamped.msg)
