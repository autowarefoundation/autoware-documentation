# Sensing

![Node diagram](images/Sensing-Bus-ODD-Architecture.drawio.svg)

## Inputs

### The sensing module contains the driver for the sensor, so there is no input

## Output

### Camera image

Image data from camera. Used by the Perception.

- [sensor_msgs/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)

### Lidar original pointcloud

Pointcloud without preprocessing. Used by the Perception and Localization.

- [sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)

### Lidar filtered pointcloud

Pointcloud after preprocessing. Used by the Perception.

- [sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)

### Radar pointcloud

Pointcloud from radar. Used by the Perception.

- [sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)

### Corrected IMU data

IMU data with yaw rate offset compensated. Used by the Localization.

- [sensor_msgs/Imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)

### Vehicle velocity

Velocity of the ego vehicle. Used by the Localization.

- [geometry_msgs/TwistWithCovarianceStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)

### GNSS pose

Initial pose of the ego vehicle from GNSS. Used by the Localization.

- [geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)
- [geometry_msgs/PoseWithCovarianceStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
