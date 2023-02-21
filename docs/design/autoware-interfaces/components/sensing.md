# Sensing

![Node diagram](images/Sensing-Bus-ODD-Architecture.drawio.svg)

## Inputs

### Camera raw image

Origin data from camera. Published by the camera driver.

- [sensor_msgs/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)

### Lidar raw pointcloud

Origin data from lidar. Published by the lidar driver.

- [sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)

### Radar scan

Origin data from radar scan. Published by the Radar driver.

- [radar_msgs/RadarScan](https://docs.ros.org/en/noetic/api/radar_msgs/html/msg/RadarScan.html)

### Radar tracks

Origin data from radar scan. Published by the Radar driver.

- [radar_msgs/RadarTracks](https://docs.ros.org/en/noetic/api/radar_msgs/html/msg/RadarTracks.html)

### IMU

Origin data from IMU. Published by the IMU driver.

- [sensor_msgs/Imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)

### GNSS

Origin data from GNSS. Published by the GNSS driver.

- [sensor_msgs/NavSatFix](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html)

## Output

### Camera image

Image data from camera. Used by the Perception.

- [sensor_msgs/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)

### Lidar filtered pointcloud

Pointcloud after preprocession. Used by the Perception.

- [sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)

### Radar pointcloud

Pointcloud frome radar. Used by the Perception.

- [sensor_msgs/PointCloud2](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)

### Corrected IMU date

Data corrected for yaw rate offset and standard deviation. Used by the Localization.

- [sensor_msgs/Imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)

### Vehicle pose

Position of the ego vehicle. Used by the Localization
- [geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)
- [geometry_msgs/PoseWithCovarianceStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)