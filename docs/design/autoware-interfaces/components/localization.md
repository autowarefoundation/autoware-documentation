# Localization

![Node diagram](images/Localization-Bus-ODD-Architecture.drawio.svg)

## Inputs

### PointCloud Map

Environment map created with point cloud, Published by the Map Server.

- sensor_msgs::msg::PointCloud2


### Initial Pose

Start pose of ego, Published by the User Interface or GNSS.

- geometry_msgs::msg::PoseStamped

### 3D-LiDAR Scanning

LiDAR scanning for NDT matching. Published by the LiDAR sensor.

- sensor_msgs::msg::PointCloud2

### GNSS 

Current Geographic coordinate of the ego. Published by GNSS sensor.

- sensor_msgs::msg::NavSatFix

### IMU

Current orientation, angular velocity and linear acceleration of ego. Published by IMU sensor.

- sensor_msgs::msg::Imu

### Vehicle Velocity Status

Current velocity of the ego vehicle. Published by the Vehicle Interface.

- autoware_auto_vehicle_msgs/msg/VelocityReport
   - std_msgs::msg::Header header;
   - float longitudinal_velocity;
   - float lateral_velocity;
   - float heading_rate;

## Outputs

### Vehicle pose 

Current position and orientation of ego.

- geometry_msgs::msg::PoseWithCovarianceStamped

### Vehicle twist

Current line velocity and angular velocity of ego.

- geometry_msgs::msg::TwistWithCovarianceStamped

### Vehicle acceleration

Current acceleration of ego.

- geometry_msgs::msg::AccelWithCovarianceStamped

### Vehicle kinematic state

Current pose and twist of ego.

- geometry_msgs::msg::Odometry

### Localization Accuracy

Diagnostics information that indicates if the localization module works properly.

TBD.