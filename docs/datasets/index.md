# Datasets

Autoware partners provide datasets for testing and development. These datasets are available for download here.

## Bus-ODD (Operational Design Domain) datasets

#### Leo Drive - ISUZU sensor data

This dataset contains data from the Isuzu bus used in the Bus ODD project.

The data contains data from following sensors:

- 1 x VLP16
- 2 x VLP32C
- 1 x Applanix POS LV 120 GNSS/INS
- 3 x Lucid Vision Triton 5.4MP cameras (left, right, front)
- Vehicle status report

It also contains /tf topic for static transformations between sensors.

##### Required message types

The GNSS data is available in `sensor_msgs/msg/NavSatFix` message type.

But also the Applanix raw messages are also included in `applanix_msgs/msg/NavigationPerformanceGsof50` and `applanix_msgs/msg/NavigationSolutionGsof49` message types.
In order to be able to play back these messages, you need to build and source the `applanix_msgs` package.

```bash
# Create a workspace and clone the repository
$ mkdir -p ~/applanix_ws/src && cd "$_"
$ git clone https://github.com/autowarefoundation/applanix.git
$ cd ..

# Build the workspace
$ colcon build --symlink-install --packages-select applanix_msgs

# Source the workspace
$ source ~/applanix_ws/install/setup.bash

# Now you can play back the messages
```

Also make sure to source Autoware Universe workspace too.

##### Download instructions

```bash
# Install awscli
$ sudo apt update && sudo apt install awscli -y

# This will download the entire dataset to the current directory
# (About 10.9GB of data)
$ aws s3 sync s3://autoware-files/collected_data/2022-08-22_leo_drive_isuzu_bags/ ./2022-08-22_leo_drive_isuzu_bags  --no-sign-request

# Optionally,
# If you instead want to download a single bag file, you can get a list of the available files with following:
$ aws s3 ls s3://autoware-files/collected_data/2022-08-22_leo_drive_isuzu_bags/ --no-sign-request
   PRE all-sensors-bag1_compressed/
   PRE all-sensors-bag2_compressed/
   PRE all-sensors-bag3_compressed/
   PRE all-sensors-bag4_compressed/
   PRE all-sensors-bag5_compressed/
   PRE all-sensors-bag6_compressed/
   PRE driving_20_kmh_2022_06_10-16_01_55_compressed/
   PRE driving_30_kmh_2022_06_10-15_47_42_compressed/

# Then you can download a single bag file with the following:
aws s3 sync s3://autoware-files/collected_data/2022-08-22_leo_drive_isuzu_bags/all-sensors-bag1_compressed/ ./all-sensors-bag1_compressed  --no-sign-request
```

#### Autocore.ai - lidar ROS2 bag file and pcap

This dataset contains pcap files and ros2 bag files from Ouster OS1-64 Lidar.
The pcap file and ros2 bag file is recorded in the same time with slight difference in duration.

[Click here to download](https://autoware-files.s3.us-west-2.amazonaws.com/collected_data/2022-04-14_autocore-lidar-bag-pcap/Lidar_Data_220414_bag_pcap.zip)

[Reference Issue](https://github.com/autowarefoundation/autoware.universe/issues/562#issuecomment-1102662448)
