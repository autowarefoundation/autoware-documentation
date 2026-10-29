# Device driver for radars

## Interface for radar devices

The radar driver converts the communication data from radars to ROS 2 topics.
The following communication types are considered:

- CAN
- CAN-FD
- Ethernet

{{ diagram("image/radar-communication.svg", "The radar device and the ROS driver exchange data in both directions.") }}

## Software Interface

Autoware supports `ros-perception/radar_msgs/msg/RadarScan.msg` and `ros-perception/radar_msgs/msg/RadarTracks.msg` for radar drivers.

{{ diagram("image/radar-driver.svg", "Inside the radar, target detection makes the scan points. Object tracking makes the tracks.") }}

The ROS driver receives the following data from the radar device:

- Scan (pointcloud)
- Tracked objects
- Diagnostics results

The ROS driver sends the following data to the radar device:

- Time synchronization
- Ego vehicle status, which the radar often needs to calculate the tracked objects

## Radar driver examples

- [ARS408 driver](https://github.com/tier4/ars408_driver)
- [Nebula](https://github.com/tier4/nebula): the `nebula_continental` package supports the Continental ARS548 and SRR520 radars. It publishes `RadarScan` and `RadarTracks`. For the ARS548, it also publishes `autoware_sensing_msgs/msg/RadarObjects`.
