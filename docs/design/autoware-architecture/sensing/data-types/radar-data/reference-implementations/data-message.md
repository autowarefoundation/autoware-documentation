# Data message for radars

## Summary

To sum up, Autoware uses radar data type as below.

- [radar_msgs/msg/RadarScan.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarScan.msg) for radar pointcloud
- [radar_msgs/msg/RadarTracks.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) for radar objects.

## Radar data message for pointcloud

### Message definition

- [ros2/msg/RadarScan.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarScan.msg)

```sh
std_msgs/Header header
radar_msgs/RadarReturn[] returns
```

- [ros2/msg/RadarReturn.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarReturn.msg)

```sh
# All variables below are relative to the radar's frame of reference.
# This message is not meant to be used alone but as part of a stamped or array message.

float32 range                            # Distance (m) from the sensor to the detected return.
float32 azimuth                          # Angle (in radians) in the azimuth plane between the sensor and the detected return.
                                         #    Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.
float32 elevation                        # Angle (in radians) in the elevation plane between the sensor and the detected return.
                                         #    Negative angles are below the sensor. For 2D radar, this will be 0.
float32 doppler_velocity                 # The doppler speeds (m/s) of the return.
float32 amplitude                        # The amplitude of the of the return (dB)
```

## Radar data message for tracked objects

### Message definition

- [radar_msgs/msg/RadarTrack.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTrack.msg)

```sh
# Object classifications (Additional vendor-specific classifications are permitted starting from 32000 eg. Car)
uint16 NO_CLASSIFICATION=0
uint16 STATIC=1
uint16 DYNAMIC=2

unique_identifier_msgs/UUID uuid            # A unique ID of the object generated by the radar.
                                            # Note: The z component of these fields is ignored for 2D tracking.
geometry_msgs/Point position                # x, y, z coordinates of the centroid of the object being tracked.
geometry_msgs/Vector3 velocity              # The velocity of the object in each spatial dimension.
geometry_msgs/Vector3 acceleration          # The acceleration of the object in each spatial dimension.
geometry_msgs/Vector3 size                  # The object size as represented by the radar sensor eg. length, width, height OR the diameter of an ellipsoid in the x, y, z, dimensions
                                            # and is from the sensor frame's view.
uint16 classification                       # An optional classification of the object (see above)
float32[6] position_covariance              # Upper-triangle covariance about the x, y, z axes
float32[6] velocity_covariance              # Upper-triangle covariance about the x, y, z axes
float32[6] acceleration_covariance          # Upper-triangle covariance about the x, y, z axes
float32[6] size_covariance                  # Upper-triangle covariance about the x, y, z axes
```

### Message usage for RadarTracks

- Object classifications

In object classifications of [radar_msgs/msg/RadarTrack.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTrack.msg), additional classifications label can be used by the number starting from 32000.

To express for [Autoware label definition](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/ObjectClassification.idl), Autoware defines object classifications for `RadarTracks.msg` as below.

```sh
uint16 UNKNOWN = 32000;
uint16 CAR = 32001;
uint16 TRUCK = 32002;
uint16 BUS = 32003;
uint16 TRAILER = 32004;
uint16 MOTORCYCLE = 32005;
uint16 BICYCLE = 32006;
uint16 PEDESTRIAN = 32007;
```

For detail implementation, please see [radar_tracks_msgs_converter](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_radar_tracks_msgs_converter).

## Note

### Survey for radar message

Depending on the sensor manufacturer and its purpose, each sensor might exchange raw, post-processed data. This section introduces a survey about the previously developed messaging systems in the open-source community. Although there are many kinds of outputs, radar mainly adopt two types as outputs, pointcloud and objects. Related discussion for message definition in ros-perception are [PR #1](https://github.com/ros-perception/radar_msgs/pull/1), [PR #2](https://github.com/ros-perception/radar_msgs/pull/2), and [PR #3](https://github.com/ros-perception/radar_msgs/pull/3). Existing open source softwares for radar are summarized in these PR.
