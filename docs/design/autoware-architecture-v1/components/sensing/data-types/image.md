# Image pre-processing design

## Overview

A camera driver publishes the camera images and the camera information. The sensing component of Autoware has no image preprocessing node in the default launch files. The perception and localization modules read the image topics of the camera driver directly.

The camera driver is not part of Autoware. The driver, or another node outside Autoware, must publish the topics that the next sections describe. In the AWSIM and CARLA sensor kits, the simulator publishes these topics.

## Message types

| Data             | Message type                                                                                                                    | Description                                                          |
| :--------------- | :------------------------------------------------------------------------------------------------------------------------------ | :------------------------------------------------------------------- |
| Image            | [`sensor_msgs/msg/Image`](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg)                     | An uncompressed image, for example `image_raw` or `image_rect_color` |
| Compressed image | [`sensor_msgs/msg/CompressedImage`](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CompressedImage.msg) | A compressed image, for example `image_raw/compressed`               |
| Camera info      | [`sensor_msgs/msg/CameraInfo`](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CameraInfo.msg)           | The intrinsic parameters and the distortion of the camera            |

## Topics that Autoware reads

All the camera topics are in the `/sensing/camera/<camera name>/` namespace.

| User                                    | Topics                                                         | Notes                                                                                              |
| :-------------------------------------- | :------------------------------------------------------------- | :------------------------------------------------------------------------------------------------- |
| Camera-based object detection           | `/sensing/camera/camera<N>/image_rect_color` and `camera_info` | The image must be rectified (the lens distortion is removed). Autoware does not rectify the image. |
| Traffic light recognition               | `/sensing/camera/<camera name>/image_raw` and `camera_info`    | The launch files use `camera6` and `camera7` as the representative cameras.                        |
| YabLoc and AR tag localizers (optional) | `/sensing/camera/traffic_light/image_raw`                      | Only when `pose_source` includes `yabloc` or `artag`                                               |

The `camera.launch.xml` file of `sample_sensor_kit_launch` shows an example. It relays the compressed image and the camera information of the traffic light camera to `/sensing/camera/traffic_light/image_raw/compressed` and `/sensing/camera/traffic_light/camera_info`. The `sensing.launch.xml` file of this sensor kit does not include `camera.launch.xml` by default.

## Decompression

Some camera drivers publish only compressed images. The traffic light recognition launch files load [`autoware_image_transport_decompressor`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_image_transport_decompressor) when `enable_image_decompressor` is `true` (default). This node reads `<image topic>/compressed` and publishes the decompressed image on `<image topic>`.

The node supports 8-bit RGB and BGR cameras, with the `encoding` parameter set to `rgb8` or `bgr8`. For other encodings, the published image does not match the camera data. The package README describes each case.

## Other image packages

The `sensing` directory of Autoware Universe has two more image packages. The launch files of `autoware_launch` do not launch them.

- [`autoware_image_diagnostics`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/sensing/autoware_image_diagnostics) divides each raw image into blocks. It finds blocks with shadow clipping, highlight clipping, blockage, or low visibility, and publishes the image state on `/diagnostics`.
- [`autoware_calibration_status_classifier`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_calibration_status_classifier) projects the LiDAR points onto the undistorted camera image. A neural network then classifies the LiDAR-camera calibration as correct or not.
