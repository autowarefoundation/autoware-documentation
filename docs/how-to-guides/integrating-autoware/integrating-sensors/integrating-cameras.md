# Integrating cameras

There is no strict requirement for the camera interfaces to be used in Autoware 
as long as the RBG color images are published in `sensor_msgs/Image` topic format.
To handle realtime video capture, users who would like to integrate their own camera into Autoware are recommended to
utilize Video for Linux version 2 (V4L2) framework, which provides a standard interfaces that allows user-space applications to interact with video capture devices.

The following figure depicts the architecture overview to integrate cameras to Autowre:
![Architecture overview](images/camera_connection_architecture_overview.svg)
## Adapt to `ros2_v4l2_camera`
[`ros2_v4l2_camera`](https://github.com/tier4/ros2_v4l2_camera) can be used to capture images from a video device node, such as `/dev/video*`,
once the video device node is recognized as a V4L2 device.

- Specify `/dev/video*` as a parameter of the node to acquire images
- topic name is arbitrary. Autoware typically assume that the topic name is `image_raw` for the images without any processing from the camera output, and `image_rect_color` for the images rectified (i.e., lens distortion corrected) after capturing.
- Representative node parameters are as follows (parameters can be differ according to the device driver imprelemtaion):

| Parameter | Description | Example Value |
| --- | --- | --- |
| `video_device` | Path to the video device node | `/dev/video0` |
| `image_size` | Resolution of captured image | `[1920, 1280]` |
| `pixel_format` | Format of pixel data from camera (currently, only `YUYV` and `UYVY` are supported) | `"YUYV"` |
| `camera_frame_id` | Frame ID for ROS messages | `"camera"` |
| `camera_info_url` | Path to camera calibration file | `"file:///path/to/camera_info.yaml"` |


### [Optional] Compress image data using `accelerated_image_processor`

## Supported/Experienced Interfaces
As a hardware interface of the camera, 
the following interfaces are introduced in this document to show how to integrate the camera into Autoware.
- Gigabit Multimedia Serial Link2 (GMSL2)
- USB
<!-- interface comparison: https://medium.com/tier-iv-tech-blog/automotive-camera-interfaces-explained-7e7d8e3ba09e -->

## How to integrate GMSL2 cameras
### 1. Implement Device driver
- Build and load the device driver. The target camera is expected to be able to be accessed via `/dev/video*` 

### 2. Trigger FSYNC signals 
- Design trigger timing (in terms of synchronization with other sensors / SoF or EoF)
- configure the components on the tranmission path, including Deserializer, Serializer, (IPS, ) Imaging sensor, so that the FSYNC signals are transmitted properly
- If GPIO is connected to the deserializer, FSYNC sigmal emission can be done using `sensor_trigger` node

## How to integrate USB cameras
### Limitation/Requirement
- most USB cameras lack of capability for triggering; which decreases time synchronization accuracy
- To capture images from the camera, it is required to be recognized as a V4l2 device node, such as `/dev/video0`


