# Integrating cameras

## Policy
There is no strict requirement for the camera interfaces to be used in Autoware 
as long as the BGR color images are published in `sensor_msgs/Image` topic format.

To handle realtime video capture, users who would like to integrate their own camera into Autoware are recommended to
utilize Video for Linux version 2 (V4L2) framework, which provides a standard interfaces that allows user-space applications to interact with video capture devices.
For instance, many cameras connected using USB are exposed as V4L2 devices by USB Video Class (UVC) drivers on which many OS supports respectively. Cameras connected using Gigabit Multimedis Serial Link2 (GMSL2)
can also be interacted with the same manner once their dedicated device drivers expose them as V4L2 devices.

The following figure depicts the architecture overview to integrate cameras to Autowre:
![Architecture overview](images/camera_connection_architecture_overview.svg)

For users who prefer cameras connected via Ethernet, it may be efficient to decode packets directly instead of relying on the V4L2 framework.
This scenario will be addressed in a separate section of this document.

## Adapt to `ros2_v4l2_camera`
[`ros2_v4l2_camera`](https://github.com/tier4/ros2_v4l2_camera) can be used to capture images from a video device node, such as `/dev/video*`,
once the video device node is recognized as a V4L2 device.

- Specify `/dev/video*` as a parameter of the node to acquire images
- Adjust `image_size` and `pixel_format` parameters to match the resolution and format of the camera
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
For logging perpurse, Compressed images, such as `sensor_msgs/CompressedImage`, are preferable in terms of reducing storage footprint,
while all perception modules of Autoware expect `sensor_msgs/Image`.

[`accelerated_image_processor`](https://github.com/tier4/accelerated_image_processor) provides accelerated compression processes.
The node takes `sensor_msgs/Image` as an input and publishes `sensor_msgs/CompressedImage` as an output.
The key parameters are as follows:

| Parameter | Description | Example Value |
| --- | --- | --- |
| `jpeg_quality` | JPEG quality | `80` |


## Supported/Experienced Interfaces
As a hardware interface of the camera, 
the following interfaces are introduced in this document to show how to integrate the camera into Autoware.

- Gigabit Multimedia Serial Link2 (GMSL2)
- USB
<!-- interface comparison: https://medium.com/tier-iv-tech-blog/automotive-camera-interfaces-explained-7e7d8e3ba09e -->

## How to integrate GMSL2 cameras
### 1. Implement Device driver
<!-- - Build and load the device driver. The target camera is expected to be able to be accessed via `/dev/video*`  -->
Device drivers are responsible for exposing the camera as a V4L2 device. The main driver creates V4L2 device nodes, such as `/dev/video*`, and handle structured defined by V4L2 framework to communicate V4L2. If the target camera adopts Serializer/Deserializer technorogy, which allows transmission over longer runs, the sub drivers to handle those components are also required.

### 2. Prepare device tree
Whereas the device tree provides the controls for hardware devices, device tree describes hardware.
The device tree provides the static information of the devcies and their connections, which the kernel uses to discover/configure the devices.

### 3. Trigger FSYNC signals
- Design trigger timing (in terms of synchronization with other sensors / SoF or EoF)
- configure the components on the tranmission path, including Deserializer, Serializer, (IPS, ) Imaging sensor, so that the FSYNC signals are transmitted properly
- If GPIO is connected to the deserializer, FSYNC sigmal emission can be done using `sensor_trigger` node

## How to integrate USB cameras
### Limitation/Requirement
- most USB cameras lack of capability for triggering; which decreases time synchronization accuracy
- To capture images from the camera, it is required to be recognized as a V4l2 device node, such as `/dev/video0`


