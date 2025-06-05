# Integrating cameras

## Camera Requirements
Autoware has no strict requirements for camera interfaces, as long as BGR color images are published in `sensor_msgs/Image` topic format.

To handle real-time video capture, users who wish to integrate their own camera into Autoware are encouraged to use the Video for Linux Version 2 (V4L2) framework.
V4L2 provides a standard interface that enables user-space applications to interact with video capture devices.
Many common camera hardware interfaces are exposed as V4L2 devices.
For example, USB cameras are widely supported via the USB Video Class (UVC) on many operating systems.
Similarly, cameras using Gigabit Multimedia Serial Link 2 (GMSL2) can be accessed through V4L2 once their dedicated drivers expose them as such.

The following figure depicts the architecture overview to integrate cameras to Autoware:
![Architecture overview](images/camera_connection_architecture_overview.svg)

For users who prefer cameras connected via Gigabit Ethernet (GigE), it may be efficient to decode packets directly instead of relying on the V4L2 framework.
This scenario will be addressed in a separate section of this document.

## Integration overview
This section describes the procedure to integrate new cameras into Autoware.

### Select interface to connect the cameras
There are several interfaces available for camera connections in Autoware. 
Based on performance characteristics, ease of integration, and operational experience, this document focuses on three major interfaces: GMSL2, USB, and GigE. 
The following table provides a qualitative comparison of the advantages and disadvantages of each interface.

|                       | GMSL2         | USB                      | GigE                     |
|-----------------------|---------------|--------------------------|--------------------------|
| Shutter trigger       | **Supported** | Requires extra equipment | Requires extra equipment |
| Plug and play         | No            | **Yes**                  | Product dependent        |
| Data transfer latency | **Ultra-Low** | Low                      | Low                      |
| Maximum bandwidth     | **6 Gbps**    | 5 Gbps@USB3.0            | 1 Gbps                   |

USB provides the most straightforward integration due to its native plug-and-play capabilities.
GigE is optimal for applications requiring long-distance transmission (up to 100 m).
GMSL2 is the preferred choice for automotive applications, offering ultra-low latency, high bandwidth, and functional safety certification options.

For automotive projects, utilizing GMSL2 as the interface for camera devices is recommended.

### Implementation requirements
Each interface type requires different components to be implemented or configured:

|                      | GMSL2                                                                                                          | USB3                                                                          | GigE                                                                                    |
|----------------------|----------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|
| Device driver        | Requires implementation of dedicated drivers for specific camera and ECU combinations to expose as V4L2 device | Native UVC driver provides V4L2 support for most cameras/ECUs                 | Not required (V4L2 protocol not applicable)                                             |
| ROS driver           | <ul> <li>`ros2_v4l2_camera`</li> <li>`gscam`</li> <ul>                                                         | <ul> <li>`ros2_v4l2_camera`</li> <li>`uvc_camera`</li> <li>`gscam`</li> </ul> | <ul> <li>`gscam` (if data is transferred via RTP)</li> <li>Dedicated driver </li> </ul> |


### Calibration
Calibration may be required to calculate the camera intrinsic parameters for some use cases, such as 3D to 2D projection (performed during traffic light recognition) and rectification.
For the calibration procedure, please refer to the documentation [here](/autoware-documentation/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-model/calibrating-sensors/intrinsic-camera-calibration/) for more details.

### Time synchronization
Some cameras support the shutter triggering feature, which enables precise time synchronization between multiple cameras and other sensors/ECUs.
The following table summarizes the effect of the time synchronization presence against major functions that use camera data in Autoware.
Although time synchronization is recommended for achieving the best performance across varying speed conditions, it could be omitted in cases where strict control of capture timing is not crucial.
<!-- See [here]() for more detail regarding time synchronization. -->

|                          | Image-based object detection/segmentation | Traffic light recognition                                      | LiDAR-camera fusion                                           |
|--------------------------|:------------------------------------------|:---------------------------------------------------------------|:--------------------------------------------------------------|
| No time synchronization  | Works fine                                | Works under low speed conditions                               | Works under low speed conditions                              |
| Time synchronization     | Works fine                                | Works across various speed conditions with the best accuracy   | Works across various speed conditions with the best accuracy  |

## Adapt to `ros2_v4l2_camera`
[`ros2_v4l2_camera`](https://github.com/tier4/ros2_v4l2_camera) can be used to capture images from a video device node, such as `/dev/video*`,
once the video device node is recognized as a V4L2 device.

- Specify `/dev/video*` as a parameter of the node to acquire images
- Adjust the `image_size` and `pixel_format` parameters to match the resolution and format of the camera
- The topic name is arbitrary. Autoware typically assumes that the topic name is `image_raw` for images without any processing from the camera output, and `image_rect_color` for images that have been rectified (i.e., lens distortion corrected) after capture.
- Representative node parameters are as follows (parameters can differ depending on the device driver implementation):

| Parameter         | Description                                                                        | Example Value                        |
|-------------------|------------------------------------------------------------------------------------|--------------------------------------|
| `video_device`    | Path to the video device node                                                      | `/dev/video0`                        |
| `image_size`      | Resolution of captured image                                                       | `[1920, 1280]`                       |
| `pixel_format`    | Format of pixel data from camera (currently, only `YUYV` and `UYVY` are supported) | `"YUYV"`                             |
| `camera_frame_id` | Frame ID for ROS messages                                                          | `"camera"`                           |
| `camera_info_url` | Path to camera calibration file                                                    | `"file:///path/to/camera_info.yaml"` |


### [Optional] Compress image data using `accelerated_image_processor`
For logging purposes, use of compressed images can drastically reduce data storage requirements and network load.
Typically, stored images will come from the `sensor_msgs/CompressedImage`topic while all perception modules of Autoware expect `sensor_msgs/Image`.

[`accelerated_image_processor`](https://github.com/tier4/accelerated_image_processor) provides accelerated compression
that offloads processing onto GPU or dedicated hardware, freeing CPU resources.
The node takes `sensor_msgs/Image` as an input and publishes `sensor_msgs/CompressedImage` as an output.
The key parameters are as follows:

| Parameter      | Description  | Example Value |
|----------------|--------------|---------------|
| `jpeg_quality` | JPEG quality | `80`          |

## How to integrate GMSL2 cameras
### 1. Implement device driver
<!-- - Build and load the device driver. The target camera is expected to be able to be accessed via `/dev/video*`  -->
Device drivers are responsible for exposing the camera as a V4L2 device. The main driver creates V4L2 device nodes, such as `/dev/video*`, and handles the structures defined by V4L2 framework to communicate V4L2. 
If the target camera adopts a serializer/deserializer, which allows transmission over longer cable lengths, the sub drivers to handle these components are also required.
cameras that use GMSL2 require an ECU or interface card with GMSL2 ports, and drivers for both the camera and ECU are usually specific to the device combination.

### 2. Prepare device tree
Where the device driver provides the controls for hardware devices, the device tree describes the hardware.
The device tree provides the static information of the devices and their connections, which the Linux kernel uses to discover/configure the devices.

### 3. Trigger FSYNC signals
FSYNC is the frame synchronization signal, which is usually triggered using external GPIO. 
The GMSL2 interface supports transmission of the FSYNC signal over the same cable as data transmission, via the FSYNC pins on the serializer/deserializer.
Synchronization implementation requires the following processes:
- Design trigger timing (in terms of synchronization with other sensors, and use of SoF or EoF)
  - SoF: Start of Frame, where the trigger is aligned with the start of image frame capture
  - EoF: End of Frame, where the trigger is aligned with the end of image frame capture
- Configure the components on the transmission path so that the FSYNC signals are transmitted properly. For GMSL2, these include the deserializer, serializer, and imaging sensor. 
- If GPIO is connected to the deserializer, the FSYNC signal emission can be done using the [`sensor_trigger`](https://github.com/tier4/sensor_trigger) node. This node provides precise timing control for FSYNC signal.

## How to integrate USB cameras
For USB cameras, the camera should be recognized as an V4L2 device just after connection thanks to Linux's UVC driver. Hence, users leverage a ROS node such as `ros2_v4l2_camera` to publish images as ROS topics.
If the USB camera supports external triggering, a GPIO trigger can be utilized with a separate trigger harness and configured in a similar way to a GMSL2 interface using the [`sensor_trigger`](https://github.com/tier4/sensor_trigger) node.

### Tips/notes
- While some USB cameras support external triggering, there is less control of shutter timing and timestamping. Whether triggering is used or not, USB cameras do not have consistent latency, which decreases time synchronization accuracy
- In some cases, the connected USB camera can be recognized as `/dev/media*`, which is associated with media controller devices and sometimes causes failure to capture frames. 
This misrecognition commonly occurs when using incompatible cables/ports; changing to cables and/or USB ports compatible with later generations (such as USB 3.0) may solve the problem.

## How to integrate GigE cameras
GigE cameras usually control streaming on the device side. Once images are streamed over the network with proper configuration, users need to decode the packets and reconstruct images on the receiver side using a ROS node such as `gscam` or a dedicated decoder node. 

### Tips/notes
In addition to the ROS node configuration, some cameras require proper network configuration, including destination IP address, netmask, maximum transmission unit (MTU), and other network parameters.
