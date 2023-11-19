# CAMERAs

## **TIER IV Automotive HDR Cameras**

![camera-tieriv.png](images/camera-tieriv.png)

[TIER IV's Automotive HDR cameras](https://sensor.tier4.jp/automotive-camera) which have ROS 2 driver and tested by TIER IV are listed below:

| Supported Products List          | MP  | FPS | Interface         | HDR           | LFM | Trigger <br> /Synchronization | Ingress <br> Protection | ROS 2 Driver | Autoware <br> Tested (Y/N) |
| -------------------------------- | --- | --- | ----------------- | ------------- | --- | ----------------------------- | ----------------------- | ------------ | -------------------------- |
| C1                               | 2.5 | 30  | GMSL2 <br> / USB3 | Y <br>(120dB) | Y   | Y                             | IP69K                   | Y            | Y                          |
| C2                               | 5.4 | 30  | GMSL2 <br> / USB3 | Y <br>(120dB) | Y   | Y                             | IP69K                   | Y            | Y                          |
| C3 <br> (to be released in 2024) | 8.3 | 30  | GMSL2 <br> / TBD  | Y <br>(120dB) | Y   | Y                             | IP69K                   | Y            | Y                          |

Link to ROS 2 driver:  
[https://github.com/tier4/ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera)

Link to product support site:  
[TIER IV Edge.Auto documentation](https://tier4.github.io/edge-auto-docs/index.html)

Link to product web site:  
[TIER IV Automotive Camera Solution](https://sensor.tier4.jp/automotive-camera)

## **FLIR Machine Vision Cameras**

![camera-flir.png](images/camera-flir.png)

FLIR Machine Vision cameras which has ROS 2 driver and tested by one or more community members are listed below:

| Supported Products List | MP           | FPS        | Interface | HDR | LFM | Trigger <br> /Synchronization | Ingress <br> Protection | ROS 2 Driver | Autoware Tested (Y/N) |
| ----------------------- | ------------ | ---------- | --------- | --- | --- | ----------------------------- | ----------------------- | ------------ | --------------------- |
| Blackfly S              | 2.0 <br> 5.0 | 22 <br> 95 | USB-GigE  | N/A | N/A | Y                             | N/A                     | Y            | -                     |
| Grasshopper3            | 2.3 <br> 5.0 | 26 <br> 90 | USB-GigE  | N/A | N/A | Y                             | N/A                     | Y            | -                     |

Link to ROS 2 driver:  
[https://github.com/berndpfrommer/flir_spinnaker_ros2](https://github.com/berndpfrommer/flir_spinnaker_ros2)

Link to company website:  
[https://www.flir.eu/iis/machine-vision/](https://www.flir.eu/iis/machine-vision/)

## **Lucid Vision Cameras**

![camera-lucid_vision.png](images/camera-lucid_vision.png)

Lucid Vision cameras which has ROS 2 driver and tested by one or more community members are listed below:

| Supported Products List | MP  | FPS  | Interface | HDR | LFM | Trigger <br> /Synchronization | Ingress <br> Protection | ROS 2 Driver | Autoware Tested (Y/N) |
| ----------------------- | --- | ---- | --------- | --- | --- | ----------------------------- | ----------------------- | ------------ | --------------------- |
| TRITON 054S             | 5.4 | 22   | GigE      | Y   | Y   | Y                             | up to IP67              | Y            | Y                     |
| TRITON 032S             | 3.2 | 35.4 | GigE      | N/A | N/A | Y                             | up to IP67              | Y            | Y                     |

Link to ROS 2 driver:  
[https://gitlab.com/leo-drive/Drivers/arena_camera](https://gitlab.com/leo-drive/Drivers/arena_camera)  
Link to company website:  
[https://thinklucid.com/triton-gige-machine-vision/](https://thinklucid.com/triton-gige-machine-vision/)

## **Allied Vision Cameras**

![camera-allied_vision.png](images/camera-allied_vision.png)

Allied Vision cameras which has ROS 2 driver and tested by one or more community members are listed below:

| Supported Products List | MP  | FPS  | Interface | HDR | LFM | Trigger <br> /Synchronization | Ingress <br> Protection | ROS 2 Driver | Autoware Tested (Y/N) |
| ----------------------- | --- | ---- | --------- | --- | --- | ----------------------------- | ----------------------- | ------------ | --------------------- |
| Mako G319               | 3.2 | 37.6 | GigE      | N/A | N/A | Y                             | N/A                     | Y            | -                     |

Link to ROS 2 driver:  
[https://github.com/neil-rti/avt_vimba_camera](https://github.com/neil-rti/avt_vimba_camera)

Link to company website:  
[https://www.alliedvision.com/en/products/camera-series/mako-g](https://www.alliedvision.com/en/products/camera-series/mako-g)

## **Neousys Technology Camera**

![images/camera-neousys.png](images/camera-neousys.png)

Neousys Technology cameras which has ROS 2 driver and tested by one or more community members are listed below:

| Supported Products List | MP  | FPS | Interface                                                                                                                                                                        | Sensor Format | Lens                                            | ROS 2 Driver | Autoware Tested (Y/N) |
| ----------------------- | --- | --- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- | ----------------------------------------------- | ------------ | --------------------- |
| AC-IMX390               | 2.0 | 30  | GMSL2 <br/> (over [PCIe-GL26 Grabber Card](https://www.neousys-tech.com/en/product/product-lines/in-vehicle-computing/vehicle-expansion-card/pcie-gl26-gmsl-frame-grabber-card)) | 1/2.7”        | 5-axis active adjustment with adhesive dispense | Y            | Y                     |

Link to ROS 2 driver:  
[https://github.com/ros-drivers/gscam](https://github.com/ros-drivers/gscam)

Link to company website:  
[https://www.neousys-tech.com/en/](https://www.neousys-tech.com/en/)
