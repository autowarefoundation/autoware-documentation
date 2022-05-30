# Running Autoware without CUDA

Two following algorithms in Autoware Universe depend on a CUDA environment.

- 2D/3D object detection
- traffic light detection

However, it is still possible to run Autoware in the environment without CUDA.
The following subsections briefly explain how to run these algorithms in such an environment.

!!! warning

    It is highly recommended to prepare a CUDA environment to achieve better performance.

## Running 2D/3D object detection without CUDA

As for the object detection system in Autoware, the following three packages require the CUDA environment:

- `lidar_centerpoint`
- `lidar_apollo_instance_segmentation`
- `tensorrt_yolo`

Instead of these modules, you can use the `euclidean_cluster` module, which is a 3D object detection algorithm that does not require a CUDA environment (refer to [the readme](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/euclidean_cluster) for detail).

## Running traffic light detection without CUDA

As for the traffic light recognition system in Autoware, there are two package that require the CUDA environment:

- `traffic_light_ssd_fine_detector`
- `traffic_light_classifier`

To run traffic light detection without CUDA, set [`enable_fine_detection` in this file](https://github.com/autowarefoundation/autoware.universe/blob/9445f3a7acd645d12a64507c3d3bfa57e74a3634/launch/tier4_perception_launch/launch/traffic_light_recognition/traffic_light.launch.xml#L3) to `false` so that `traffic_light_ssd_fine_detector` be disabled and the traffic light detection solely be executed by `map_based_traffic_light_detector`.

To run traffic light classification without CUDA, set [`use_gpu` in this file](https://github.com/autowarefoundation/autoware.universe/blob/9445f3a7acd645d12a64507c3d3bfa57e74a3634/perception/traffic_light_classifier/launch/traffic_light_classifier.launch.xml#L7) to `false` so that `traffic_light_classifier` use classification algorithm that does not use CUDA environment or GPU.