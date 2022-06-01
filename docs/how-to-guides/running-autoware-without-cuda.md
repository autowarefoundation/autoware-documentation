# Running Autoware without CUDA

Although CUDA installation is recommended to achieve better performance for object detection and traffic light recognition in Autoware Universe, it is possible to run these algorithms without CUDA. 
The following subsections briefly explain how to run each algorithm in such an environment.

## Running 2D/3D object detection without CUDA

Autoware Universe's object detection can be run using one of five possible configurations:

- `lidar_centerpoint`
- `lidar_apollo_instance_segmentation`
- `lidar-apollo` + `tensorrt_yolo`
- `lidar-centerpoint` + `tensorrt_yolo`
- `euclidean_cluster`

Of these five configurations, only the last one (`euclidean_cluster`) can be run without CUDA. For more details, refer to the [`euclidean_cluster` module's README file](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/euclidean_cluster).

## Running traffic light detection without CUDA

For traffic light recognition (both detection and classification), there are two modules that require CUDA:

- `traffic_light_ssd_fine_detector`
- `traffic_light_classifier`

To run traffic light detection without CUDA, set [`enable_fine_detection` to `false` in the traffic light launch file](https://github.com/autowarefoundation/autoware.universe/blob/9445f3a7acd645d12a64507c3d3bfa57e74a3634/launch/tier4_perception_launch/launch/traffic_light_recognition/traffic_light.launch.xml#L3). Doing so disables the `traffic_light_ssd_fine_detector` such that traffic light detection is handled by the `map_based_traffic_light_detector` module instead.

To run traffic light classification without CUDA, set [`use_gpu` to `false` in the traffic light classifier launch file](https://github.com/autowarefoundation/autoware.universe/blob/9445f3a7acd645d12a64507c3d3bfa57e74a3634/perception/traffic_light_classifier/launch/traffic_light_classifier.launch.xml#L7). Doing so will force the `traffic_light_classifier` to use a different classification algorithm that does not require CUDA or a GPU.