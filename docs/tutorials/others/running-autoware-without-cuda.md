# Running Autoware without CUDA

Although CUDA installation is recommended to achieve better performance for object detection and traffic light recognition in Autoware Universe, it is possible to run these algorithms without CUDA.
The following subsections briefly explain how to run each algorithm in such an environment.

To install Autoware without the NVIDIA libraries (CUDA, cuDNN, and TensorRT), run the Ansible playbook with `--skip-tags nvidia`. See the [source installation](../../installation/autoware/source-installation.md) page. The packages that need CUDA or TensorRT check for these libraries at build time. If CMake does not find them, the packages leave out their GPU code.

The launch files on this page are in the `autoware_launch` package of the [autoware_launch](https://github.com/autowarefoundation/autoware_launch) repository (version 0.53.0 in Autoware 2.0.0).

## Running 3D object detection without CUDA

The `lidar_detection_model` argument of [`component_perception.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/component_perception.launch.xml) selects the LiDAR object detector. Its value has the form `model_type/model_name`. If you leave out the `model_name` part, the default parameters of the model are used. The model types are:

- `centerpoint` (the default)
- `transfusion`
- `bevfusion`
- `pointpainting`
- `apollo`
- `clustering`

All model types except `clustering` run a neural network with TensorRT and need CUDA. With `clustering`, Autoware starts no neural network detector. It uses only the rule-based detector, which is built on the [`euclidean_cluster`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_euclidean_cluster) package and runs on the CPU.

The top-level launch files, for example `autoware.launch.xml` and `logging_simulator.launch.xml`, do not pass `lidar_detection_model` to the component launch file. To select `clustering`, change the default value in `autoware_launch/launch/components/component_perception.launch.xml`:

```xml
<arg name="lidar_detection_model" default="clustering" description="If the model name is not declared, the default model in the detection composition will be used"/>
```

Keep the other GPU options of the same file at their default value `false`:

- `use_cuda_ground_segmentation`: the CUDA backend of ground segmentation
- `use_semantic_segmentation_ptv3`: the PTv3 neural network for ground segmentation
- `cuda_pointcloud_preprocessing`: the CUDA backend of the common pointcloud downsampling

If you did not build the workspace with `--symlink-install`, rebuild `autoware_launch` after you change a launch file.

## Running traffic light detection without CUDA

Traffic light recognition has two steps: detection finds the traffic lights in the camera image, and classification finds their color and arrows. The launch file [`component_traffic_light.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/component_traffic_light.launch.xml) starts both steps.

To run traffic light detection without CUDA, set the `traffic_light_recognition/use_high_accuracy_detection` argument of `autoware.launch.xml` to `false`. Then the YOLOX-based detectors (`autoware_traffic_light_fine_detector` or the whole-image detector), which need CUDA, do not start. Instead, the [`autoware_traffic_light_map_based_detector`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_map_based_detector) package calculates where the traffic lights appear in the image from the vector map. The default value of this argument is `true` in `autoware.launch.xml` and `false` in `e2e_simulator.launch.xml`. For other launch files, change the default value in `autoware.launch.xml`:

```xml
<arg name="traffic_light_recognition/use_high_accuracy_detection" default="false" description="enable to use high accuracy detection for traffic light recognition"/>
```

To run traffic light classification without CUDA, set the classifier type to `0` (HSV filter) in `component_traffic_light.launch.xml`. There is one argument for car traffic lights and one for pedestrian traffic lights:

```xml
<arg name="traffic_light_recognition/classification/car/classifier_type" default="0" description="0=HSVFilter, 1=CNN, 2=LAMPClassifier"/>
<arg name="traffic_light_recognition/classification/pedestrian/classifier_type" default="0" description="0=HSVFilter, 1=CNN, 2=LampRecognizer"/>
```

The default value `1` selects a CNN classifier, and `2` selects a lamp recognizer. Both use TensorRT and need CUDA. The HSV filter classifies the color of the light in the HSV color space and runs on the CPU. For details, see the [`autoware_traffic_light_classifier`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_classifier) README.
