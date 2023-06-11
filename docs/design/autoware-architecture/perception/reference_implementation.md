# Perception component Reference Implementation Design Doc

autowareを起動すると、default parameterが読み込まれ、Reference Implementationが起動される。ここでは、defaultで起動するReference Implementationがどのようにdesignされているか記述する。

_Note that some implementation does not adhere to the high-level architecture design and require updating._

For more details, please refer to the design documents in each package.

## Architecture

This diagram describes the architecture of the reference implementation.

![overall-perception-architecture](image/reference-implementation-perception-diagram.drawio.svg)

- **Detection**: Detects the pose and velocity of dynamic objects such as vehicles and pedestrians.
  - **Detector**: Triggers object detection processing frame by frame.
  - **Interpolator**: Maintains stable object detection. Even if the output from Detector suddenly becomes unavailable, Interpolator uses the output from the Tracking module to maintain the detection results without missing any objects.
- **Tracking**: Associates detected results across multiple frames.
- **Prediction**: Predicts trajectories of dynamic objects.

The following diagram describes the reference implementation of the Perception component. By adding new modules or extending the functionalities, various ODDs can be supported.

![reference-implementation](image/perception-diagram.png)

### Internal interface in the perception component

- **Obstacle Segmentation to Object Recognition**
  - Point Cloud: A Point Cloud observed in the current frame, where the ground and outliers are removed.
- **Obstacle Segmentation to Occupancy Grid Map**
  - Ground filtered Point Cloud: A Point Cloud observed in the current frame, where the ground is removed.
- **Occupancy Grid Map to Obstacle Segmentation**
  - Occupancy Grid Map: This is used for filtering outlier.

### Important Parameters

| Package           | Parameter               | Type   | Description                                                                                                                                                                                                                                            |
| ----------------- | ----------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| perception launch | `mode`                  | string | detectionのmode. options: `camera_lidar_radar_fusion`, `camera_lidar_fusion`, `lidar_radar_fusion`, `lidar` or `radar` <https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_perception_launch/launch/perception.launch.xml> |
| perception launch | `lidar_detection_model` | string | lidar detection modelのmodel. options: `centerpoint`, `apollo`, `pointpainting`, `clustering`                                                                                                                                                          |
| perception launch | `use_object_filter`     | string | Whether to use filters for detected objects. They can filter objects bu using either vector maps or relative positions.                                                                                                                                |
