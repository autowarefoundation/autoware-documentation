# Radar faraway dynamic objects detection with radar objects

## Overview

This diagram describes the pipeline for radar faraway dynamic object detection.

{{ diagram("image/faraway-object-detection.svg", "The pipeline for radar faraway dynamic object detection.") }}

The noise filter and the message converter run in Sensing (`radar_tracks_postprocessor.launch.xml` of `autoware_sensing_launch`). The velocity, range, and vector map filters run in Perception (`radar_filter.launch.xml` of `autoware_perception_launch`). The result goes to the `radar` channel of the multi-object tracker on `/perception/object_recognition/detection/radar/far_objects`.

## Reference implementation

### Crossing filter

- [`autoware_radar_tracks_noise_filter`](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_radar_tracks_noise_filter)

This package filters the noise objects that cross in front of the ego vehicle. These objects are most likely ghost objects. Radar measures the x-axis velocity (the Doppler velocity) well, but not the y-axis velocity. If the y-axis velocity of a `RadarTrack` is more than `velocity_y_threshold` (7.0 m/s in `autoware_sensing_launch`), the filter treats it as noise. The filter works on `radar_msgs/msg/RadarTracks`, before the message converter.

The package `autoware_radar_crossing_objects_noise_filter`, which did this on `DetectedObjects`, was removed from Autoware Universe.

### Velocity filter and range filter

- [`autoware_object_sorter`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_object_sorter)

Static objects include many noise like the objects reflected from ground.
In many cases for radars, dynamic objects can be detected stably.
For some radars, ghost objects sometimes occur for near objects.

`autoware_object_sorter` filters the objects by velocity and by range, for each class. It replaces the removed packages `autoware_object_velocity_splitter` and `autoware_object_range_splitter`. In `detected_object_sorter.param.yaml` of `autoware_perception_config`, the radar filter keeps these objects:

- `UNKNOWN`, `CAR`, `TRUCK`, `BUS`, `TRAILER`, `MOTORCYCLE`, `BICYCLE`, and `PEDESTRIAN`: velocity of at least 5.5 m/s, and distance from 80 m to 1000 m.
- `ANIMAL` and `HAZARD`: distance from 30 m to 100 m, with no velocity limit.

### Vector map filter

- [`object_lanelet_filter`](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_detected_object_validation/object-lanelet-filter.md)

In most cases, vehicles drive in the drivable area.
The `object_lanelet_filter` of `autoware_detected_object_validation` removes the objects that are outside the drivable area defined by the vector map. For radar objects, `radar_lanelet_filter.param.yaml` enables the polygon overlap filter and the elevation filter (from 0 m to 5 m above the lanelet surface).

Note that if you use `object_lanelet_filter` for radar faraway detection, you need to define drivable area in a vector map other than the area where autonomous car run.

### Radar object clustering

The package `autoware_radar_object_clustering` combined several radar detections of one object into one object. It was removed from Autoware Universe. The current pipeline sends the radar objects to the multi-object tracker without this clustering step.

## Note

### Parameter tuning

Detection performed only by Radar applies various strong noise processing.
Therefore, there is a trade-off that if you strengthen the noise processing, things that you originally wanted to detect will disappear, and if you weaken it, your self-driving system will be unable to start because the object will be in front of you all the time due to noise.
It is necessary to adjust parameters while paying attention to this trade-off.

### Limitation

- Elevated railway, vehicles for multi-level intersection

If you use 2D radars (The radar can detect in xy-axis 2D coordinate, but can not have z-axis detection) and driving area has elevated railway or vehicles for multi-level intersection, the radar process detects these and these have a bad influence to planning results.
In addition, for now, elevated railway is detected as vehicle because the radar process doesn't have label classification feature and it leads to unintended behavior.
