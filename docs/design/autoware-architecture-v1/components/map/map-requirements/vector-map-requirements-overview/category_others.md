---
title: Others
---

The drawings on this page use left-hand traffic, as in Japan: vehicles drive on the left side of the road, and a right turn crosses the opposing lane. For right-hand traffic, swap left and right. In the map drawings, a colored area is a Lanelet, and a line with dots is a Linestring and its points.

---

## Requirements

### vm-07-01 Vector Map creation range

#### Detail of requirements <!-- omit in toc -->

Create all Lanelets within the sensor range of the vehicle. This includes the Lanelets on roads that the vehicle does not drive on, and the Lanelets that intersect with the Lanelet of the vehicle.

But if one of the following conditions is true, you must create the lanelets for a range of at least 10 meters:

- The vehicle drives on the priority lane through an intersection without traffic lights.
- The vehicle drives straight or turns left through an intersection with traffic lights.

Refer to [vm-03-04](category_intersection.md#vm-03-04-lanelet-creation-in-the-intersection) for more about intersection requirements.

The minimum of 10 meters is a requirement of this map specification. No Autoware parameter uses this value. The intersection module looks for vehicles only on the attention lanes in the map: the lanes that conflict with the path of the vehicle, and their preceding lanes up to `common.attention_area_length` (200 m by default). If the Lanelets on the crossing road are short, the attention area is also short.

##### Behavior of Autoware <!-- omit in toc -->

Autoware detects approaching vehicles and plans a route to avoid collisions.

##### Caution

Check the range of the sensors on your vehicle.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-07-01_1.svg", "Lanelets on all the lanes in the sensor range, also on the crossing road.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-07-01_2.svg", "Lanelets only on the route of the vehicle.") }}

---

### vm-07-02 Range of detecting pedestrians who enter the road

#### Detail of requirements <!-- omit in toc -->

Autoware has a feature that detects sudden entries from the roadside. It tracks pedestrians and cyclists outside the road boundaries. When they are likely to enter the road, the vehicle slows down to prevent collisions.

A Linestring of one of the following types tells Autoware that the objects on the other side of the line cannot jump out into the road:

- guard_rail
- wall
- fence

##### Supplemental information <!-- omit in toc -->

With the default parameters of `autoware_launch`, the run out module cuts the predicted paths of objects at _guard_rail_ and _fence_ Linestrings (`cut_predicted_paths.strict_linestring_types`). `map_based_prediction` cuts the predicted paths of crosswalk users at _fence_ Linestrings.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-05-04_1.svg", "A fence Linestring between the road and the area next to it.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [map_based_prediction - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_map_based_prediction/)
- [Run Out - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/)

---

### vm-07-03 Guardrails, guard pipes, fences

#### Detail of requirements <!-- omit in toc -->

When you create a Linestring for guardrails or guard pipes (_type: guard_rail_), put it where the part that sticks out most on the road side meets the ground, measured straight down.

Use the same position rules for Linestrings of fences (_type:fence_).

##### Supplemental information <!-- omit in toc -->

With the default parameters of `autoware_launch`, the obstacle velocity limiter uses _guard_rail_ Linestrings as static obstacles (`obstacles.static_map_tags`). It decreases the velocity of the vehicle when the vehicle drives close to a guardrail, for example in a curve. The run out module cuts the predicted paths of objects at _guard_rail_ and _fence_ Linestrings (see [vm-07-02](#vm-07-02-range-of-detecting-pedestrians-who-enter-the-road)). The drivable area expansion does not use these Linestrings by default. Its parameter `avoid_linestring.types` contains only _road_border_ and _curbstone_.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-07-03_1.svg", "The guard_rail Linestring is under the part of the guardrail that sticks out most toward the road.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-07-03_2.svg", "The guard_rail Linestring is at the posts of the guardrail, not under its outer edge.") }}

#### Related Autoware module

- [Obstacle Velocity Limiter - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module/)
- [Run Out - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/)
- [Drivable Area design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_drivable_area_design/)

---

### vm-07-04 Ellipsoidal height

#### Detail of requirements <!-- omit in toc -->

The height of a Point must be the ellipsoidal height (WGS84), in meters.

{{ diagram("../assets/vm-07-04_height_en.svg", "Ellipsoidal height and orthometric height of a point.") }}

#### Preferred vector map <!-- omit in toc -->

The height of a Point is the distance from the ellipsoid surface to the ground.

#### Incorrect vector map <!-- omit in toc -->

The height of a Point is the orthometric height, the distance from the geoid to the ground.

##### Supplemental information <!-- omit in toc -->

The `vertical_datum` field of `map_projector_info.yaml` tells Autoware the height reference of the map. Set it to `WGS84` for a map that meets this requirement. `autoware_gnss_poser` converts the GNSS height into this datum. For the file format, see the [map component design](../../index.md#projection-information).
