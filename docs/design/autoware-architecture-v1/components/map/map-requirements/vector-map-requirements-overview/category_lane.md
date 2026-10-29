---
title: Lane
---

<!-- cspell:ignore centerlines -->

The drawings on this page use left-hand traffic, as in Japan: vehicles drive on the left side of the road. For right-hand traffic, swap left and right. In the map drawings, a colored area is a Lanelet, and a line with dots is a Linestring and its points.

---

## Requirements

### vm-01-01 Lanelet basics

#### Detail of requirements <!-- omit in toc -->

The road's Lanelets must comply with the following requirements.

- _subtype:road_
- _location:urban_, for public roads
- Align the Lanelet's direction with the direction in which the vehicle moves. (You can show the Lanelet direction as arrows in [Vector Map Builder](https://docs.web.auto/en/user-manuals/vector-map-builder/screen-layout#project-tab).)
- Set whether a lane change is permitted or not, according to [vm-01-02](#vm-01-02-allowance-for-lane-changes).
- Set the Linestring IDs for the Lanelet's left_bound and right_bound. See [vm-01-03](#vm-01-03-linestring-sharing).
- tag: _one_way=yes_. Autoware does not support _one_way=no_. For a narrow road on which the vehicle drives in both directions, see [vm-01-14](#vm-01-14-roads-with-no-centerline-2).
- Connect the Lanelet to the next and the previous Lanelets, except at the start or the end of the road. Two Lanelets are connected when the end points of the left and right bounds of the first Lanelet are the start points of the bounds of the next Lanelet.
- Position the points (x, y, z) of the Lanelet to align with the PCD Map, both laterally and in elevation. The height of a Point must be the ellipsoidal height (WGS84). Refer to [vm-07-04](category_others.md#vm-07-04-ellipsoidal-height).

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-01.svg", "A road Lanelet, its direction, and its left and right bounds.") }}

---

### vm-01-02 Allowance for lane changes

#### Detail of requirements <!-- omit in toc -->

Add a tag to the Lanelet's Linestring that permits or prohibits a lane change.

- Permit _lane_change=yes_
- Prohibit _lane_change=no_

Set the Linestring _subtype_ according to the type of line.

- _solid_
- _dashed_

!!! note

    Lanelet2 reads the _lane_change_ tag first. If a Linestring has no _lane_change_ tag, Lanelet2 permits a lane change only across a Linestring with _type:line_thin_ or _type:line_thick_ and _subtype:dashed_. The subtypes _dashed_solid_ and _solid_dashed_ permit a lane change in one direction only.

##### Referenced from Japan's Road Traffic Law <!-- omit in toc -->

- White dashed lines: lane changes and overtaking are permitted.
- White solid lines: lane changes and overtaking are permitted.
- Yellow solid lines: lane changes are not permitted.

{{ diagram("../assets/vm-01-02.svg", "The three types of lane lines and the tags of their Linestrings.") }}

#### Related Autoware module

- [Lane Change design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/)
- [Static Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/)
- [Dynamic Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/)
- [Out of lane design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/)

---

### vm-01-03 Linestring sharing

#### Detail of requirements <!-- omit in toc -->

When you create Lanelets that are physically next to each other, make them share the Linestring between them.

##### Behavior of Autoware <!-- omit in toc -->

Autoware finds the Lanelet next to the current Lanelet through the shared Linestring. If the two Lanelets share a Linestring, the following behaviors become possible:

- The vehicle moves out of its lane to avoid obstacles.
- The vehicle drives slightly out of its lane in a curve.
- The vehicle changes lanes.

{{ diagram("../assets/vm-01-03_1.svg", "A road with two lanes in each direction.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-03_2.svg", "Each lane line is one Linestring that the two Lanelets next to it share.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-03_3.svg", "Each Lanelet has its own Linestrings, so two Linestrings lie at each lane line.") }}

#### Related Autoware module

- [Lane Change design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/)
- [Static Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/)
- [Dynamic Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/)
- [Out of lane design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module/)

---

### vm-01-04 Sharing of the centerline of lanes for opposing traffic

#### Detail of requirements <!-- omit in toc -->

When the vehicle's Lanelet and the opposite Lanelet physically touch, the two Lanelets must share the Linestring of the road center line. For that purpose, the two Lanelets must have the same length.

##### Behavior of Autoware <!-- omit in toc -->

The vehicle can avoid an obstacle through the opposite lane. Autoware finds the opposite Lanelet through the shared Linestring. The static obstacle avoidance module uses the opposite lane when its parameter `use_lane_type` is `opposite_direction_lane`. This value is the default.

{{ diagram("../assets/vm-01-04_1.svg", "The vehicle moves into the opposite lane to pass a parked vehicle.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-04_2.svg", "The two Lanelets share the Linestring of the road center line and have the same length.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-04_3.svg", "The two directions have separate Linestrings, and the Lanelets do not have the same length.") }}

---

### vm-01-05 Lane geometry

#### Detail of requirements <!-- omit in toc -->

The geometry of the road Lanelet must comply with the following:

- The left and right Linestrings must follow the lane lines of the road.
- The end of a Lanelet must meet the start of the next Lanelet in a straight line.
- The outline must be smooth, not jagged or bumpy, except for L-shaped cranks.

{{ diagram("../assets/vm-01-05_1.svg", "A curved road with one lane in each direction.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-05_2.svg", "The bounds of the Lanelet follow the lane lines and are smooth.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-05_3.svg", "The end of the Lanelet is not straight, and its right bound is uneven and leaves the lane line.") }}

---

### vm-01-06 Line position (1)

#### Detail of requirements <!-- omit in toc -->

Put the Linestring of the road center line in the exact middle of the road marking.

{{ diagram("../assets/vm-01-06_1.svg", "A road with one lane in each direction and a dashed road center line.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-06_2.svg", "Close-up: the Linestring is in the middle of the marking.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-06_3.svg", "Close-up: the Linestring is at the edge of the marking.") }}

---

### vm-01-07 Line position (2)

#### Detail of requirements <!-- omit in toc -->

If the road has lines on its outer sides, put each Linestring in the middle of the line.

{{ diagram("../assets/vm-01-07_1.svg", "A road with wide edge lines on both sides.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-07_2.svg", "Each Linestring is in the middle of its edge line.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

---

### vm-01-08 Line position (3)

#### Detail of requirements <!-- omit in toc -->

If the road has no lines on its outer sides, put the Linestring 0.5 m inside the road edge.

{{ diagram("../assets/vm-01-08_1.svg", "A road with no lines on its outer sides.") }}

##### Caution

The width depends on the laws of your country.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-08_2.svg", "Each outer Linestring is 0.5 m inside the road edge.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

---

### vm-01-09 Speed limits

#### Detail of requirements <!-- omit in toc -->

In the following cases, add a speed limit (_tag:speed_limit_) in km/h to the Lanelet (_subtype:road_) on which the vehicle drives.

- A speed limit road sign exists.
- You want a lower speed limit, for example on narrow roads.

If a Lanelet has no _speed_limit_ tag, Lanelet2 uses a default speed limit from its _location_ and _subtype_. For _location:urban_ and _subtype:road_, the default is 50 km/h.

Autoware settings and behaviors control the following. They do not need tags:

- The maximum velocity of the vehicle
- Slower speeds where they are necessary, for example in curves and on downhill roads

{{ diagram("../assets/vm-01-09_1.svg", "A speed limit road sign.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-09_2.svg", "The Lanelet next to the sign has speed_limit 20.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

---

### vm-01-10 Centerline

#### Detail of requirements <!-- omit in toc -->

By default, Autoware drives along the midpoints between the left and right Linestrings of a Lanelet.

If the vehicle must drive to the left or right of this midpoint for a reason, add a centerline to the Lanelet. The centerline is a Linestring that the Lanelet refers to with the role _centerline_. Give the centerline a smooth shape that the vehicle can drive.

By default, the map loader of Autoware (parameter `use_waypoints`, `true` by default) stores the centerline from the map as the _waypoints_ of the Lanelet. Only the path planning of the ego vehicle uses these waypoints. Other modules, for example the prediction of other vehicles, use the midpoints between the bounds.

{{ diagram("../assets/vm-01-10_1.svg", "The vehicle moves to the right for a short distance, then back.") }}

##### Caution

The centerline of a Lanelet is a different concept from the road center line (the line that divides the lanes of opposite directions).

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-10_2.svg", "The centerline moves to the right and back with smooth diagonal parts.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-10_3.svg", "The centerline moves in sharp steps that the vehicle cannot drive.") }}

---

### vm-01-11 Centerline connection (1)

#### Detail of requirements <!-- omit in toc -->

When you add centerlines to several Lanelets one after the other, connect the centerlines.

{{ diagram("../assets/vm-01-11_1.svg", "The vehicle moves to the right and back two times on a long road.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-11_2.svg", "The two centerlines meet at the same point on the Lanelet border.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-11_3.svg", "The two centerlines do not meet at the Lanelet border.") }}

---

### vm-01-12 Centerline connection (2)

#### Detail of requirements <!-- omit in toc -->

If a Lanelet with a centerline connects to Lanelets without one, put the start and end points of the centerline in the middle of the Lanelet. Give the centerline a smooth shape that the vehicle can drive.

{{ diagram("../assets/vm-01-12_1.svg", "The vehicle moves to the right and back in the middle part of the road only.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-12_2.svg", "The centerline starts and ends in the middle of the Lanelet.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-12_3.svg", "The centerline does not start and end in the middle of the Lanelet.") }}

---

### vm-01-13 Roads with no centerline (1)

#### Detail of requirements <!-- omit in toc -->

If a road has no road center line but is wide enough for the vehicle and an oncoming vehicle to pass each other, put two Lanelets side by side, with the shared Linestring in the middle of the road.

{{ diagram("../assets/vm-01-13_1.svg", "A wide road without a road center line.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-13_2.svg", "Two Lanelets side by side share one Linestring in the middle of the road.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

---

### vm-01-14 Roads with no centerline (2)

#### Detail of requirements <!-- omit in toc -->

Apply this requirement if all the following conditions are satisfied:

- The road is a single lane without a road center line, and it is too narrow for the vehicle and an oncoming vehicle to pass each other.
- No vehicles other than the autonomous vehicle enter this road.
- The autonomous vehicle drives forth and back on this road.

Requirement for Vector Map creation:

- Put two Lanelets on top of each other. The two Lanelets use the same two Linestrings in opposite directions: the left bound of one Lanelet is the right bound of the other.

##### Supplementary information

- Whether this case applies depends on the local operation policies and the vehicle specifications. Decide it with the map requestor.
- The bidirectional traffic module of the behavior path planner finds such pairs of Lanelets. On these Lanelets, it moves the path of the vehicle to the left of the road center. If it detects an oncoming vehicle, it moves the vehicle to the left side, stops it, and waits until the oncoming vehicle has passed. It cannot handle cases in which the vehicles cannot pass each other this way.

{{ diagram("../assets/vm-01-14_1.svg", "The vehicle drives outward on a narrow road and later comes back on it.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-14_2.svg", "Two Lanelets in opposite directions lie on top of each other and use the same Linestrings.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-14_3.svg", "The road is split into a left part and a right part, with the Lanelets side by side.") }}

#### Related Autoware module

- [Bidirectional Traffic - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module/)

---

### vm-01-15 Road shoulder

#### Detail of requirements <!-- omit in toc -->

If the road has a shoulder, put a Lanelet for the road shoulder (_subtype:road_shoulder_) on it. You do not need to create this Lanelet inside intersections.

The road shoulder's Lanelet and the sidewalk share the Linestring between them (_type:road_border_).

A road shoulder Lanelet must not be next to another road shoulder Lanelet.

A road Lanelet must be next to the road shoulder Lanelet.

##### Behavior of Autoware <!-- omit in toc -->

- Autoware can start from the road shoulder and can also stop on the road shoulder at the goal.
- The Autoware parameter `margin_from_boundary` of the goal planner sets the margin between the vehicle and the edge of the road shoulder at the goal. You do not need to consider it when you create the Vector Map.
- Autoware does not park on the road shoulder Lanelet where the vehicle overlaps with any of the following:
  - A Polygon of a _no_parking_area_
  - A Polygon of a _no_stopping_area_
  - Areas near an intersection and in an intersection
  - A crosswalk

The Linestring at the boundary of the road shoulder does not need _tag:lane_change=yes_.

{{ diagram("../assets/vm-01-15_1.svg", "A curved road with a road shoulder and a sidewalk on each side.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-15_2.svg", "Road shoulder Lanelets, and road_border Linestrings between the road shoulders and the sidewalks.") }}

#### Incorrect vector map <!-- omit in toc -->

Do not create a road shoulder Lanelet for a road without a road shoulder.

{{ diagram("../assets/vm-01-15_3.svg", "A road shoulder Lanelet on a road that has a side strip, not a road shoulder.") }}

#### Related Autoware module

- [Static Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/)
- [Dynamic Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/)
- [Start Planner design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_start_planner_module/)
- [Goal Planner design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/)

---

### vm-01-16 Road shoulder Linestring sharing

#### Detail of requirements <!-- omit in toc -->

The Lanelets of the road shoulder and of the road next to it must share a Linestring.

{{ diagram("../assets/vm-01-15_1.svg", "A curved road with a road shoulder and a sidewalk on each side.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-16_2.svg", "Each road shoulder Lanelet shares the Linestring at the edge line with the road Lanelet next to it.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Static Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/)
- [Dynamic Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/)
- [Goal Planner design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/)

---

### vm-01-17 Side strip

#### Detail of requirements <!-- omit in toc -->

Put a Lanelet (_subtype:pedestrian_lane_) on the side strip. A side strip is the strip outside the edge line of a road that has no sidewalk. You do not need to create this Lanelet inside intersections.

The outer Linestring of the side strip's Lanelet must have _type:road_border_.

{{ diagram("../assets/vm-01-17_1.svg", "A curved road with a side strip on each side and no sidewalk.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-17_2.svg", "Pedestrian lane Lanelets on the side strips, with road_border Linestrings on their outer sides.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

---

### vm-01-18 Side strip Linestring sharing

#### Detail of requirements <!-- omit in toc -->

The Lanelet of the side strip and the road Lanelet next to it must share a Linestring.

{{ diagram("../assets/vm-01-17_1.svg", "A curved road with a side strip on each side and no sidewalk.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-18_2.svg", "Each side strip Lanelet shares the Linestring at the edge line with the road Lanelet next to it.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

---

### vm-01-19 Walkway

#### Detail of requirements <!-- omit in toc -->

Put a sidewalk Lanelet (_subtype:walkway_) only where the path of the vehicle crosses a sidewalk, for example at the entrance of a private area. Do not put one where the vehicle does not cross the sidewalk.

The Lanelet (_subtype:walkway_) must cover the part of the sidewalk that the vehicle crosses, plus 3 m before and 3 m after it.

##### Behavior of Autoware <!-- omit in toc -->

The walkway module stops the vehicle before a _walkway_ Lanelet on its path. Then the vehicle continues.

{{ diagram("../assets/vm-01-19_1.svg", "The vehicle crosses the sidewalk to enter a private area.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-01-19_2.svg", "A walkway Lanelet covers the crossing and 3 m on each side of it.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)
- [Walkway design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_walkway_module/)
