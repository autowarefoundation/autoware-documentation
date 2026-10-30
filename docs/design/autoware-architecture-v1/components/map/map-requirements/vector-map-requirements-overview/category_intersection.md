---
title: Intersection
---

The drawings on this page use left-hand traffic, as in Japan: vehicles drive on the left side of the road, and a right turn crosses the opposing lane. For right-hand traffic, swap left and right.

---

## Requirements

### vm-03-01 Intersection criteria

#### Detail of requirements <!-- omit in toc -->

Essential criteria for the construction of an intersection:

- Encircle the drivable area at the intersection with a Polygon (_type:intersection_area_).
- Add _turn_direction_ to all Lanelets in the intersection.
- Ensure that all lanelets in the intersection are tagged:
  - _key:intersection_area_
  - _value: Polygon's ID_
- Attach _right_of_way_ to the necessary Lanelets.
- Also, it is necessary to appropriately create traffic lights, crosswalks, and stop lines.

For detailed information, refer to the respective requirements on this page.

##### Autoware modules <!-- omit in toc -->

- The intersection module uses _turn_direction_ and _right_of_way_. It plans the velocity to avoid collisions with other vehicles, and it takes the traffic lights into account. The blind spot module uses _turn_direction_ to check for bicycles and pedestrians on the side of a turning vehicle.
- The static obstacle avoidance module uses the _intersection_area_ tag. It can plan a path that leaves the lane inside the intersection area (parameter `use_intersection_areas`, `true` by default). The lane change module also reads the _intersection_area_ tag. The intersection module can use the intersection area to detect vehicles outside the lanelets (parameter `use_intersection_area`, `false` by default).

#### Preferred vector map <!-- omit in toc -->

None in particular.

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)
- [Blind Spot design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/)
- [Static Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/)
- [Lane Change - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/)

---

### vm-03-02 Lanelet's turn direction and virtual linestring

#### Detail of requirements <!-- omit in toc -->

Add the following tag to the Lanelets in the intersection:

- turn_direction : straight
- turn_direction : left
- turn_direction : right

Also, if the left or right Linestrings of Lanelets at the intersection lack road paintings, designate these as _type:virtual_.

##### Behavior of Autoware <!-- omit in toc -->

Autoware turns on the turn signal (blinker) before a Lanelet with _turn_direction_ left or right. By default, the turn signal starts when the distance to the start of that Lanelet is less than 30 m plus the distance that the vehicle drives in 3 s at its current speed. The parameters are `turn_signal_intersection_search_distance` and `turn_signal_search_time` of the behavior path planner. To change the distance for one Lanelet, add the following tag to it. This value then replaces the whole default distance.

- key: _turn_signal_distance_
- value: numerical value (m)

{{ diagram("../assets/vm-03-02_1.svg", "A T-junction. A second road joins the main road from the south.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-02_2.svg", "The lanelets in the junction have turn_direction. The unpainted bound of the left turn is a virtual linestring.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)
- [Blind Spot design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/)
- [Turn signal design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_turn_signal_design/)

---

### vm-03-03 Lanelet width in the intersection

#### Detail of requirements <!-- omit in toc -->

Lanelets in the intersection should have a consistent width. Additionally, draw Linestrings with smooth curves.

The shape of this curve must be determined by the Vector Map creator.

{{ diagram("../assets/vm-03-03_1.svg", "The ego vehicle turns right at an intersection with traffic lights.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-03_2.svg", "Correct: the lanelet has a constant width and smooth linestrings.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-03_3.svg", "Incorrect: the width changes, and the linestrings are not smooth.") }}

---

### vm-03-04 Lanelet creation in the intersection

#### Detail of requirements <!-- omit in toc -->

Create all Lanelets in the intersection, including lanelets not driven by the vehicle. Additionally, link stop lines and traffic lights to the Lanelets appropriately.

Refer also to the creation scope [vm-07-01](category_others.md#vm-07-01-vector-map-creation-range)

##### Behavior of Autoware <!-- omit in toc -->

Autoware uses lanelets to predict the movements of other vehicles and plan the vehicle's velocity accordingly. Therefore, it is necessary to create all lanelets in the intersection.

{{ diagram("../assets/vm-03-04_1.svg", "The map has a lanelet for every lane and every turn. The ego vehicle turns left.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-04_2.svg", "Correct: all lanelets in the intersection exist.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-04_3.svg", "Incorrect: only the lanelets of the ego vehicle exist.") }}

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)

---

### vm-03-05 Lanelet division in the intersection

#### Detail of requirements <!-- omit in toc -->

Create the Lanelets in the intersection as a single object without dividing them.

{{ diagram("../assets/vm-03-05_1.svg", "The ego vehicle turns right at an intersection with traffic lights.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-05_2.svg", "Correct: one lanelet goes from the entrance to the exit.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-05_3.svg", "Incorrect: the turn is divided into two lanelets.") }}

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)

---

### vm-03-06 Guide lines in the intersection

#### Detail of requirements <!-- omit in toc -->

If there are guide lines in the intersection, draw the Lanelet following them.

In cases where the Lanelets branches off, begin the branching at the end of the guide line. However, it is not necessary to share points or linestrings between Lanelets.

A guide line is a road painting. Give the Linestring along a dashed guide line _type:line_thin_ and _subtype:dashed_, not _type:virtual_. Use _type:virtual_ only for the bounds without road paintings, as in [vm-03-02](#vm-03-02-lanelets-turn-direction-and-virtual-linestring). [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LinestringTagging.md) defines _virtual_ as a non-physical lane boundary. Camera-based localization (YabLoc) uses _line_thin_ Linestrings as road markings, but not _virtual_ Linestrings. Lanelet2 permits a lane change across a _line_thin_ _dashed_ Linestring. To prohibit it, add _lane_change=no_ (see [vm-01-02](category_lane.md#vm-01-02-allowance-for-lane-changes)).

{{ diagram("../assets/vm-03-06_1.svg", "Guide lines for a right turn. On the right, the lane branches into two lanes.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-06_2.svg", "The lanelets follow the guide lines. The branch starts at the end of the guide lines.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)

---

### vm-03-07 Multiple lanelets in the intersection

#### Detail of requirements <!-- omit in toc -->

When connecting multiple lanes with Lanelets at an intersection, those Lanelets should be made adjacent to each other without crossing.

{{ diagram("../assets/vm-03-07_1.svg", "Two vehicles turn right side by side.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-07_2.svg", "Correct: the lanelets are next to each other and do not cross.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-07_3.svg", "Incorrect: the lanelets cross each other.") }}

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)

---

### vm-03-08 Intersection area range

#### Detail of requirements <!-- omit in toc -->

Encircle the intersection's drivable area with a Polygon (_type:intersection_area_). The boundary of this intersection's Polygon should be defined by the objects below.

- Linestrings (_type:road_border_)
- Straight lines at the connection points of lanelets in the intersection.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-08_1.svg", "The intersection_area polygon follows the road borders and the lanelet connection boundaries.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)
- [Static Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/)
- [Lane Change - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_lane_change_module/)

---

### vm-03-09 Range of Lanelet in the intersection

#### Detail of requirements <!-- omit in toc -->

Determine the start and end positions of lanelets in the intersection (henceforth the boundaries of lanelet connections) based on the stop line's position.

- For cases with a painted stop line:
  - The stop line's linestring (_type:stop_line_) position must align with the lanelet's start.
  - Extend the lanelet's end to where the opposing lane's stop line would be.
- Without a painted stop line:
  - Use a drawn linestring (_type:stop_line_) to establish positions as if there were a painted stop line.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-09_1.svg", "Each lanelet starts at its stop line and ends at the extended line of the opposite stop line.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)

---

### vm-03-10 Right of way (with signal)

#### Detail of requirements <!-- omit in toc -->

Set the regulatory element 'right_of_way' for Lanelets that meet all of the following criteria:

- Lanelets in the intersection with a _turn_direction_ of _right_ or _left_.
- Lanelets that intersect with the vehicle's lanelet.
- There are traffic lights at the intersection.

Set to _yield_ those lanelets in the intersection that intersect the vehicle's lanelet, and set to _yield_ those lanelets that do not share the same signal change timing with the vehicle. Also, if the vehicle is turning left, set the opposing vehicle's right-turn lane to _yield_. There is no need to set _yield_ for lanelets where the vehicle goes straight (_turn_direction:straight_).

{{ diagram("../assets/vm-03-10_1.svg", "The ego vehicle turns left (left) or right (right) at an intersection with traffic lights.") }}

#### Preferred vector map <!-- omit in toc -->

##### The vehicle turns left <!-- omit in toc -->

{{ diagram("../assets/vm-03-10_2.svg", "Left turn: the crossing lane of the other signal phase and the opposing right turn are yield lanes.") }}

##### The vehicle turns right <!-- omit in toc -->

{{ diagram("../assets/vm-03-10_3.svg", "Right turn: the crossing lanes of the other signal phase are yield lanes.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)
- [How to set the RightOfWay tag - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/#how-towhy-set-rightofway-tag)

---

### vm-03-11 Right of way (without signal)

#### Detail of requirements <!-- omit in toc -->

Set the regulatory element 'right_of_way' for Lanelets that meet all of the following criteria:

- Lanelets in the intersection with a _turn_direction_ of _right_ or _left_. A lanelet with _turn_direction_ _straight_ also needs it in the cases below.
- Lanelets that intersect with the vehicle's lanelet.
- There are **no** traffic lights at the intersection.

{{ diagram("../assets/vm-03-11_1.svg", "Case 1: the ego vehicle is on the priority road. Case 2: it is on the non-priority road.") }}

#### Preferred vector map <!-- omit in toc -->

##### Case 1: the vehicle is on the priority lane <!-- omit in toc -->

{{ diagram("../assets/vm-03-11_2.svg", "Case 1: the crossing lanes and the opposing right turn are yield lanes.") }}

##### Case 2: the vehicle is on the non-priority lane <!-- omit in toc -->

A regulatory element is not necessary. However, when the vehicle goes straight, it has relative priority over other vehicles turning right from the opposing non-priority road. Therefore, settings for _right_of_way_ and _yield_ are required in this case.

{{ diagram("../assets/vm-03-11_3.svg", "Case 2: the opposing right turn is the yield lane.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)
- [How to set the RightOfWay tag - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/#how-towhy-set-rightofway-tag)

---

### vm-03-12 Right of way supplements

#### Detail of requirements <!-- omit in toc -->

##### Why it's necessary to configure 'right_of_way' <!-- omit in toc -->

Without the 'right_of_way' setting, Autoware interprets other lanes intersecting its path as having priority. Therefore, as long as there are other vehicles in the crossing lane, Autoware cannot enter the intersection regardless of signal indications.

An example of a problem: Even when our signal allows proceeding, our vehicle waits beforehand if other vehicles are waiting at a red light where the opposing lane intersects with a right-turn lane.

#### Preferred vector map <!-- omit in toc -->

None in particular.

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)

---

### vm-03-13 Merging from private area, sidewalk

#### Detail of requirements <!-- omit in toc -->

Set _location=private_ for Lanelets within private property.

Give the private Lanelet that joins the public road a _turn_direction_ tag, as in [vm-03-02](#vm-03-02-lanelets-turn-direction-and-virtual-linestring). Without this tag, the intersection module does not stop the vehicle before the public road.

When a road, which enters or exits private property, intersects with a sidewalk, create a Lanelet for that sidewalk (_subtype:walkway_).

##### Behavior of Autoware <!-- omit in toc -->

- The vehicle stops temporarily before entering the sidewalk. The walkway module does this for a _subtype:walkway_ Lanelet.
- The vehicle comes to a stop before merging onto the public road. The intersection module does this ("merge from private") for a Lanelet on the path that has _location=private_ and a _turn_direction_ tag, when the next Lanelet on the path is not private.

{{ diagram("../assets/vm-03-13_1.svg", "Vehicles enter and leave a private area across the sidewalk.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-13_2.svg", "Private lanelets have location=private, and the crossed sidewalk is a walkway lanelet.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Merge from private - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/#merge-from-private)
- [Walkway - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_walkway_module/)

---

### vm-03-14 Road marking

#### Detail of requirements <!-- omit in toc -->

If there is a stop line ahead of the guide lines in the intersection, ensure the following:

- Create a Lanelet for the guide lines.
- The Lanelet for the guide lines references a Regulatory Element (_subtype:road_marking_).
- The Regulatory Element refers to the _stop_line_'s Linestring.

The intersection module uses this stop line as the default stop line of the Lanelet.

Refer to [Web.Auto Documentation - Creation of Regulatory Element](https://docs.web.auto/en/user-manuals/vector-map-builder/how-to-use/edit-maps#creation-of-regulatory-element) for the method of creation in Vector Map Builder.

{{ diagram("../assets/vm-03-14_1.svg", "A stop line at the end of the guide lines for a right turn.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-14_2.svg", "The lanelet refers to a road_marking regulatory element, which refers to the stop line.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Intersection - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/)

---

### vm-03-15 Exclusive bicycle lane

#### Detail of requirements <!-- omit in toc -->

If an exclusive bicycle lane exists, create a Lanelet with _subtype:bicycle_lane_ for it. The bicycle lane and the road Lanelet next to it must share a Linestring. In left-hand traffic, the bicycle lane is on the left side of the road Lanelet. In right-hand traffic, it is on the right side. A bicycle lane connects only to other bicycle lanes. Autoware supports _subtype:bicycle_lane_ in maps with _format_version_ 2 or later. See [Bicycle lane in the autoware_lanelet2_extension documentation](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/extra_lanelet_subtypes.md#bicycle-lane-format_version--2).

Older maps model the bicycle lane as a Lanelet with _subtype:road_. In that case, set _lane_change = no_ on the Linestring that the bicycle lane shares with the road. For bicycle lanes at intersections, assign a _yield_ designation beneath the _right_of_way_ for lanes that intersect with the vehicle's left-turn lane. (Refer to [vm-03-10](category_intersection.md#vm-03-10-right-of-way-with-signal) and [vm-03-11](category_intersection.md#vm-03-11-right-of-way-without-signal) for right_of_way).

##### Behavior of Autoware <!-- omit in toc -->

The blind spot module checks the side of a turning vehicle for bicycles and pedestrians. It uses the _road_shoulder_ or _bicycle_lane_ Lanelet next to the outermost road Lanelet on the turning side. If there is none, it uses the outermost road Lanelet.

{{ diagram("../assets/vm-03-15_1.svg", "The road sign for an exclusive bicycle lane in Japan.") }}

{{ diagram("../assets/vm-03-15_2.svg", "A road with an exclusive bicycle lane on each side.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-03-15_3.svg", "Each bicycle lane is a bicycle_lane lanelet that shares a linestring with the road lanelet.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Blind Spot design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/)
