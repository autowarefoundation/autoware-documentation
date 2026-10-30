---
title: Area
---

The drawings on this page use left-hand traffic, as in Japan: vehicles drive on the left side of the road. For right-hand traffic, swap left and right. In the map drawings, a colored area is a Lanelet, a dashed area is a Polygon, and a line with dots is a Linestring and its points.

---

## Requirements

### vm-06-01 Buffer Zone

#### Detail of requirements <!-- omit in toc -->

Create a Polygon (_type:hatched_road_markings_) when a Buffer Zone (also known as a zebra zone) is painted on the road surface.

- If the Buffer Zone is next to a Lanelet, share Points between them.
- If the Buffer Zone is at an intersection, make its Polygon overlap the Polygon of the intersection (_intersection_area_).

##### Behavior of Autoware <!-- omit in toc -->

To avoid obstacles, Autoware can use the Buffer Zone as a drivable area and drive through it. The static and dynamic obstacle avoidance modules do this when their parameter `use_hatched_road_markings` is `true` (the default in `autoware_launch`).

##### Caution

- Vehicles must not drive through safety zones. Do not confuse Buffer Zones with safety zones.
- Do not create a Polygon for a Buffer Zone where static objects, such as poles, block the vehicles, even if the Buffer Zone is painted on the surface. Create Buffer Zones only where vehicles can pass.

{{ diagram("../assets/vm-06-01_1.svg", "A Buffer Zone: a hatched marking that vehicles can drive on.") }}

{{ diagram("../assets/vm-06-01_2.svg", "A safety zone: vehicles must not drive through it.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-06-01_3.svg", "hatched_road_markings Polygons on a straight road and at an intersection.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Static Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/)
- [Dynamic Avoidance - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/)

---

### vm-06-02 No parking signs

#### Detail of requirements <!-- omit in toc -->

In a Vector Map, you can prohibit parking in specific areas, but still permit temporary stops.

Create a reference from a Lanelet (_subtype:road_) to a Regulatory Element (_subtype:no_parking_area_), and make this Regulatory Element refer to a Polygon (_type:no_parking_area_).

Refer to [Web.Auto Documentation - Creation of No Parking Area](https://docs.web.auto/en/user-manuals/vector-map-builder/how-to-use/edit-maps#creation-of-no-parking-area) for the method of creation in Vector Map Builder.

##### Behavior of Autoware <!-- omit in toc -->

The goal planner does not select a goal in a _no_parking_area_. As a result, Autoware does not park the vehicle there.

The goal planner reads only the Regulatory Elements of the Lanelets in which it looks for a goal. These are the road shoulder Lanelets around the goal, or the outermost road Lanelet if the road has no road shoulder. If the road has a road shoulder, the road shoulder Lanelet must also refer to the Regulatory Element (_subtype:no_parking_area_).

{{ diagram("../assets/vm-06-02_1.svg", "A no parking sign and the yellow line along the curb.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-06-02_2.svg", "The road Lanelet refers to a no_parking_area Regulatory Element and its Polygon.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Goal Planner design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/)

---

### vm-06-03 No stopping signs

#### Detail of requirements <!-- omit in toc -->

Where a sign prohibits parking and stopping, create the same objects as for vm-06-02.

Create a reference from a Lanelet (_subtype:road_) to a Regulatory Element (_subtype:no_parking_area_), and make this Regulatory Element refer to a Polygon (_type:no_parking_area_).

Refer to [Web.Auto Documentation - Creation of No Parking Area](https://docs.web.auto/en/user-manuals/vector-map-builder/how-to-use/edit-maps#creation-of-no-parking-area) for the method of creation in Vector Map Builder.

##### Behavior of Autoware <!-- omit in toc -->

The goal planner does not select a goal in a _no_parking_area_. As a result, Autoware does not park the vehicle there. A _no_parking_area_ does not prevent temporary stops. To prevent all stops, use a _no_stopping_area_ (vm-06-04).

{{ diagram("../assets/vm-06-03_1.svg", "A no parking and no stopping sign and the yellow line along the curb.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-06-03_2.svg", "The road Lanelet refers to a no_parking_area Regulatory Element and its Polygon.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Goal Planner design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/)

---

### vm-06-04 No stopping sections

#### Detail of requirements <!-- omit in toc -->

Vehicles can stop on the road for signals or traffic congestion. But in a Vector Map, you can prohibit all stops (temporary stops, parking, and idling) in specific areas.

Create a reference from a Lanelet (_subtype:road_) to a Regulatory Element (_subtype:no_stopping_area_), and make this Regulatory Element refer to a Polygon (_type:no_stopping_area_). The Regulatory Element can also refer to one Linestring (_type:stop_line_) with the role _ref_line_. The vehicle stops at this line if it cannot pass through the area.

Refer to [Web.Auto Documentation - Creation of No Stopping Area](https://docs.web.auto/en/user-manuals/vector-map-builder/how-to-use/edit-maps#creation-of-no-stopping-area) for the method of creation in Vector Map Builder.

##### Behavior of Autoware <!-- omit in toc -->

The vehicle does not make temporary stops in a _no_stopping_area_. If a stopped vehicle ahead blocks the area, the vehicle stops before the area. The goal planner does not select a goal in a _no_stopping_area_, so the vehicle cannot park there.

The no stopping area module reads the Regulatory Element from the Lanelets on the path of the vehicle. The goal planner reads it only from the Lanelets in which it looks for a goal, as in vm-06-02. If the road has a road shoulder, the road shoulder Lanelet must also refer to the Regulatory Element (_subtype:no_stopping_area_).

{{ diagram("../assets/vm-06-04_1.svg", "A no stopping zone: the vehicle must drive through without stopping.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-06-04_2.svg", "The road Lanelet refers to a no_stopping_area Regulatory Element, its Polygon, and an optional stop line.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [No Stopping Area design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module/)
- [Goal Planner design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/)

---

### vm-06-05 Detection area

#### Detail of requirements <!-- omit in toc -->

Autoware finds obstacles in the Detection Area: points of the point cloud or predicted objects. If it finds obstacles, the vehicle stops at the stop line and stays there until the obstacles go away. To use this function, add the Detection Area element to the Vector Map.

Create a reference from a Lanelet (_subtype:road_) to a Regulatory Element (_subtype:detection_area_). Make this Regulatory Element refer to a Polygon (_type:detection_area_) with the role _refers_ and to a Linestring (_type:stop_line_) with the role _ref_line_. One Regulatory Element can refer to more than one detection area Polygon.

Refer to [Web.Auto Documentation - Creation of Detection Area](https://docs.web.auto/en/user-manuals/vector-map-builder/how-to-use/edit-maps#creation-of-detection-area) for the method of creation in Vector Map Builder.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-06-05_1.svg", "The detection_area Regulatory Element refers to the stop line and to the detection area Polygon.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Detection Area - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module/)
