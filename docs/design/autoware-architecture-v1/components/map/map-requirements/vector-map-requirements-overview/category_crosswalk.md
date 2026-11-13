---
title: Crosswalk
---

The drawings on this page use left-hand traffic, as in Japan: vehicles drive on the left side of the road. For right-hand traffic, swap left and right. In the map drawings, a colored area is a Lanelet, a dashed area is a Polygon, and a line with dots is a Linestring and its points.

There are two types of requirements for crosswalks. Both can apply to a single crosswalk.

- [vm-05-01](category_crosswalk.md#vm-05-01-crosswalks-across-the-road) : Crosswalks across the road
- [vm-05-02](category_crosswalk.md#vm-05-02-crosswalks-with-pedestrian-signals) : Crosswalks with pedestrian signals

A crosswalk at an intersection must also meet the requirements of the intersection.

---

## Requirements

### vm-05-01 Crosswalks across the road

#### Detail of requirements <!-- omit in toc -->

Necessary requirements for creation:

1. Create a Lanelet for the crosswalk (_subtype:crosswalk_).
2. If there is a stop line before the crosswalk, create a Linestring (_type:stop_line_). Create stop lines for the opposing traffic lane in the same way.
3. Create a Polygon to cover the crosswalk. The regulatory element refers to it with the role _crosswalk_polygon_. Autoware does not read a type tag on this Polygon.
4. The Lanelet of the road refers to the regulatory element (_subtype:crosswalk_). The regulatory element refers to the crosswalk Lanelet (role _refers_), the stop line Linestrings (role _ref_line_), and the Polygon (role _crosswalk_polygon_).

##### Supplemental information <!-- omit in toc -->

- Link the regulatory element to the lanelet(s) of the road that intersects with the crosswalk.
- The stop lines linked to the regulatory element do not have to be on the road Lanelets linked with the regulatory element.

##### Behavior of Autoware <!-- omit in toc -->

When pedestrians or cyclists are on the crosswalk, Autoware stops before the stop line and waits for them to pass. When they have cleared the area, Autoware starts to move forward.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-05-01_1.svg", "The crosswalk Regulatory Element refers to the crosswalk Lanelet, the crosswalk Polygon, and the stop lines.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Crosswalk - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/)

---

### vm-05-02 Crosswalks with pedestrian signals

#### Detail of requirements <!-- omit in toc -->

Necessary requirements for creation:

- Create a Lanelet (_subtype:crosswalk_, _participant:pedestrian_).
- Create a Traffic Light Linestring. If multiple traffic lights exist, create multiple Linestrings.
  - Linestring
    - _type:traffic_light_
    - _subtype_: see the supplemental information
    - _height_:value
- Make sure that the crosswalk Lanelet refers to a Regulatory Element (_subtype:traffic_light_). Also make sure that the Regulatory Element refers to the Linestrings (_type:traffic_light_).

##### Supplemental information <!-- omit in toc -->

Autoware knows that a traffic light is for pedestrians because a crosswalk Lanelet refers to its Regulatory Element. It does not use the subtype for this. The subtype controls the camera detection:

- If the camera must detect the pedestrian light, give the Linestring a subtype other than _solid_, for example _subtype:red_green_. The map-based detector skips traffic light Linestrings that have no subtype or that have _subtype:solid_.
- If the Linestring has _subtype:solid_, as in the example of the [Autoware Lanelet2 extension](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/extra_regulatory_elements.md#traffic-lights-for-crosswalks), the camera does not detect it. The crosswalk traffic light estimator then estimates the state of the pedestrian light from the vehicle traffic lights.

Refer to [vm-04-02](category_traffic_light.md#vm-04-02-traffic-light-position-and-size) for more about the traffic light object.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-05-02_1.svg", "The crosswalk Lanelet refers to a traffic_light Regulatory Element for the pedestrian lights.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Crosswalk - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/)

---

### vm-05-03 Deceleration for safety at crosswalks

#### Detail of requirements <!-- omit in toc -->

To make the vehicle slow down to a safe speed when it passes a crosswalk, add the following tags to the crosswalk Lanelet (_subtype:crosswalk_):

- _safety_slow_down_speed_ [m/s]: The maximum velocity while crossing.
- _safety_slow_down_distance_ [m]: The start of the area where the maximum speed applies, measured from the front bumper of the vehicle to the crosswalk.

The vehicle slows down even when there are no pedestrians or cyclists near the crosswalk. If the Lanelet has _safety_slow_down_speed_ but no _safety_slow_down_distance_, the crosswalk module uses a distance of 2.0 m.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-05-03_2.svg", "The safety slow down tags are on the crosswalk Lanelet.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Crosswalk - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/)

---

### vm-05-04 Fences

#### Detail of requirements <!-- omit in toc -->

Autoware detects pedestrians and bicycles that cross the crosswalk, and also those that can cross it. But areas near the crosswalk where many people move, such as fenced kindergartens, playgrounds, or parks, can affect the crosswalk detection. The predicted paths of people and bicycles from these areas can go to the crosswalk.

Surround the areas that are not connected to the crosswalk with a Linestring (_type:fence_). You do not need to link this Linestring to anything.

If there is a guardrail, wall, or fence between the road and the sidewalk, with another fence behind it, you can omit the second fence. But do not omit fences in the areas around crosswalks.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-05-04_1.svg", "A fence Linestring separates the area from the road.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [map_based_prediction - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_map_based_prediction/)
