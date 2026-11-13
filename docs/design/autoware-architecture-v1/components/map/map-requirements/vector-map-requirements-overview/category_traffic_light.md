---
title: Traffic light
---

The drawings on this page use left-hand traffic, as in Japan: vehicles drive on the left side of the road, and a right turn crosses the opposing lane. For right-hand traffic, swap left and right. In the map drawings, a colored area is a Lanelet, and a line with dots is a Linestring and its points.

---

## Requirements

### vm-04-01 Traffic light basics

#### Detail of requirements <!-- omit in toc -->

When you create traffic lights in a vector map, meet the following requirements:

- Road Lanelet (_subtype:road_). Quantity: one.
- Traffic Light. Multiple instances possible.
  - Traffic light Linestring (_type:traffic_light_).
  - Traffic light bulbs Linestring (_type:light_bulbs_).
  - Stop line Linestring (_type:stop_line_).
- Regulatory element for traffic lights (_subtype:traffic_light_). The road Lanelet refers to it. It refers to the traffic light (_traffic_light_ with the role _refers_, _light_bulbs_ with the role _light_bulbs_) and to the stop line (_stop_line_ with the role _ref_line_). Quantity: one.

Refer to [Web.Auto Documentation - Creation of a traffic light and a stop line](https://docs.web.auto/en/user-manuals/vector-map-builder/how-to-use/edit-maps#creation-of-a-traffic-light-and-a-stop-line) for the method of creation in Vector Map Builder.

Refer to vm-04-02 and vm-04-03 for the specifications of traffic light and traffic light bulb objects.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-04-01_1.svg", "The traffic_light Regulatory Element refers to the stop line, the traffic light, and its light bulbs.") }}

If there is a crosswalk at the intersection, make the road Lanelet and the crosswalk Lanelet intersect and overlap.

{{ diagram("../assets/vm-04-01_2.svg", "The road Lanelet and the crosswalk Lanelet overlap.") }}

#### Related Autoware module

- [Traffic Light design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/)
- [autoware_traffic_light_map_based_detector - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_traffic_light_map_based_detector/)

---

### vm-04-02 Traffic light position and size

#### Detail of requirements <!-- omit in toc -->

Create traffic lights with Linestring.

- _type:traffic_light_
- _subtype:red_yellow_green_

Make the Linestring (from the start point to the end point) match the bottom edge of the traffic light. The start point is the left end, and the end point is the right end. Make sure that the 3D coordinates of the Linestring give the correct height of the traffic light above the ground.

Use _tag:height_ for the size of the traffic light. For example, for 50 cm, write _tag:height=0.5_. This height is the size of the traffic light, not its position.

##### Supplemental information <!-- omit in toc -->

The map-based detector of the traffic light recognition skips traffic light Linestrings that have no subtype or that have _subtype:solid_. The camera then does not detect these lights. Autoware does not read other subtype values, such as _red_yellow_green_.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-04-02_1.svg", "The traffic_light Linestring runs along the bottom edge of the light, and the height tag gives its size.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Traffic Light design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/)
- [autoware_traffic_light_map_based_detector - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_traffic_light_map_based_detector/)

---

### vm-04-03 Traffic light lamps

#### Detail of requirements <!-- omit in toc -->

To let the system detect the color of traffic lights, create the color scheme and the arrangement of the lamps accurately as objects. Show the position of each lamp with a Point. For colored lamps, use the _color_ tag for the color. For arrow lamps, use the _arrow_ tag for the direction.

- _tag: color = red, yellow, green_
- _tag: arrow = up, right, left, up_right, up_left_

Create a Linestring from the Points of the lamps.

- _type: light_bulbs_
- _tag: traffic_light_id_ = the ID of the traffic light Linestring (_type:traffic_light_)

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-04-03_1.svg", "The light_bulbs Linestring connects the centers of the lamps. Each Point has a color tag, and the arrow lamp also has an arrow tag.") }}

The order of the Points of the lamps can be 1→2→3→4 or 4→3→2→1. Both are acceptable.

##### Supplemental information <!-- omit in toc -->

The map-based signal filter of `autoware_traffic_light_multi_camera_fusion` reads these tags. It also accepts the color _white_ and the arrows _down_, _down_left_, _down_right_, and _cross_. It ignores a Point without a _color_ tag.

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Traffic Light design - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/)
- [autoware_traffic_light_multi_camera_fusion - Autoware Universe Documentation](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_traffic_light_multi_camera_fusion/)
