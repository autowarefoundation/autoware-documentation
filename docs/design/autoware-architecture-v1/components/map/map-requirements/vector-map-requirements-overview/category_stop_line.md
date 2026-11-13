---
title: Stop line
---

The drawings on this page use left-hand traffic, as in Japan: vehicles drive on the left side of the road. For right-hand traffic, swap left and right. In the map drawings, a colored area is a Lanelet, and a line with dots is a Linestring and its points.

---

## Requirements

### vm-02-01 Stop line alignment

#### Detail of requirements <!-- omit in toc -->

Place the Linestring (_type:stop_line_) for the stop line on the edge of the white line that the vehicle reaches first.

Refer to [Web.Auto Documentation - Creation and edit of a stop point (StopPoint)](https://docs.web.auto/en/user-manuals/vector-map-builder/how-to-use/edit-maps#creation-and-edit-of-a-stop-point-stoppoint) for the method of creation in Vector Map Builder.

{{ diagram("../assets/vm-02-01_1.svg", "A painted stop line across the lane of the vehicle.") }}

##### Supplemental information <!-- omit in toc -->

A stop_line Linestring alone does not stop the vehicle. A Regulatory Element must refer to it. For a stop line with a stop sign, use a Regulatory Element (_subtype:traffic_sign_), as in vm-02-02. The stop line module of Autoware uses only the stop lines of traffic_sign Regulatory Elements with a stop_sign. Traffic lights, crosswalks, detection areas, and no stopping areas refer to their stop lines through their own Regulatory Elements.

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-02-01_2.svg", "The stop_line Linestring is on the near edge of the white line.") }}

#### Incorrect vector map <!-- omit in toc -->

{{ diagram("../assets/vm-02-01_3.svg", "The stop_line Linestring is in the middle of the white line.") }}

#### Related Autoware module

- [Stop Line design - Autoware Core Documentation](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/)

---

### vm-02-02 Stop sign

#### Detail of requirements <!-- omit in toc -->

Where there is no stop line on the road but a stop sign exists, place a Linestring as the stop line next to the sign.

Create a reference from a Lanelet (_subtype:road_) to a Regulatory Element (_subtype:traffic_sign_). Make this Regulatory Element refer to a Linestring (_type:stop_line_) with the role _ref_line_ and to a Linestring (_type:traffic_sign_, _subtype:stop_sign_) with the role _refers_.

{{ diagram("../assets/vm-02-02_1.svg", "A stop sign next to the lane, without a painted stop line.") }}

#### Preferred vector map <!-- omit in toc -->

{{ diagram("../assets/vm-02-02_2.svg", "The traffic_sign Regulatory Element refers to the stop line and to the stop sign.") }}

#### Incorrect vector map <!-- omit in toc -->

None in particular.

#### Related Autoware module

- [Stop Line design - Autoware Core Documentation](https://autowarefoundation.github.io/autoware_core/main/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/)
