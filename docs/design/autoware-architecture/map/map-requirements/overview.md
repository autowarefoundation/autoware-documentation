# Vector Map creation requirement specifications

## Overview

Autoware relies on high-definition point cloud maps and vector maps of the driving environment to perform various tasks such as localization, route planning, traffic light detection, and predicting the trajectories of pedestrians and other vehicles.

A vector map contains highly accurate information about a road network, lane geometry, and traffic lights. It is required for route planning, traffic light detection, and predicting the trajectories of other vehicles and pedestrians.

Vector Map uses [lanelet2_extension](https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md), which is based on the [lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) format and extended for Autoware.

The primitives (basic components) used in Vector Map are explained in [Web.Auto Docs - What is Lanelet2](https://docs.web.auto/en/user-manuals/vector-map-builder/introduction#what-is-lanelet2). The following **Vector Map creation requirement specifications** are written on the premise of these knowledge.

This specification is a set of requirements for the creation of Vector Map(s) to ensure that Autoware drives safely and autonomously as intended by the user. It does not cover how to operate specific map creation tools.

- [Vector Map Builder - how to use](https://docs.web.auto/user-manuals/vector-map-builder/how-to-use/edit-maps)

## Handling of the Requirement Specification

Which requirements apply entirely depends on the configuration of the Autoware system on a vehicle. Before creating a Vector Map, it is necessary to clearly determine in advance how you want the vehicle with the implemented system to behave in various environments.

Next, you must comply with the laws of the country where the autonomous driving vehicle will be operating. It is your responsibility to choose which of the following requirements to apply according to the laws.

!!!Caution - The examples of the road signs and road surface markings are used in Japan. Please replace them with those used in your respective countries. - The values for range and distance indicated are minimum values. Please determine values that comply with the laws of your country. Furthermore, these minimum values may change depending on the maximum velocity of the autonomous driving vehicle.

## List of Requirement Specifications

| Category                                              | ID       | Requirements                                            |
| ----------------------------------------------------- | -------- | ------------------------------------------------------- |
| [Category Lane](./category_lane.md)                   | vm-01-01 | Lanelet basics                                          |
|                                                       | vm-01-02 | Allowance for lane changes                              |
|                                                       | vm-01-03 | Linestring sharing                                      |
|                                                       | vm-01-04 | Sharing of the centerline of lanes for opposing traffic |
|                                                       | vm-01-05 | Lane geometry                                           |
|                                                       | vm-01-06 | Line position (1)                                       |
|                                                       | vm-01-07 | Line position (2)                                       |
|                                                       | vm-01-08 | Line position (3)                                       |
|                                                       | vm-01-09 | Speed limits                                            |
|                                                       | vm-01-10 | Centerline                                              |
|                                                       | vm-01-11 | Centerline connection (1)                               |
|                                                       | vm-01-12 | Centerline connection (2)                               |
|                                                       | vm-01-13 | Roads with no centerline (1)                            |
|                                                       | vm-01-14 | Roads with no centerline (2)                            |
|                                                       | vm-01-15 | Road shoulder                                           |
|                                                       | vm-01-16 | Road shoulder Linestring sharing                        |
|                                                       | vm-01-17 | Side strip                                              |
|                                                       | vm-01-18 | Side strip Linestring sharing                           |
|                                                       | vm-01-19 | Walkway                                                 |
| [Category Stop Line](./category_stop_line.md)         | vm-02-01 | Stop line alignment                                     |
|                                                       | vm-02-02 | Stop sign                                               |
| [Category Intersection](./category_intersection.md)   | vm-03-01 | Intersection criteria                                   |
|                                                       | vm-03-02 | Lanelet's turn direction and virtual                    |
|                                                       | vm-03-03 | Lanelet width in the intersection                       |
|                                                       | vm-03-04 | Lanelet creation in the intersection                    |
|                                                       | vm-03-05 | Lanelet division in the intersection                    |
|                                                       | vm-03-06 | Guide lines in the intersection                         |
|                                                       | vm-03-07 | Multiple lanelets in the intersection                   |
|                                                       | vm-03-08 | Intersection Area range                                 |
|                                                       | vm-03-09 | Range of Lanelet in the intersection                    |
|                                                       | vm-03-10 | Right of way (with signal)                              |
|                                                       | vm-03-11 | Right of way (without signal)                           |
|                                                       | vm-03-12 | Right of way supplements                                |
|                                                       | vm-03-13 | Merging from private area, sidewalk                     |
|                                                       | vm-03-14 | Road marking                                            |
|                                                       | vm-03-15 | Exclusive bicycle lane                                  |
| [Category Traffic Light](./category_traffic_light.md) | vm-04-01 | Traffic light basics                                    |
|                                                       | vm-04-02 | Traffic light position and size                         |
|                                                       | vm-04-03 | Traffic light lamps                                     |
| [Category Crosswalk](./category_crosswalk.md)         | vm-05-01 | Crosswalks across the road                              |
|                                                       | vm-05-02 | Crosswalks with pedestrian signals                      |
|                                                       | vm-05-03 | Deceleration for safety at crosswalks                   |
|                                                       | vm-05-04 | Fences                                                  |
| [Category Area](./category_area.md)                   | vm-06-01 | Buffer Zone                                             |
|                                                       | vm-06-02 | No parking signs                                        |
|                                                       | vm-06-03 | No stopping signs                                       |
|                                                       | vm-06-04 | No stopping sections                                    |
|                                                       | vm-06-05 | Detection area                                          |
| [Category Others](./category_others.md)               | vm-07-01 | Vector Map creation range                               |
|                                                       | vm-07-02 | Range of detecting pedestrians who enter the road       |
|                                                       | vm-07-03 | Guardrails, guard pipes, fences                         |
|                                                       | vm-07-04 | Ellipsoidal height                                      |
