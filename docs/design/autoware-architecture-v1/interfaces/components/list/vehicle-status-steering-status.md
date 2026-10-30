---
last_updated: 2026-01-21
interface_type: topic
interface_name: /vehicle/status/steering_status
data_type_name: autoware_vehicle_msgs/msg/SteeringReport
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/SteeringReport.msg
rate: 10 (design value)
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current steering status of the vehicle.

The rate of 10 Hz in the table is a design value. The [`autoware_component_interface_specs`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_component_interface_specs/include/autoware/component_interface_specs/vehicle.hpp) package does not define a rate. Known publishers send the status faster: [`autoware_simple_planning_simulator`](https://github.com/autowarefoundation/autoware_simple_planning_simulator/blob/1.1.0/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp) publishes it at 40 Hz (`timer_sampling_time_ms: 25`), and [`pacmod_interface`](https://github.com/tier4/pacmod_interface/blob/main/pacmod_interface/src/pacmod_interface/pacmod_interface.cpp) publishes it once for each set of PACMod reports. The consumers use the latest message that they received. For example, [`autoware_trajectory_follower_node`](https://github.com/autowarefoundation/autoware_universe/blob/0.53.1/control/autoware_trajectory_follower_node/src/controller_node.cpp) and [`autoware_vehicle_cmd_gate`](https://github.com/autowarefoundation/autoware_universe/blob/0.53.1/control/autoware_vehicle_cmd_gate/src/vehicle_cmd_gate.cpp) read the latest steering angle and do not check its age.

## Message

See the [message definition]({{ data_type_link }}) for details.

## Errors

Unknown status: If the vehicle interface cannot get the status due to connection loss, etc., the status is stopped and a diagnostic error is reported.

Hardware Fault: If the vehicle platform reports a sensor fault, a diagnostic error is reported.

## Support

This interface is required.

## Limitations

Conversion accuracy: This value is typically calculated from the Steering Wheel Angle using a steering gear ratio (which may be variable/non-linear). Therefore, the accuracy depends on the correctness of the gear ratio model parameters.

Mechanical play: Due to mechanical backlash in the steering column, small movements of the steering wheel might not result in actual tire movement, causing a discrepancy between this reported status and the physical tire angle.

Tire deformation: This value represents the kinematic angle of the wheel assembly, not the actual slip angle of the tire contact patch.

## Use cases

- Control the vehicle for autonomous driving.
- Display current steering status to the operator.

## Requirement

- Support getting the current steering status of the vehicle.
- Report the error as diagnostics if the status cannot be received or an unknown status is received.

## Design

Coordinate system & sign convention: The interface follows the standard vehicle coordinate system (ISO 8855 / ROS REP-103).

- Positive (+) indicates **counter-clockwise** rotation (left turn).
- Negative (-) indicates **clockwise** rotation (right turn).

Data source: Reports the steering tire angle (average of front wheels or virtual center wheel).
Typically converted from the steering wheel angle sensor data using a linear or variable gear ratio model.

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
