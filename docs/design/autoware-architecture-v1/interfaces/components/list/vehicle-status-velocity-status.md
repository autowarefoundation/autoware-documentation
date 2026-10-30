---
last_updated: 2026-01-21
interface_type: topic
interface_name: /vehicle/status/velocity_status
data_type_name: autoware_vehicle_msgs/msg/VelocityReport
data_type_link: https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_vehicle_msgs/msg/VelocityReport.msg
rate: 10 (design value)
qos_reliability: reliable
qos_durability: volatile
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Get the current velocity status of the vehicle.

The rate of 10 Hz in the table is a design value. The [`autoware_component_interface_specs`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/common/autoware_component_interface_specs/include/autoware/component_interface_specs/vehicle.hpp) package does not define a rate. Known publishers send the status faster: [`autoware_simple_planning_simulator`](https://github.com/autowarefoundation/autoware_simple_planning_simulator/blob/1.1.0/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp) publishes it at 40 Hz (`timer_sampling_time_ms: 25`), and [`pacmod_interface`](https://github.com/tier4/pacmod_interface/blob/main/pacmod_interface/src/pacmod_interface/pacmod_interface.cpp) publishes it once for each set of PACMod reports. The consumers need a steady rate. For example, [`autoware_gyro_odometer`](https://github.com/autowarefoundation/autoware_core/blob/1.10.0/localization/autoware_gyro_odometer/src/gyro_odometer.cpp) gets this velocity through [`autoware_vehicle_velocity_converter`](https://github.com/autowarefoundation/autoware_core/tree/1.10.0/sensing/autoware_vehicle_velocity_converter). It stops its output when the latest vehicle twist is more than `message_timeout_sec` (0.2 s by default) older than the latest IMU message.

## Message

See the [message definition]({{ data_type_link }}) for details.

## Errors

Unknown status: If the vehicle interface cannot get the status due to connection loss, etc., the status is stopped and a diagnostic error is reported.

Hardware Fault: If the vehicle platform reports a sensor fault, a diagnostic error is reported.

## Support

This interface is required.

## Limitations

Wheel speed: The vehicle interface often computes the longitudinal velocity from the wheel speeds. For example, `pacmod_interface` uses the mean speed of the rear wheels and the tire radius, and it takes the sign from the gear report. Wheel slip and an incorrect tire radius cause errors. The `speed_scale_factor` parameter of `autoware_vehicle_velocity_converter` can correct a scale error.

Heading rate: The vehicle interface can compute the heading rate instead of measuring it. For example, `pacmod_interface` computes it from the velocity, the steering angle, and the wheelbase with a kinematic model. `autoware_gyro_odometer` uses the angular velocity of the IMU instead.

Lateral velocity: Some vehicle interfaces do not set the lateral velocity. For example, `pacmod_interface` does not set the `lateral_velocity` field, so its value is always 0.

## Use cases

- Control the vehicle for autonomous driving.
- Display current velocity status to the operator.

## Requirement

- Support getting the current velocity status of the vehicle.
- Report the error as diagnostics if the status cannot be received or an unknown status is received.

## Design

Coordinate system & sign convention: The interface follows the standard vehicle coordinate system (ISO 8855 / ROS REP-103).

- Longitudinal velocity: Positive (+) indicates **forward** motion. Negative (-) indicates **backward** motion.
- Lateral velocity: Positive (+) indicates motion to the **left**. Negative (-) indicates motion to the **right**.
- Heading rate: Positive (+) indicates **counter-clockwise** rotation (left turn).

## History

| Date       | Description                      |
| ---------- | -------------------------------- |
| 2026-01-21 | First release in the new format. |
