# How to integrate Autoware with a differential drive vehicle

## 1. Introduction

Currently, Autoware assumes that vehicles use an Ackermann kinematic model with Ackermann steering.
Thus, the Autoware interfaces adopt the Ackermann commands format for the output of the control module (see [the definition of AckermannControlCommands.idl](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_control_msgs/msg/AckermannControlCommand.idl) for details).

However, it is possible to integrate Autoware with a vehicle that follows a differential drive kinematic model, as commonly used by small mobile robots.

## 2. Procedure

One simple way of using Autoware with a differential drive vehicle is to create a `vehicle_interface` package that translates Ackermann commands to differential drive commands.
Here are two points that you need to consider:

- Create `vehicle_interface` package for differential drive vehicle
- Set an appropriate `wheel_base`

### 2.1 Create a `vehicle_interface` package for differential drive vehicle

An Ackermann command in Autoware consists of two main control inputs:

- steering angle ($\omega$)
- velocity ($v$)

Conversely, a typical differential drive command consists of the following inputs:

- left wheel velocity ($v_l$)
- right wheel velocity ($v_r$)

For example, the Ackermann command can be converted to the differential drive command with the following equations:

$$
v_l = v - \frac{l\omega}{2},
v_r = v + \frac{l\omega}{2}
$$

where $l$ denotes wheel tread.

For general requirements for `vehicle_interface` package, please refer to [the description of `vehicle_interface`](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-interface/) for detail.

### 2.2 Set an appropriate `wheel_base`

A differential drive robot does not necessarily have front and rear wheels, which means that the wheelbase (the horizontal distance between the axles of the front and rear wheels) cannot be defined. However, Autoware expects `wheel_base` to be set in `vehicle_info.param.yaml` with some value.
Thus, you need to set a pseudo value for `wheel_base`.

The appropriate pseudo value for `wheel_base` depends on the size of your vehicle.
Setting it to be the same value as `wheel_tread` is one possible choice.

!!! warning
    - If `wheel_base` is too large, the vehicle may restrict the vehicle's motion. 
    - Conversely, Autoware Universe is not guaranteed to work properly when the `wheel_base` value is too small.

## 3. Known issues

### Motion model incompatibility

Since Autoware assumes that vehicles use a steering system, it is not possible to take advantage of the flexibility of a differential drive system's motion model.

For example, when planning a parking maneuver with the `freespace_planner` module, Autoware may drive the differential drive vehicle forward and backward, even if the vehicle can be parked with a simpler trajectory that uses pure rotational movement.