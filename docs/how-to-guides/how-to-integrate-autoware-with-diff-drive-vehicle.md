# How to integrate Autoware with differential drive vehicle

## 1. Integrate Autoware with differential drive vehicle

Currently, Autoware assumes the vehicle to follow an Ackermann kinematic model that uses Ackermann steering.
Thus, the Autoware interfaces adopts Ackermann commands format for the output of control module (see [the definition of AckermannControlCommands.idl](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_control_msgs/msg/AckermannControlCommand.idl) for details).

However, it is possible to integrate Autoware with a vehicle that follows a differential drive kinematic model, which is common especially for small mobile robots.

## 2. Procedure

This section briefly explains one of the ways to use Autoware for differential drive vehicles.

One simple way is to create a `vehicle_interface` package that translate Ackermann command to differential drive command.
Ackermann command in Autoware mainly consists of two main control inputs:

- steering angle ($\omega$)
- velocity ($v$)

while the typical differential drive command consists of the following inputs:

- left wheel velocity ($v_l$)
- right wheel velocity ($v_r$)

For example, Ackermann command can be converted to differential drive command with the following equations:

$$
v_l = v - \frac{l\omega}{2}, 
v_r = v + \frac{l\omega}{2}
$$

where $l$ denotes wheel tread.

For general requirements for `vehicle_interface` package, please refer to [the description of `vehicle_interface`](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-interface/) for detail.

## 3. Known issues

### Motion model incompatibility

As mentioned above, Autoware assumes the vehicle to be a steering system and not a differential drive system.
Due to this reason, when you apply Autoware to differential drive vehicles, Autoware cannot take full advantage of the flexibility of their motion model.

For example, `freespace_planner` for parking scenario in planning module may drive the differential drive vehicle forward and backward,
while the vehicle may be possible park with much more simple trajectory with pure rotation movement.
