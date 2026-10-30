# Ackermann kinematic model

Autoware now supports control inputs for vehicles based on an Ackermann kinematic model.  
This section introduces you a brief concept of the Ackermann kinematic model
and explains how Autoware controls it.
Please remember,
Autoware control output (`/control/command/control_cmd`)
publishes lateral and longitudinal commands according to the Ackermann kinematic model.

- If your vehicle does not suit the Ackermann kinematic model, you have to modify the control commands. [Another document gives you an example of how to convert your Ackermann kinematic model control inputs into a differential drive model.](customizing-for-differential-drive-model.md)

## Geometry

The basic style of the Ackermann kinematic model has four wheels with an Ackermann link on the front,
and it is powered by the rear wheels.  
The key point of Ackermann kinematic model is
that the axes of all wheels intersect at the same point,
which means
all wheels will trace a circular trajectory with a different radii but a common center point
(See the figure below).  
Therefore,
this model has a great advantage that it minimizes the slippage of the wheels
and prevents tires from getting worn soon.

In general,
the Ackermann kinematic model accepts the longitudinal speed $v$ and the steering angle $\phi$ as inputs.
In autoware, $\phi$ is positive if it is steered counterclockwise,
so the steering angle in the figure below is actually negative.

{{ diagram("images/ackermann-kinematic-model.svg", "An Ackermann kinematic model: on the left the vehicle drives straight ahead, and on the right it steers to the right.") }}

### Control

Autoware publishes the control command on the `/control/command/control_cmd` topic.
The type of this topic is [`autoware_control_msgs/msg/Control`](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Control.msg). It contains a lateral command and a longitudinal command:

```bash title="Control"
builtin_interfaces/Time stamp
builtin_interfaces/Time control_time
autoware_control_msgs/Lateral lateral
autoware_control_msgs/Longitudinal longitudinal
```

where,

```bash title="Lateral"
builtin_interfaces/Time stamp
builtin_interfaces/Time control_time
float32 steering_tire_angle
float32 steering_tire_rotation_rate
bool is_defined_steering_tire_rotation_rate
```

```bash title="Longitudinal"
builtin_interfaces/Time stamp
builtin_interfaces/Time control_time
float32 velocity
float32 acceleration
float32 jerk
bool is_defined_acceleration
bool is_defined_jerk
```

See [Lateral.msg](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Lateral.msg) and [Longitudinal.msg](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_control_msgs/msg/Longitudinal.msg) for details.

The vehicle interface should realize these control commands through your vehicle's control device.

Autoware also sends gear, turn indicator, and hazard light commands (see the [vehicle interface design](../../../design/autoware-architecture-v1/interfaces/components/vehicle-interface.md)). The vehicle interface module must also handle these commands if the vehicle has devices for them.
