# Control

![Node diagram](images/Control-Bus-ODD-Architecture.drawio.svg)

## Overview

Control stack plays a role in transferring the decision from "brain" to "hand / foot". From the previous planning component, the autonomous vehicle acquires the task trajectory. So the following module, i.e. Control stack is supposed to focus on how to maneuver the vehicle to follow the received trajectory.

## Role

These are critical roles of Control stack:

- Derive the lateral command, i.e. how to manipulate the steering wheel
- Derive the longitudinal command, i.e. how to step on the pedal (gas/brake)
- Integrate all the control signals to the vehicle interface, including:
  - lateral command
  - longitudinal command
  - Turn indicators
  - Hazard lights
  - Gear
  - etc..

## Input

The table below summarizes the overall input into Control stack:

| Input              | Topic Name(Data Type)                                                                                                                                                      | Explanation                                                                                                                                                                                                                                                                                                         |
| ------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Trajectory         | `/planning/scenario_planning/trajectory`<br>(_autoware_auto_planning_msgs/msg/Trajectory_)                                                                                 | This is the local sequence of pose, twist and acceleration that Control stack must follow. This must be smooth, and kinematically possible to follow by the Control stack. By default, the trajectory is 10 seconds long at 0.1 second resolution.                                                                  |
| Vehicle Kinematics | `/localization/kinematic_state`<br>(_nav_msgs/msg/Odometry_)                                                                                                               | This includes vehicle's pose and velocity information. Both lateral and longitudinal commands are calculated based on current velocity. For more details, refer to the following description of node implementation.                                                                                                |
| Map data           | `/map/vector_map`<br>(_autoware_auto_mapping_msgs/msg/HADMapBin_)                                                                                                          | This includes all static information about the environment, such as: <ul><li>Lane connection information used for route planning from starting position to goal position</li><li>Lane geometry to generate reference path used to calculate trajectory </li><li> All information related to traffic rules</li></ul> |
| Vehicle steering   | `/vehicle/status/steering_status`<br>(_autoware_auto_vehicle_msgs/msg/SteeringReport_)                                                                                     | This includes the current steering information that is to feed for lateral algorithm.                                                                                                                                                                                                                               |
| Emergency state    | `/system/emergency/emergency_state`<br>(_autoware_auto_system_msgs/msg/EmergencyState_)                                                                                    | This includes information on the emergency state to switch normal driving to emergency handler or not.                                                                                                                                                                                                              |
| Hazard lights      | `/planning/hazard_lights_cmd`<br>`/external/selected/hazard_lights_cmd`<br>`/system/emergency/hazard_lights_cmd`<br>(_autoware_auto_vehicle_msgs/msg/HazardLightsCommand_) | These include information on how to switch the hazard lights, and will be selected by a vehicle gate node.                                                                                                                                                                                                          |
| Turn indicators    | `/planning/turn_indicators_cmd`<br>`/external/selected/turn_indicators_cmd`<br>(_autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand_)                                    | These include information on how to switch the turn indicators, and will be selected by a vehicle gate node.                                                                                                                                                                                                        |
| Engage             | `/autoware/engage`<br>(_autoware_auto_vehicle_msgs/msg/Engage_)                                                                                                            | This includes the information on whether vehicle enters autonomous driving or not.                                                                                                                                                                                                                                  |

## Output

The table below summarizes the final output from Control stack:

| Output          | Topic(Data Type)                                                                                   | Explanation                                                                                                                                                                 |
| --------------- | -------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Control command | `/control/command/control_cmd`<br>(_autoware_auto_control_msgs/msg/AckermannControlCommand_)       | This is the control command published to vehicle interface including: <ul><li>Velocity</li><li>Acceleration</li><li>Steering angle</li><li>Steering rotation rate</li></ul> |
| Turn indicators | `/control/command/turn_indicators_cmd`<br>(_autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand_) | This is the output to control turn signals of the vehicle.                                                                                                                  |
| Hazard lights   | `/control/command/hazard_lights_cmd`<br>(_autoware_auto_vehicle_msgs/msg/HazardLightsCommand_)     | This is the output to control hazard signals of the vehicle.                                                                                                                |
| Gear            | `/control/command/gear_cmd`<br>(_autoware_auto_vehicle_msgs/msg/GearCommand_)                      | This is the output to control gear of the vehicle.                                                                                                                          |

## Implementation

For more details, please refer to the design documents in each package.

- [control performance analysis](https://autowarefoundation.github.io/autoware.universe/main/control/control_performance_analysis/): analyze the tracking performance of a control module and monitor the driving status of the vehicle.
- [external cmd selector](https://autowarefoundation.github.io/autoware.universe/main/control/external_cmd_selector/): publish command according to current mode.
- [joy controller](https://autowarefoundation.github.io/autoware.universe/main/control/joy_controller/): convert a joy msg to autoware commands (e.g. steering wheel, shift, turn signal, engage) for a vehicle.
- [lane departure checker](https://autowarefoundation.github.io/autoware.universe/main/control/lane_departure_checker/): check if vehicle follows a trajectory.
- [obstacle collision checker](https://autowarefoundation.github.io/autoware.universe/main/control/obstacle_collision_checker/): check obstacle collision for predicted trajectory and publish diagnostic errors if collision is found.
- [shift decider](https://autowarefoundation.github.io/autoware.universe/main/control/shift_decider/): decide gear from ackermann control command.
- [MPC lateral controller](https://autowarefoundation.github.io/autoware.universe/main/control/trajectory_follower/design/mpc_lateral_controller-design/):generate lateral control commands (steering angle and steering rate) when following a path using mpc algorithm.
- [PID longitudinal controller](https://autowarefoundation.github.io/autoware.universe/main/control/trajectory_follower/design/pid_longitudinal_controller-design/): compute the target acceleration to achieve the target velocity set at each point of the target trajectory using a feed-forward/back control.
- [vehicle cmd gate](https://autowarefoundation.github.io/autoware.universe/main/control/vehicle_cmd_gate/):get information from emergency handler, planning module, external controller, and send a msg to vehicle.
- [simple trajectory follower](https://autowarefoundation.github.io/autoware.universe/main/control/trajectory_follower_nodes/design/simple_trajectory_follower-design/): a simple follower node, flexible to use.

## Remark

- The performance of control algorithm is affected by the parameter configuration to a certain extent.
- According to different ODD scenarios, the parameters of algorithm may differ a lot.
- The vehicle interface is supposed to be compatible with the output of Control stack.
