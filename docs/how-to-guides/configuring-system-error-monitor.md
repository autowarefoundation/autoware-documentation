# Configuring System Error Monitor

## Overview
Although Autoware repository has CI tests to do integration testing, there are always unkown error causing function failure.
This may include:
* Nodes terminates abnormally
* Nodes freezes without crashing and stops publishing message or causes drop in publish rate of message.
* Nodes functioning beyond it's designed state (e.g., losing its localization pose, large deviation from planned trajectory, etc)

In order to detect such failure and trigger minimum risk manuever, we have system monitoring module.
This page explains how users can configure `system_error_monitor` for Autoware for their use case.

## Configuring Parameters

### Turning on Emergency Handler function
By default, Contorl module will ignore any emergency control sent from System Monitor Module.
This is because it is not always safe to make a sudden stop at node failure, and we would like the user to be aware of the function when it is turned on.

In order to turn on its function, modify `use_emergency_handling` parameter to true in [vehicle_cmd_gate](https://github.com/autowarefoundation/autoware.universe/tree/main/control/vehicle_cmd_gate) package.
For example, if you are using autoware.launch.xml to launch Autoware, modify the parameter in this [file](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml).

### Adding new topic monitoring
By default, only few of the topics are monitored as default.
You can add new topic monitor by modifying `topics.yaml` file passed to [component_state_monitor](https://github.com/autowarefoundation/autoware.universe/tree/main/system/component_state_monitor) launch file.

For example, if you are using autoware.launch.xml to launch Autoware, modify the parameter in this [file](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/system/component_state_monitor/topics.yaml) to add the new topic `/planning/scenario_planning/lane_driving/behavior_planning/path` as monitoring target:

```
- module: planning
  mode: [online, planning_simulation]
  type: autonomous
  args:
    node_name_suffix: scenario_planning_path
    topic: /planning/scenario_planning/lane_driving/behavior_planning/path
    topic_type: autoware_auto_planning_msgs/msg/Path
    best_effort: false
    transient_local: false
    warn_rate: 5.0
    error_rate: 1.0
    timeout: 1.0
```
