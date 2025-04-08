# List of Autoware AD API

| API                                                                                              | Release      | Method          |
| ------------------------------------------------------------------------------------------------ | ------------ | --------------- |
| [/api/fail_safe/mrm_state](./api/fail_safe/mrm_state.md)                                         | v1.1.0       | notification    |
| [/api/fail_safe/rti_state](./api/fail_safe/rti_state.md)                                         | not released | notification    |
| [/api/interface/version](./api/interface/version.md)                                             | v1.0.0       | function call   |
| [/api/localization/initialization_state](./api/localization/initialization_state.md)             | v1.0.0       | notification    |
| [/api/localization/initialize](./api/localization/initialize.md)                                 | v1.0.0       | function call   |
| [/api/manual/local/command/acceleration](./api/manual/local/command/acceleration.md)             | v1.8.0       | realtime stream |
| [/api/manual/local/command/gear](./api/manual/local/command/gear.md)                             | v1.8.0       | notification    |
| [/api/manual/local/command/hazard_lights](./api/manual/local/command/hazard_lights.md)           | v1.8.0       | notification    |
| [/api/manual/local/command/pedals](./api/manual/local/command/pedals.md)                         | v1.8.0       | realtime stream |
| [/api/manual/local/command/steering](./api/manual/local/command/steering.md)                     | v1.8.0       | realtime stream |
| [/api/manual/local/command/turn_indicators](./api/manual/local/command/turn_indicators.md)       | v1.8.0       | notification    |
| [/api/manual/local/command/velocity](./api/manual/local/command/velocity.md)                     | v1.8.0       | realtime stream |
| [/api/manual/local/control_mode/list](./api/manual/local/control_mode/list.md)                   | v1.8.0       | function call   |
| [/api/manual/local/control_mode/select](./api/manual/local/control_mode/select.md)               | v1.8.0       | function call   |
| [/api/manual/local/control_mode/status](./api/manual/local/control_mode/status.md)               | v1.8.0       | notification    |
| [/api/manual/local/operator/heartbeat](./api/manual/local/operator/heartbeat.md)                 | v1.8.0       | realtime stream |
| [/api/manual/remote/command/acceleration](./api/manual/remote/command/acceleration.md)           | v1.8.0       | realtime stream |
| [/api/manual/remote/command/gear](./api/manual/remote/command/gear.md)                           | v1.8.0       | notification    |
| [/api/manual/remote/command/hazard_lights](./api/manual/remote/command/hazard_lights.md)         | v1.8.0       | notification    |
| [/api/manual/remote/command/pedals](./api/manual/remote/command/pedals.md)                       | v1.8.0       | realtime stream |
| [/api/manual/remote/command/steering](./api/manual/remote/command/steering.md)                   | v1.8.0       | realtime stream |
| [/api/manual/remote/command/turn_indicators](./api/manual/remote/command/turn_indicators.md)     | v1.8.0       | notification    |
| [/api/manual/remote/command/velocity](./api/manual/remote/command/velocity.md)                   | v1.8.0       | realtime stream |
| [/api/manual/remote/control_mode/list](./api/manual/remote/control_mode/list.md)                 | v1.8.0       | function call   |
| [/api/manual/remote/control_mode/select](./api/manual/remote/control_mode/select.md)             | v1.8.0       | function call   |
| [/api/manual/remote/control_mode/status](./api/manual/remote/control_mode/status.md)             | v1.8.0       | notification    |
| [/api/manual/remote/operator/heartbeat](./api/manual/remote/operator/heartbeat.md)               | v1.8.0       | realtime stream |
| [/api/motion/accept_start](./api/motion/accept_start.md)                                         | not released | function call   |
| [/api/motion/state](./api/motion/state.md)                                                       | not released | notification    |
| [/api/operation_mode/change_to_autonomous](./api/operation_mode/change_to_autonomous.md)         | v1.0.0       | function call   |
| [/api/operation_mode/change_to_local](./api/operation_mode/change_to_local.md)                   | v1.0.0       | function call   |
| [/api/operation_mode/change_to_remote](./api/operation_mode/change_to_remote.md)                 | v1.0.0       | function call   |
| [/api/operation_mode/change_to_stop](./api/operation_mode/change_to_stop.md)                     | v1.0.0       | function call   |
| [/api/operation_mode/disable_autoware_control](./api/operation_mode/disable_autoware_control.md) | v1.0.0       | function call   |
| [/api/operation_mode/enable_autoware_control](./api/operation_mode/enable_autoware_control.md)   | v1.0.0       | function call   |
| [/api/operation_mode/state](./api/operation_mode/state.md)                                       | v1.0.0       | notification    |
| [/api/perception/objects](./api/perception/objects.md)                                           | not released | realtime stream |
| [/api/planning/cooperation/get_policies](./api/planning/cooperation/get_policies.md)             | not released | function call   |
| [/api/planning/cooperation/set_commands](./api/planning/cooperation/set_commands.md)             | not released | function call   |
| [/api/planning/cooperation/set_policies](./api/planning/cooperation/set_policies.md)             | not released | function call   |
| [/api/planning/steering_factors](./api/planning/steering_factors.md)                             | not released | realtime stream |
| [/api/planning/velocity_factors](./api/planning/velocity_factors.md)                             | not released | realtime stream |
| [/api/routing/change_route](./api/routing/change_route.md)                                       | v1.5.0       | function call   |
| [/api/routing/change_route_points](./api/routing/change_route_points.md)                         | v1.5.0       | function call   |
| [/api/routing/clear_route](./api/routing/clear_route.md)                                         | v1.0.0       | function call   |
| [/api/routing/route](./api/routing/route.md)                                                     | v1.0.0       | notification    |
| [/api/routing/set_route](./api/routing/set_route.md)                                             | v1.0.0       | function call   |
| [/api/routing/set_route_points](./api/routing/set_route_points.md)                               | v1.0.0       | function call   |
| [/api/routing/state](./api/routing/state.md)                                                     | v1.0.0       | notification    |
| [/api/system/diagnostics/status](./api/system/diagnostics/status.md)                             | v1.3.0       | realtime stream |
| [/api/system/diagnostics/struct](./api/system/diagnostics/struct.md)                             | v1.3.0       | notification    |
| [/api/system/heartbeat](./api/system/heartbeat.md)                                               | v1.3.0       | realtime stream |
| [/api/vehicle/dimensions](./api/vehicle/dimensions.md)                                           | v1.1.0       | function call   |
| [/api/vehicle/doors/command](./api/vehicle/doors/command.md)                                     | v1.2.0       | function call   |
| [/api/vehicle/doors/layout](./api/vehicle/doors/layout.md)                                       | v1.2.0       | function call   |
| [/api/vehicle/doors/status](./api/vehicle/doors/status.md)                                       | v1.2.0       | notification    |
| [/api/vehicle/kinematics](./api/vehicle/kinematics.md)                                           | v1.1.0       | realtime stream |
| [/api/vehicle/metrics](./api/vehicle/metrics.md)                                                 | not released | reliable stream |
| [/api/vehicle/status](./api/vehicle/status.md)                                                   | v1.4.0       | realtime stream |
