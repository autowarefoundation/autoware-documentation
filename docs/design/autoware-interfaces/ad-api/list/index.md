# List of Autoware AD API

| API                                                                                              | Release      | Method          |
| ------------------------------------------------------------------------------------------------ | ------------ | --------------- |
| [/api/fail_safe/mrm_state](./api/fail_safe/mrm_state.md)                                         | v1.1.0       | notification    |
| [/api/fail_safe/rti_state](./api/fail_safe/rti_state.md)                                         | not released | notification    |
| [/api/interface/version](./api/interface/version.md)                                             | v1.0.0       | function call   |
| [/api/local/command/acceleration](./api/local/command/acceleration.md)                           | not released | realtime stream |
| [/api/local/command/gear](./api/local/command/gear.md)                                           | not released | notification    |
| [/api/local/command/hazard_lights](./api/local/command/hazard_lights.md)                         | not released | notification    |
| [/api/local/command/pedals](./api/local/command/pedals.md)                                       | not released | realtime stream |
| [/api/local/command/steering](./api/local/command/steering.md)                                   | not released | realtime stream |
| [/api/local/command/turn_indicators](./api/local/command/turn_indicators.md)                     | not released | notification    |
| [/api/local/command/velocity](./api/local/command/velocity.md)                                   | not released | realtime stream |
| [/api/local/control_mode/list](./api/local/control_mode/list.md)                                 | not released | function call   |
| [/api/local/control_mode/select](./api/local/control_mode/select.md)                             | not released | function call   |
| [/api/local/control_mode/status](./api/local/control_mode/status.md)                             | not released | notification    |
| [/api/local/operator/status](./api/local/operator/status.md)                                     | not released | realtime stream |
| [/api/localization/initialization_state](./api/localization/initialization_state.md)             | v1.0.0       | notification    |
| [/api/localization/initialize](./api/localization/initialize.md)                                 | v1.0.0       | function call   |
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
| [/api/remote/command/acceleration](./api/remote/command/acceleration.md)                         | not released | realtime stream |
| [/api/remote/command/gear](./api/remote/command/gear.md)                                         | not released | notification    |
| [/api/remote/command/hazard_lights](./api/remote/command/hazard_lights.md)                       | not released | notification    |
| [/api/remote/command/pedals](./api/remote/command/pedals.md)                                     | not released | realtime stream |
| [/api/remote/command/steering](./api/remote/command/steering.md)                                 | not released | realtime stream |
| [/api/remote/command/turn_indicators](./api/remote/command/turn_indicators.md)                   | not released | notification    |
| [/api/remote/command/velocity](./api/remote/command/velocity.md)                                 | not released | realtime stream |
| [/api/remote/control_mode/list](./api/remote/control_mode/list.md)                               | not released | function call   |
| [/api/remote/control_mode/select](./api/remote/control_mode/select.md)                           | not released | function call   |
| [/api/remote/control_mode/status](./api/remote/control_mode/status.md)                           | not released | notification    |
| [/api/remote/operator/status](./api/remote/operator/status.md)                                   | not released | realtime stream |
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
| [/api/vehicle/status](./api/vehicle/status.md)                                                   | v1.4.0       | realtime stream |
