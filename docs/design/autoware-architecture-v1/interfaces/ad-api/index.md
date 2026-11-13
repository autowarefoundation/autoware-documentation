# Autoware AD API

## Overview

Autoware AD API is the interface for operating the vehicle from outside the autonomous driving system, for example from a fleet management system (FMS) or a human machine interface (HMI).
See the [overall interface design of Autoware](../index.md) for the concepts and the communication methods.

- The [list of APIs](list/index.md) shows every API with its release version and communication method.
- The [types](types/index.md) page shows the message and service types, which the `autoware_adapi_msgs` repository defines.
- The [release notes](release.md) show the changes of each interface version. Autoware 2.0.0 provides the interface version 1.9.1.

## Default implementation

Autoware provides a default implementation of AD API. It starts with Autoware when the `launch_api` argument of `autoware.launch.xml` is `true`, which is the default.

| Package                                                                                                                                       | Repository        | Provides                                                                                        |
| :-------------------------------------------------------------------------------------------------------------------------------------------- | :---------------- | :---------------------------------------------------------------------------------------------- |
| [`autoware_adapi_specs`](https://github.com/autowarefoundation/autoware_core/tree/main/api/autoware_adapi_specs)                              | Autoware Core     | The name, type, and QoS of each API, as C++ headers.                                            |
| [`autoware_default_adapi`](https://github.com/autowarefoundation/autoware_core/tree/main/api/autoware_default_adapi)                          | Autoware Core     | The interface, localization, and routing APIs.                                                  |
| [`autoware_default_adapi_universe`](https://github.com/autowarefoundation/autoware_universe/tree/main/system/autoware_default_adapi_universe) | Autoware Universe | The other APIs, and the launch file that starts all API nodes in the `/adapi/node/*` namespace. |
| [`autoware_adapi_adaptors`](https://github.com/autowarefoundation/autoware_core/tree/main/api/autoware_adapi_adaptors)                        | Autoware Core     | Adaptors that convert the RViz initial pose and goal topics to AD API calls.                    |

## User stories

The user stories are service scenarios that AD API assumes. AD API is designed based on these scenarios.
Each scenario is realized by a combination of use cases described later.
If there are scenarios that cannot be covered, please discuss adding a user story.

<div class="grid cards aw-cards" markdown>

- :fa-cl-s fa-bus: **[Bus service](stories/bus-service.md)**

  A bus goes around the designated stops.

- :fa-cl-s fa-taxi: **[Taxi service](stories/taxi-service.md)**

  A taxi picks up passengers and drives them to their destination.

</div>

## Use cases

Use cases are partial scenarios derived from the user story and generically designed.
Service providers can combine these use cases to define user stories and check if AD API can be applied to their own scenarios.

<div class="grid cards aw-cards aw-cards--small" markdown>

- :fa-cl-s fa-power-off: **[Launch and terminate](use-cases/launch-terminate.md)**

  Start and stop Autoware with the AD API.

- :fa-cl-s fa-location-crosshairs: **[Initialize the pose](use-cases/initialize-pose.md)**

  Give the initial pose for localization.

- :fa-cl-s fa-toggle-on: **[Change the operation mode](use-cases/change-operation-mode.md)**

  Switch between stop, autonomous, and manual modes.

- :fa-cl-s fa-route: **[Drive to the designated position](use-cases/drive-designated-position.md)**

  Set a route and drive to the goal.

- :fa-cl-s fa-door-open: **[Get on and get off](use-cases/get-on-off.md)**

  Open and close the doors for passengers.

- :fa-cl-s fa-gauge-high: **[Vehicle monitoring](use-cases/vehicle-monitoring.md)**

  Watch the vehicle status, plans, and objects.

- :fa-cl-s fa-hand: **[Vehicle operation](use-cases/vehicle-operation.md)**

  Support the decisions of Autoware.

- :fa-cl-s fa-heart-pulse: **[System monitoring](use-cases/system-monitoring.md)**

  Check the communication and the diagnostics.

- :fa-cl-s fa-gamepad: **[Manual control](use-cases/manual-control/index.md)**

  Drive the vehicle manually through Autoware.

</div>

## Features

| Feature                                          | Description                                                                |
| :----------------------------------------------- | :------------------------------------------------------------------------- |
| [Interface](features/interface.md)               | The interface version of AD API.                                           |
| [Operation mode](features/operation_mode.md)     | The operation mode (stop, autonomous, local, remote) and Autoware control. |
| [Routing](features/routing.md)                   | The route to the goal.                                                     |
| [Localization](features/localization.md)         | The initialization of localization.                                        |
| [Motion](features/motion.md)                     | The stop and start of the vehicle.                                         |
| [Planning factors](features/planning-factors.md) | The reasons why the vehicle stops, slows down, or steers.                  |
| [Perception](features/perception.md)             | The objects that Autoware detects.                                         |
| [Fail-safe](features/fail-safe.md)               | The minimal risk maneuver (MRM) and its requests.                          |
| [Vehicle status](features/vehicle-status.md)     | The kinematics, status, metrics, and dimensions of the vehicle.            |
| [Vehicle doors](features/vehicle-doors.md)       | The layout, status, and commands of the doors.                             |
| [Control](features/control.md)                   | The commands that Autoware sends to the vehicle.                           |
| [Cooperation](features/cooperation.md)           | Operator decisions for the planning scenes.                                |
| [Heartbeat](features/heartbeat.md)               | The heartbeat of Autoware.                                                 |
| [Diagnostics](features/diagnostics.md)           | The diagnostic graph of Autoware.                                          |
| [Manual control](features/manual-control.md)     | Manual driving commands from a local or remote operator.                   |
