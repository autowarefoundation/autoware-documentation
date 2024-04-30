# Autoware AD API

## Overview

Autoware AD API is the interface for operating the vehicle from outside the autonomous driving system.
[See here for the overall interface design of Autoware.](../index.md)

## User stories

The user stories are service scenarios that AD API assumes. AD API is designed based on these scenarios.
Each scenario is realized by a combination of use cases described later.
If there are scenarios that cannot be covered, please discuss adding a user story.

- [Bus service](./stories/bus-service.md)
- [Taxi service](./stories/taxi-service.md)

## Use cases

Use cases are partial scenarios derived from the user story and generically designed.
Service providers can combine these use cases to define user stories and check if AD API can be applied to their own scenarios.

- [Launch and terminate](./use-cases/launch-terminate.md)
- [Initialize the pose](./use-cases/initialize-pose.md)
- [Change the operation mode](./use-cases/change-operation-mode.md)
- [Drive to the designated position](./use-cases/drive-designated-position.md)
- [Get on and get off](./use-cases/get-on-off.md)
- [Vehicle monitoring](./use-cases/vehicle-monitoring.md)
- [Vehicle operation](./use-cases/vehicle-operation.md)
- [System monitoring](./use-cases/system-monitoring.md)

## Features

- [Interface](./features/interface.md)
- [Operation Mode](./features/operation_mode.md)
- [Routing](./features/routing.md)
- [Localization](./features/localization.md)
- [Motion](./features/motion.md)
- [Planning](./features/planning-factors.md)
- [Perception](./features/perception.md)
- [Fail-safe](./features/fail-safe.md)
- [Vehicle status](./features/vehicle-status.md)
- [Vehicle doors](./features/vehicle-doors.md)
- [Cooperation](./features/cooperation.md)
- [Heartbeat](./features/heartbeat.md)
