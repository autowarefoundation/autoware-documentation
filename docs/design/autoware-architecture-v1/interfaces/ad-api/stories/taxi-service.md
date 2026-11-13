# User story of taxi service

## Overview

This user story is a taxi service that picks up passengers and drives them to their destination.

## Scenario

| Step | Operation                                                  | Use case                                                                      |
| ---- | ---------------------------------------------------------- | ----------------------------------------------------------------------------- |
| 1    | Start up the autonomous driving system.                    | [Launch and terminate](../use-cases/launch-terminate.md)                      |
| 2    | Drive the vehicle from the garage to the waiting position. | [Change the operation mode](../use-cases/change-operation-mode.md)            |
| 3    | Initialize the pose of the vehicle.                        | [Initialize the pose](../use-cases/initialize-pose.md)                        |
| 4    | Enable autonomous control.                                 | [Change the operation mode](../use-cases/change-operation-mode.md)            |
| 5    | Drive the vehicle to the position to pick up.              | [Drive to the designated position](../use-cases/drive-designated-position.md) |
| 6    | Get on the vehicle.                                        | [Get on and get off](../use-cases/get-on-off.md)                              |
| 7    | Drive the vehicle to the destination.                      | [Drive to the designated position](../use-cases/drive-designated-position.md) |
| 8    | Get off the vehicle.                                       | [Get on and get off](../use-cases/get-on-off.md)                              |
| 9    | Drive the vehicle to the waiting position.                 | [Drive to the designated position](../use-cases/drive-designated-position.md) |
| 10   | Return to step 5 if there is another request.              |                                                                               |
| 11   | Drive the vehicle from the waiting position to the garage. | [Change the operation mode](../use-cases/change-operation-mode.md)            |
| 12   | Shut down the autonomous driving system.                   | [Launch and terminate](../use-cases/launch-terminate.md)                      |
