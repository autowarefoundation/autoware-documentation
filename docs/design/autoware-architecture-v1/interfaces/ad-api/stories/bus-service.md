# User story of bus service

## Overview

This user story is a bus service that goes around the designated stops.

## Scenario

| Step | Operation                                                  | Use case                                                                      |
| ---- | ---------------------------------------------------------- | ----------------------------------------------------------------------------- |
| 1    | Start up the autonomous driving system.                    | [Launch and terminate](../use-cases/launch-terminate.md)                      |
| 2    | Drive the vehicle from the garage to the waiting position. | [Change the operation mode](../use-cases/change-operation-mode.md)            |
| 3    | Initialize the pose of the vehicle.                        | [Initialize the pose](../use-cases/initialize-pose.md)                        |
| 4    | Enable autonomous control.                                 | [Change the operation mode](../use-cases/change-operation-mode.md)            |
| 5    | Drive the vehicle to the next bus stop.                    | [Drive to the designated position](../use-cases/drive-designated-position.md) |
| 6    | Get on and off the vehicle.                                | [Get on and get off](../use-cases/get-on-off.md)                              |
| 7    | Return to step 5 unless it's the last bus stop.            |                                                                               |
| 8    | Drive the vehicle to the waiting position.                 | [Drive to the designated position](../use-cases/drive-designated-position.md) |
| 9    | Drive the vehicle from the waiting position to the garage. | [Change the operation mode](../use-cases/change-operation-mode.md)            |
| 10   | Shut down the autonomous driving system.                   | [Launch and terminate](../use-cases/launch-terminate.md)                      |
