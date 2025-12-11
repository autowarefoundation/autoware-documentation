# User story of bus service

## Overview

This user story is a bus service that goes around the designated stops.

## Scenario

| Step | Operation                                                  | Use Case                                                                      |
| ---- | ---------------------------------------------------------- | ----------------------------------------------------------------------------- |
| 1    | Startup the autonomous driving system.                     | [Launch and terminate](../use-cases/launch-terminate.md)                      |
| 2    | Drive the vehicle from the garage to the waiting position. | [Change the operation mode](../use-cases/change-operation-mode.md)            |
| 3    | Enable autonomous control.                                 | [Change the operation mode](../use-cases/change-operation-mode.md)            |
| 4    | Drive the vehicle to the next bus stop.                    | [Drive to the designated position](../use-cases/drive-designated-position.md) |
| 5    | Get on and off the vehicle.                                | [Get on and get off](../use-cases/get-on-off.md)                              |
| 6    | Return to step 4 unless it's the last bus stop.            |                                                                               |
| 7    | Drive the vehicle to the waiting position.                 | [Drive to the designated position](../use-cases/drive-designated-position.md) |
| 8    | Drive the vehicle from the waiting position to the garage. | [Change the operation mode](../use-cases/change-operation-mode.md)            |
| 9    | Shutdown the autonomous driving system.                    | [Launch and terminate](../use-cases/launch-terminate.md)                      |
