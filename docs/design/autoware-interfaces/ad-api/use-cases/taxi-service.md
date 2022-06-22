# Use cases of bus service

## Overview

This use case is a taxi service that picks up passengers and drives them to their destination.

## Scenario

| Step | Operation                                                  | Use Case                                                         |
| ---- | ---------------------------------------------------------- | ---------------------------------------------------------------- |
| 1    | Startup the autonomous driving system.                     | [Launch and terminate](launch-terminate.md)                      |
| 2    | Drive the vehicle from the garage to the waiting position. | [Change the operation mode](change-operation-mode.md)            |
| 3    | Enable autonomous control.                                 | [Change the operation mode](change-operation-mode.md)            |
| 4    | Drive the vehicle to the position to pick up.              | [Drive to the designated position](drive-designated-position.md) |
| 5    | Get on the vehicle.                                        | [Get on and get off](get-on-off.md)                              |
| 6    | Drive the vehicle to the destination.                      | [Drive to the designated position](drive-designated-position.md) |
| 7    | Get off the vehicle.                                       | [Get on and get off](get-on-off.md)                              |
| 8    | Drive the vehicle to the waiting position.                 | [Drive to the designated position](drive-designated-position.md) |
| 9    | Return to step 4 if there is another request.              |                                                                  |
| 10   | Drive the vehicle from the waiting position to the garage. | [Change the operation mode](change-operation-mode.md)            |
| 11   | Shutdown the autonomous driving system.                    | [Launch and terminate](launch-terminate.md)                      |
