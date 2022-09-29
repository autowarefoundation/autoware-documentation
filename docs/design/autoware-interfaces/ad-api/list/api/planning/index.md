# Planning API

- {{ link_ad_api('/api/planning/velocity_factors') }}
- {{ link_ad_api('/api/planning/steering_factors') }}

## Description

This API manages the planned behavior of the vehicle.
Applications can notify the vehicle behavior to the people around and visualize it for operator and passengers.

## Velocity factors

The velocity factors is an array of information on the behavior that the vehicle stops (or slows down).
Each factor has a type shown below, pose, distance from the vehicle head to that pose, status, and detailed data depending on its type.
As the vehicle approaches the stop position, this factor appears with a status of APPROACHING.
And when the vehicle reaches that position and stops, the status will be STOPPED.
The pose indicates the stop position or the vehicle head if the stop position cannot be calculated.

![velocity-factors](./docs/velocity-factors.drawio.svg)

| Factor Type                 | Description                                                              |
| --------------------------- | ------------------------------------------------------------------------ |
| SURROUNDING_OBSTACLE        | There are obstacles immediately around the vehicle.                      |
| ROUTE_OBSTACLE              | There are obstacles along the route ahead.                               |
| INTERSECTION                | There are obstacles in other lanes in the path.                          |
| CROSSWALK                   | There are obstacles on the crosswalk.                                    |
| REAR_CHECK                  | There are obstacles behind that would be in a human driver's blind spot. |
| USER_DEFINED_DETECTION_AREA | There are obstacles in the predefined detection area.                    |
| NO_STOPPING_AREA            | There is not enough space beyond the no stopping area.                   |
| STOP_SIGN                   | A stop by a stop sign.                                                   |
| TRAFFIC_SIGNAL              | A stop by a traffic signal.                                              |
| V2I_GATE_CONTROL_ENTER      | A stop by a V2I gate entering.                                           |
| V2I_GATE_CONTROL_LEAVE      | A stop by a V2I gate leaving.                                            |
| MERGE                       | A stop before merging lanes.                                             |
| SIDEWALK                    | A stop before crossing the sidewalk.                                     |
| LANE_CHANGE                 | A lane change.                                                           |
| AVOIDANCE                   | A path change to avoid an obstacle in the current lane.                  |
| EMERGENCY_OPERATION         | A stop by emergency instruction from the operator.                       |

## Steering factors

The steering factors is an array of information on the maneuver that requires use of turn indicators, such as turning left or right.
Each factor has a type shown below, pose, distance from the vehicle head to that pose, status, and detailed data depending on its type.
As the vehicle approaches the position to start steering, this factor appears with a status of APPROACHING.
And when the vehicle reaches that position, the status will be TURNING.
The pose indicates the start position when APPROACHING and the end position when TURNING.

![steering-factors-1](./docs/steering-factors-1.drawio.svg)

In cases such as lane change and avoidance, the vehicle will start steering at any position in the range depending on the situation.
As the vehicle approaches the start position of the range, this factor appears with a status of APPROACHING.
And when the vehicle reaches that position, the status will be TRYING.
Then, when it is possible, the vehicle will start steering and the status will be TURNING.
The pose indicates the start of the range (A) when APPROACHING and the end of the range (B) when TRYING.
The position to end steering (C to D) for TURNING depends on the position to start steering.

![steering-factors-2](./docs/steering-factors-2.drawio.svg)

| Factor Type           | Description                                                              |
| --------------------- | ------------------------------------------------------------------------ |
| INTERSECTION          | A turning left or right at an intersection.                              |
| LANE_CHANGE           | A lane change.                                                           |
| AVOIDANCE_PATH_CHANGE | A path change to avoid an obstacle in the current lane.                  |
| AVOIDANCE_PATH_RETURN | A path change to return to the original lane after avoiding an obstacle. |
| STATION               | T.B.D. (bus stop)                                                        |
| PULL_OUT              | T.B.D.                                                                   |
| PULL_OVER             | T.B.D.                                                                   |
| EMERGENCY_OPERATION   | A path change by emergency instruction from the operator.                |
