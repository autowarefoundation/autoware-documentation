# Planning factors

## Related API

- {{ link_ad_api('/api/planning/velocity_factors') }}
- {{ link_ad_api('/api/planning/steering_factors') }}

## Description

This API manages the planned behavior of the vehicle.
Applications can notify the vehicle behavior to the people around and visualize it for operator and passengers.

Each factor has a behavior type. The `PlanningBehavior` message defines the common values that the tables below show. An implementation can also use other values for its own behaviors.
Some behavior types have sequence and details as additional information. The `PlanningSequence` message defines `change` and `return` for the avoidance behavior.

## Velocity factors

The velocity factors is an array of information on the behavior that the vehicle stops or slows down.

| Behavior                    | Description                                                                                  |
| --------------------------- | -------------------------------------------------------------------------------------------- |
| surrounding-obstacle        | There are obstacles immediately around the vehicle.                                          |
| route-obstacle              | There are obstacles along the route ahead.                                                   |
| intersection                | There are obstacles in other lanes in the path.                                              |
| crosswalk                   | There are obstacles on the crosswalk.                                                        |
| rear-check                  | There are obstacles behind that would be in a human driver's blind spot.                     |
| user-defined-attention-area | There are obstacles in the predefined attention area.                                        |
| roundabout                  | There are obstacles in the roundabout.                                                       |
| run-out                     | There are objects that move toward the path of the vehicle.                                  |
| no-stopping-area            | There is not enough space beyond the no stopping area.                                       |
| no-drivable-lane            | A stop before a lane that is outside the operational design domain (ODD).                    |
| stop-sign                   | A stop by a stop sign.                                                                       |
| traffic-signal              | A stop by a traffic signal.                                                                  |
| virtual-traffic-light       | A stop by a virtual traffic light, for example a shutter or a V2X device at an intersection. |
| merge                       | A stop before merging lanes.                                                                 |
| sidewalk                    | A stop before crossing the sidewalk.                                                         |
| adaptive-cruise             | A slowdown to keep the distance to the vehicle ahead.                                        |
| lane-change                 | A lane change.                                                                               |
| avoidance                   | A path change to avoid an obstacle in the current lane.                                      |
| start-planner               | A start from the road shoulder.                                                              |
| goal-planner                | A stop at the goal, for example after a pull over to the road shoulder.                      |

Each factor also provides the status, the pose of the stop position, and the distance from the vehicle to that pose.
The pose uses the frame of the message header. The default implementation uses the `map` frame.
As the vehicle approaches the stop position, this factor appears with a status of APPROACHING.
And when the vehicle reaches that position and stops, the status will be STOPPED.

{{ diagram("planning-factors/velocity-factors.svg", "A velocity factor for a stop in front of an obstacle.") }}

## Steering factors

The steering factors is an array of information on the maneuver that requires use of turn indicators, such as turning left or right.
Each factor has a behavior type which is described below and steering direction.

| Behavior      | Description                                                                 |
| ------------- | --------------------------------------------------------------------------- |
| intersection  | A turning left or right at an intersection.                                 |
| lane-change   | A lane change.                                                              |
| avoidance     | A path change to avoid an obstacle. It has a sequence of change and return. |
| start-planner | A path from the road shoulder to the center of the lane.                    |
| goal-planner  | A path to the goal, for example a pull over to the road shoulder.           |

Each factor also provides the status, two poses, and the distances from the vehicle to these poses.
As the vehicle approaches the position to start steering, this factor appears with a status of APPROACHING.
And when the vehicle reaches that position, the status will be TURNING.
The poses indicate the start and end position of the section where the status is TURNING.

{{ diagram("planning-factors/steering-factors-1.svg", "A steering factor for a left turn at an intersection.") }}

In cases such as lane change and avoidance, the vehicle will start steering at any position in the range depending on the situation.
For these types, the section where the status is TURNING will be updated dynamically and the poses will follow that.

{{ diagram("planning-factors/steering-factors-2.svg", "A steering factor for a lane change. The section can move forward while the vehicle drives (red).") }}

## Default implementation

The `planning` node of `autoware_default_adapi_universe` makes these factors from the planning factors that the planning modules publish on the `/planning/planning_factors/*` topics.
It maps each module to a behavior type, for example `blind_spot` to `rear-check` and `detection_area` to `user-defined-attention-area`. It skips the factors of modules that it does not map.
It sets the behavior, the pose, the distance, the status, and the direction. It does not set the sequence, the detail, or the cooperation status.
