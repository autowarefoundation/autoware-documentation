# Motion API

- {{ link_ad_api('/api/motion/state') }}
- {{ link_ad_api('/api/motion/factors') }}

## Description

This API manages the behavior that the vehicle plans.

## States

![motion-state](./state.drawio.svg)

| State    | Description                                          |
| -------- | ---------------------------------------------------- |
| STOPPED  | The vehicle is stopped.                              |
| STARTING | The vehicle is about to start (it is still stopped). |
| MOVING   | The vehicle is moving.                               |

## Factors

The motion factors are information on the behavior that the vehicle plans.
They are sorted in ascending order of distance.
There are two types of factors, stop and direction change.
For each type, the meanings of the data members are as follows.

- stop type

  | Name   | Description                                                   |
  | ------ | ------------------------------------------------------------- |
  | pose   | The pose of the stop point.                                   |
  | reason | Reason (e.g. stop line, crosswalk, obstacle, traffic signal). |
  | status | Whether the vehicle is stopped due to this factor.            |

- direction change type

  | Name   | Description                                                  |
  | ------ | ------------------------------------------------------------ |
  | pose   | The pose to turn on/off the blinker.                         |
  | reason | Reason (e.g. turning, lane change, avoidance).               |
  | status | Whether the direction change has started due to this factor. |
