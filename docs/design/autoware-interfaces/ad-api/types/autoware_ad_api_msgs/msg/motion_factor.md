# autoware_ad_api_msgs/msg/MotionFactor

## Definition

```txt
# constants for reason
uint16 INTERSECTION=1
uint16 MERGE_FROM_PRIVATE_ROAD=2
uint16 CROSSWALK=3
uint16 WALKWAY=4
uint16 STOP_LINE=5
uint16 DETECTION_AREA=6
uint16 NO_STOPPING_AREA=7
uint16 TRAFFIC_LIGHT=8
uint16 OBSTACLE_STOP=9
uint16 SURROUND_OBSTACLE_CHECK=10
uint16 BLIND_SPOT=11
uint16 BLOCKED_BY_OBSTACLE=12
uint16 STOPPING_LANE_CHANGE=13
uint16 REMOTE_EMERGENCY_STOP=14
uint16 VIRTUAL_TRAFFIC_LIGHT=15

# constants for status
uint16 STOP_FALSE=1
uint16 STOP_TRUE=2

# variables
geometry_msgs/Pose pose
float32 distance
uint16 reason
uint16 status
string detail
```

## This type uses

None
