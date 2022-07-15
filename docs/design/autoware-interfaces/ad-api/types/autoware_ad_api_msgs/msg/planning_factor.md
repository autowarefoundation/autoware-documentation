# autoware_ad_api_msgs/msg/PlanningFactor

## Definition

```txt
# constants for type
uint16 SURROUND_OBSTACLE = 1
uint16 FRONT_OBSTACLE = 2
uint16 INTERSECTION = 3
uint16 CROSSWALK = 4
uint16 REAR_CHECK = 5
uint16 USER_DEFINED_DETECTION_AREA = 6
uint16 NO_STOPPING_AREA = 7
uint16 STOP_SIGN = 8
uint16 TRAFFIC_LIGHT = 9
uint16 V2I_GATE_CONTROL = 10
uint16 FROM_PRIVATE_ROAD = 11
uint16 SIDEWALK = 12
uint16 LANE_CHANGE = 13
uint16 AVOIDANCE1 = 14
uint16 AVOIDANCE2 = 15
uint16 DIRECTION_CHANGE = 16
uint16 EMERGENCY_STOP_OPERATION = 17

# constants for status
uint16 STOPPING = 1
uint16 STOPPED = 2
uint16 APPROACHING = 3
uint16 ACTIVATING = 4
uint16 ACTIVATED = 5

# variables
geometry_msgs/Pose pose
float32 distance
uint16 type
uint16 status
string detail
```

## This type uses

None
