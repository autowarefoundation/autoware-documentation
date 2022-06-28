# autoware_ad_api_msgs/msg/OperationModeNotice

## Definition

```txt
# constants for selector modes, skip 1 to match ChangeOperationMode.
uint16 UNKNOWN = 0
uint16 STOP = 2
uint16 AUTONOMOUS = 3
uint16 LOCAL = 4
uint16 REMOTE = 5

# variables
uint16 selector_mode
bool is_driver_control
bool is_in_transition
```

## This type uses

None
