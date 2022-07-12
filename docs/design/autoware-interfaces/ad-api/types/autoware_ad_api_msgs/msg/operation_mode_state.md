# autoware_ad_api_msgs/msg/OperationModeState

## Definition

```txt
# constants for operation_mode
uint16 UNKNOWN = 0
uint16 STOP = 1
uint16 AUTONOMOUS = 2
uint16 LOCAL = 3
uint16 REMOTE = 4

# variables
uint16 operation_mode
bool is_autoware_control_enabled
bool is_in_transition
bool change_to_stop
bool change_to_autonomous
bool change_to_local
bool change_to_remote
```

## This type uses

None
