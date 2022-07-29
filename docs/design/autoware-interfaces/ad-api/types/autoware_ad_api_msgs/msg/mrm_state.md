# autoware_ad_api_msgs/msg/MrmState

## Definition

```txt
# constants for both
uint16 UNKNOWN = 0

# constants for stamp
uint16 NONE = 1
uint16 OPERATING = 2
uint16 SUCCEEDED = 3
uint16 FAILED = 4

# constants for behavior
uint16 NONE = 101
uint16 EMERGENCY_STOP = 102
uint16 COMFORTABLE_STOP = 103

# variables
builtin_interfaces/Time stamp
uint16 state
uint16 behavior
```

## This type uses

None
