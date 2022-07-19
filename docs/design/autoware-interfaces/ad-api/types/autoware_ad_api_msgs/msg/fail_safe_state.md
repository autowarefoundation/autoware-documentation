# autoware_ad_api_msgs/msg/FailSafeState

## Definition

```txt
# constants for both
uint16 UNKNOWN = 0

# constants for stamp
uint16 NORMAL = 1
uint16 MRM_OPERATING = 2
uint16 MRM_SUCCEEDED = 3
uint16 MRM_FAILED = 4

# constants for behavior
uint16 NONE = 101
uint16 SUDDEN_STOP = 102
uint16 COMFORTABLE_STOP = 103

# variables
builtin_interfaces/Time stamp
uint16 state
uint16 behavior
```

## This type uses

None
