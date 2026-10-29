# Heartbeat

## Related API

- {{ link_ad_api('/api/system/heartbeat') }}

## Description

This API is used to check whether applications and Autoware are communicating properly.
The message contains timestamp and sequence number to check for communication delays, order, and losses.
Autoware publishes the heartbeat at 10 Hz. The sequence number wraps at 65535.

This API sends the heartbeat from Autoware to the application. For manual control, the application sends its own heartbeat to Autoware with the operator heartbeat API. See [manual control](manual-control.md).
