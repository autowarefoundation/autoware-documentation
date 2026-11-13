# Initialize the pose

Autoware needs a global pose as the initial guess for localization. The application gives this pose with the localization API, or it lets Autoware use the GNSS pose.

## Related API

- [Localization](../features/localization.md)
- {{ link_ad_api('/api/localization/initialize') }}
- {{ link_ad_api('/api/localization/initialization_state') }}

## Sequence

### Initialize the pose with an input pose

The user gives the pose, for example in a GUI. The application puts the pose in the request. RViz uses this method: `initial_pose_adaptor` in `autoware_adapi_adaptors` converts the `/initialpose` topic to an API call.

{{ diagram("sequence/initialize-pose-input.svg", "Initialize the pose with a pose from the user.") }}

### Initialize the pose with GNSS

The application sends the request with an empty pose array. Autoware then uses the latest GNSS pose as the initial guess. This method needs a GNSS receiver.

{{ diagram("sequence/initialize-pose-gnss.svg", "Initialize the pose with the GNSS pose.") }}

## Notes

- In both methods, Autoware refines the initial guess with the pose estimator (NDT or YabLoc) if one is enabled.
- The request fails with `ERROR_UNSAFE` if the vehicle is not stopped, when the stop check of `autoware_pose_initializer` is enabled.
- If the initialization fails, the state changes to `UNINITIALIZED`. Send the request again.
