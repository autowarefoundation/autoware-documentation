# Vehicle status

## Related API

- {{ link_ad_api('/api/vehicle/kinematics') }}
- {{ link_ad_api('/api/vehicle/status') }}
- {{ link_ad_api('/api/vehicle/metrics') }}
- {{ link_ad_api('/api/vehicle/dimensions') }}
- {{ link_ad_api('/api/vehicle/specs') }}

## Kinematics

This is an estimate of the vehicle kinematics. The vehicle position is necessary for applications to schedule dispatches.
Also, using velocity and acceleration, applications can find vehicles that need operator assistance, such as stuck or brake suddenly.
The message has the pose in the map frame and the geographic position (latitude, longitude, and altitude). If the map uses local coordinates without a geographic projection, the geographic position is not available (NaN).

## Status

This is the status provided by the vehicle, such as indicators and steering.
These are mainly used for visualization and remote control.

## Metrics

This is metrics data provided by the vehicle, such as remaining energy.
The remaining energy can be used for vehicle dispatch scheduling.
The default implementation publishes it every 10 seconds (`update_rate` is 0.1 Hz).

## Dimensions

The vehicle dimensions are used to know the actual distance between the vehicle and objects because the vehicle position in kinematics is the coordinates of the base link. This is necessary for visualization when supporting vehicles remotely.
See [vehicle dimensions](../../components/vehicle-dimensions.md) for the definition of each value.

## Specs

The vehicle specs give other values of the vehicle, for example the maximum steering tire angle.
This API is not released yet, and the default implementation does not provide it.
