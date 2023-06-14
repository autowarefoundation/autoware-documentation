# Vehicle status

## Related API

- {{ link_ad_api('/api/vehicle/kinematics') }}
- {{ link_ad_api('/api/vehicle/status') }}
- {{ link_ad_api('/api/vehicle/dimensions') }}
- {{ link_ad_api('/api/vehicle/doors/layout') }}
- {{ link_ad_api('/api/vehicle/doors/status') }}

## Kinematics

This is an estimate of the vehicle kinematics. The vehicle position is necessary for applications to schedule dispatches.
Also, using velocity and acceleration, applications can find vehicles that need operator assistance, such as stuck or brake suddenly.

## Status

This is the status provided by the vehicle. The indicators and steering are mainly used for visualization and remote control.
The remaining energy can be also used for vehicle scheduling.

## Doors

This feature is available if the vehicle provides a software interface for the doors.
It can be used to create user interfaces for passengers or to control sequences at bus stops.

## Dimensions

The vehicle dimensions are used to know the actual distance between the vehicle and objects because the vehicle position in kinematics is the coordinates of the base link. This is necessary for visualization when supporting vehicles remotely.
