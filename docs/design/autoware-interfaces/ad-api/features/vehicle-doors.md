# Vehicle doors

## Related API

- {{ link_ad_api('/api/vehicle/doors/layout') }}
- {{ link_ad_api('/api/vehicle/doors/status') }}
- {{ link_ad_api('/api/vehicle/doors/command') }}

## Description

This feature is available if the vehicle provides a software interface for the doors.
It can be used to create user interfaces for passengers or to control sequences at bus stops.

Applications first get their layout to control doors.
The door layout provides an array of information for each door such as the following.

| Index | Description | Roles           |
| ----- | ----------- | --------------- |
| 0     | front right |                 |
| 1     | front left  | GET_ON          |
| 2     | rear right  | GET_ON, GET_OFF |
| 3     | rear left   | GET_OFF         |
