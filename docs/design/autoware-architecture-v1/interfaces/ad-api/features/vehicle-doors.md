# Vehicle doors

## Related API

- {{ link_ad_api('/api/vehicle/doors/layout') }}
- {{ link_ad_api('/api/vehicle/doors/status') }}
- {{ link_ad_api('/api/vehicle/doors/command') }}

## Description

This feature is available if the vehicle provides a software interface for the doors.
It can be used to create user interfaces for passengers or to control sequences at bus stops.

## Layout

Each door in a vehicle is assigned an array index. This assignment is vehicle dependent.
The layout API returns this information.
The description field is a string to display in the user interface, etc.
This is an arbitrary string and is not recommended to use for processing in applications.
Use the roles field to know doors for getting on and off.
Below is an example of the information returned by the layout API.

| Index | Description | Roles           |
| ----- | ----------- | --------------- |
| 0     | front right | -               |
| 1     | front left  | GET_ON          |
| 2     | rear right  | GET_OFF         |
| 3     | rear left   | GET_ON, GET_OFF |

## Status

The status API provides an array of door status. This array order is consistent with the layout API.

## Control

Use the command API to control doors.
Unlike the status and layout APIs, array index do not correspond to doors.
The command has a field to specify the target door index.
