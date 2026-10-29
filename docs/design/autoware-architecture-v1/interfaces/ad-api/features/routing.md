# Routing

## Related API

- {{ link_ad_api('/api/routing/state') }}
- {{ link_ad_api('/api/routing/route') }}
- {{ link_ad_api('/api/routing/clear_route') }}
- {{ link_ad_api('/api/routing/set_route_points') }}
- {{ link_ad_api('/api/routing/set_route') }}
- {{ link_ad_api('/api/routing/change_route_points') }}
- {{ link_ad_api('/api/routing/change_route') }}

## Description

This API manages destination and waypoints. Note that waypoints are not like stops and just points passing through.
In other words, Autoware does not support the route with multiple stops, the application needs to split it up and switch them.
There are two ways to set the route. The one is a generic method that uses pose ({{ link_ad_api('/api/routing/set_route_points') }}), another is a map-dependent method that uses lanelet segments ({{ link_ad_api('/api/routing/set_route') }}).
The start of the route is always the current pose of the vehicle.

## States

{{ diagram("routing/state.svg", "The route state changes with the set, change, and clear requests, and with the arrival at the goal.") }}

| State    | Description                                        |
| -------- | -------------------------------------------------- |
| UNSET    | The route is not set. Waiting for a route request. |
| SET      | The route is set.                                  |
| ARRIVED  | The vehicle has arrived at the destination.        |
| CHANGING | Trying to change the route.                        |

The set APIs accept a request only in the `UNSET` state, and the change APIs only in the `SET` state.
In the default implementation, Autoware changes the operation mode to stop when the vehicle arrives at the goal in autonomous mode.
The clear API fails while the vehicle uses the route, that is, while it moves in autonomous mode under Autoware control.

## Options

The route set and change APIs have route options that allow applications to choose several behaviors regarding route planning.
See the sections below for supported options and details.

### allow_goal_modification

**[v1.1.0]** Autoware tries to look for an alternate goal when goal is unreachable (for example, when there is an obstacle on the given goal). When setting a route from the API, applications can choose whether they allow Autoware to adjust goal pose in such situation. When set false, Autoware can get stuck until the given goal becomes reachable.
