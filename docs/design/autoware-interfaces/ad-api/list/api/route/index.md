# Route API

- {{ link_ad_api('/api/route/state') }}
- {{ link_ad_api('/api/route/clear') }}
- {{ link_ad_api('/api/route/set') }}
- {{ link_ad_api('/api/route/lanelet/set') }}
- {{ link_ad_api('/api/route/lanelet/notice') }}

## Description

This API manages destination and waypoints. Note that waypoints are not like stops and just points passing through.
In other words, Autoware does not support the route with multiple stops, the application needs to split it up and switch them.
There are two ways to set the route. The one is a generic method that uses pose, another is a map-dependent.

## States

![route-state](./state.drawio.svg)

| State    | Description                                        |
| -------- | -------------------------------------------------- |
| WAITING  | The route is not set. Waiting for a route request. |
| SET      | The route is set.                                  |
| ARRIVED  | The vehicle has arrived at the destination.        |
| CHANGING | Trying to change the route. Not implemented yet.   |
