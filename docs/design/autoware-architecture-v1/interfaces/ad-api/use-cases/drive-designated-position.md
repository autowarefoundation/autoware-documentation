# Drive to the designated position

The application sets a route to the goal and starts autonomous driving. Autoware drives to the goal and stops there.

## Related API

- [Operation mode](../features/operation_mode.md)
- [Routing](../features/routing.md)

## Sequence

{{ diagram("sequence/drive-designated-position.svg", "Set a route, start autonomous driving, and clear the route at the goal.") }}

<div class="aw-steps" markdown>

1. **Set the route.** When the route state is `UNSET`, call {{ link_ad_api('/api/routing/set_route_points') }} with the goal pose and the waypoints, or {{ link_ad_api('/api/routing/set_route') }} with lanelet segments. The route state changes to `SET`.
2. **Wait until the autonomous mode is available.** Autoware checks its components, for example the route, the localization, and the planning output. When all checks pass, `is_autonomous_mode_available` of {{ link_ad_api('/api/operation_mode/state') }} becomes `true`.
3. **Start.** Call {{ link_ad_api('/api/operation_mode/change_to_autonomous') }}. Autoware starts to drive along the route.
4. **Arrive.** At the goal, the route state changes to `ARRIVED`. In the default implementation, the routing node of AD API then changes the operation mode to stop.
5. **Clear the route.** Call {{ link_ad_api('/api/routing/clear_route') }}. The route state changes to `UNSET`, and the application can set the next route.

</div>

To change the goal while the vehicle drives, use {{ link_ad_api('/api/routing/change_route_points') }} or {{ link_ad_api('/api/routing/change_route') }}. See [routing](../features/routing.md) for the route states.
