# Manual control

## Description

To operate a vehicle without a steering wheel or steering wheel, or to develop a remote operation system, an interface to manually drive a vehicle using Autoware is required.

## Requirements

- The vehicle can transition to a safe state when a communication problem occurs.
- Supports the following commands and statuses.
  - Pedal, acceleration or velocity as longitudinal control
  - Steering tire angle as lateral control
  - Gear
  - Turn indicators
  - Hazard lights
- The following are under consideration.
  - Headlights
  - Wipers
  - Parking brake
  - Horn

## Sequence

```plantuml
{% include 'design/autoware-interfaces/ad-api/use-cases/manual-control/sequence.plantuml' %}
```

## Related features

- [Manual control](../../features/manual-control.md)
- [Vehicle status](../../features/vehicle-status.md)
