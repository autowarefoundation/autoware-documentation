# Vehicle operation

## Request to intervene

Request to intervene (RTI) is a feature that requires the operator to switch to manual driving mode. It is also called Take Over Request (TOR).
Interfaces for RTI are currently being discussed. For now assume that manual driving is requested if the MRM state is not NORMAL.
See [fail-safe](../features/fail-safe.md) for details.

## Request to cooperate

Request to cooperate (RTC) is a feature that the operator supports the decision in autonomous driving mode.
Autoware usually drives the vehicle using its own decisions, but the operator may prefer to make their own decisions in complex situations.
Since RTC only overrides the decision and does not need to change operation mode, the vehicle can continue autonomous driving, unlike RTC.
See [cooperation](../features/cooperation.md) for details.
