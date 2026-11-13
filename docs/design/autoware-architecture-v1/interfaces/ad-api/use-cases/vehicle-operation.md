# Vehicle operation

## Request to intervene

Request to intervene (RTI) is a feature that requires the operator to switch to manual driving mode. It is also called Take Over Request (TOR).
AD API defines {{ link_ad_api('/api/fail_safe/rti_state') }} for RTI, but this API is not released yet, and the default implementation does not provide it.
For now, assume that manual driving is requested if the MRM state is not `NORMAL`.
See [fail-safe](../features/fail-safe.md) for details.

## Request to cooperate

Request to cooperate (RTC) is a feature that the operator supports the decision in autonomous driving mode.
Autoware usually drives the vehicle using its own decisions, but the operator may prefer to make their own decisions in complex situations.
Since RTC only overrides the decision and does not need to change operation mode, the vehicle can continue autonomous driving, unlike RTI.
The cooperation APIs are not released yet, and the default implementation does not provide them.
See [cooperation](../features/cooperation.md) for details.

## Request a minimal risk maneuver

An application can ask Autoware to bring the vehicle to a safe state, for example when the application detects a problem that Autoware cannot see.
Send the request with {{ link_ad_api('/api/fail_safe/mrm_request/send') }}.
See [fail-safe](../features/fail-safe.md) for details.
