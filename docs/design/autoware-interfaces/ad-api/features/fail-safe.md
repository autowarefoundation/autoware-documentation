# Fail-safe

## Related API

- {{ link_ad_api('/api/fail_safe/mrm_state') }}
- {{ link_ad_api('/api/fail_safe/rti_state') }}

## Description

This API manages the behavior related to the abnormality of the vehicle.
It provides the state of Request to Intervene (RTI), Minimal Risk Maneuver (MRM) and Minimal Risk Condition (MRC).
As shown below, Autoware has the gate to switch between the command during normal operation and the command during abnormal operation.
For safety, Autoware switches the operation to MRM when an abnormality is detected.
Since the required behavior differs depending on the situation, MRM is implemented in various places as a specific mode in a normal module or as an independent module.
The fail-safe module selects the behavior of MRM according to the abnormality and switches the gate output to that command.

![fail-safe-architecture](./fail-safe/architecture.drawio.svg)

## MRM state

The MRM state indicates whether MRM is operating and its current behavior.
This state also provides success or failure of the operation. Generally, MRM will switch to another behavior if it fails.

![mrm-state](./fail-safe/mrm-state.drawio.svg)

| State     | Description                                                |
| --------- | ---------------------------------------------------------- |
| NONE      | MRM is not operating.                                      |
| OPERATING | MRM is operating because an abnormality has been detected. |
| SUCCEEDED | MRM succeeded. The vehicle is in a safe condition.         |
| FAILED    | MRM failed. The vehicle is still in an unsafe condition.   |

There is a dependency between MRM behaviors. For example, it switches from a comfortable stop to a emergency stop, but not the other way around.
This is service dependent. Autoware supports the following transitions by default.

![mrm-behavior](./fail-safe/mrm-behavior.drawio.svg)

| State            | Description                                                               |
| ---------------- | ------------------------------------------------------------------------- |
| NONE             | MRM is not operating or is operating but no special behavior is required. |
| COMFORTABLE_STOP | The vehicle will stop quickly with a comfortable deceleration.            |
| EMERGENCY_STOP   | The vehicle will stop immediately with as much deceleration as possible.  |

## RTI state

The RTI state indicates whether RTI is requested. If for some reason autonomous driving cannot continue, Autoware will request a change to manual driving.
The following states are provided to properly handle RTI from multiple vehicles.

| State         | Description                                                                               |
| ------------- | ----------------------------------------------------------------------------------------- |
| NONE          | RTI is not requested.                                                                     |
| MRM_OPERATING | MRM is operating. Since MRC has not been achieved, immediate intervention is recommended. |
| MRM_COMPLETED | MRM is completed. Since MRC has been achieved, give priority to vehicles with urgency.    |
