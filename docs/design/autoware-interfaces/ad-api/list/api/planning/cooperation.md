# Cooperation API

- {{ link_ad_api('/api/planning/velocity_factors') }}
- {{ link_ad_api('/api/planning/steering_factors') }}
- {{ link_ad_api('/api/planning/cooperation/set_commands') }}
- {{ link_ad_api('/api/planning/cooperation/set_defaults') }}
- {{ link_ad_api('/api/planning/cooperation/get_defaults') }}

## Description

Some planning modules can receive the operator decision and reflect it in their behavior.
These modules have their own decisions, but their behavior is based on the operator decisions.
The operator can check the module decision and change the operator decision if necessary.
If the module supports this feature, [cooperation status](../../../types/autoware_adapi_v1_msgs/msg/CooperationStatus.md) is provided in [velocity factors or steering factors](./index.md).

![cooperation-architecture](./docs/cooperation-architecture.drawio.svg)

## Scene ID

The cooperation status contains an ID to distinguish scenes.
This ID is generated for each scene that requires decision and doesn't change until the scene is completed or canceled.
To set the operator decision, it needs to specify the ID by selecting the target scene from the cooperation status.
In practice, the application can hides the specification of the ID and provides an abstracted interface to the operator.

## Decisions

The module decision is either deactivate or activate. The operator decision is also evaluated in one of these.
Its meaning depends on the module and is shown in the table below.
These decisions are designed to assign behavior to activate that is considered high risk.

| Factor Type     | Deactivate    | Activate        |
| --------------- | ------------- | --------------- |
| velocity (stop) | stop          | pass            |
| steering (path) | keep the path | change the path |

The operator decision is actually either deactivate, activate, autonomous, or none.
If the operator selects deactivate or activate, the module decision is ignored and the operator decision is used instead.
If the operator selects autonomous, the module decision is used.
The none is the initial value of the operator decision and means that the operator has not selected any decision.
If the operator decision is none, it is evaluated according to the cooperation policies set for each module type.

The cooperation policy is either required or optional, and is initialized by system settings.
The required policy evaluates none decision as deactivate to minimize risk.
Therefore, the operator decision is required to continue driving.
The optional policy evaluates none decision as autonomous to continue driving.
This allows the vehicle to drive without the operator decision.
The cooperation policies can also be changed by the operator.
Note that this setting is common per module, so changing it will affect all scenes in the same module.

![cooperation-state](./docs/cooperation-state.drawio.svg)

## Examples

This is an example of cooperation for lane change module. The behaviors by the combination of decisions are as follows.

| Operator decision | Policy   | Module decision | Description                                                                                                    |
| ----------------- | -------- | --------------- | -------------------------------------------------------------------------------------------------------------- |
| deactivate        | -        | -               | The operator instructs to keep lane regardless the module decision. So it keeps lane by operator decision.     |
| activate          | -        | -               | The operator instructs to change lane regardless the module decision. So it changes lane by operator decision. |
| autonomous        | -        | deactivate      | The operator instructs to follow the module decision. So it keeps lane by module decision.                     |
| autonomous        | -        | activate        | The operator instructs to follow the module decision. So it changes lane by module decision.                   |
| none              | required | -               | The required policy is used because no operator instruction. So it keeps lane the same as deactivate.          |
| none              | optional | deactivate      | The optional policy is used because no operator instruction. So it keeps lane by module decision.              |
| none              | optional | activate        | The optional policy is used because no operator instruction. So it change lane by module decision.             |
