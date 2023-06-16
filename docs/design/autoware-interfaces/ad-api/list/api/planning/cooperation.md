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

The operator decision is actually either deactivate, activate, autonomous, or default.
If the operator selects deactivate or activate, the module decision is ignored and the operator decision is used instead.
If the operator selects autonomous, the module decision is used.
The default is the initial value of the operator decision.
If the operator decision is default, it is evaluated according to the default policy set for each module type.

The default policy is either deactivate or autonomous, and is initialized by system settings.
These meanings are the same as the operator decision.
The purpose of the default policy is to set whether the operator decision is required.
The deactivate policy is a defensive setting and not perform risky behavior until instructed by the operator.
The autonomous policy can perform both behavior without operator instruction.
Also, the operator can change the default policies.

![cooperation-state](./docs/cooperation-state.drawio.svg)

## Examples

This is an example of cooperation for lane change module. The behaviors by the combination of decisions are as follows.

| Operator decision | Default policy | Module decision | Description                                                                                                    |
| ----------------- | -------------- | --------------- | -------------------------------------------------------------------------------------------------------------- |
| deactivate        | -              | -               | The operator instructs to keep lane regardless the module decision. So it keeps lane by operator decision.     |
| activate          | -              | -               | The operator instructs to change lane regardless the module decision. So it changes lane by operator decision. |
| autonomous        | -              | deactivate      | The operator instructs to follow the module decision. So it keeps lane by module decision.                     |
| autonomous        | -              | activate        | The operator instructs to follow the module decision. So it changes lane by module decision.                   |
| default           | deactivate     | -               | The deactivate default policy is used because no operator instruction. So it keeps lane by deactivate policy.  |
| default           | autonomous     | deactivate      | The autonomous default policy is used because no operator instruction. So it keeps lane by module decision.    |
| default           | autonomous     | activate        | The autonomous default policy is used because no operator instruction. So it change lane by module decision.   |
