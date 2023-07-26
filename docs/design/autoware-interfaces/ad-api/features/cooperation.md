# Cooperation

## Related API

- {{ link_ad_api('/api/planning/velocity_factors') }}
- {{ link_ad_api('/api/planning/steering_factors') }}
- {{ link_ad_api('/api/planning/cooperation/set_commands') }}
- {{ link_ad_api('/api/planning/cooperation/set_policies') }}
- {{ link_ad_api('/api/planning/cooperation/get_policies') }}

## Description

Request to cooperate (RTC) is a feature that the operator supports the decision in autonomous driving mode.
The planning component manages each situation that requires decision as a scene.
Autoware usually drives the vehicle using its own decisions, but the operator may prefer to make their decisions in experiments and complex situations.

Each scene has an ID that doesn't change until the scene is completed or canceled.
The operator can override the decision of the target scene using this ID.
In practice, the user interface application can hides the specification of the ID and provides an abstracted interface to the operator.

For example, in the situation in the diagram below, change lanes twice and turn left at the intersection.
There are three scenes and each has a decision to change or keep the lane, turn left or wait.
Here Autoware decides not to change lanes a second time due to the obstacle, so the vehicle will stop there.

![cooperation-scenes](./cooperation/scenes.drawio.svg)

## Architecture

Modules that support RTC have the operator decision and cooperation policy in addition to the module decision as shown below.
These modules use the merged decision that is determined by these values when planning vehicle behavior.
See decisions section for details of these values.
The cooperation policy is used when there is no operator decision and has a default value set by the system settings.
If the module supports RTC, these information are available in [velocity factors or steering factors](./planning-factors.md) as [cooperation status](../types/autoware_adapi_v1_msgs/msg/CooperationStatus.md).

![cooperation-architecture](./cooperation/architecture.drawio.svg)

## Sequence

Here is an example sequence that overrides the scene decision to force a lane change. This assumes the second scene in the example situation given earlier.

1. A module creates a scene with generated ID BBBB when approaching a place where a lane change is needed.
2. The scene determines the module decision from the current situation.
3. The scene determines the merged decision. Since there is no operator decision, so the cooperation policy is used.
4. The scene plans the vehicle to keep the lane.
5. The scene sends a cooperation status.
6. The operator receives the cooperation status.
7. The operator sends a cooperation command.
8. The scene receives the cooperation command and update the operator decision.
9. The scene updates the module decision from the current situation.
10. The scene updates the merged decision. The received operator decision is used.
11. The scene plans the vehicle to change the lane.

## Decisions

There are four variables used for RTC, each of which takes the value shown in the table.

| Name               | Values                                 |
| ------------------ | -------------------------------------- |
| merged decision    | deactivate, activate                   |
| module decision    | deactivate, activate                   |
| operator decision  | deactivate, activate, autonomous, none |
| cooperation policy | required, optional                     |

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

![cooperation-state](./cooperation/state.drawio.svg)

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
