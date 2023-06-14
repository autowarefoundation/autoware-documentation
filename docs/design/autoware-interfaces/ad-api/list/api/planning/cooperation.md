# Cooperation API

- {{ link_ad_api('/api/planning/velocity_factors') }}
- {{ link_ad_api('/api/planning/steering_factors') }}
- {{ link_ad_api('/api/planning/cooperation/set_commands') }}
- {{ link_ad_api('/api/planning/cooperation/set_defaults') }}
- {{ link_ad_api('/api/planning/cooperation/get_defaults') }}

## Description

Some planning modules can receive the operator's decision and reflect it in their behavior.
These modules have their own decisions, but use the the merged decision of theirs and operator's.
The operator can check the module's decision and override the decision if necessary.
If the module supports this feature, [cooperation status](../../../types/autoware_adapi_v1_msgs/msg/CooperationStatus.md) is provided in [velocity factors or steering factors](./index.md).

![cooperation-architecture](./docs/cooperation-architecture.drawio.svg)

## Scene ID

The cooperation status contains an ID to distinguish scenes.
This ID is generated for each scene that requires decision and doesn't change until the scene is completed or canceled.
To set the operator's decision, it needs to specify the ID by selecting the target scene from the cooperation status.
In practice, the application can hides the specification of the ID and provides an abstracted interface to the operator.

## Decisions

The modules that support cooperation have their own decisions that is either deactivate or activate.
Its meaning depends on the module and is shown in the table below.
The merged decision will also be either of these, and the module will decide the behavior using it.

| Factor Type | Deactivate    | Activate        |
| ----------- | ------------- | --------------- |
| velocity    | stop          | pass            |
| steering    | keep the path | change the path |

The operator's decision is either deactivate, activate, autonomous, or undecided.
If the operator selects deactivate or activate, the module's decision is ignored and the operator's is used instead.
If the operator selects autonomous, the module's decision is used.
The undecided is the initial state of the operator's decision.
If the operator's decision is undecided, the default decision is used instead when merging decisions.
Default decisions are set for each module type, and the operator can also select it.

![cooperation-state](./docs/cooperation-state.drawio.svg)
