# Cooperation API

- {{ link_ad_api('/api/planning/velocity_factors') }}
- {{ link_ad_api('/api/planning/steering_factors') }}
- {{ link_ad_api('/api/planning/cooperation/set_decisions') }}
- {{ link_ad_api('/api/planning/cooperation/set_default') }}
- {{ link_ad_api('/api/planning/cooperation/get_default') }}

## Description

Some planning modules can receive the operator's decision and reflect it in their behavior.
These modules have their own decisions, but use the the merged decision of theirs and operator's.
The operator can check the module's decision and override the decision if necessary.

![cooperation-architecture](./docs/cooperation-architecture.drawio.svg)

## Decisions

The modules that support cooperation have their own decisions that is either deactivate or activate.
These are provided as cooperation status in velocity factors or steering factors.
Its meaning depends on the module and is shown in the table below.
The merged decision will also be either of these, and the module will decide the behavior using it.

| Factor Type | Deactivate    | Activate        |
| ----------- | ------------- | --------------- |
| velocity    | stop          | pass            |
| steering    | keep the path | change the path |

The operator's decision is either deactivate, activate, autonomous, or undecided.
If the operator selects deactivate or activate, the module's decision is ignored and the operator's is used instead.
If the operator selects autonomous, the module's decision is used.
The undecided is the initial state of the operator's decision and is evaluated as the default decision when merging.
The operator can also override the default decision to use when undecided.

![cooperation-state](./docs/cooperation-state.drawio.svg)
