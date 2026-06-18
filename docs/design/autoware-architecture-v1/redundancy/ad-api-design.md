---
hide:
	- navigation
	- toc
---

# AD API Design for Redundant ECUs

## Purpose

This page defines the high-level AD API design for a redundant ECU configuration.
It addresses three main design concerns during Main/Sub switching:

- Applying the same service request consistently to both Main and Sub systems
- Handling MRM recovery requests while hiding internal differences between redundant and non-redundant configurations
- Preventing API output collisions between Main and Sub

## Design Principles

Internal architectural differences (redundant vs. non-redundant) should not be exposed to external API users.
From the API perspective, recovery from MRM state back to autonomous operation should be handled through a consistent request interface.

To achieve this, the design introduces:

- Standardized service fan-out (`generic_service_divider`)
- Unified MRM reset orchestration (`mrm_reset_manager`)
- Active-ECU-based API output filtering (`redundancy_adapi_switcher`)

## Existing vs Added Elements

### Existing Elements (from the Single-ECU Configuration)

- `command_mode_decider`
  - Receives services such as `change_operation_mode` and `change_autoware_control`
  - For node-level responsibilities, see [switching-connection-design.md](switching-connection-design.md)
- `aggregator` diagnostic latch behavior
  - Once diagnostics become `diag ERROR`, the state is latched
  - Even if diagnostics later return to `diag OK`, the latch is not cleared automatically
  - Latch release requires an explicit service call after human safety confirmation

### Added Elements (for Redundant Configuration)

- `generic_service_divider`
- `mrm_reset_manager`
- `redundancy_adapi_switcher`

## Service API Architecture Diagram

![Service API architecture](image/service-api-architecture.drawio.svg)

[Open in draw.io for fullscreen]({{ "/design/autoware-architecture-v1/redundancy/image/service-api-architecture.drawio.svg" | drawio }})

## Service Fan-Out Design (`generic_service_divider`)

In Main/Sub switching, a key requirement is to apply the same request coherently to both ECUs.

For example, the following service calls to `command_mode_decider` must be applied to both Main and Sub:

- `change_operation_mode`
- `change_autoware_control`

`generic_service_divider` republishes one incoming service request to multiple clients.
This allows one request to be fanned out into ECU-specific calls (Main-targeted and Sub-targeted).

This is not limited to `command_mode_decider`.
Any service that must be applied simultaneously to both ECUs can be added via plugins.
For example, in a configuration where `ekf_localizer` is deployed on Sub ECU, a `trigger_node` service for initial pose setup can be added as a fan-out target.

## Unified MRM Reset Design (`mrm_reset_manager`)

### Background

MRM recovery involves both redundant-system resets and existing-system resets:

- ECU switching-function reset
  - When resuming autonomous driving by returning Main ECU to Active after a Main→Sub switch
  - When initializing the internal state of the ECU switching function itself
- `aggregator` diagnostic latch release
  - To avoid unintended automatic resume and require explicit operator judgment after safety confirmation

### API Unification

These internal reset paths are hidden behind a single external request: `/system/mrm/reset`.
`mrm_reset_manager` receives this request and translates it into:

- ECU switching reset request
- Diagnostic latch release request

Note: `/system/mrm/reset` is currently a design proposal and is not yet supported as an AD API endpoint.

### Initialization-Phase Control

`mrm_reset_manager` also controls initialization-phase enable/disable behavior.
Initialization is considered complete when:

- Localization is set
- Route is set
- Autoware Control is `on`

During initialization:

- `aggregator` latch behavior is turned `off` to avoid latching transient startup `diag ERROR`
- `redundancy_switcher_interface` is notified via `set_initializing` to suppress unnecessary ECU switching

After initialization completes, `set_initializing` is updated to enable normal behavior.

In summary, `mrm_reset_manager` serves two roles:

- API entry point for MRM-related external reset requests
- Orchestrator for enabling/disabling reset target functions according to initialization phase

## Minimum Sub-ECU API Set and Output Switching

When switching from Main to Sub, Sub ECU provides a minimum API subset required for failure-related HMI reporting:

- `/api/system/diagnostics/status`
- `/api/system/diagnostics/struct`
- `/api/fail_safe/mrm_state`
- `/api/system/heartbeat`

Sub ECU runs `default_adapi` for this limited scope.
(As a design option, emitting the full API set from both Main and Sub is also possible.)

To avoid simultaneous output collisions, `redundancy_adapi_switcher` applies output filtering:

- If Main ECU is Active: stop API source-topic output from Sub ECU
- If Sub ECU is Active: stop API source-topic output from Main ECU

## Information Flow (Summary)

1. An external service request is received and fanned out by `generic_service_divider` to Main and Sub.
2. When ECU switching is relevant, `redundancy_switcher_interface` handles switching-function integration.
3. An MRM recovery request (`/system/mrm/reset`) is received by `mrm_reset_manager` and translated into switching reset + diagnostic latch release requests.
4. During startup initialization, `mrm_reset_manager` controls latch behavior and switching-function enable state.
5. API outputs are gated by `redundancy_adapi_switcher` so only the Active ECU side is exposed.

## Constraints and Assumptions

- This page covers high-level design; detailed service definitions and implementation procedures are out of scope.
- Node/service names may change during implementation.
- This design assumes ECU switching integration defined in [switching-connection-design.md](switching-connection-design.md).
