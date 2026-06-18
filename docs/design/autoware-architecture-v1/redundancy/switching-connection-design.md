# Connection Design Between the Switching Function and Autoware

## Purpose

This page defines the high-level design for connecting an external ECU switching function (External Switching Function) to Autoware.
The scope covers responsibility boundaries between vehicle behavior selection nodes running on both Main and Sub ECUs and the node that bridges to the switching function.

## Overall Architecture

This section first distinguishes between elements that already exist in the single-ECU configuration and elements added for redundancy.

### Existing Elements in the Single-ECU Configuration (Prerequisites)

The following elements are pre-existing MRM switching components. They are not newly introduced for ECU redundancy.

- Nodes
  - `command_mode_decider`
  - `command_mode_switcher`
  - `aggregator`
- Topics
  - `command_mode_request`
  - `command_mode_status`
  - `command_mode_availability`

Roles of existing elements:

- `command_mode_decider`: Generates behavior-switch requests (`command_mode_request`)
- `command_mode_switcher`: Determines which behavior is enabled based on requests and current state
- `aggregator`: Aggregates per-command-mode executability and publishes `command_mode_availability`
- `command_mode_request`: Represents behavior-switch requests
- `command_mode_status`: Represents the current state of each command mode
- `command_mode_availability`: Represents availability of each command mode

### Elements Added for Redundancy

- Added node: `redundancy_switcher_interface`
- Added topic: `active_control_unit`

`redundancy_switcher_interface` connects to the external ECU switching function and shares switching results with existing MRM nodes via `active_control_unit`.
This enables behavior selection with ECU switching while minimizing changes to the existing MRM design.

### Nodes Launched on Each ECU

Both Main ECU and Sub ECU launch the following:

- Existing MRM nodes: `command_mode_decider` / `command_mode_switcher` / `aggregator`
- Added node: `redundancy_switcher_interface`

In addition, `command_mode_status` and `command_mode_availability` are shared between ECUs to maintain decision consistency across both systems.

## High-Level Connection Diagram

![ECU switching architecture](image/ecu-switching-architecture.drawio.svg)

## Details of Added Elements

### `redundancy_switcher_interface` (Added Node)

`redundancy_switcher_interface` is a bridge node placed between existing MRM nodes and the external ECU switching function.

#### Connection to Existing Nodes

- Receives `command_mode_request` from `command_mode_decider` and converts it to an ECU-switching request when needed
- Passes `active_control_unit` received from the external switching function to `command_mode_switcher`

#### Connection to the External ECU Switching Function

- Forwards Autoware-side requests (for example, MRM requests and switch reset requests) to the external interface
- Receives switching results from the external switching function and normalizes them into a representation usable by Autoware

#### Design Intent

The switching decision/execution logic is not embedded into existing MRM nodes.
Instead, it is centralized in `redundancy_switcher_interface`, making responsibility boundaries explicit.

#### Plugin Architecture (Interchangeable Switching Function Integration)

The connection between `redundancy_switcher_interface` and the external switching function is implemented through replaceable plugins.
Each plugin implements:

- Interface connectivity to a target switching function
- Conversion from switching-function-specific protocols to Autoware’s common abstraction

This approach keeps core Autoware node responsibilities stable while isolating switching-function-specific differences inside plugins.

### `active_control_unit` (Added Topic)

`active_control_unit` is the key topic introduced to propagate ECU switching results inside Autoware.

- Meaning: Indicates which control unit (Main/Sub) is currently Active
- Publisher: `redundancy_switcher_interface`
- Consumer: `command_mode_switcher`, for `network gate` Open/Closed control

`command_mode_switcher` control examples:

- Main ECU is Active: Main-side `network gate = Open`, Sub-side `Closed`
- Sub ECU is Active: Sub-side `network gate = Open`, Main-side `Closed`

This allows existing MRM behavior-selection logic to remain intact while incorporating ECU switching outcomes.

If `active_control_unit` is undefined, inconsistent, or empty, the system treats it as a transient or abnormal switching state,
and transitions to safe-side behavior (restricted executable modes and upstream notification).

## Information Flow

1. `command_mode_status` and `command_mode_availability` are continuously shared between ECUs to maintain consistency.
2. `command_mode_decider` generates a switching request.
3. `redundancy_switcher_interface` forwards the request to the external ECU switching function.
4. The external switching function returns active control unit information, which `redundancy_switcher_interface` normalizes into `active_control_unit`.
5. `redundancy_switcher_interface` provides `active_control_unit` to `command_mode_switcher`.
6. `command_mode_switcher` updates the `network gate` (Open/Closed) and controls executable behavior accordingly.

## Constraints and Assumptions

- This page covers high-level design; detailed topic/service definitions and state machine implementation are out of scope.
- Switching-function internals and hardware fault detection are responsibilities of the external function side.
- Communication-method differences across implementations are absorbed by plugins.
