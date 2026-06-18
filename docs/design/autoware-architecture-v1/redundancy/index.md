# Redundant ECU Configuration Design

## Purpose

This page defines the high-level design for operating Autoware in a redundant ECU configuration.
It focuses on cases where switching from Main ECU to Sub ECU is required due to software faults, hardware failures, or related conditions.

The design has two primary goals:

- Improve system safety by increasing tolerance against system and hardware failures
- Increase architectural flexibility for higher ASIL targets (including decomposition-oriented design options)

## Background and Motivation

In Autoware Architecture v1, redundancy was treated as future work.
In practical deployment, however, a single-ECU failure can directly stop autonomous driving functions,
so the architecture must improve operational continuity (availability) under ECU faults.

Redundant architecture is particularly effective for:

- Reducing single points of failure (SPOF) in primary compute platforms
- Improving operational continuity under fault conditions
- Expanding design options for safety and certification (ASIL) strategies

## Scope

### In scope

- Redundant architecture design based on a dual-ECU Main/Sub configuration
- Design of abstraction layers related to switching
- Coordination design between switching behavior and Autoware components
- Impact assessment and design updates for existing AD API behavior

### Out of scope

- Implementation/provision of the switching function itself
- Vehicle-specific physical hardware design
- Formal ISO 26262 work products
- Byzantine fault handling

## About the External Switching Function

In this design, the external mechanism that performs physical switching between Main and Sub ECUs is called the **External Switching Function**.
It is outside Autoware, and Autoware does not implement it (out of scope).

Autoware is designed with this external switching function as a prerequisite.
Requirements expected from the switching function are defined in [Requirements for the Switching Function (Prerequisites)](#requirements-for-the-switching-function-prerequisites).

## Requirements

### Requirements for the Switching Function (Prerequisites)

The following requirements must be satisfied by the external switching function to enable redundant operation in Autoware.
These are external requirements; Autoware functional requirements are defined assuming they are met.

| ID | Requirement | Notes |
|----|-------------|-------|
| EXT-01 | Switching behavior shall be deterministic | The same input shall always produce the same switching result |
| EXT-02 | Switching shall complete within a bounded time | A timeout upper bound must be defined |
| EXT-03 | A single ECU or network fault shall not cause failure of the switching function itself | Single faults must be contained and not propagated into the switching function |
| EXT-04 | Current switching state (Active/Standby assignment) shall be shared with both Main and Sub ECUs | ECU-side logic must be able to identify current role |
| EXT-05 | Switching requests from ECU side shall be accepted | ECU-originated switching triggers must be supported |
| EXT-06 | Exactly one ECU shall be Active at any time | Avoiding simultaneous output (split-brain) is the switching function’s responsibility |

### Functional Requirements for Autoware

Assuming EXT requirements are satisfied, Autoware shall fulfill the following:

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-01 | Autoware shall provide ECU identity and heartbeat information to the switching function | Required for ECU identification by the switching function |
| FR-02 | Main and Sub internal systems shall report local health information to the switching function and support switching-necessity judgment | Includes process status, communication status, and freshness of critical topics |
| FR-03 | Main and Sub ECUs shall detect and recognize abnormalities of the switching function (for example communication loss or functional outage) | Reduces risk of the switching function becoming an undetected SPOF |
| FR-04 | The architecture shall allow Sub ECU to become Active when Main ECU hardware fails | Covers hardware faults in addition to software faults |
| FR-05 | ECU switching results (Active side changes) shall be reflected in Autoware vehicle behavior selection (modes, MRM, etc.) | Ensures post-switch behavior continuity |
| FR-06 | Sub ECU shall continue to provide a minimum AD API capability even after ECU switching | Minimizes impact on external applications |

### Non-Functional Requirements

| ID | Requirement | Notes |
|----|-------------|-------|
| NFR-01 | Switching-related events and decision data shall be recorded as auditable logs | Required for safety analysis and debugging |

## Detailed Design Pages

Details beyond the high-level requirements are split into the following pages:

- [Connection Design Between the Switching Function and Autoware](switching-connection-design.md)
- [AD API Design for Redundant ECUs](ad-api-design.md)
