<!-- cspell:ignore EPDMS -->

# Assessment of safety and benchmarks

This page lists the tools that check and measure the planners and the learned models of Autoware today. It describes only work that exists, with links to the source. A common benchmark for all generators of Architecture 2.0 does not exist yet.

The tools are in three groups:

<div class="aw-features aw-features--small" markdown>

- :fa-cl-s fa-shield-halved: **Checks at run time**

  Nodes that check each trajectory while Autoware drives, and stop or reject unsafe output.

- :fa-cl-s fa-gauge-high: **Metrics**

  Nodes and tools that compute metrics, live or from recorded data.

- :fa-cl-s fa-road: **Simulation tests**

  Simulators and scenario frameworks for closed-loop tests.

</div>

## Checks at run time

| Package                                                                                                                                       | What it does                                                                                                                                                                                                                                                 | Status in Autoware 2.0.0                        |
| :-------------------------------------------------------------------------------------------------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------- |
| [`autoware_planning_validator`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/planning_validator)              | Checks the trajectory before control uses it, for example the latency, the trajectory shape, and collisions at intersections and from the rear.                                                                                                              | Runs with both values of `planning_setting`     |
| [`autoware_trajectory_modifier`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_modifier)   | Its modifier plugins add stop points for obstacles and traffic lights to the candidates of the Diffusion Planner.                                                                                                                                            | Runs with `planning_setting:=diffusion_planner` |
| [`autoware_trajectory_validator`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_validator) | The safety gate of the selector. Its plugins reject candidates that break the vehicle limits, cross uncrossable map boundaries, or break traffic light and crosswalk rules. A plugin can also run in shadow mode: it reports its result but does not reject. | Not launched                                    |
| [`autoware_trajectory_ranker`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/planning/autoware_trajectory_ranker)       | Scores the candidates from the risk level that the validator gives, the source of each candidate, and optional quality metrics for comfort, progress, and consistency.                                                                                       | Not launched                                    |

The [Generator-Selector framework](detailed-architectural-interface.md#selector) page describes how the selector uses the validator and the ranker.

## Metrics

### Online evaluators

The packages in the [`evaluator/`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/evaluator) folder of Autoware Universe compute metrics while Autoware runs:

- `autoware_planning_evaluator`: metrics of the planning quality. The planning launch file always starts it. With `planning_setting:=diffusion_planner`, it reads the output of the Diffusion Planner.
- `autoware_control_evaluator`: metrics of the control quality. The control launch file starts it when `launch_control_evaluator` is `true`, which is the default.
- `autoware_perception_online_evaluator`: metrics of the perception output. It starts when `use_perception_online_evaluator` is `true`. The default is `false`.
- `autoware_localization_evaluator` and `autoware_kinematic_evaluator`: metrics of the localization and of the vehicle motion.
- `autoware_scenario_simulator_v2_adapter`: converts the metrics to messages that scenario_simulator_v2 can use as pass and fail conditions.

### Offline evaluation of planners

[`autoware_planning_data_analyzer`](https://github.com/autowarefoundation/autoware_tools/tree/main/planning/autoware_planning_data_analyzer) in `autoware_tools` evaluates trajectories from rosbag data, without a simulator (open-loop evaluation). It computes trajectory errors, for example the average displacement error (ADE) and the final displacement error (FDE). It also computes the Extended Predictive Driver Model Score (EPDMS), a score of comfort, progress, and safety.

### Benchmarks of learned models

[AutoE2E](https://github.com/autowarefoundation/auto_e2e) publishes an open-loop benchmark of open-weights E2E driving models on the KITScenes-Multimodal validation set. It compares the ADE and the FDE at 3 seconds. The community benchmark runs in [AutoE2E issue 210](https://github.com/autowarefoundation/auto_e2e/issues/210).

## Simulation tests

| Tool                                                                                                                                 | What it does                                                                                                                                                                                                                                                            |
| :----------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2)                                                              | Runs OpenSCENARIO scenarios with Autoware in closed loop. The `simulator.repos` file of Autoware 2.0.0 pins version 22.0.0. The [scenario test simulation](../../../demos/scenario-simulation/scenario-simulator/scenario-test-simulation.md) page shows how to run it. |
| [driving_log_replayer_v2](https://github.com/tier4/driving_log_replayer_v2)                                                          | Replays recorded data through Autoware and evaluates the result. The [Driving Log Replayer](../../../demos/scenario-simulation/driving-log-replayer.md) page gives the build and run steps.                                                                             |
| [`autoware_sample_scenarios`](https://github.com/autowarefoundation/autoware_sample_scenarios)                                       | A minimal sample OpenSCENARIO file for the sample planning map.                                                                                                                                                                                                         |
| [`autoware_carla_interface`](https://github.com/autowarefoundation/autoware_universe/tree/0.53.1/simulator/autoware_carla_interface) | Connects Autoware to the CARLA simulator.                                                                                                                                                                                                                               |
| [`autoware_carla_scenario`](https://github.com/autowarefoundation/autoware_carla_scenario)                                           | A scenario test framework for Autoware on CARLA. It evaluates pass and fail conditions and records the results and a trajectory log of each run.                                                                                                                        |
| [TeraSim](https://github.com/autowarefoundation/TeraSim)                                                                             | A fork of TeraSim for co-simulation of Autoware, CARLA, and TeraSim. TeraSim makes naturalistic and adversarial background traffic to find rare unsafe events.                                                                                                          |

The design discussion [CARLA × Autoware integration architecture for closed-loop evaluation](https://github.com/orgs/autowarefoundation/discussions/7277) proposes how these CARLA tools fit together.
