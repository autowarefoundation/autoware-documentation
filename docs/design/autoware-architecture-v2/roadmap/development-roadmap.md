# Development roadmap

This page gives the status of the work on Architecture 2.0 at the Autoware 2.0.0 release, with the sources. It does not give dates for future work. The plans and their status change in the GitHub issues and discussions that this page links to.

## Generator-selector migration

The migration to the generator-selector framework follows the plan in [Introducing Generator + Selector Architecture for Modular Trajectory Planning](https://github.com/orgs/autowarefoundation/discussions/6301). The issue [Enable generator-selector planning framework](https://github.com/autowarefoundation/autoware/issues/6292) tracks the work.

| Phase | Plan                                                                    | Status in Autoware 2.0.0                                                                                                                                                                                                                                                       |
| :---- | :---------------------------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0     | Add the new messages                                                    | Done. `CandidateTrajectories` and the related messages are in `autoware_internal_planning_msgs`.                                                                                                                                                                               |
| 1     | Publish the old and the new trajectory topics                           | Done. The old topic is not in 2.0.0 (see Phase 6).                                                                                                                                                                                                                             |
| 2     | The controller uses only `/planning/trajectory`                         | Done. The trajectory follower, the control validator, and the vehicle command gate read `/planning/trajectory`.                                                                                                                                                                |
| 3     | Add the generator                                                       | Done. The Diffusion Planner and `autoware_trajectory_modifier` run with `planning_setting:=diffusion_planner`.                                                                                                                                                                 |
| 4     | Add the selector                                                        | The `autoware_trajectory_selector` package is in Autoware Universe, but the launch files do not start it.                                                                                                                                                                      |
| 5     | Enable the generator and the selector as launch options                 | Partly done. The `planning_setting` launch argument selects the generator. The launch file has a placeholder for the selector.                                                                                                                                                 |
| 6     | Integrate the full modular pipeline and remove the old trajectory topic | Partly done. No package or launch file of the 2.0.0 workspace uses the old topic `/planning/scenario_planning/trajectory`. The selector does not run, so the full pipeline is not in 2.0.0. In March 2026, the issue reported a prototype that was in tests on a real vehicle. |

The original plan gave dates in 2025 for each phase. The work took longer. In March 2026, the [comment on the migration issue](https://github.com/autowarefoundation/autoware/issues/6292#issuecomment-4031693678) reported that Phase 5 passed its tests on a real vehicle, and that a large change of the launch configuration was in progress. The issue [New launch package: autoware_launch_v2](https://github.com/autowarefoundation/autoware/issues/6721) and the [`autoware_launch_v2`](https://github.com/autowarefoundation/autoware_launch_v2) repository track this launch work.

## Steps of the stack architecture

The [Autonomous driving stack architecture](autonomous-driving-stack-architecture.md) page describes four steps from the robotics stack to a learned hybrid stack. This table gives the status of each step at the Autoware 2.0.0 release.

| Step                                    | Status in Autoware 2.0.0                                                                                                                                                 |
| :-------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Step 1: learned planner                 | In the release. You can select the Diffusion Planner with `planning_setting:=diffusion_planner`.                                                                         |
| Step 2: learned perception and planning | Not in the release.                                                                                                                                                      |
| Step 3: learned driving                 | `autoware_tensorrt_vad` is in Autoware Universe, but the default launch files do not start it. Other E2E models are in separate repositories of the Autoware Foundation. |
| Step 4: learned hybrid approach         | Not in the release. The roadmap marks it as under development by industry.                                                                                               |

## Where the work is discussed

- [GitHub Discussions of the Autoware Foundation](https://github.com/orgs/autowarefoundation/discussions): design proposals, announcements, and the notes of the working group meetings.
- [AutoE2E onboarding guide](https://github.com/autowarefoundation/auto_e2e/blob/main/ONBOARDING.md): AutoE2E is developed in the Autoware Robotaxi working group. This guide tells how to join its weekly meetings.
- [E2E + Rule-Based Hybrid Autonomous Driving Platform on Autoware](https://github.com/orgs/autowarefoundation/discussions/6915): a proposal for a hybrid of an E2E model and a rule-based safety layer.
