# Evaluation

These pages describe how to measure the quality of Autoware in a simulation and with recorded data. Use them to find out how well a component works, and to check that a simulator is compatible with Autoware. The [simulation overview](../index.md) shows how to start each kind of simulation.

## Pages in this section

- [Localization evaluation in an urban environment](urban-environment-localization.md): a test of the NDT-based localization of Autoware on the [Istanbul Open Dataset](../../datasets/index.md#istanbul-open-dataset). The route has tunnels and bridges. The page compares the results with a ground truth and explains how to repeat the test.
- [Digital twin simulation: compatibility test](../digital-twin-simulation/compatibility-test.md): the `simulator_compatibility_test` package of `autoware_tools`. It checks that a simulator receives the vehicle commands of Autoware and reports the vehicle status back.

## Evaluator packages

The `evaluator` folder of [autoware_universe](https://github.com/autowarefoundation/autoware_universe/tree/main/evaluator) has nodes that calculate metrics while Autoware runs. You can use them in all kinds of simulation and on a real vehicle.

| Package                                                                                                                                                   | Purpose                                                                                                         |
| --------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| [`autoware_planning_evaluator`](https://autowarefoundation.github.io/autoware_universe/main/evaluator/autoware_planning_evaluator/)                       | Calculates metrics on the quality of planning. It publishes them in real time and saves them to a JSON file.    |
| [`autoware_control_evaluator`](https://autowarefoundation.github.io/autoware_universe/main/evaluator/autoware_control_evaluator/)                         | Calculates metrics on the quality of control and on the kinematics and position of the ego vehicle.             |
| [`autoware_localization_evaluator`](https://autowarefoundation.github.io/autoware_universe/main/evaluator/autoware_localization_evaluator/)               | Calculates metrics on the performance of localization.                                                          |
| [`autoware_perception_online_evaluator`](https://autowarefoundation.github.io/autoware_universe/main/evaluator/autoware_perception_online_evaluator/)     | Evaluates the output of perception online, without annotations.                                                 |
| [`autoware_kinematic_evaluator`](https://autowarefoundation.github.io/autoware_universe/main/evaluator/autoware_kinematic_evaluator/)                     | Calculates statistics of the velocity of the ego vehicle.                                                       |
| [`autoware_scenario_simulator_v2_adapter`](https://autowarefoundation.github.io/autoware_universe/main/evaluator/autoware_scenario_simulator_v2_adapter/) | Converts messages of Autoware, for example metrics, into `UserDefinedValue` messages for scenario_simulator_v2. |

The `tier4_metrics_rviz_plugin` package of [autoware_tools](https://github.com/autowarefoundation/autoware_tools/tree/main/evaluation/tier4_metrics_rviz_plugin) shows the output of `planning_evaluator` in an RViz panel.

## Related pages

- [Planning evaluation using scenarios](../../tutorials/others/planning-evaluation-using-scenarios.md): create scenarios and evaluate planning with Scenario Simulator and the Autoware Evaluator.
- [Defining temporal performance metrics on components](../../tutorials/others/defining-temporal-performance-metrics.md): choose the message rates and response times to measure.
- [An example procedure for adding and evaluating a new node](../../tutorials/others/an-example-procedure-for-adding-and-evaluating-a-new-node.md): evaluate a new node with a rosbag and in a simulator.
- [Evaluating the controller performance](../../tutorials/integrating-autoware/tuning-parameters-and-performance/evaluation/evaluating-controller-performance/index.md) and [evaluating the real-time performance](../../tutorials/integrating-autoware/tuning-parameters-and-performance/evaluation/evaluating-real-time-performance/index.md) of Autoware on your vehicle.
- [Driving Log Replayer](../scenario-simulation/driving-log-replayer.md): evaluate Autoware with recorded data.
