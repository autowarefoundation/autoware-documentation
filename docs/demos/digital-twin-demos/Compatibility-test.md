# Compatibility test

The compatibility test tool is used to verify whether a digital-twin simulator works correctly with Autoware.

## `simulator_compatibility_test` package

- This tool is built on **ROS 2 Humble**.
- As of **Jan 26th, 2023**, it has been tested only with [**MORAI SIM: Drive Demo**](MORAI_Sim-tutorial.md).
- For full source code and instructions, see:
  [autoware_tools/simulator/simulator_compatibility_test](https://github.com/autowarefoundation/autoware_tools/tree/main/simulator/simulator_compatibility_test#simulator_compatibility_test)

!!! note

    The test suite includes a set of manual and automated test cases that check whether a simulator correctly publishes and receives the Autoware-required messages (e.g., control mode, gear, velocity, steering, turn indicators, hazard lights).
    These tests verify bidirectional communication by sending control commands and checking the simulator’s reported status.

!!! info

    Simulators other than MORAI SIM can also use the **common manual tests** (`test_sim_common_manual_testing`).

!!! tip

    Developers may extend these by creating simulator-specific automated versions similar to the MORAI SIM test set.
