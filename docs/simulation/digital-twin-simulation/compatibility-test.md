# Compatibility test

The compatibility test tool is used to verify whether a digital-twin simulator works correctly with Autoware.

## `simulator_compatibility_test` package

- The package is part of [autoware_tools](https://github.com/autowarefoundation/autoware_tools). Autoware 2.0.0 uses `autoware_tools` 0.8.0, which `repositories/tools.repos` imports.
- It is a ROS 2 Python package. The test cases use `pytest` and `rclpy`.
- As of January 26, 2023, it had been tested only with [**MORAI SIM: Drive Demo**](MORAI_Sim-tutorial.md).
- For full source code and instructions, see:
  [autoware_tools/simulator/simulator_compatibility_test](https://github.com/autowarefoundation/autoware_tools/tree/main/simulator/simulator_compatibility_test#simulator_compatibility_test)

!!! note

    The test suite includes a set of manual and automated test cases that check whether a simulator correctly publishes and receives the Autoware-required messages (e.g., gear, velocity, steering, turn indicators, hazard lights).
    These tests verify bidirectional communication by sending control commands and checking the simulator’s reported status.

!!! info

    Simulators other than MORAI SIM can also use the **common manual tests** (`test_sim_common_manual_testing`).

!!! tip

    Developers may extend these by creating simulator-specific automated versions similar to the MORAI SIM test set.

## Test sets

The package has three folders of tests:

- `test_base`: shared methods that the other test sets use.
- `test_sim_common_manual_testing`: the basic tests. Any simulator can use them, but they have no test automation. You start the simulator and load a map and the ego vehicle yourself.
- `test_morai_sim`: an automated version of the common tests for MORAI SIM: Drive. It contains code that is specific to MORAI SIM: Drive.

## Test cases

Each test case sends a command to the simulator and reads the status that the simulator reports back. The test passes if the reported status matches the command.

| Test file                                    | What the test does                                                                                                    | Command topic                          | Status topic                             |
| -------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- | -------------------------------------- | ---------------------------------------- |
| `test_02_change_gear_and_report.py`          | Changes the gear to P, N, R, and D.                                                                                   | `/control/command/gear_cmd`            | `/vehicle/status/gear_status`            |
| `test_03_longitudinal_command_and_report.py` | Commands 5 m/s and 1 m/s², and checks after 10 seconds that the velocity is above 5 m/s. Then it commands 0 m/s.      | `/control/command/control_cmd`         | `/vehicle/status/velocity_status`        |
| `test_04_lateral_command_and_report.py`      | Sets the steering tire angle to +30 degrees (left) and to -30 degrees (right), and back to 0 degrees after each step. | `/control/command/control_cmd`         | `/vehicle/status/steering_status`        |
| `test_05_turn_indicators_cmd_and_report.py`  | Turns on the left and the right turn indicator, and then turns both off.                                              | `/control/command/turn_indicators_cmd` | `/vehicle/status/turn_indicators_status` |
| `test_06_hazard_lights_cmd_and_report.py`    | Turns the hazard lights on and off.                                                                                   | `/control/command/hazard_lights_cmd`   | `/vehicle/status/hazard_lights_status`   |

The commands use the `autoware_control_msgs` and `autoware_vehicle_msgs` message types, for example `autoware_control_msgs/msg/Control` and `autoware_vehicle_msgs/msg/GearCommand`. The package also has command line tools for the control mode, for example `control_mode_command` and `control_mode_report`. They use the `/control/command/control_mode_cmd` and `/vehicle/status/control_mode` topics.

## Run the common manual tests

1. Import `autoware_tools` into your Autoware workspace and build the package:

   ```bash
   cd ~/autoware
   vcs import src < repositories/tools.repos
   colcon build --packages-select simulator_compatibility_test
   source install/setup.bash
   ```

2. Start your simulator. Load a map and the ego vehicle.

3. Go to the folder of the common tests and run one test case, for example the gear test:

   ```bash
   cd src/tools/simulator/simulator_compatibility_test/test_sim_common_manual_testing
   python -m pytest test_02_change_gear_and_report.py
   ```

4. Make sure that the simulator shows the expected behavior, for example the gear changes to P, N, R, and D. If the simulator has a GUI, it shows the changes.

5. Make sure that the `pytest` output shows that the test passed.

Do steps 3 to 5 again for each test case.
