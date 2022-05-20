# Random test simulation

Note : Before running the test, please follow the [instruction](installation.md) about installation and building of Autoware Universe and Scenario_simulator_v2

## Running steps

1. Move to project directory, where the project is build.

2. Source the workspace setup script

   ```bash
   source install/setup.bash
   ```

3. Run

   ```bash
   ros2 launch random_test_runner random_test.launch.py \
   architecture_type:=awf/universe \
   sensor_model:=sample_sensor_kit \
   vehicle_model:=sample_vehicle
   ```

![random_test_runner](images/random_test_runner.png)

For more information about supported parameters, refer to the ![random_test_runner documentation].(<https://github.com/tier4/scenario_simulator_v2/blob/master/test_runner/random_test_runner/Readme.md#node-parameters>)
