# Scenario test simulation

!!! note

    Running the Scenario Simulator requires some additional steps on top of building and installing Autoware, so make sure that [Scenario Simulator installation](installation.md) has been completed first before proceeding.

## Running steps

1. Move to the workspace directory where Autoware and the Scenario Simulator have been built.

2. Source the workspace setup script:

   ```bash
   source install/setup.bash
   ```

3. Run the simulation:

   ```bash
   ros2 launch scenario_test_runner scenario_test_runner.launch.py \
     architecture_type:=awf/universe \
     record:=false \
     scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml' \
     sensor_model:=sample_sensor_kit \
     vehicle_model:=sample_vehicle \
     rviz_config:=$($(ros2 pkg prefix autoware_launch)/share/autoware_launch/rviz/scenario_simulator.rviz)
   ```

![scenario_test_runner](images/scenario_test_runner.png)

[Reference video tutorials](https://user-images.githubusercontent.com/102840938/206996920-758b62ae-270a-497c-8a72-f9e4867df695.mp4)
