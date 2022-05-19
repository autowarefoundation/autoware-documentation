# Scenario test simulation

Before running the test, please follow the [instruction](installation.md) about installation and building of Autoware Universe and Scenario_simulator_v2

1. Move to project directory, where the project is build.

2. Source the workspace setup script
   ```bash
   source install/setup.bash
   ```
3. Run 

   ```bash
   ros2 launch scenario_test_runner scenario_test_runner.launch.py \
   architecture_type:=awf/universe \
   record:=false \
   scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml' \
   sensor_model:=sample_sensor_kit \
   vehicle_model:=sample_vehicle
   ``` 
