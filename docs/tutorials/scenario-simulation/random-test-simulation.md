# Random test simulation

Before running the test, please follow the [instruction](installation.md) about installation and building of Autoware Universe and Scenario_simulator_v2

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
