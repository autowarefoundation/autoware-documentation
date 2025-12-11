# Planning simulator demo with Autoware Launch GUI

This section provides a step-by-step guide on using the Autoware Launch GUI for planning simulations, offering an alternative to the command-line instructions provided in the Basic simulations section.

## Getting Started with Autoware Launch GUI

1. **Installation:** Ensure you have installed the Autoware Launch GUI. [Installation instructions](https://github.com/autowarefoundation/autoware-launch-gui#installation).

2. **Launching the GUI:** Open the Autoware Launch GUI from your applications menu.

   ![GUI screenshot for launching the GUI](images/launch-gui/launch_gui_main.png)

## Launching a Planning Simulation

### Lane Driving Scenario

1. **Set Autoware Path:** In the GUI, set the path to your Autoware installation.

   ![GUI_screenshot_for_setting_Autoware_path](images/launch-gui/launch_gui_setup.png)

2. **Select Launch File:** Choose `planning_simulator.launch.xml` for the lane driving scenario.

   ![GUI screenshot for selecting launch file](images/launch-gui/selecting_launch_file.png)

3. **Customize Parameters:** Adjust parameters such as `map_path`, `vehicle_model`, and `sensor_model` as needed.

   ![GUI screenshot for customizing parameters](images/launch-gui/customizing-parameters1.png)
   ![GUI screenshot for customizing parameters](images/launch-gui/customizing-parameters2.png)

4. **Start Simulation:** Click the launch button to start the simulation.

   ![GUI screenshot for starting simulation](images/launch-gui/starting_simulation.png)

5. **Any Scenario:** From here, you can follow the instructions in the [Planning Scenario Simulations](index.md#basic-simulations).

## Monitoring and Managing the Simulation

- **Real-Time Monitoring:** Use the GUI to monitor CPU/Memory usage and Autoware logs in real-time.
- **Profile Management:** Save your simulation profiles for quick access in future simulations.
- **Adjusting Parameters:** Easily modify simulation parameters on-the-fly through the GUI.
