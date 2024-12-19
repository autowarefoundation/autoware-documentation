# Planning simulation

## Preparation

Download and unpack a sample map.

- You can also download [the map](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view?usp=sharing) manually.

```bash
gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
```

!!! Note

    Sample map: Copyright 2020 TIER IV, Inc.

Check if you have `~/autoware_data` folder and files in it.

```bash
$ cd ~/autoware_data
$ ls -C -w 30
image_projection_based_fusion
lidar_apollo_instance_segmentation
lidar_centerpoint
tensorrt_yolo
tensorrt_yolox
traffic_light_classifier
traffic_light_fine_detector
traffic_light_ssd_fine_detector
yabloc_pose_initializer
```

If not, please, follow [Manual downloading of artifacts](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts).

Change the [maximum velocity](https://github.com/autowarefoundation/autoware_launch/blob/c03bd4bdb70117efffc328e5fe6e57426f169b3b/autoware_launch/config/planning/scenario_planning/common/common.param.yaml#L3), that is 15km/h by default.

## Basic simulations

!!! info "[Using Autoware Launch GUI](#using-autoware-launch-gui)"

    If you prefer a graphical user interface (GUI) over the command line for launching and managing your simulations, refer to the `Using Autoware Launch GUI` section at the end of this document for a step-by-step guide.

### Lane driving scenario

#### 1. Launch Autoware

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

!!! warning

    Note that you cannot use `~` instead of `$HOME` here.

    If `~` is used, the map will fail to load.

![after-autoware-launch](https://github.com/vish0012/autoware-documentation/blob/3d3ed61f1a835fba0acc94edc9d46c2de441a260/docs/tutorials/ad-hoc-simulation/images/planning/others/first-overview.png)

If you encounter a situation where the simulation is running but the view is not visible in Autoware like below, please follow these steps:

![after-autoware-launch-error](https://github.com/vish0012/autoware-documentation/blob/27844480495b187cca173191f48a4d9a14ba6c49/docs/tutorials/ad-hoc-simulation/images/planning/others/first%20error%20view.png)

Double-Click on 'TopDownOrtho' View
Navigate to the Views section on the right panel and double-click on the TopDownOrtho option to properly display the simulation.

Manually Select the View (If Needed)
If the view still does not appear, try the following:

Go to the Views section.
Select a different view (e.g., ThirdPersonFollower) and then re-select TopDownOrtho.
By following these steps, the simulation view should load correctly.

#### 2. Set an initial pose for the ego vehicle

![set-initial-pose](https://github.com/vish0012/autoware-documentation/blob/3d3ed61f1a835fba0acc94edc9d46c2de441a260/docs/tutorials/ad-hoc-simulation/images/planning/others/After%20Initializing%202D%20Pose.png)

a) Click the `2D Pose estimate` button in the toolbar, or hit the `P` key.

b) In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the initial pose. An image representing the vehicle should now be displayed.

!!! warning

    Remember to set the initial pose of the car in the same direction as the lane.

    To confirm the direction of the lane, check the arrowheads displayed on the map.

#### 3. Set a goal pose for the ego vehicle

a) Click the `2D Goal Pose` button in the toolbar, or hit the `G` key.

b) In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the goal pose. If done correctly, you will see a planned path from initial pose to goal pose.

![set-goal-pose](https://github.com/vish0012/autoware-documentation/blob/428c5b41b7c4d91cb17275b496627b2eb2fa77cd/docs/tutorials/ad-hoc-simulation/images/planning/others/goal%20pose.png)

#### 4. Start the ego vehicle

Now you can start the ego vehicle driving by clicking the `AUTO` button on `OperationMode` in `AutowareStatePanel`.
Alteratively, you can manually start the vehicle by running the following command:

```bash
source ~/autoware/install/setup.bash
ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {}
```

After that, you can see `AUTONOMOUS` sign on `OperationMode` and `AUTO` button is grayed out.

![start-driving](https://github.com/vish0012/autoware-documentation/blob/a8fbf0ea404bed9126256c1a4eb09e9744c6ab35/docs/tutorials/ad-hoc-simulation/images/planning/others/start%20goal%20pose.png)

### Parking scenario

1. Set an initial pose and a goal pose, and engage the ego vehicle.

   ![after-set-goal-pose](https://github.com/vish0012/autoware-documentation/blob/0914e979e43bb257ebcfa5b4c984a7acfb17785e/docs/tutorials/ad-hoc-simulation/images/planning/others/parking%20initial%20pose.png)

2. When the vehicle approaches the goal, it will switch from lane driving mode to parking mode.
3. After that, the vehicle will reverse into the destination parking spot.

   ![parking-maneuver](https://github.com/vish0012/autoware-documentation/blob/1bfab84885a0b932d45924ced8ac42bf390364b9/docs/tutorials/ad-hoc-simulation/images/planning/others/parking%20moving%201.png)
   ![parking-maneuver 2](https://github.com/vish0012/autoware-documentation/blob/1bfab84885a0b932d45924ced8ac42bf390364b9/docs/tutorials/ad-hoc-simulation/images/planning/others/parking%20moving%202.png)

### Pull out and pull over scenario

1. In a pull out scenario, set the ego vehicle at the road shoulder.

   ![pullover-pullout](https://github.com/vish0012/autoware-documentation/blob/3581b0b9b4bbdd680a3e85407fd24ee9a3a4cd3a/docs/tutorials/ad-hoc-simulation/images/planning/others/pull%20out%20scenario.png)

2. Set a goal and then engage the ego vehicle.

3. In a pull over scenario, similarly set the ego vehicle in a lane and set a goal on the road shoulder.

### Lane change scenario

1. Download and unpack Nishishinjuku map.

   ```bash
   gdown -O ~/autoware_map/ 'https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip'
   unzip -d ~/autoware_map ~/autoware_map/nishishinjuku_autoware_map.zip
   ```

2. Launch autoware with Nishishinjuku map with following command:

   ```bash
   source ~/autoware/install/setup.bash
   ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/nishishinjuku_autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
   ```

   ![open-nishishinjuku-map](https://github.com/vish0012/autoware-documentation/blob/760b51f1b543bdedf7a523ede91aca7f3f446872/docs/tutorials/ad-hoc-simulation/images/planning/others/nishi%20shinjuku%20map%20.png)

3. Set an initial pose and a goal pose in adjacent lanes.

   ![set-position-and-goal](https://github.com/vish0012/autoware-documentation/blob/7cc6b96690ae21f933ba329285e8fb1b06d20d7a/docs/tutorials/ad-hoc-simulation/images/planning/others/nishishinjuku%20set%20pose%20.png)

4. Engage the ego vehicle. It will make a lane change along the planned path.

   ![lane-changing](https://github.com/vish0012/autoware-documentation/blob/7cc6b96690ae21f933ba329285e8fb1b06d20d7a/docs/tutorials/ad-hoc-simulation/images/planning/others/nishi%20shinjuku%20lane%20change%20.png)

### Avoidance scenario

1. Set an initial pose and a goal pose in the same lane. A path will be planned.

   ![set-position-and-goal](https://github.com/vish0012/autoware-documentation/blob/068fdb362f76b60a44b2c84e1572c0a53747fdb9/docs/tutorials/ad-hoc-simulation/images/planning/others/avoidence%20set%20goal.png)

2. Set a "2D Dummy Bus" on the roadside. A new path will be planned.

   ![set-dummy-bus](https://github.com/vish0012/autoware-documentation/blob/65b8b8b6aa0fe5d141b9e33c4e86d581776fb849/docs/tutorials/ad-hoc-simulation/images/planning/others/2d%20dummy%20bus.png)

3. Engage the ego vehicle. It will avoid the obstacle along the newly planned path.

## Advanced Simulations

### Placing dummy objects

1. Click the `2D Dummy Car` or `2D Dummy Pedestrian` button in the toolbar.
2. Set the pose of the dummy object by clicking and dragging on the map.
3. Set the velocity of the object in `Tool Properties -> 2D Dummy Car/Pedestrian` panel.

   !!! note

   Changes to the `velocity` parameter will only affect objects placed after the parameter is changed.

   ![set-dummy-car](https://github.com/vish0012/autoware-documentation/blob/4176dd22bdbd9220268c23514cb6716c4c3a3f98/docs/tutorials/ad-hoc-simulation/images/planning/others/advacnce%20dumy%20object%20.png)

4. Delete any dummy objects placed in the view by clicking the `Delete All Objects` button in the toolbar.

5. Click the `Interactive` button in the toolbar to make the dummy object interactive.

   ![set-interactive-dummy-car](https://github.com/vish0012/autoware-documentation/blob/95f0be14a5291ac8921cdb7ccac2e9c1c55c9b50/docs/tutorials/ad-hoc-simulation/images/planning/others/interactive%20dummy%20.png)

6. For adding an interactive dummy object, press `SHIFT` and click the `right click`.
7. For deleting an interactive dummy object, press `ALT` and click the `right click`.
8. For moving an interactive dummy object, hold the `right click` drag and drop the object.

   ![move-interactive-dummy-car](https://github.com/vish0012/autoware-documentation/blob/51e9413dcc21b3ac3c871d4f0a4e28b78863aacf/docs/tutorials/ad-hoc-simulation/images/planning/others/moving%20interactive%20demo%20.png)

### Traffic light recognition simulation

By default, traffic lights on the map are all treated as if they are set to green. As a result, when a path is created that passed through an intersection with a traffic light, the ego vehicle will drive through the intersection without stopping.

The following steps explain how to set and reset traffic lights in order to test how the Planning component will respond.

#### Set traffic light

1. Go to `Panels -> Add new panel`, select `TrafficLightPublishPanel`, and then press `OK`.

2. In `TrafficLightPublishPanel`, set the `ID` and color of the traffic light.

3. Click the `SET` button.
   ![set-traffic-light](https://github.com/vish0012/autoware-documentation/blob/08e809bf1283e3ac993b405a89c4c411ca4639a6/docs/tutorials/ad-hoc-simulation/images/planning/others/traffic%20light.png)
   Then you can seen down left side
   ![set-traffics-light 2](https://github.com/vish0012/autoware-documentation/blob/89c8929f870c70b1470a6c9eb5f549342833e7fa/docs/tutorials/ad-hoc-simulation/images/planning/others/traffic%20pannel%20down%20.png)

4. Finally, click the `PUBLISH` button to send the traffic light status to the simulator. Any planned path that goes past the selected traffic light will then change accordingly.

![send-traffic-light-color](https://github.com/vish0012/autoware-documentation/blob/637dfac7a5dae71db5ce0be458df98579d27e394/docs/tutorials/ad-hoc-simulation/images/planning/others/traffic%2023.png)

By default, Rviz should display the ID of each traffic light on the map. You can have a closer look at the IDs by zooming in the region or by changing the View type.

In case the IDs are not displayed, try the following troubleshooting steps:

a) In the `Displays` panel, find the `traffic_light_id` topic by toggling the triangle icons next to `Map > Lanelet2VectorMap > Namespaces`.

b) Check the `traffic_light_id` checkbox.

c) Reload the topic by clicking the `Map` checkbox twice.

![see-traffic-light-ID](https://github.com/vish0012/autoware-documentation/blob/8cbd6c7c78f224b7bc514fa8f49d276b9883adfc/docs/tutorials/ad-hoc-simulation/images/planning/others/last%2045.png)

#### Update/Reset traffic light

You can update the color of the traffic light by selecting the next color (in the image it is `GREEN`) and clicking `SET` button. In the image the traffic light in front of the ego vehicle changed from `RED` to `GREEN` and the vehicle restarted.

![after-traffic-light-color-update](https://github.com/vish0012/autoware-documentation/blob/e730cad28f0a4113624f0b3afe300dcee8af454b/docs/tutorials/ad-hoc-simulation/images/planning/others/traffic%20publish%20green%20.png)

To remove a traffic light from `TrafficLightPublishPanel`, click the `RESET` button.

[Reference video tutorials](https://drive.google.com/file/d/1bs_dX1JJ76qHk-SGvS6YF9gmekkN8fz7/view?usp=sharing)

## Using Autoware Launch GUI

This section provides a step-by-step guide on using the Autoware Launch GUI for planning simulations, offering an alternative to the command-line instructions provided in the Basic simulations section.

### Getting Started with Autoware Launch GUI

1. **Installation:** Ensure you have installed the Autoware Launch GUI. [Installation instructions](https://github.com/autowarefoundation/autoware-launch-gui#installation).

2. **Launching the GUI:** Open the Autoware Launch GUI from your applications menu.

   ![GUI screenshot for launching the GUI](images/planning/launch-gui/launch_gui_main.png)

### Launching a Planning Simulation

#### Lane Driving Scenario

1. **Set Autoware Path:** In the GUI, set the path to your Autoware installation.

   ![GUI_screenshot_for_setting_Autoware_path](images/planning/launch-gui/launch_gui_setup.png)

2. **Select Launch File:** Choose `planning_simulator.launch.xml` for the lane driving scenario.

   ![GUI screenshot for selecting launch file](images/planning/launch-gui/selecting_launch_file.png)

3. **Customize Parameters:** Adjust parameters such as `map_path`, `vehicle_model`, and `sensor_model` as needed.

   ![GUI screenshot for customizing parameters](images/planning/launch-gui/customizing-parameters1.png)
   ![GUI screenshot for customizing parameters](images/planning/launch-gui/customizing-parameters2.png)

4. **Start Simulation:** Click the launch button to start the simulation.

   ![GUI screenshot for starting simulation](images/planning/launch-gui/starting_simulation.png)

5. **Any Scenario:** From here, you can follow the instructions in the

- Lane driving scenario: [Lane Driving Scenario](#lane-driving-scenario)
- Parking scenario: [Parking scenario](#parking-scenario)
- Lane change scenario: [Lane change scenario](#lane-change-scenario)
- Avoidance scenario: [Avoidance scenario](#avoidance-scenario)
- Advanced Simulations: [Advanced Simulations](#advanced-simulations)

### Monitoring and Managing the Simulation

- **Real-Time Monitoring:** Use the GUI to monitor CPU/Memory usage and Autoware logs in real-time.
- **Profile Management:** Save your simulation profiles for quick access in future simulations.
- **Adjusting Parameters:** Easily modify simulation parameters on-the-fly through the GUI.

## Want to Try Autoware with Your Custom Map?

The above content describes the process for conducting some operations in the planning simulator using a sample map. If you are interested in running Autoware with maps of your own environment, please visit the [How to Create Vector Map](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-maps/#creating-maps) section for guidance.

![psim-custom-map](images/planning/others/psim-custom-map.png)
