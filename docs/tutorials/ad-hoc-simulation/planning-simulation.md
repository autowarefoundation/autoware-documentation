# Planning simulation

## Preparation

Download and unpack a sample map.

- Click [here](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view?usp=sharing) to download.
- Unpack it by running the following command.

```bash
unzip -d ~/Downloads/ ~/Downloads/sample-map-planning.zip
```

!!! Note

    Sample map: Copyright 2020 TIER IV, Inc.

## Basic simulations

### Lane driving scenario

#### 1. Launch Autoware

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Downloads/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

!!! warning

    Note that you cannot use `~` instead of `$HOME` here.

    If `~` is used, the map will fail to load.

![after-autoware-launch](images/planning/lane-following/after-autoware-launch.png)

#### 2. Add Autoware State Panel

This panel is useful when running planning simulations. To add the panel, click `Panels -> Add new panel`, select `AutowareStatePanel`, and then click `OK`.

![after-autoware-launch](images/planning/lane-following/open-autoware-state-panel.png)

#### 3. Set an initial pose for the ego vehicle

![set-initial-pose](images/planning/lane-following/set-initial-pose.png)

a) Click the `2D Pose estimate` button in the toolbar, or hit the `P` key.

b) In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the initial pose. An image representing the vehicle should now be displayed.

!!! warning

    Remember to set the initial pose of the car in the same direction as the lane.

    To confirm the direction of the lane, check the arrowheads displayed on the map.

#### 4. Set a goal pose for the ego vehicle

a) Click the `2D Goal Pose` button in the toolbar, or hit the `G` key.

b) In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the goal pose. If done correctly, you will see a planned path from initial pose to goal pose.

![set-goal-pose](images/planning/lane-following/set-goal-pose.png)

#### 5. Engage the ego vehicle

Now you can start the ego vehicle driving by clicking the `Engage` button in `AutowareStatePanel`. Alteratively, you can manually engage the vehicle by running the following command:

```bash
source ~/autoware/install/setup.bash
ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage "engage: true" -1
```

![start-driving](images/planning/lane-following/engage-and-start-planning.png)

### Parking scenario

1. Set an initial pose and a goal pose, and engage the ego vehicle.

   ![after-set-goal-pose](images/planning/parking/after-set-goal-pose.png)

2. When the vehicle approaches the goal, it will switch from lane driving mode to parking mode.
3. After that, the vehicle will reverse into the destination parking spot.

   ![parking-maneuver](images/planning/parking/parking-maneuver.png)

## Advanced Simulations

### Placing dummy objects

1. Click the `2D Dummy Car` or `2D Dummy Pedestrian` button in the toolbar.

2. Set the pose of the dummy object by clicking and dragging on the map.

3. Set the velocity of the object in `Tool Properties -> 2D Dummy Car/Pedestrian` panel.

!!! note

    Changes to the `velocity` parameter will only affect objects placed after the parameter is changed.

![set-dummy-car](images/planning/lane-following/place-dummy-car.png) 4. Delete any dummy objects placed in the view by clicking the `Delete All Objects` button in the toolbar.

### Traffic light recognition simulation

By default, traffic lights on the map are all treated as if they are set to green. As a result, when a path is created that passed through an intersection with a traffic light, the ego vehicle will drive through the intersection without stopping.

The following steps explain how to set and reset traffic lights in order to test how the Planning component will respond.

#### Set traffic light

1. Go to `Panels -> Add new panel`, select `TrafficLightPublishPanel`, and then press `OK`.

2. In `TrafficLightPublishPanel`, set the `ID` and color of the traffic light.

3. Click the `SET` button.
   ![set-traffic-light](images/planning/lane-following/set-traffic-light.png)

4. Finally, click the `PUBLISH` button to send the traffic light status to the simulator. Any planned path that goes past the selected traffic light will then change accordingly.

![send-traffic-light-color](images/planning/lane-following/send-traffic-light-color.png)

By default, Rviz should display the ID of each traffic light on the map. You can have a closer look at the IDs by zooming in the region or by changing the View type.

In the event that the IDs are not displayed, try the following troubleshooting steps:

a) In the `Displays` panel, find the `traffic_light_id` topic by toggling the triangle icons next to `Map > Lanelet2VectorMap > Namespaces`.

b) Check the `traffic_light_id` checkbox.

c) Reload the topic by clicking the `Map` checkbox twice.

![see-traffic-light-ID](images/planning/lane-following/see-traffic-light-ID.png)

#### Update/Reset traffic light

You can update the color of the traffic light by selecting the next color (in the image it is `GREEN`) and clicking `SET` button. In the image the traffic light in front of the ego vehicle changed from `RED` to `GREEN` and the vehicle restarted.

![after-traffic-light-color-update](images/planning/lane-following/after-traffic-light-color-update.png)

To remove a traffic light from `TrafficLightPublishPanel`, click the `RESET` button.
