# Planning simulation

## Preparation

Download and unpack a sample map.

- Click [here](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view?usp=sharing) to download.
- Unpack it by running the following command.

```bash
unzip -d ~/Downloads/ ~/Downloads/sample-map-planning.zip
```

### Note

- Sample map: Copyright 2020 TIER IV, Inc.

## Basic Simulations

### Lane Driving

#### Launch Autoware

```sh
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Downloads/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

Note that you cannot use `~` instead of `$HOME` here.

![after-autoware-launch](images/planning/lane-following/after-autoware-launch.png)

#### Add Autoware State Panel

This panel is useful when running planning simulations. To add the panel, click `Panels -> Add new panel`, select `AutowareStatePanel`, and then click `OK`.

![after-autoware-launch](images/planning/lane-following/open-autoware-state-panel.png)

#### Set an initial pose for the ego vehicle

![set-initial-pose](images/planning/lane-following/set-initial-pose.png)

1. Click the `2D Pose estimate` button in the toolbar, or hit the `P` key.
2. In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the initial pose. An image representing the vehicle should now be displayed.

!!! warning

    Remember to set the initial pose of the car in the same direction as the lane. To confirm the direction of the lane, check the arrowheads displayed on the map.

#### Set a goal pose for the ego vehicle

1. Click the `2D Goal Pose` button in the toolbar, or hit the `G` key.
2. In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the goal pose. If done correctly, you will see a planned path from initial pose to goal pose.

![set-goal-pose](images/planning/lane-following/set-goal-pose.png)

#### Engage the ego vehicle

Now you can start the ego vehicle driving by clicking the `Engage` button in `AutowareStatePanel`. Alteratively, you can manually engage the vehicle by running the following command:

```bash
source ~/autoware/install/setup.bash
ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage "engage: true" -1
```

![start-driving](images/planning/lane-following/engage-and-start-planning.png)

### Parking maneuvers

1. Set an initial pose and a goal pose, and engage the ego vehicle.

   ![after-set-goal-pose](images/planning/parking/after-set-goal-pose.png)

2. When the vehicle approaches the goal, it will change to the parking mode.
3. After that, the vehicle will start driving backwards and park at the destination parking spot.

   ![parking-maneuver](images/planning/parking/parking-maneuver.png)

## Advanced Simulations

### Dummy objects

To place a dummy car/pedestrian, click the `2D Dummy Car` and/or `2D Dummy Pedestrian` button in the toolbar, and then set its pose by dragging on the map. You can set its velocity in `Tool Properties -> 2D Dummy Car/Pedestrian` panel. In the image the velocity is set to 0.

![set-dummy-car](images/planning/lane-following/place-dummy-car.png)

To delete any dummy objects placed in the view, click the `Delete All Objects` button in the toolbar.

### Traffic light simulation

#### Set traffic light

To simulate traffic light recognition, go to `Panels -> Add new panel`, select `TrafficLightPublishPanel`, and then press `OK`. Then in the panel you need to set the `ID` and color of the traffic light.

You can check the ID of each traffic light on Rviz. If the IDs do not pop up, select `Map/Lanelet2VectorMap/Namespaces/traffic_light_id` topic, check the checkbox, and then **reload** this topic by clicking the checkbox of `Map` twice to re-visualize the IDs (see the image below). You can have a closer look at the IDs by zooming in the region or by changing the View type.

![see-traffic-light-ID](images/planning/lane-following/see-traffic-light-ID.png)

And then you can select the ID of the desired traffic light from the `ID` _Combobox_. The image below shows a traffic light whose `ID` is 34836 and its color is set to `RED`.

![set-traffic-light](images/planning/lane-following/set-traffic-light.png)

Finally, click the `PUBLISH` button to send the traffic light status to the simulator. Any planned path that goes past the selected traffic light will then change accordingly.

![send-traffic-light-color](images/planning/lane-following/send-traffic-light-color.png)

#### Reset traffic light

If you previously set a traffic light to `RED` or `AMBER`, you can reset the color of the traffic light to `GREEN` and let the ego vehicle make a turn at the intersection.

![after-traffic-light-color-update](images/planning/lane-following/after-traffic-light-color-update.png)
