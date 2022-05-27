# Planning simulation

## Preparation

1. Download and unpack a sample map.

   - Click [here](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view?usp=sharing) to download.
   - Unpack it by running the following command.

   ```bash
   unzip -d ~/Downloads/ ~/Downloads/sample-map-planning.zip
   ```

### Note

- Sample map: Copyright 2020 TIER IV, Inc.

## How to run a planning simulation

### Launch Autoware

```sh
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Downloads/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

Note that you cannot use `~` instead of `$HOME` here.

![after-autoware-launch](images/planning/lane-following/after-autoware-launch.png)

### Open Autoware State Panel

This panel is useful when we start planning simulation. Click `Panels -> Add new panel`, select `AutowareStatePanel`, and then press `OK`.

![after-autoware-launch](images/planning/lane-following/open-autoware-state-panel.png)

### Set an initial pose for the ego vehicle

1. Click the `2D Pose estimate` button in the toolbar, or hit the `P` key.
2. In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the initial pose. The car should be placed in the right direction according to the lane(you can check the arrow on the lane). Now you will see a car on the Rviz.

![set-initial-pose](images/planning/lane-following/set-initial-pose.png)

### Set a goal pose for the ego vehicle

1. Click the `2D Nav Goal` button in the toolbar, or hit the `G` key
2. In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the goal pose. Keep in mind to set the goal pose aligned with the lane direction. Otherwise the path will not be planned. Now you will see a planned path.

![set-goal-pose](images/planning/lane-following/set-goal-pose.png)

### Place a dummy object

You can skip this step.

To place a dummy car/pedestrian, click the `2D Dummy Car` and/or `2D Dummy Pedestrian` button in the toolbar, and then set its pose by dragging on the map. You can set its velocity in `Tool Properties -> 2D Dummy Car/Pedestrian` panel. In the image the velocity is set to 0.

![set-dummy-car](images/planning/lane-following/place-dummy-car.png)

To delete those objects, click the `Delete All Objects` button in the toolbar.

### Set traffic light

You can skip this step.

To simulate traffic light recognition, go to `Panels -> Add new panel`, select `TrafficLightPublishPanel`, and then press `OK`. Then in the panel you need to set the `ID` and color of the traffic light.

You can check the `ID` of the traffic light by searching for `traffic` tag in `lanelet2_map.osm`. In the image there is one traffic light whose `ID` is 34836 and its color is `SET` to `RED`.

![set-traffic-light](images/planning/lane-following/set-traffic-light.png)

By clicking `PUBLISH` button the traffic light status is sent to the simulator, and the planned path changes accordingly.

![send-traffic-light-color](images/planning/lane-following/send-traffic-light-color.png)

### Engage the ego vehicle

Now you can start driving by clicking `Engage` button in `AutowareStatePanel`. Or you can also manually `engage` the vehicle by running this command.

```bash
source ~/autoware/install/setup.bash
ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage "engage: true" -1
```

![start-driving](images/planning/lane-following/engage-and-start-planning.png)

### Reset traffic light

You can reset the color of the traffic light and let the vehicle make a turn at the intersection.

![after-traffic-light-color-update](images/planning/lane-following/after-traffic-light-color-update.png)

## How to simulate parking maneuvers

1. Set an initial pose and a goal pose, and engage the ego vehicle.

   ![after-set-goal-pose](images/planning/parking/after-set-goal-pose.png)

2. When the vehicle approaches the goal, it will change to the parking mode.

   ![change-to-parking-mode](images/planning/parking/change-to-parking-mode.png)

3. After that, the vehicle will start driving backwards and park at the destination parking spot.

   ![backward-driving](images/planning/parking/backward-driving.png)

   ![reach-goal](images/planning/parking/reach-goal.png)
