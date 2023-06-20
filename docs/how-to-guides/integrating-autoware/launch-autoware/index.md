# Launch Autoware

!!! warning

    Under Construction

This section briefly explains how to run your vehicle with Autoware.

## Install Autoware

Follow the [installation steps of Autoware](../../installation/).

## Launch Autoware

Launch Autoware with the following command:

```bash
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=YOUR_VEHICLE sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP
```

このとき、実行時引数で立ち上げるモードを選ぶことができます。
詳細は下位ページを見てください。

## Set initial pose

If GNSS is available, Autoware automatically initializes the vehicle's pose.

If not, you need to set the initial pose using the RViz GUI.

1. Click the 2D Pose estimate button in the toolbar, or hit the P key
2. In the 3D View pane, click and hold the left mouse button, and then drag to set the direction for the initial pose.

## Set goal pose

Set a goal pose for the ego vehicle.

1. Click the 2D Nav Goal button in the toolbar, or hit the G key
2. In the 3D View pane, click and hold the left mouse button, and then drag to set the direction for the goal pose.
   If successful, you will see the calculated planning path on RViz.

## Engage

In your terminal, execute the following command.

```bash
source ~/autoware.YOURS/install/setup.bash
ros2 topic pub /autoware.YOURS/engage autoware_auto_vehicle_msgs/msg/Engage "engage: true" -1
```

You can also engage via RViz with "AutowareStatePanel".
The panel can be found in `Panels > Add New Panel > tier4_state_rviz_plugin > AutowareStatePanel`.

Now the vehicle should drive along the calculated path!