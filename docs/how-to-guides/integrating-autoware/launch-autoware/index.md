# Launch Autoware

!!! warning

    Under Construction

This section explains how to run your vehicle with Autoware.

## Install Autoware

Follow the [installation steps of Autoware](../../../installation/).

## Launch Autoware

Launch Autoware with the following command:

```bash
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=YOUR_VEHICLE sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP
```

どのコンポーネントを立ち上げるかをコマンドライン引数で指定することが可能です。
たとえばlocalizationの動作確認をするためにperceptionとplanningとcontrolを立ち上げる必要がない場合は次のように書くことができます

```bash
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=YOUR_VEHICLE sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP \
    launch_perception:=false \
    launch_planning:=false \
    launch_control:=false
```

基本的なコマンドラインオプションは[autoware.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml)に書かれています。

コマンドライン引数の中には各コンポーネントでどのような手法を起動するかを切り変えるためのオプションも存在します。
`localization_mode` や `perception_mode` がそれにあたります。
各コンポーネントごとのオプションの詳細は下位ページを見てください。

## Set initial pose

If GNSS is available, Autoware automatically initializes the vehicle's pose.

If not もしくは自動初期位置推定が誤った位置を出力した場合は, you need to set the initial pose using the RViz GUI.

1. Click the 2D Pose estimate button in the toolbar, or hit the P key

    ![2D Pose estiamte](images/2d_pose_estimate.png)


2. In the 3D View pane, click and hold the left mouse button, and then drag to set the direction for the initial pose.


## Set goal pose

Set a goal pose for the ego vehicle.

1. Click the 2D Nav Goal button in the toolbar, or hit the G key

    ![2D Pose estiamte](images/2d_pose_estimate.png)

2. In the 3D View pane, click and hold the left mouse button, and then drag to set the direction for the goal pose.
   If successful, you will see the calculated planning path on RViz.

    ![route planning](images/route_planning_is_complete.png){width="800"}

## Engage

In your terminal, execute the following command.

```bash
source ~/autoware.YOURS/install/setup.bash
ros2 topic pub /autoware.YOURS/engage autoware_auto_vehicle_msgs/msg/Engage "engage: true" -1
```


You can also engage via RViz with "AutowareStatePanel".
The panel can be found in `Panels > Add New Panel > tier4_state_rviz_plugin > AutowareStatePanel`.


ルートが計算されると`AUTO`ボタンが活性化します。それを押せばEngageがされます。

![autoware state panel](images/autoware_state_panel_before.png)

Now the vehicle should drive along the calculated path!

走行中はStatePanelは以下のようになります。ここで`STOP`を押せば走行を停止することができます。

![autoware state panel](images/autoware_state_panel_after.png)