# Localization mode

!!! warning

    Under Construction

自己位置推定はデフォルトではndt_scan_matcherが起動します。
ここでは、ndt_scan_matcher以外の位置推定手法の起動方法を説明します。

## How to launch YabLoc

YabLocはカメラベースの位置推定手法です。
YabLocをndt_scan_matcherに代わってpose_estimatorとして利用する場合はAutowareの起動時に次のように`localization_mode:=yabloc`を追加してください。

```bash
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=YOUR_VEHICLE sensor_kit:=YOUR_SENSOR_KIT map_path:=/PATH/TO/YOUR/MAP \
    localization_mode:=yabloc
```

`localization_mode`はデフォルトでは`ndt`であり、特に指定をしない場合は`localization_mode:=ndt`として扱われます。
このコマンドライン引数を指定することにより、自動でYabLoのノードが起動し、NDTのノードは起動しないようになります。

YabLocの詳細は [YabLoc Guide](yabloc-guide.md)を見てください。

## How to launch Eagleye

Eagleyeはtwist_estimatorとしてのオプションでもあり、pose_estimatorとtwist_estimator両方の代替オプションでもあります。
現状ではコマンドライン引数からはEagleyeの利用は指定できません。

Eagleyeの起動方法の詳細は [Eagleye Guide](eagleye-guide.md)を見てください。