# /api/external/set/initialize_pose_auto

## Classification

- Behavior: Service
- DataType: tier4_external_api_msgs/srv/InitializePoseAuto

## Description

GNSS による位置姿勢をもとに、車両の位置姿勢を初期化・再設定する。

## Requirement

位置姿勢に関する事前状態を考慮せず、GNSS による位置姿勢のみを用いて車両の位置姿勢を初期化・再設定する。
