# How to integrate Autoware with differential drive vehicle

## 1. Integrate Autoware with differential drive vehicle

Currently, Autoware assumes the robot to be driven by Ackermann Command

（編注：Ackermannコマンドの説明少し図とかで簡単に描きたい）

However, it is also possible to integrate Autoware with differential drive robot, by properly translating Ackermann Command to diff-drive command

## 2. How-to

Create a vehicle_interface package that translates Ackermann Command to differential drive commands.



## 3. Known issues

### Motion model incompatibility

As mentioned above, Autoware assumes the vehicle to adopt steering system that is controlled with Ackermann commands.
Due to this reason, when you apply Autoware to differential drive vehicles, Autoware cannot take full advantage of the flexibility of their motion model.
For example, `freespace_planner` for parking scenario in planning module may drive the differential drive vehicle forward and backward, 
while the vehicle may be possible park with much more simple trajectory with pure rotation movement.
