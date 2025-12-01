# Installation of related tools

## Calibration tools

### Calibration tools provided by TIER IV

Once the Autoware is installed, you can install the calibration tools provided by TIER IV as follows:

```bash
cd autoware
wget https://raw.githubusercontent.com/tier4/CalibrationTools/refs/heads/tier4/universe/calibration_tools_autoware.repos
vcs import src < calibration_tools.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Check the [README.md file](https://github.com/tier4/CalibrationTools/blob/tier4/universe/README.md) for a more detailed explanation.
