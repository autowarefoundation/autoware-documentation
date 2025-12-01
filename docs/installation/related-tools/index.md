# Installation of related tools

## Calibration Tool

### Install Calibration of TIER IV

After installing [autoware](https://github.com/tier4/autoware) (please see [source-installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/) page), execute the following commands:

```bash
cd autoware
wget https://raw.githubusercontent.com/tier4/CalibrationTools/tier4/universe/calibration_tools.repos
vcs import src < calibration_tools.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Check [here](https://github.com/tier4/CalibrationTools/blob/tier4/universe/README.md) for a detailed explanation.
