# Lidar-Lidar Calibration

## Overview

In this tutorial,
we will explain lidar-lidar calibration over [mapping-based lidar-lidar calibration tool](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/how_to_extrinsic_mapping_based.md) of Tier IV's
CalibrationTools.
Also,
[map-based lidar-lidar calibration method](https://github.com/tier4/CalibrationTools/blob/tier4/universe/sensor/docs/how_to_extrinsic_map_based.md) included in the Tier IV's tools.
[LL-Calib](https://github.com/autocore-ai/calibration_tools/tree/main/lidar-lidar-calib) on GitHub,
provided by [AutoCore](https://autocore.ai/),
is a lightweight toolkit for online/offline 3D LiDAR to LiDAR calibration.
It's based on local mapping and "GICP" method to derive the relation between main and sub lidar.
If you want more details about these methods such as usage, troubleshooting etc. please check the links above.

!!! warning

    Please get initial calibration results from [Generic Calibration](./generic-calibration.md) section, it is important for getting accurate results from this tool.
    We will use initial calibration parameters that we calculated on previous step on this tutorial.
