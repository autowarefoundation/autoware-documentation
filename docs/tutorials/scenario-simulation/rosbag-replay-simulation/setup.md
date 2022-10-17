# Setup

!!! note

    Running the Driving Log Replayer requires some additional steps on top of building and installing Autoware, so make sure that [Driving Log Replayer installation](installation.md) has been completed first before proceeding.

    Sample map: Copyright 2020 TIER IV, Inc.

    Sample Dataset: Copyright 2022 TIER IV, Inc.

## Set up resources

1. Download and unpack a sample map.

   The same maps as in planning-simulation are used, so if you have already downloaded them, you do not need to do this step.

   ```bash
   gdown -O ~/autoware_map/sample-map-planning.zip 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
   unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
   ```

   You can also download [the map](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view) manually.

2. Download and unpack a dataset.

   The driving_log_replayer tutorial uses the same dataset, so this step is not necessary if you have already downloaded it from other evaluation.

   ```bash
   gdown -O ~/driving_log_replayer_data/sample_dataset.tar.zst 'https://docs.google.com/uc?export=download&id=1UjMWZj5Yc55O7BZiGHa0ikZGhwmcfPiS'
   tar -I zstd -xvf ~/driving_log_replayer_data/sample_dataset.tar.zst -C ~/driving_log_replayer_data/
   ```

   You can also download [the dataset](https://drive.google.com/file/d/1UjMWZj5Yc55O7BZiGHa0ikZGhwmcfPiS/view) manually.

3. Copy sample setting

   The driving_log_replayer tutorial uses the same setting, so this step is not necessary if you have already copied it from other evaluation.

   ```bash
   cp ~/autoware/src/simulator/driving_log_replayer/docs/sample/.driving_log_replayer.config.toml ~/
   ```
