# Obstacle Segmentation Evaluation

## Preparation

1. Copy sample scenario

   ```bash
   mkdir -p ~/driving_log_replayer_data/obstacle_segmentation/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/docs/sample/obstacle_segmentation/scenario.yaml ~/driving_log_replayer_data/obstacle_segmentation/sample
   ```

2. Copy bag file from dataset

   ```bash
   mkdir -p ~/driving_log_replayer_data/obstacle_segmentation/sample/t4_dataset
   cp -r ~/driving_log_replayer_data/sample_dataset ~/driving_log_replayer_data/obstacle_segmentation/sample/t4_dataset
   ```

## How to run

1. Run the simulation

   ```bash
   driving_log_replayer simulation run -p obstacle_segmentation --rate 0.5
   ```

   ![obstacle_segmentation](images/obstacle_segmentation.png)

2. Check the results

   Results are displayed in the terminal like below.
   The number of tests will vary slightly depending on PC performance and CPU load conditions, so slight differences are not a problem.

   ```bash
    test case 1 / 1 : use case: sample_dataset
    --------------------------------------------------
    TestResult: Failed
    Detection Failed: detection: 557 / 681 -> 81.79% detection_warn: 0 non_detection: 681 / 681 -> 100.00%
   ```

For more information, refer to the [driving_log_replayer documentation](https://tier4.github.io/driving_log_replayer/).
