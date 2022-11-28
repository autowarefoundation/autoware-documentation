# /api/external/get/localization_scores

## Classification

- Behavior: Topic
- DataType: tier4_external_api_msgs/msg/LocalizationScoreArray

## Description

自己位置推定のスコアを取得する。取得できるスコアは以下の通り。

- Transform Probability (TP)
- Nearest Voxel Transformation Likelihood (NVTL)

## Requirement

内部の実装でサポートされている自己位置推定のスコアを提供すること。
