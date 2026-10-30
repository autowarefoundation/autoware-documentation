# Lane detection methods

## Overview

This document describes some of the most common lane detection methods used in the autonomous driving industry.
Lane detection is a crucial task in autonomous driving, as it is used to determine the boundaries of the road and the
vehicle's position within the lane.

## Methods

This document covers the methods under two categories: lane detection methods and multitask detection methods.

!!! note

    The results have been obtained using pre-trained models. Training the model with your own data will yield more
    successful results.

    The tables list the pre-trained configurations that were tested on a campus video and a road video.
    The result videos of these tests were published on YouTube, but they are no longer available.

### Lane detection methods

#### CLRerNet

This work introduces LaneIoU, which improves confidence score accuracy by considering local lane angles, and CLRerNet,
a novel detector leveraging LaneIoU.

- **Paper**: [CLRerNet: Improving Confidence of Lane Detection with LaneIoU](https://arxiv.org/abs/2305.08366)
- **Code**: [GitHub](https://github.com/hirotomusiker/CLRerNet)

| Method   | Backbone | Dataset | Confidence |
| -------- | -------- | ------- | ---------- |
| CLRerNet | dla34    | culane  | 0.4        |
| CLRerNet | dla34    | culane  | 0.1        |
| CLRerNet | dla34    | culane  | 0.01       |

#### CLRNet

This work introduces Cross Layer Refinement Network (CLRNet) to fully utilize high-level semantic and low-level detailed
features in lane detection.
CLRNet detects lanes with high-level features and refines them with low-level details.
Additionally, ROIGather technique and Line IoU loss significantly enhance localization accuracy,
outperforming state-of-the-art methods.

- **Paper**: [CLRNet: Cross Layer Refinement Network for Lane Detection](https://arxiv.org/abs/2203.10350)
- **Code**: [GitHub](https://github.com/Turoad/CLRNet)

| Method | Backbone  | Dataset  | Confidence |
| ------ | --------- | -------- | ---------- |
| CLRNet | dla34     | culane   | 0.2        |
| CLRNet | dla34     | culane   | 0.1        |
| CLRNet | dla34     | culane   | 0.01       |
| CLRNet | dla34     | llamas   | 0.4        |
| CLRNet | dla34     | llamas   | 0.2        |
| CLRNet | dla34     | llamas   | 0.1        |
| CLRNet | resnet18  | llamas   | 0.4        |
| CLRNet | resnet18  | llamas   | 0.2        |
| CLRNet | resnet18  | llamas   | 0.1        |
| CLRNet | resnet18  | tusimple | 0.2        |
| CLRNet | resnet18  | tusimple | 0.1        |
| CLRNet | resnet34  | culane   | 0.1        |
| CLRNet | resnet34  | culane   | 0.05       |
| CLRNet | resnet101 | culane   | 0.2        |
| CLRNet | resnet101 | culane   | 0.1        |

#### FENet

This research introduces Focusing Sampling, Partial Field of View Evaluation, Enhanced FPN architecture,
and Directional IoU Loss, addressing challenges in precise lane detection for autonomous driving.
Experiments show that Focusing Sampling, which emphasizes distant details crucial for safety,
significantly improves both benchmark and practical curved/distant lane recognition accuracy over uniform approaches.

- **Paper**: [FENet: Focusing Enhanced Network for Lane Detection](https://arxiv.org/abs/2312.17163)
- **Code**: [GitHub](https://github.com/HanyangZhong/FENet)

| Method   | Backbone | Dataset | Confidence |
| -------- | -------- | ------- | ---------- |
| FENet v1 | dla34    | culane  | 0.2        |
| FENet v1 | dla34    | culane  | 0.1        |
| FENet v1 | dla34    | culane  | 0.05       |
| FENet v2 | dla34    | culane  | 0.2        |
| FENet v2 | dla34    | culane  | 0.1        |
| FENet v2 | dla34    | culane  | 0.05       |
| FENet v2 | dla34    | llamas  | 0.4        |
| FENet v2 | dla34    | llamas  | 0.2        |
| FENet v2 | dla34    | llamas  | 0.1        |
| FENet v2 | dla34    | llamas  | 0.05       |

### Multitask detection methods

#### YOLOPv2

This work proposes an efficient multi-task learning network for autonomous driving,
combining traffic object detection, drivable road area segmentation, and lane detection.
YOLOPv2 model achieves new state-of-the-art performance in accuracy and speed on the BDD100K dataset,
halving the inference time compared to previous benchmarks.

- **Paper**: [YOLOPv2: Better, Faster, Stronger for Panoptic Driving Perception](https://arxiv.org/abs/2208.11434)
- **Code**: [GitHub](https://github.com/CAIC-AD/YOLOPv2)

#### HybridNets

This work introduces HybridNets, an end-to-end perception network for autonomous driving.
It optimizes segmentation heads and box/class prediction networks using a weighted bidirectional feature network.
HybridNets achieves good performance on BDD100K and Berkeley DeepDrive datasets, outperforming state-of-the-art methods.

- **Paper**: [HybridNets: End-to-End Perception Network](https://arxiv.org/abs/2203.09035)
- **Code**: [GitHub](https://github.com/datvuthanh/HybridNets)

#### TwinLiteNet

This work introduces TwinLiteNet, a lightweight model designed for driveable area and lane line segmentation in
autonomous driving.

- **Paper**: [TwinLiteNet: An Efficient and Lightweight Model for Driveable Area and Lane Segmentation in Self-Driving Cars](https://arxiv.org/abs/2307.10705)
- **Code**: [GitHub](https://github.com/chequanghuy/TwinLiteNet)

## Citation

```bibtex
@article{honda2023clrernet,
title={CLRerNet: Improving Confidence of Lane Detection with LaneIoU},
author={Hiroto Honda and Yusuke Uchida},
journal={arXiv preprint arXiv:2305.08366},
year={2023},
}
```

```bibtex
@InProceedings{Zheng_2022_CVPR,
    author    = {Zheng, Tu and Huang, Yifei and Liu, Yang and Tang, Wenjian and Yang, Zheng and Cai, Deng and He, Xiaofei},
    title     = {CLRNet: Cross Layer Refinement Network for Lane Detection},
    booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
    month     = {June},
    year      = {2022},
    pages     = {898-907}
}
```

```bibtex
@article{wang&zhong_2024fenet,
      title={FENet: Focusing Enhanced Network for Lane Detection},
      author={Liman Wang and Hanyang Zhong},
      year={2024},
      eprint={2312.17163},
      archivePrefix={arXiv},
      primaryClass={cs.CV}
}
```

```bibtex
@misc{vu2022hybridnets,
      title={HybridNets: End-to-End Perception Network},
      author={Dat Vu and Bao Ngo and Hung Phan},
      year={2022},
      eprint={2203.09035},
      archivePrefix={arXiv},
      primaryClass={cs.CV}
}
```

```bibtex
@INPROCEEDINGS{10288646,
  author={Che, Quang-Huy and Nguyen, Dinh-Phuc and Pham, Minh-Quan and Lam, Duc-Khai},
  booktitle={2023 International Conference on Multimedia Analysis and Pattern Recognition (MAPR)},
  title={TwinLiteNet: An Efficient and Lightweight Model for Driveable Area and Lane Segmentation in Self-Driving Cars},
  year={2023},
  volume={},
  number={},
  pages={1-6},
  doi={10.1109/MAPR59823.2023.10288646}
}
```
