# Lane Detection Methods

## Overview

This document describes some of the most common lane detection methods used in the autonomous driving industry.
Lane detection is a crucial task in autonomous driving, as it is used to determine the boundaries of the road and the
vehicle's position within the lane.

## Methods

This document covers the methods under two categories: lane detection methods and multitask detection methods.

!!! note

    The results have been obtained using pre-trained models. Training the model with your own data will yield more 
    successful results.

### Lane Detection Methods

#### CLRerNet

This work introduce LaneIoU, which improves confidence score accuracy by considering local lane angles, and CLRerNet,
a novel detector leveraging LaneIoU.

- **Paper**: [CLRerNet: Improving Confidence of Lane Detection with LaneIoU](https://arxiv.org/abs/2305.08366)
- **Code**: [GitHub](https://github.com/hirotomusiker/CLRerNet)

| Method   | Backbone | Dataset | Confidence | Campus Video                                | Road Video                                  |
|----------|----------|---------|------------|---------------------------------------------|---------------------------------------------|
| CLRerNet | dla34    | culane  | 0.4        | ![type:video](https://youtu.be/bfuHuoembGg) | ![type:video](https://youtu.be/9r_IEg_IkJ8) |
| CLRerNet | dla34    | culane  | 0.1        | ![type:video](https://youtu.be/XVonhGmvt8Q) | ![type:video](https://youtu.be/5P6-yqCPAns) |
| CLRerNet | dla34    | culane  | 0.01       | ![type:video](https://youtu.be/Sp599_HyegU) | ![type:video](https://youtu.be/2tz9gXNIjqs) |

#### CLRNet

This work introduce Cross Layer Refinement Network (CLRNet) to fully utilize high-level semantic and low-level detailed
features in lane detection.
CLRNet detects lanes with high-level features and refines them with low-level details.
Additionally, ROIGather technique and Line IoU loss significantly enhance localization accuracy,
outperforming state-of-the-art methods.

- **Paper**: [CLRNet: Cross Layer Refinement Network for Lane Detection](https://arxiv.org/abs/2203.10350)
- **Code**: [GitHub](https://github.com/Turoad/CLRNet)

| Method | Backbone  | Dataset  | Confidence | Campus Video                                | Road Video                                  |
|--------|-----------|----------|------------|---------------------------------------------|---------------------------------------------|
| CLRNet | dla34     | culane   | 0.2        | ![type:video](https://youtu.be/n2HpKlOKGvc) | ![type:video](https://youtu.be/K6-AHSraopc) |
| CLRNet | dla34     | culane   | 0.1        | ![type:video](https://youtu.be/BWdCEFy6k3w) | ![type:video](https://youtu.be/dHrzsIotVWA) |
| CLRNet | dla34     | culane   | 0.01       | ![type:video](https://youtu.be/5iNo2VMD9os) | ![type:video](https://youtu.be/nl4Lthr1mT8) |
| CLRNet | dla34     | llamas   | 0.4        | ![type:video](https://youtu.be/IxzJ7TfaSrk) | ![type:video](https://youtu.be/IxDdOI5efl0) |
| CLRNet | dla34     | llamas   | 0.2        | ![type:video](https://youtu.be/vaR8tAgB1Ew) | ![type:video](https://youtu.be/vc8-kslVi34) |
| CLRNet | dla34     | llamas   | 0.1        | ![type:video](https://youtu.be/LkDeZQOItqw) | ![type:video](https://youtu.be/r-O92vxSXuw) |
| CLRNet | resnet18  | llamas   | 0.4        | ![type:video](https://youtu.be/nqSupblM89o) | ![type:video](https://youtu.be/py1S5fDIC5E) |
| CLRNet | resnet18  | llamas   | 0.2        | ![type:video](https://youtu.be/rrNoXck6YLc) | ![type:video](https://youtu.be/KHaS9GXueJg) |
| CLRNet | resnet18  | llamas   | 0.1        | ![type:video](https://youtu.be/J-gU1xbba28) | ![type:video](https://youtu.be/5U3O0iaUWF4) |
| CLRNet | resnet18  | tusimple | 0.2        | ![type:video](https://youtu.be/HfZYdADQsPM) | ![type:video](https://youtu.be/syse16SpafY) |
| CLRNet | resnet18  | tusimple | 0.1        | ![type:video](https://youtu.be/o3w3wL_f-GY) | ![type:video](https://youtu.be/O2HwNfTJvSQ) |
| CLRNet | resnet34  | culane   | 0.1        | ![type:video](https://youtu.be/6IgkfJsCjWA) | ![type:video](https://youtu.be/LgU3mQniP8c) |
| CLRNet | resnet34  | culane   | 0.05       | ![type:video](https://youtu.be/eLLcPrEpy84) | ![type:video](https://youtu.be/fPoP3uFpzRw) |
| CLRNet | resnet101 | culane   | 0.2        | ![type:video](https://youtu.be/FODj_M-RRC4) | ![type:video](https://youtu.be/5fG8ApvTFD4) |
| CLRNet | resnet101 | culane   | 0.1        | ![type:video](https://youtu.be/i0Bu2-Ef8T8) | ![type:video](https://youtu.be/DWy4HeHyZYQ) |

### FENet

This research introduces Focusing Sampling, Partial Field of View Evaluation, Enhanced FPN architecture,
and Directional IoU Loss, addressing challenges in precise lane detection for autonomous driving.
Experiments show that Focusing Sampling, which emphasizes distant details crucial for safety,
significantly improves both benchmark and practical curved/distant lane recognition accuracy over uniform approaches.

- **Paper**: [FENet: Focusing Enhanced Network for Lane Detection](https://arxiv.org/abs/2312.17163)
- **Code**: [GitHub](https://github.com/HanyangZhong/FENet)

| Method   | Backbone | Dataset | Confidence | Campus Video                                | Road Video                                  |
|----------|----------|---------|------------|---------------------------------------------|---------------------------------------------|
| FENet v1 | dla34    | culane  | 0.2        | ![type:video](https://youtu.be/eGHgxf-8mcg) | ![type:video](https://youtu.be/YMKCWLWq2Ww) |
| FENet v1 | dla34    | culane  | 0.1        | ![type:video](https://youtu.be/em3eaZ6RKZM) | ![type:video](https://youtu.be/bCjEUtoIYac) |
| FENet v1 | dla34    | culane  | 0.05       | ![type:video](https://youtu.be/_3gwLW54aHw) | ![type:video](https://youtu.be/24hjuNlZBIQ) |
| FENet v2 | dla34    | culane  | 0.2        | ![type:video](https://youtu.be/Z4WPJ9Cop2w) | ![type:video](https://youtu.be/d3bovsjF2tE) |
| FENet v2 | dla34    | culane  | 0.1        | ![type:video](https://youtu.be/vbE1wNIc1Js) | ![type:video](https://youtu.be/ezWGPTSbBAw) |
| FENet v2 | dla34    | culane  | 0.05       | ![type:video](https://youtu.be/sJvyR6jrlpY) | ![type:video](https://youtu.be/XKuJ-YoVusY) |
| FENet v2 | dla34    | llamas  | 0.4        | ![type:video](https://youtu.be/_GrUe6phC7U) | ![type:video](https://youtu.be/_YLDS-gTg2w) |
| FENet v2 | dla34    | llamas  | 0.2        | ![type:video](https://youtu.be/G59KpXE-2OI) | ![type:video](https://youtu.be/3MaNauiPAxQ) |
| FENet v2 | dla34    | llamas  | 0.1        | ![type:video](https://youtu.be/cre7XhUF7IM) | ![type:video](https://youtu.be/vGKDraGFamM) |
| FENet v2 | dla34    | llamas  | 0.05       | ![type:video](https://youtu.be/TNpBmidhChQ) | ![type:video](https://youtu.be/Z67DTfoppVo) |

## Multitask Detection Methods

### YOLOPv2

This work proposes an efficient multi-task learning network for autonomous driving,
combining traffic object detection, drivable road area segmentation, and lane detection.
YOLOPv2 model achieves new state-of-the-art performance in accuracy and speed on the BDD100K dataset,
halving the inference time compared to previous benchmarks.

- **Paper**: [YOLOPv2: Better, Faster, Stronger for Panoptic Driving Perception](https://arxiv.org/abs/2208.11434)
- **Code**: [GitHub](https://github.com/CAIC-AD/YOLOPv2)

| Method  | Campus Video                                | Road Video                                  |
|---------|---------------------------------------------|---------------------------------------------|
| YOLOPv2 | ![type:video](https://youtu.be/iovwTg3cisA) | ![type:video](https://youtu.be/UzkCnI0Sx7c) |

### HybridNets

This work introduces HybridNets, an end-to-end perception network for autonomous driving.
It optimizes segmentation heads and box/class prediction networks using a weighted bidirectional feature network.
HybridNets achieves good performance on BDD100K and Berkeley DeepDrive datasets, outperforming state-of-the-art methods.

- **Paper**: [HybridNets: End-to-End Perception Network](https://arxiv.org/abs/2203.09035)
- **Code**: [GitHub](https://github.com/datvuthanh/HybridNets)

| Method     | Campus Video                                | Road Video                                  |
|------------|---------------------------------------------|---------------------------------------------|
| HybridNets | ![type:video](https://youtu.be/ph9TKSiWvd4) | ![type:video](https://youtu.be/aNsm4Uj1gcA) |

### TwinLiteNet

This work introduces TwinLiteNet, a lightweight model designed for driveable area and lane line segmentation in
autonomous driving.

- **Paper
  **: [TwinLiteNet: An Efficient and Lightweight Model for Driveable Area and Lane Segmentation in Self-Driving Cars](https://arxiv.org/abs/2307.10705)
- **Code**: [GitHub](https://github.com/chequanghuy/TwinLiteNet)

| Method      | Campus Video                                | Road Video                                  |
|-------------|---------------------------------------------|---------------------------------------------|
| Twinlitenet | ![type:video](https://youtu.be/hDIcbBup7ww) | ![type:video](https://youtu.be/4J9zSoVxw-Q) |

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