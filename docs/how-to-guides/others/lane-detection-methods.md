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

| Method   | Backbone | Dataset | Confidence | Campus Video                                             | Road Video                                               |
|----------|----------|---------|------------|----------------------------------------------------------|----------------------------------------------------------|
| CLRerNet | dla34    | culane  | 0.4        | ![type:video](https://www.youtube.com/embed/bfuHuoembGg) | ![type:video](https://www.youtube.com/embed/9r_IEg_IkJ8) |
| CLRerNet | dla34    | culane  | 0.1        | ![type:video](https://www.youtube.com/embed/XVonhGmvt8Q) | ![type:video](https://www.youtube.com/embed/5P6-yqCPAns) |
| CLRerNet | dla34    | culane  | 0.01       | ![type:video](https://www.youtube.com/embed/Sp599_HyegU) | ![type:video](https://www.youtube.com/embed/2tz9gXNIjqs) |

#### CLRNet

This work introduce Cross Layer Refinement Network (CLRNet) to fully utilize high-level semantic and low-level detailed
features in lane detection.
CLRNet detects lanes with high-level features and refines them with low-level details.
Additionally, ROIGather technique and Line IoU loss significantly enhance localization accuracy,
outperforming state-of-the-art methods.

- **Paper**: [CLRNet: Cross Layer Refinement Network for Lane Detection](https://arxiv.org/abs/2203.10350)
- **Code**: [GitHub](https://github.com/Turoad/CLRNet)

| Method | Backbone  | Dataset  | Confidence | Campus Video                                             | Road Video                                               |
|--------|-----------|----------|------------|----------------------------------------------------------|----------------------------------------------------------|
| CLRNet | dla34     | culane   | 0.2        | ![type:video](https://www.youtube.com/embed/n2HpKlOKGvc) | ![type:video](https://www.youtube.com/embed/K6-AHSraopc) |
| CLRNet | dla34     | culane   | 0.1        | ![type:video](https://www.youtube.com/embed/BWdCEFy6k3w) | ![type:video](https://www.youtube.com/embed/dHrzsIotVWA) |
| CLRNet | dla34     | culane   | 0.01       | ![type:video](https://www.youtube.com/embed/5iNo2VMD9os) | ![type:video](https://www.youtube.com/embed/nl4Lthr1mT8) |
| CLRNet | dla34     | llamas   | 0.4        | ![type:video](https://www.youtube.com/embed/IxzJ7TfaSrk) | ![type:video](https://www.youtube.com/embed/IxDdOI5efl0) |
| CLRNet | dla34     | llamas   | 0.2        | ![type:video](https://www.youtube.com/embed/vaR8tAgB1Ew) | ![type:video](https://www.youtube.com/embed/vc8-kslVi34) |
| CLRNet | dla34     | llamas   | 0.1        | ![type:video](https://www.youtube.com/embed/LkDeZQOItqw) | ![type:video](https://www.youtube.com/embed/r-O92vxSXuw) |
| CLRNet | resnet18  | llamas   | 0.4        | ![type:video](https://www.youtube.com/embed/nqSupblM89o) | ![type:video](https://www.youtube.com/embed/py1S5fDIC5E) |
| CLRNet | resnet18  | llamas   | 0.2        | ![type:video](https://www.youtube.com/embed/rrNoXck6YLc) | ![type:video](https://www.youtube.com/embed/KHaS9GXueJg) |
| CLRNet | resnet18  | llamas   | 0.1        | ![type:video](https://www.youtube.com/embed/J-gU1xbba28) | ![type:video](https://www.youtube.com/embed/5U3O0iaUWF4) |
| CLRNet | resnet18  | tusimple | 0.2        | ![type:video](https://www.youtube.com/embed/HfZYdADQsPM) | ![type:video](https://www.youtube.com/embed/syse16SpafY) |
| CLRNet | resnet18  | tusimple | 0.1        | ![type:video](https://www.youtube.com/embed/o3w3wL_f-GY) | ![type:video](https://www.youtube.com/embed/O2HwNfTJvSQ) |
| CLRNet | resnet34  | culane   | 0.1        | ![type:video](https://www.youtube.com/embed/6IgkfJsCjWA) | ![type:video](https://www.youtube.com/embed/LgU3mQniP8c) |
| CLRNet | resnet34  | culane   | 0.05       | ![type:video](https://www.youtube.com/embed/eLLcPrEpy84) | ![type:video](https://www.youtube.com/embed/fPoP3uFpzRw) |
| CLRNet | resnet101 | culane   | 0.2        | ![type:video](https://www.youtube.com/embed/FODj_M-RRC4) | ![type:video](https://www.youtube.com/embed/5fG8ApvTFD4) |
| CLRNet | resnet101 | culane   | 0.1        | ![type:video](https://www.youtube.com/embed/i0Bu2-Ef8T8) | ![type:video](https://www.youtube.com/embed/DWy4HeHyZYQ) |

#### FENet

This research introduces Focusing Sampling, Partial Field of View Evaluation, Enhanced FPN architecture,
and Directional IoU Loss, addressing challenges in precise lane detection for autonomous driving.
Experiments show that Focusing Sampling, which emphasizes distant details crucial for safety,
significantly improves both benchmark and practical curved/distant lane recognition accuracy over uniform approaches.

- **Paper**: [FENet: Focusing Enhanced Network for Lane Detection](https://arxiv.org/abs/2312.17163)
- **Code**: [GitHub](https://github.com/HanyangZhong/FENet)

| Method   | Backbone | Dataset | Confidence | Campus Video                                             | Road Video                                               |
|----------|----------|---------|------------|----------------------------------------------------------|----------------------------------------------------------|
| FENet v1 | dla34    | culane  | 0.2        | ![type:video](https://www.youtube.com/embed/eGHgxf-8mcg) | ![type:video](https://www.youtube.com/embed/YMKCWLWq2Ww) |
| FENet v1 | dla34    | culane  | 0.1        | ![type:video](https://www.youtube.com/embed/em3eaZ6RKZM) | ![type:video](https://www.youtube.com/embed/bCjEUtoIYac) |
| FENet v1 | dla34    | culane  | 0.05       | ![type:video](https://www.youtube.com/embed/_3gwLW54aHw) | ![type:video](https://www.youtube.com/embed/24hjuNlZBIQ) |
| FENet v2 | dla34    | culane  | 0.2        | ![type:video](https://www.youtube.com/embed/Z4WPJ9Cop2w) | ![type:video](https://www.youtube.com/embed/d3bovsjF2tE) |
| FENet v2 | dla34    | culane  | 0.1        | ![type:video](https://www.youtube.com/embed/vbE1wNIc1Js) | ![type:video](https://www.youtube.com/embed/ezWGPTSbBAw) |
| FENet v2 | dla34    | culane  | 0.05       | ![type:video](https://www.youtube.com/embed/sJvyR6jrlpY) | ![type:video](https://www.youtube.com/embed/XKuJ-YoVusY) |
| FENet v2 | dla34    | llamas  | 0.4        | ![type:video](https://www.youtube.com/embed/_GrUe6phC7U) | ![type:video](https://www.youtube.com/embed/_YLDS-gTg2w) |
| FENet v2 | dla34    | llamas  | 0.2        | ![type:video](https://www.youtube.com/embed/G59KpXE-2OI) | ![type:video](https://www.youtube.com/embed/3MaNauiPAxQ) |
| FENet v2 | dla34    | llamas  | 0.1        | ![type:video](https://www.youtube.com/embed/cre7XhUF7IM) | ![type:video](https://www.youtube.com/embed/vGKDraGFamM) |
| FENet v2 | dla34    | llamas  | 0.05       | ![type:video](https://www.youtube.com/embed/TNpBmidhChQ) | ![type:video](https://www.youtube.com/embed/Z67DTfoppVo) |

### Multitask Detection Methods

#### YOLOPv2

This work proposes an efficient multi-task learning network for autonomous driving,
combining traffic object detection, drivable road area segmentation, and lane detection.
YOLOPv2 model achieves new state-of-the-art performance in accuracy and speed on the BDD100K dataset,
halving the inference time compared to previous benchmarks.

- **Paper**: [YOLOPv2: Better, Faster, Stronger for Panoptic Driving Perception](https://arxiv.org/abs/2208.11434)
- **Code**: [GitHub](https://github.com/CAIC-AD/YOLOPv2)

| Method  | Campus Video                                             | Road Video                                               |
|---------|----------------------------------------------------------|----------------------------------------------------------|
| YOLOPv2 | ![type:video](https://www.youtube.com/embed/iovwTg3cisA) | ![type:video](https://www.youtube.com/embed/UzkCnI0Sx7c) |

#### HybridNets

This work introduces HybridNets, an end-to-end perception network for autonomous driving.
It optimizes segmentation heads and box/class prediction networks using a weighted bidirectional feature network.
HybridNets achieves good performance on BDD100K and Berkeley DeepDrive datasets, outperforming state-of-the-art methods.

- **Paper**: [HybridNets: End-to-End Perception Network](https://arxiv.org/abs/2203.09035)
- **Code**: [GitHub](https://github.com/datvuthanh/HybridNets)

| Method     | Campus Video                                             | Road Video                                               |
|------------|----------------------------------------------------------|----------------------------------------------------------|
| HybridNets | ![type:video](https://www.youtube.com/embed/ph9TKSiWvd4) | ![type:video](https://www.youtube.com/embed/aNsm4Uj1gcA) |

#### TwinLiteNet

This work introduces TwinLiteNet, a lightweight model designed for driveable area and lane line segmentation in
autonomous driving.

- **Paper
  **: [TwinLiteNet: An Efficient and Lightweight Model for Driveable Area and Lane Segmentation in Self-Driving Cars](https://arxiv.org/abs/2307.10705)
- **Code**: [GitHub](https://github.com/chequanghuy/TwinLiteNet)

| Method      | Campus Video                                             | Road Video                                               |
|-------------|----------------------------------------------------------|----------------------------------------------------------|
| Twinlitenet | ![type:video](https://www.youtube.com/embed/hDIcbBup7ww) | ![type:video](https://www.youtube.com/embed/4J9zSoVxw-Q) |

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
