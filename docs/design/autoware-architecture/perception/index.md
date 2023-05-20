# Perception component design

!!! warning

    Under Construction

## Overview

Perception はsensingとlocalization, mapの入力を受け取り，semanticな情報（e.g. 物体の認識，tracking, prediction, 地面との分離，信号認識）を付加し，planningにわたす

## Requirements

**Goals:**

**Non-goals:**

## High-level architecture

This diagram describes the high-level architecture of the Perception Component.

![overall-perception-architecture](image/high-level-perceptipn-diagram.drawio.svg)

The Perception component consists of the following sub-components:

- **Object Recognition**:車や歩行者などの動物体の認識を行います。このモジュールはさらに、Detection、 Tracking、Predictionの3つの機能に分割されています。
  - **Detection**
    - 物体を認識
      - **Detector**
      - **Interpolator**
  - **Tracking**
    - 追跡
  - **Prediction**
    - 予測
- **Obstacle Segmentation**
  - 動物体に加え、静止障害物のような、衝突したくないものを検出します。例えば工事用のコーンはこのモジュールで認識しています。
- **Occupancy Grid Map**
  - 死角（情報が得られない、動物体が飛び出してくる可能性のある領域）を検出
- **Traffic Light Recognition**
  - 信号認識

## Component interface

The following describes the input/output concept between Perception Component and other components. See the [Perception Component Interface (WIP)](../../autoware-interfaces/components/perception.md) page for the current implementation.

### Input to the perception component

- **From Sensing**
- **From Localization**
- **From Map**
  - Vector map: lanelet filterに使用
  - Point Cloud Map: compare map filterに使用

### Output from the perception component

- **To Planning**

### Internal interface in the perception component

- **Obstacle Segmentation to Object Recognition**

- **Obstacle Segmentation to Occupancy Grid Map**

## How to add new modules (WIP)

## Supported Functions

## Reference Implementation

The following diagram describes the reference implementation of the Perception component. By adding new modules or extending the functionalities, various ODDs can be supported.

_Note that some implementation does not adhere to the high-level architecture design and require updating._

![reference-implementation](image/high-level-perception-diagram.drawio.svg)

For more details, please refer to the design documents in each package.

### Important Parameters

### Notation
