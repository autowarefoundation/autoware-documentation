# Training and Deploying Models

## Overview

The Autoware offers a comprehensive array of machine learning models, tailored for a wide range of tasks including 2D and 3D object detection,
traffic light recognition and more. These models have been meticulously trained utilizing **[open-mmlab](https://github.com/open-mmlab)**'s extensive repositories.
By leveraging the provided scripts and following the training steps, you have the capability to train these models using your own dataset,
tailoring them to your specific needs.

Furthermore, you will find the essential conversion scripts to deploy your trained models into Autoware using the **[mmdeploy](https://github.com/open-mmlab/mmdeploy)** repository.

## Training traffic light classifier model

The traffic light classifier model within the Autoware has been trained using the **[mmlab/pretrained](https://github.com/open-mmlab/mmpretrain)** repository.
The Autoware offers pretrained models based on EfficientNet-b1 and MobileNet-v2 architectures.
To fine-tune these models, a total of 83,400 images were employed, comprising 58,600 for training,
14,800 for evaluation, and 10,000 for testing. These images represent Japanese traffic lights and were trained using TIER IV's internal dataset.

| Name            | Input Size | Test Accuracy |
| --------------- | ---------- | ------------- |
| EfficientNet-b1 | 128 x 128  | 99.76%        |
| MobileNet-v2    | 224 x 224  | 99.81%        |

Comprehensive training instructions for the traffic light classifier model are detailed within
the readme file accompanying **"traffic_light_classifier"** package. These instructions will guide you through
the process of training the model using your own dataset. To facilitate your training, we have also provided
an example dataset containing three distinct classes (green, yellow, red), which you can leverage during the training process.

Detailed instructions for training the traffic light classifier model can be found **[here](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_traffic_light_classifier/README.md)**.

## Training CenterPoint 3D object detection model

The CenterPoint 3D object detection model within the Autoware has been trained using the **[autowarefoundation/mmdetection3d](https://github.com/autowarefoundation/mmdetection3d/blob/main/projects/AutowareCenterPoint/README.md)** repository.

To train custom CenterPoint models and convert them into ONNX format for deployment in Autoware, please refer to the instructions provided in the README file included with Autoware's
**[lidar_centerpoint](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_lidar_centerpoint/)** package. These instructions will provide a step-by-step guide for training the CenterPoint model.

In order to assist you with your training process, we have also included an example dataset in the TIER IV dataset format.

This dataset contains 600 lidar frames and covers 5 classes, including 6905 cars, 3951 pedestrians, 75 cyclists, 162 buses, and 326 trucks.

You can utilize this example dataset to facilitate your training efforts.

## Training a YOLOX model for `autoware_traffic_light_fine_detector`

To train a custom YOLOX model for use with the **Autoware Traffic Light Fine Detector**, please refer to the official YOLOX and Autoware training guides listed below. These documents provide the required setup, data preparation steps, and export instructions.

???+ abstract "Relevant documentation"

    <div class="grid cards" markdown>

    -   **Autoware traffic_light_fine_detector README**

        ---

        Training overview, model requirements, and ONNX export details:
        [autoware_universe/perception/autoware_traffic_light_fine_detector/README.md](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_fine_detector/README.md)

    -   **YOLOX Custom Dataset Training Guide**

        ---

        Instructions for preparing datasets, configuring experiments, and launching training:
        [YOLOX/train_custom_data.md](https://github.com/Megvii-BaseDetection/YOLOX/blob/main/docs/train_custom_data.md)

    </div>

### 📦 Example Dataset (VOC Format)

To assist with training, an example dataset containing **1062 cropped traffic-light images with Pascal VOC annotations** is available here:

[:fa-cl-s fa-cloud-arrow-down: Download the sample traffic light dataset (3 MB)](https://autoware-files.s3.us-west-2.amazonaws.com/dataset/traffic_light_detection_sample_dataset.tar.gz){ .md-button }

- Use the YOLOX documentation to set up your environment and prepare your dataset (VOC or COCO).
- Train a YOLOX model using your custom traffic-light data or the provided sample dataset.
- After training, export the model to **ONNX** following the instructions in the Autoware `traffic_light_fine_detector` README.
- Replace or integrate the exported ONNX model within the Autoware package for deployment.
