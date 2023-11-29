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

Detailed instructions for training the traffic light classifier model can be found **[here](https://github.com/autowarefoundation/autoware.universe/blob/main/perception/traffic_light_classifier/README.md)**.

<!--

Training traffic light detection model and lidar CenterPoint model will be added there.

-->

## Training yolox detection model for Traffic_light_fine_detector package

To train custom yolox traffic light detection models and convert them into ONNX format for deployment in Autoware, please refer to the instructions provided in the README file included with the
**"traffic_light_fine_detector"** package. These instructions will provide a step-by-step guide for training yolox model.

In order to assist you with your training process, we have also included a sample dataset in the Pascal VOC format.
This dataset contains 1062 cropped images of traffic lights and annotations.
You can utilize **[this example dataset](https://autoware-files.s3.us-west-2.amazonaws.com/dataset/traffic_light_detection_sample_dataset.tar.gz)** to facilitate your training efforts.

Detailed instructions for training the traffic light detector model can be found **[here](https://github.com/autowarefoundation/autoware.universe/blob/main/perception/traffic_light_fine_detector/README.md)**.