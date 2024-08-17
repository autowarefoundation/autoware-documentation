# Rosbag2 Anonymizer

## Overview

Autoware provides a tool ([autoware_rosbag2_anonymizer](https://github.com/autowarefoundation/autoware_rosbag2_anonymizer)) to anonymize ROS 2 bag files.
This tool is useful when you want to share your data with Autoware community but want to keep the privacy of the data.

With this tool you can blur any object (faces, license plates, etc.) in your bag files, and you can get a new bag file
with the blurred images.

## Installation

### Clone the repository

```bash
git clone https://github.com/autowarefoundation/autoware_rosbag2_anonymizer.git
cd autoware_rosbag2_anonymizer
```

### Download the pretrained models

```bash
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth

wget https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/GroundingDINO_SwinB.cfg.py
wget https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/groundingdino_swinb_cogcoor.pth

wget https://github.com/autowarefoundation/autoware_rosbag2_anonymizer/releases/download/v0.0.0/yolov8x_anonymizer.pt
wget https://github.com/autowarefoundation/autoware_rosbag2_anonymizer/releases/download/v0.0.0/yolo_config.yaml
```

### Install ROS 2 mcap dependencies if you will use mcap files

!!! warning

    Be sure you have installed the ROS 2 on your system.

```bash
sudo apt install ros-humble-rosbag2-storage-mcap
```

### Install `autoware_rosbag2_anonymizer` tool

Before installing the tool, you should update the pip package manager.

```bash
python3 -m pip install pip -U
```

Then, you can install the tool with the following command.

```bash
python3 -m pip install .
```

## Configuration

Define prompts in the `validation.json` file. The tool will use these prompts to detect objects. You can add your prompts
as dictionaries under the prompts key. Each dictionary should have two keys:

- `prompt`: The prompt that will be used to detect the object. This prompt will be blurred in the anonymization process.
- `should_inside`: This is a list of prompts that object should be inside. If the object is not inside the prompts, the
  tool will not blur the object.

```json
{
  "prompts": [
    {
      "prompt": "license plate",
      "should_inside": ["car", "bus", "..."]
    },
    {
      "prompt": "human face",
      "should_inside": ["person", "human body", "..."]
    }
  ]
}
```

You should set your configuration in the configuration files under config folder according to the usage. Following
instructions will guide you to set each configuration file.

- `config/anonymize_with_unified_model.yaml`

```yaml
rosbag:
  input_bags_folder: "/path/to/input_bag_folder" # Path to the input folder which contains ROS 2 bag files
  output_bags_folder: "/path/to/output_folder" # Path to the output ROS 2 bag folder
  output_save_compressed_image: True # Save images as compressed images (True or False)
  output_storage_id: "sqlite3" # Storage id for the output bag file (`sqlite3` or `mcap`)

grounding_dino:
  box_threshold: 0.1 # Threshold for the bounding box (float)
  text_threshold: 0.1 # Threshold for the text (float)
  nms_threshold: 0.1 # Threshold for the non-maximum suppression (float)

open_clip:
  score_threshold: 0.7 # Validity threshold for the OpenCLIP model (float

yolo:
  confidence: 0.15 # Confidence threshold for the YOLOv8 model (float)

bbox_validation:
  iou_threshold: 0.9 # Threshold for the intersection over union (float), if the intersection over union is greater than this threshold, the object will be selected as inside the validation prompt

blur:
  kernel_size: 31 # Kernel size for the Gaussian blur (int)
  sigma_x: 11 # Sigma x for the Gaussian blur (int)
```

- `config/yolo_create_dataset.yaml`

```yaml
rosbag:
  input_bags_folder: "/path/to/input_bag_folder" # Path to the input ROS 2 bag files folder

dataset:
  output_dataset_folder: "/path/to/output/dataset" # Path to the output dataset folder
  output_dataset_subsample_coefficient: 25 # Subsample coefficient for the dataset (int)

grounding_dino:
  box_threshold: 0.1 # Threshold for the bounding box (float)
  text_threshold: 0.1 # Threshold for the text (float)
  nms_threshold: 0.1 # Threshold for the non-maximum suppression (float)

open_clip:
  score_threshold: 0.7 # Validity threshold for the OpenCLIP model (float

bbox_validation:
  iou_threshold: 0.9 # Threshold for the intersection over union (float), if the intersection over union is greater than this threshold, the object will be selected as inside the validation prompt
```

- `config/yolo_train.yaml`

```yaml
dataset:
  input_dataset_yaml: "path/to/data.yaml" # Path to the config file of the dataset, which is created in the previous step

yolo:
  epochs: 100 # Number of epochs for the YOLOv8 model (int)
  model: "yolov8x.pt" # Select the base model for YOLOv8 ('yolov8x.pt' 'yolov8l.pt', 'yolov8m.pt', 'yolov8n.pt')
```

- `config/yolo_anonymize.yaml`

```yaml
rosbag:
  input_bag_path: "/path/to/input_bag/bag.mcap" # Path to the input ROS 2 bag file with 'mcap' or 'sqlite3' extension
  output_bag_path: "/path/to/output_bag_file" # Path to the output ROS 2 bag folder
  output_save_compressed_image: True # Save images as compressed images (True or False)
  output_storage_id: "sqlite3" # Storage id for the output bag file (`sqlite3` or `mcap`)

yolo:
  model: "path/to/yolo/model" # Path to the trained YOLOv8 model file (`.pt` extension) (you can download the pre-trained model from releases)
  config_path: "path/to/input/data.yaml" # Path to the config file of the dataset, which is created in the previous step
  confidence: 0.15 # Confidence threshold for the YOLOv8 model (float)

blur:
  kernel_size: 31 # Kernel size for the Gaussian blur (int)
  sigma_x: 11 # Sigma x for the Gaussian blur (int)
```

## Usage

The tool provides two options to anonymize images in ROS 2 bag files.

### Option 1: Anonymize with Unified Model

You should provide a single rosbag and tool anonymize images in rosbag with a unified model. The model is a combination
of GroundingDINO, OpenCLIP, YOLOv8 and SegmentAnything. If you don't want to use pre-trained YOLOv8 model, you can
follow the instructions in the second option to train your own YOLOv8 model.

You should set your configuration in config/anonymize_with_unified_model.yaml file.

```bash
python3 main.py config/anonymize_with_unified_model.yaml --anonymize_with_unified_model
```

### Option 2: Anonymize Using the YOLOv8 Model Trained on a Dataset Created with the Unified Model

#### Step 1: Create a Dataset

Create an initial dataset with the unified model. You can provide multiple ROS 2 bag files to create a dataset. After
running the following command, the tool will create a dataset in YOLO format.

You should set your configuration in config/yolo_create_dataset.yaml file.

```bash
python3 main.py config/yolo_create_dataset.yaml --yolo_create_dataset
```

#### Step 2: Manually Label the Missing Labels

The dataset which is created in the first step has some missing labels. You should label the missing labels manually.

#### Step 3: Split the Dataset

Split the dataset into training and validation sets. Give the path to the dataset folder which is created in the first
step.

```bash
autoware-rosbag2-anonymizer-split-dataset /path/to/dataset/folder
```

#### Step 4: Train the YOLOv8 Model

Train the YOLOv8 model with the dataset which is created in the first step.

You should set your configuration in config/yolo_train.yaml file.

```bash
python3 main.py config/yolo_train.yaml --yolo_train
```

#### Step 5: Anonymize Images in ROS 2 Bag Files

Anonymize images in ROS 2 bag files with the trained YOLOv8 model. If you want to anonymize your ROS 2 bag file with only
YOLOv8 model, you should use following command. But we recommend to use the unified model for better results. You can
follow the Option 1 for the unified model with the YOLOv8 model trained by you.

You should set your configuration in config/yolo_anonymize.yaml file.

```bash
python3 main.py config/yolo_anonymize.yaml --yolo_anonymize
```

## Share Your Anonymized Data

After anonymizing your data, you can share your anonymized data with the Autoware community. If you want to share your
data with the Autoware community, you should create an issue and pull request to
the [Autoware Documentation repository](https://github.com/autowarefoundation/autoware-documentation).

## Citation

```bibtex
@article{liu2023grounding,
  title={Grounding dino: Marrying dino with grounded pre-training for open-set object detection},
  author={Liu, Shilong and Zeng, Zhaoyang and Ren, Tianhe and Li, Feng and Zhang, Hao and Yang, Jie and Li, Chunyuan and Yang, Jianwei and Su, Hang and Zhu, Jun and others},
  journal={arXiv preprint arXiv:2303.05499},
  year={2023}
}
```

```bibtex
@article{kirillov2023segany,
  title={Segment Anything},
  author={Kirillov, Alexander and Mintun, Eric and Ravi, Nikhila and Mao, Hanzi and Rolland, Chloe and Gustafson, Laura and Xiao, Tete and Whitehead, Spencer and Berg, Alexander C. and Lo, Wan-Yen and Doll{\'a}r, Piotr and Girshick, Ross},
  journal={arXiv:2304.02643},
  year={2023}
}
```

```bibtex
@software{ilharco_gabriel_2021_5143773,
  author       = {Ilharco, Gabriel and
                  Wortsman, Mitchell and
                  Wightman, Ross and
                  Gordon, Cade and
                  Carlini, Nicholas and
                  Taori, Rohan and
                  Dave, Achal and
                  Shankar, Vaishaal and
                  Namkoong, Hongseok and
                  Miller, John and
                  Hajishirzi, Hannaneh and
                  Farhadi, Ali and
                  Schmidt, Ludwig},
  title        = {OpenCLIP},
  month        = jul,
  year         = 2021,
  note         = {If you use this software, please cite it as below.},
  publisher    = {Zenodo},
  version      = {0.1},
  doi          = {10.5281/zenodo.5143773},
  url          = {https://doi.org/10.5281/zenodo.5143773}
}
```