# Rosbag2 anonymizer

## Overview

Autoware provides a tool ([autoware_rosbag2_anonymizer](https://github.com/autowarefoundation/autoware_rosbag2_anonymizer)) to anonymize ROS 2 bag files.
This tool is useful when you want to share your data with Autoware community but want to keep the privacy of the data.

With this tool you can blur any object (faces, license plates, and other objects) in your bag files, and you can get a new bag file with the blurred images.

The tool combines four models: GroundingDINO, OpenCLIP, Segment Anything 2 (SAM 2), and YOLO.

## Installation

### Clone the repository

```bash
git clone https://github.com/autowarefoundation/autoware_rosbag2_anonymizer.git
cd autoware_rosbag2_anonymizer
```

### Download the pretrained models

```bash
wget https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_small.pt

wget https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/GroundingDINO_SwinB.cfg.py
wget https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/groundingdino_swinb_cogcoor.pth

wget https://github.com/autowarefoundation/autoware_rosbag2_anonymizer/releases/download/v1.0.0/yolo11x_anonymizer.pt
wget https://github.com/autowarefoundation/autoware_rosbag2_anonymizer/releases/download/v1.0.0/yolo_config.yaml
```

### Install ROS 2 mcap dependencies if you will use mcap files

!!! warning

    Install ROS 2 on your system first. Then source its setup file, for example `source /opt/ros/jazzy/setup.bash`. The command below needs the `ROS_DISTRO` variable that this file sets.

```bash
sudo apt install ros-$ROS_DISTRO-rosbag2-storage-mcap
```

### Install `autoware_rosbag2_anonymizer` tool

The upstream README recommends a Python virtual environment for the tool:

```bash
python3 -m venv .venv
. .venv/bin/activate
```

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

The examples show the keys that the code of the tool reads. The model paths point to the files that you downloaded in the installation step. The configuration files in the repository can miss keys or use old key names. Copy the keys from the examples on this page.

- `config/anonymize_with_unified_model.yaml`

```yaml
rosbag:
  input_bags_folder: "/path/to/input_bag_folder" # Path to the input folder which contains ROS 2 bag files
  output_bags_folder: "/path/to/output_folder" # Path to the output ROS 2 bag folder
  output_save_compressed_image: True # Save images as compressed images (True or False)
  output_storage_id: "sqlite3" # Storage id for the output bag file (`sqlite3` or `mcap`)

grounding_dino:
  config_path: "./GroundingDINO_SwinB.cfg.py" # Path to the GroundingDINO configuration file
  checkpoint_path: "./groundingdino_swinb_cogcoor.pth" # Path to the GroundingDINO weights
  box_threshold: 0.1 # Threshold for the bounding box (float)
  text_threshold: 0.1 # Threshold for the text (float)
  nms_threshold: 0.1 # Threshold for the non-maximum suppression (float)

segment_anything_2:
  model_cfg: "configs/sam2.1/sam2.1_hiera_s.yaml" # SAM 2 model configuration
  checkpoint_path: "./sam2.1_hiera_small.pt" # Path to the SAM 2 weights

openclip:
  run: True # Use OpenCLIP to validate the detections (True or False)
  model_name: "ViT-B-32" # OpenCLIP model name
  pretrained_model: "laion2b_s34b_b79k" # OpenCLIP pretrained weights
  score_threshold: 0.7 # Validity threshold for the OpenCLIP model (float)

yolo:
  model: "./yolo11x_anonymizer.pt" # Path to the YOLO model file (`.pt` extension)
  config_path: "./yolo_config.yaml" # Path to the YOLO dataset configuration file
  confidence: 0.15 # Confidence threshold for the YOLO model (float)

bbox_validation:
  iou_threshold: 0.9 # Threshold for the intersection over union (float), if the intersection over union is greater than this threshold, the object will be selected as inside the validation prompt

blur:
  region: "mask" # Blur the segmentation mask or the whole bounding box (`mask` or `bbox`)
  kernel_size: 31 # Kernel size for the Gaussian blur (int)
  sigma_x: 19 # Sigma x for the Gaussian blur (int)

debug:
  show_on_image: True # Show the detections on the images (True or False)
  print_on_terminal: True # Print the detections on the terminal (True or False)
```

- `config/yolo_create_dataset.yaml`

```yaml
rosbag:
  input_bags_folder: "/path/to/input_bag_folder" # Path to the input ROS 2 bag files folder

dataset:
  output_dataset_folder: "/path/to/output/dataset" # Path to the output dataset folder
  output_dataset_subsample_coefficient: 25 # Subsample coefficient for the dataset (int)

grounding_dino:
  config_path: "./GroundingDINO_SwinB.cfg.py"
  checkpoint_path: "./groundingdino_swinb_cogcoor.pth"
  box_threshold: 0.1 # Threshold for the bounding box (float)
  text_threshold: 0.1 # Threshold for the text (float)
  nms_threshold: 0.1 # Threshold for the non-maximum suppression (float)

openclip:
  run: True # Use OpenCLIP to validate the detections (True or False)
  model_name: "ViT-B-32"
  pretrained_model: "laion2b_s34b_b79k"
  score_threshold: 0.7 # Validity threshold for the OpenCLIP model (float)

yolo:
  model: "./yolo11x_anonymizer.pt"
  config_path: "./yolo_config.yaml"
  confidence: 0.15 # Confidence threshold for the YOLO model (float)

bbox_validation:
  iou_threshold: 0.9 # Threshold for the intersection over union (float), if the intersection over union is greater than this threshold, the object will be selected as inside the validation prompt

debug:
  show_on_image: False
  print_on_terminal: False
```

- `config/yolo_train.yaml`

```yaml
dataset:
  input_dataset_yaml: "path/to/data.yaml" # Path to the config file of the dataset, which is created in the previous step

yolo:
  epochs: 100 # Number of epochs for the YOLO model (int)
  model: "yolo11x.pt" # Select the base model for YOLO ('yolo11x.pt', 'yolo11l.pt', 'yolo11m.pt', 'yolo11s.pt', 'yolo11n.pt')
```

- `config/yolo_anonymize.yaml`

```yaml
rosbag:
  input_bag_path: "/path/to/input_bag/bag.mcap" # Path to the input ROS 2 bag file with 'mcap' or 'sqlite3' extension
  output_bag_path: "/path/to/output_bag_file" # Path to the output ROS 2 bag folder
  output_save_compressed_image: True # Save images as compressed images (True or False)
  output_storage_id: "sqlite3" # Storage id for the output bag file (`sqlite3` or `mcap`)

yolo:
  model: "path/to/yolo/model" # Path to the trained YOLO model file (`.pt` extension) (you can download the pre-trained model from releases)
  config_path: "path/to/input/data.yaml" # Path to the config file of the dataset, which is created in the previous step
  confidence: 0.15 # Confidence threshold for the YOLO model (float)

segment_anything_2:
  model_cfg: "configs/sam2.1/sam2.1_hiera_s.yaml"
  checkpoint_path: "./sam2.1_hiera_small.pt"

blur:
  region: "mask" # Blur the segmentation mask or the whole bounding box (`mask` or `bbox`)
  kernel_size: 31 # Kernel size for the Gaussian blur (int)
  sigma_x: 9 # Sigma x for the Gaussian blur (int)

debug:
  show_on_image: False
  print_on_terminal: False
```

- `config/validation.yaml`

```yaml
dataset:
  input_dataset_yaml: "path/to/data.yaml" # Path to the config file of the dataset in YOLO format
  max_samples: -1 # Number of samples to validate (int), -1 for all samples in the dataset

grounding_dino:
  config_path: "./GroundingDINO_SwinB.cfg.py"
  checkpoint_path: "./groundingdino_swinb_cogcoor.pth"
  box_threshold: 0.1 # Threshold for the bounding box (float)
  text_threshold: 0.1 # Threshold for the text (float)
  nms_threshold: 0.1 # Threshold for the non-maximum suppression (float)

openclip:
  run: True # Use OpenCLIP to validate the detections (True or False)
  model_name: "ViT-B-32"
  pretrained_model: "laion2b_s34b_b79k"
  score_threshold: 0.7 # Validity threshold for the OpenCLIP model (float)

yolo:
  model: "./yolo11x_anonymizer.pt"
  config_path: "./yolo_config.yaml"
  confidence: 0.15 # Confidence threshold for the YOLO model (float)

bbox_validation:
  iou_threshold: 0.9 # Threshold for the intersection over union (float), if the intersection over union is greater than this threshold, the object will be selected as inside the validation prompt
```

## Usage

The tool provides two options to anonymize images in ROS 2 bag files.

!!! warning

    If your ROS 2 bag file includes custom message types from Autoware or any other packages, you should source the
    their workspaces before running the tool.

    You can source Autoware workspace with the following command.
    ```bash
    source /path/to/your/workspace/install/setup.bash
    ```

### Option 1: Anonymize with the unified model

Give the folder that contains your ROS 2 bag files, and the tool anonymizes the images in each bag file with a unified model. The model is a combination of GroundingDINO, OpenCLIP, YOLO, and SAM 2. If you do not want to use the pre-trained YOLO model, follow the instructions in the second option to train your own YOLO model.

You should set your configuration in config/anonymize_with_unified_model.yaml file.

```bash
python3 main.py config/anonymize_with_unified_model.yaml --anonymize_with_unified_model
```

### Option 2: Anonymize using the YOLO model trained on a dataset created with the unified model

#### Step 1: Create a dataset

Create an initial dataset with the unified model. You can provide multiple ROS 2 bag files to create a dataset. After
running the following command, the tool will create a dataset in YOLO format.

You should set your configuration in config/yolo_create_dataset.yaml file.

```bash
python3 main.py config/yolo_create_dataset.yaml --yolo_create_dataset
```

#### Step 2: Manually label the missing labels

The dataset which is created in the first step has some missing labels. You should label the missing labels manually.
You can use the following example tools to label the missing labels:

- [label-studio](https://github.com/HumanSignal/label-studio)
- [Roboflow](https://roboflow.com/) (You can use the free version)

#### Step 3: Split the dataset

Split the dataset into training and validation sets. Give the path to the dataset folder which is created in the first
step.

```bash
autoware-rosbag2-anonymizer-split-dataset /path/to/dataset/folder
```

#### Step 4: Train the YOLO model

Train the YOLO model with the dataset which is created in the first step.

You should set your configuration in config/yolo_train.yaml file.

```bash
python3 main.py config/yolo_train.yaml --yolo_train
```

#### Step 5: Anonymize images in ROS 2 bag files

Anonymize the images in ROS 2 bag files with the trained YOLO model. To use only the YOLO model, run the following command. For better results, use the unified model of Option 1 with your trained YOLO model.

You should set your configuration in config/yolo_anonymize.yaml file.

```bash
python3 main.py config/yolo_anonymize.yaml --yolo_anonymize
```

### Validate the tool

You can measure the results of the unified model on a dataset in YOLO format. The validation calculates the precision, the recall, and the mAP scores of the dataset. Set your configuration in the `config/validation.yaml` file, then run:

```bash
python3 main.py config/validation.yaml --validation
```

For the metrics and the results that the authors measured, see the [validation document](https://github.com/autowarefoundation/autoware_rosbag2_anonymizer/blob/main/docs/validation.md) of the tool.

## Troubleshooting

- **Error 1**: `torch.OutOfMemoryError: CUDA out of memory`

```bash
torch.OutOfMemoryError: CUDA out of memory. Tried to allocate 1024.00 MiB. GPU 0 has a total capacity of 10.87 GiB of which 1010.88 MiB is free. Including non-PyTorch memory, this process has 8.66 GiB memory in use. Of the allocated memory 8.21 GiB is allocated by PyTorch, and 266.44 MiB is reserved by PyTorch but unallocated. If reserved but unallocated memory is large try setting PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True to avoid fragmentation.  See documentation for Memory Management  (https://pytorch.org/docs/stable/notes/cuda.html#environment-variables)
```

This error occurs when the GPU memory is not enough to run the model. You can add the following environment variable to
avoid this error.

```bash
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
```

For the GPU memory that each model needs, see the [time and requirement analysis](https://github.com/autowarefoundation/autoware_rosbag2_anonymizer/blob/main/docs/time_and_requirent_analysis.md) of the tool.

- **Error 2**: The tool takes a long time to process the images.

The tool runs several models on each image (GroundingDINO, OpenCLIP, YOLO, and SAM 2), so it can take a long time. The [time and requirement analysis](https://github.com/autowarefoundation/autoware_rosbag2_anonymizer/blob/main/docs/time_and_requirent_analysis.md) shows the time that each model needs.

## Share your anonymized data

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
